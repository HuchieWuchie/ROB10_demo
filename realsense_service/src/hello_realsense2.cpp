#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>

#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
//import sensor_msgs.point_cloud2 as pc2
#include "sensor_msgs/PointField.h"
#include <cv_bridge/cv_bridge.h>

#include "realsense_service/capture.h"
#include "realsense_service/depth.h"
#include "realsense_service/intrinsics.h"
// intrinsics.srv is not used
#include "realsense_service/pointcloud.h"
#include "realsense_service/rgb.h"
#include "realsense_service/uvSrv.h"

class RealsenseServer{
  public:
    int cam_width;
    int cam_height;
    int tempFilterSize;
    std::string baseService;
    //captured information variables initialized to 0 or empty
    int color_frame;
    int scolor_image;
    int depth_frame;
    int depth_image;
    int cloudGeometry;
    int cloudColor;
    int cloudGeometryStatic;
    int cloudColorStatic;
    int colorImageStatic;
    int depthImageStatic;
    int uv;
    int uvStatic;
    int frame_no;
    // TODO
    //int framesRGB = [];
    //int framesDepth = [];

    //self.br = CvBridge()
    cv_bridge::CvImagePtr br;

    // TODO
    /*self.FIELDS_XYZ = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]*/

    //Initialize ROS
    ros::NodeHandle n;
    ros::ServiceServer serviceCapture;
    ros::ServiceServer serviceCaptureDepth;
    ros::ServiceServer serviceCaptureRGB;
    ros::ServiceServer serviceUVStatic;
    ros::ServiceServer servicePointCloudStatic;

    ros::Publisher pubPointCloudGeometryStatic;
    ros::Publisher pubStaticRGB;
    ros::Publisher pubStaticDepth;
    ros::Publisher pubPointCloudGeometryStaticRGB;
    //ros::Rate loop_rate(30);

    bool updateStatic(realsense_service::capture::Request& req, realsense_service::capture::Response& res){
      std::cout <<"TODO" << std::endl;
      return true;
    }

    bool serviceSendDepthImageStatic(realsense_service::depth::Request& req, realsense_service::depth::Response& res){
      std::cout <<"TODO" << std::endl;
      return true;
    }

    bool serviceSendRGBImageStatic(realsense_service::rgb::Request& req, realsense_service::rgb::Response& res){
      std::cout <<"TODO" << std::endl;
      return true;
    }

    bool serviceGetUVStatic(realsense_service::uvSrv::Request& req, realsense_service::uvSrv::Response& res){
      std::cout <<"TODO" << std::endl;
      return true;
    }

    bool serviceGetPointCloud(realsense_service::pointcloud::Request& req, realsense_service::pointcloud::Response& res){
      std::cout <<"TODO" << std::endl;
      return true;
    }

    //Initialize realsense package
    rs2::config cfg;
    rs2::pipeline pipe;

    RealsenseServer();
    void initializeRealsense();
    void update();
};

//CONSTRUCTOR, CHANGE VALUES HERE
RealsenseServer::RealsenseServer() {
    cam_width = 1280;
    cam_height = 720;
    tempFilterSize = 10;
    baseService = "/sensors/realsense";
    color_frame = 0;
    scolor_image = 0;
    depth_frame = 0;
    depth_image = 0;
    cloudGeometry = 0;
    cloudColor = 0;
    cloudGeometryStatic = 0;
    cloudColorStatic = 0;
    colorImageStatic = 0;
    depthImageStatic = 0;
    uv = 0;
    uvStatic = 0;
    // TODO
    //framesRGB = [];
    //framesDepth = [];

    // TOD
    /*self.FIELDS_XYZ = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]*/

    //Initialize ROS
    //serviceCapture = rospy.Service(self.baseService + '/capture', capture, self.updateStatic)
    serviceCapture = n.advertiseService(baseService + "/capture", &RealsenseServer::updateStatic, this);
    //self.serviceCaptureDepth = rospy.Service(self.baseService + '/depth', depth, self.serviceSendDepthImageStatic)
    serviceCaptureDepth = n.advertiseService(baseService + "/depth", &RealsenseServer::serviceSendDepthImageStatic, this);
    //self.serviceCaptureRGB = rospy.Service(self.baseService + '/rgb', rgb, self.serviceSendRGBImageStatic)
    serviceCaptureRGB = n.advertiseService(baseService + "/rgb", &RealsenseServer::serviceSendRGBImageStatic, this);
    //self.serviceUVStatic = rospy.Service(self.baseService + '/pointcloud/static/uv', uvSrv, self.serviceUVStatic)
    serviceUVStatic = n.advertiseService(baseService + "/pointcloud/static/uv", &RealsenseServer::serviceGetUVStatic, this);
    //self.servicePointCloudStatic = rospy.Service(self.baseService + '/pointcloud/static', pointcloud, self.servicePointCloud)*/
    servicePointCloudStatic = n.advertiseService(baseService + "/pointcloud/static", &RealsenseServer::serviceGetPointCloud, this);

    pubPointCloudGeometryStatic = n.advertise<sensor_msgs::PointCloud2>(baseService + "/pointcloudGeometry/static", 1);
    pubStaticRGB = n.advertise<sensor_msgs::Image>(baseService + "/rgb/static", 1);
    pubStaticDepth = n.advertise<sensor_msgs::Image>(baseService + "/depth/static", 1);
    pubPointCloudGeometryStaticRGB = n.advertise<std_msgs::Float32MultiArray>(baseService + "/pointcloudGeometry/static/rgb", 1);
    ros::Rate loop_rate(30);
    frame_no = 0;

    cfg.enable_stream(RS2_STREAM_COLOR, cam_width, cam_height, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, cam_width, cam_height, RS2_FORMAT_Z16, 30);
}

void RealsenseServer::initializeRealsense(){
    std::cout << "initialize" << std::endl;
    pipe.start(cfg);
    int count = 0;
    //HIGH ACCURACY PRESET EXPLAINED -https://dev.intelrealsense.com/docs/d400-series-visual-presets
    //DO WE STILL WANT THIS?

    while (ros::ok()) {
      RealsenseServer::update();
    }
}

void RealsenseServer::update(){
  rs2::frameset frames = pipe.wait_for_frames();
  //rs2::frame frame = frames.first(RS2_STREAM_DEPTH);
  auto frame_depth = frames.get_depth_frame();
  auto frame_color = frames.get_color_frame();
  if (frame_depth && frame_color){
    //frame.get_data();
    float dist_to_center = frame_depth.get_distance(cam_width / 2, cam_height / 2);
    std::cout << "The camera is facing an object " << dist_to_center << " meters away" << std::endl;

    // ALIGN THE STREAMS
    rs2::align align(RS2_STREAM_COLOR);
    rs2::frameset aligned_frames = align.process(frames);

    // GET POINTCLOUD
    rs2::pointcloud pc;
    rs2::points points = pc.calculate(aligned_frames.get_depth_frame());
    pc.map_to(aligned_frames.get_color_frame());
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "realsense_service_cpp_node");
  RealsenseServer RSObj;
  RSObj.initializeRealsense();
  //while (ros::ok()) {
    //count++;
  //}
  /*
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    rs2::context ctx;
    std::cout << "hello from librealsense - " << RS2_API_VERSION_STR << std::endl;
    std::cout << "You have " << ctx.query_devices().size() << " RealSense devices connected" << std::endl;

    while (ros::ok())
    {
      std_msgs::String msg;
      msg.data = "hello";
      chatter_pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
    }*/

    return 0;
}