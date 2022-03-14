#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>

#include <iostream>
#include <sstream>

#include "ros/ros.h"
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
    // TODO
    /*self.serviceCapture = rospy.Service(self.baseService + '/capture', capture, self.updateStatic)
    self.serviceCaptureDepth = rospy.Service(self.baseService + '/depth', depth, self.serviceSendDepthImageStatic)
    self.serviceCaptureRGB = rospy.Service(self.baseService + '/rgb', rgb, self.serviceSendRGBImageStatic)
    self.serviceUVStatic = rospy.Service(self.baseService + '/pointcloud/static/uv', uvSrv, self.serviceUVStatic)
    self.servicePointCloudStatic = rospy.Service(self.baseService + '/pointcloud/static', pointcloud, self.servicePointCloud)*/
    ros::Publisher pubPointCloudGeometryStatic;
    ros::Publisher pubStaticRGB;
    ros::Publisher pubStaticDepth;
    ros::Publisher pubPointCloudGeometryStaticRGB;
    //ros::Rate loop_rate(30);

    //CONSTRUCTOR
    RealsenseServer();
    //Initialize realsense package
    void initializeRealsense();
};

//CONSTRUCTOR, CHANGE VALUES HERE
RealsenseServer::RealsenseServer() {
    std::cout << "CONSTRUCTOR\r";
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
    // TODO
    /*self.serviceCapture = rospy.Service(self.baseService + '/capture', capture, self.updateStatic)
    self.serviceCaptureDepth = rospy.Service(self.baseService + '/depth', depth, self.serviceSendDepthImageStatic)
    self.serviceCaptureRGB = rospy.Service(self.baseService + '/rgb', rgb, self.serviceSendRGBImageStatic)
    self.serviceUVStatic = rospy.Service(self.baseService + '/pointcloud/static/uv', uvSrv, self.serviceUVStatic)
    self.servicePointCloudStatic = rospy.Service(self.baseService + '/pointcloud/static', pointcloud, self.servicePointCloud)*/
    pubPointCloudGeometryStatic = n.advertise<sensor_msgs::PointCloud2>(baseService + "/pointcloudGeometry/static", 1);
    pubStaticRGB = n.advertise<sensor_msgs::Image>(baseService + "/rgb/static", 1);
    pubStaticDepth = n.advertise<sensor_msgs::Image>(baseService + "/dept/static", 1);
    pubPointCloudGeometryStaticRGB = n.advertise<std_msgs::Float32MultiArray>(baseService + "/pointcloudGeometry/static/rgb", 1);
    ros::Rate loop_rate(30);
    frame_no = 0;
}

void RealsenseServer::initializeRealsense(){
    std::cout << "initialize \r";
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, cam_width, cam_height, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, cam_width, cam_height, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    pipe.start(cfg);

    rs2::frameset frames = pipe.wait_for_frames();
    //rs2::frame frame = frames.first(RS2_STREAM_DEPTH);
    rs2::depth_frame depth_frame = frames.get_depth_frame();
    if (depth_frame){
      //frame.get_data();
      float dist_to_center = depth_frame.get_distance(cam_width / 2, cam_height / 2);
      std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "realsense_service");
  RealsenseServer RealsenseServerObj;
  while (ros::ok()) {
    RealsenseServerObj.initializeRealsense();
  }

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
