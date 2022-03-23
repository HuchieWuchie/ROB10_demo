#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>

#include <iostream>
#include <sstream>
#include <unistd.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "realsense_service/capture.h"
#include "realsense_service/depth.h"
#include "realsense_service/intrinsics.h"
// intrinsics.srv is not used
#include "realsense_service/pointcloud.h"
#include "realsense_service/rgb.h"
#include "realsense_service/uvSrv.h"

typedef std::tuple<uint8_t, uint8_t, uint8_t> RGB_tuple;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class RealsenseServer{
  public:
    int cam_width;
    int cam_height;
    //int tempFilterSize;
    std::string baseService;
    bool capture;
    std::vector<RGB_tuple> cloudColor;
    std::vector<int> u, v;

    //Initialize ROS variables and functions
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

    bool updateStatic(realsense_service::capture::Request& req, realsense_service::capture::Response& res){
      if (req.capture.data){
        ros::Time time_now = ros::Time::now();
        std::cout<<"[" << time_now <<"] Updating statics "<<std::endl;
        update();
        res.success.data = true;
      } else {
        std::cout<<"Client contacted the server but did not asked to capture new statics"<<std::endl;
        res.success.data = false;
      }
      return true;
    }

    bool serviceSendDepthImageStatic(realsense_service::depth::Request& req, realsense_service::depth::Response& res){
      //auto frame_depth = aligned_frames.get_depth_frame();
      //std::cout<<"width " << frame_depth.get_width() << std::endl;
      //std::cout<<"height " << frame_depth.get_height() << std::endl;
      // NOT SURE IF CV_16UC1 OR CV_8UC1, MORE OPTIONS https://stackoverflow.com/questions/13428689/whats-the-difference-between-cvtype-values-in-opencv
      rs2::depth_frame frame_depth(processed_frame);
      cv::Mat image(cv::Size(frame_depth.get_width(), frame_depth.get_height()), CV_8UC1, (void*)frame_depth.get_data(), cv::Mat::AUTO_STEP);
      //cv::Mat image(cv::Size(frame_depth.get_width(), frame_depth.get_height()), CV_32F, (void*)frame_depth.get_data(), cv::Mat::AUTO_STEP);
      cv::imwrite("my_img.png", image);
      sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
      res.img = *img_msg;
      return true;
    }

    bool serviceSendRGBImageStatic(realsense_service::rgb::Request& req, realsense_service::rgb::Response& res){
      rs2::video_frame frame_color(processed_frame);
      cv::Mat image(cv::Size(frame_color.get_width(), frame_color.get_height()), CV_8UC3, (void*)frame_color.get_data(), cv::Mat::AUTO_STEP);
      //cv::imwrite("my_img.png", image);
      sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      res.img = *img_msg;
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

    //Initialize realsense variables and function
    rs2::config cfg;
    rs2::pipeline pipe;
    /*REMOVE WHEN DONE*/ rs2::frameset frames;
    /*REMOVE WHEN DONE*/ rs2::frameset aligned_frames;
    rs2::pointcloud pc;
    /*REMOVE WHEN DONE*/ rs2::points points;
    //const rs2::texture_coordinate* uv;
    rs2::frame processed_frame;

    RealsenseServer();
    void initializeRealsense();
    void update();
    void generateStatics();
    void publishPointcloud();
    PointCloud::Ptr points_to_pcl(const rs2::points& points);
    /*TUPLE COULD BE TURNED INTO AN ARRAY*/ RGB_tuple get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);

};

//CONSTRUCTOR, CHANGE VALUES HERE
RealsenseServer::RealsenseServer() {
    cam_width = 1280;
    cam_height = 720;
    baseService = "/sensors/realsense";

    //Initialize ROS
    serviceCapture = n.advertiseService(baseService + "/capture", &RealsenseServer::updateStatic, this);
    serviceCaptureDepth = n.advertiseService(baseService + "/depth", &RealsenseServer::serviceSendDepthImageStatic, this);
    serviceCaptureRGB = n.advertiseService(baseService + "/rgb", &RealsenseServer::serviceSendRGBImageStatic, this);
    serviceUVStatic = n.advertiseService(baseService + "/pointcloud/static/uv", &RealsenseServer::serviceGetUVStatic, this);
    servicePointCloudStatic = n.advertiseService(baseService + "/pointcloud/static", &RealsenseServer::serviceGetPointCloud, this);

    pubPointCloudGeometryStatic = n.advertise<sensor_msgs::PointCloud2>(baseService + "/pointcloudGeometry/static", 1);
    pubStaticRGB = n.advertise<sensor_msgs::Image>(baseService + "/rgb/static", 1);
    pubStaticDepth = n.advertise<sensor_msgs::Image>(baseService + "/depth/static", 1);
    pubPointCloudGeometryStaticRGB = n.advertise<std_msgs::Float32MultiArray>(baseService + "/pointcloudGeometry/static/rgb", 1);

    cfg.enable_stream(RS2_STREAM_COLOR, cam_width, cam_height, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, cam_width, cam_height, RS2_FORMAT_Z16, 30);
}

void RealsenseServer::initializeRealsense(){
    std::cout << "initializing" << std::endl;
    pipe.start(cfg);
    //DROP STARTUP FRAMES
    for(int i = 0; i < 50; i++){
      pipe.wait_for_frames();
    }
    //RealsenseServer::update();
    //HIGH ACCURACY PRESET EXPLAINED -https://dev.intelrealsense.com/docs/d400-series-visual-presets
    //DO WE STILL WANT THIS?
}

void RealsenseServer::update(){
    frames = pipe.wait_for_frames();
    auto frame_depth = frames.get_depth_frame();
    auto frame_color = frames.get_color_frame();

    if (frame_depth && frame_color){
      // ALIGN THE STREAMS
      rs2::align align(RS2_STREAM_COLOR);
      aligned_frames = align.process(frames);

      // filtering CHANGE AS NEEDED
      rs2::hole_filling_filter hole_filter(2);
      rs2::decimation_filter dec_filter;
      rs2::threshold_filter thr_filter;
      thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.3f);
      thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 1.0f);
      processed_frame = hole_filter.process(dec_filter.process(thr_filter.process(aligned_frames.get_depth_frame())));

      //rs2::depth_frame filteredDepthFrame(processed_frame);
      //float dist_to_center = filteredDepthFrame.get_distance(filteredDepthFrame.get_width()/2, filteredDepthFrame.get_height()/2);
      //std::cout << "The camera is facing an object " << dist_to_center << " meters away" << std::endl;
      //GET POINTCLOUD
      RealsenseServer::generateStatics();
    }
}

void RealsenseServer::generateStatics(){
  //Clean previous values
  u.clear();
  v.clear();
  cloudColor.clear();

  //https://github.com/Resays/xyz_rgb_realsense/tree/master
  rs2::video_frame color(processed_frame);
  rs2::depth_frame depth(processed_frame);
  pc.map_to(color);
  points = pc.calculate(depth);

  //uv = points.get_texture_coordinates();
  const rs2::texture_coordinate* uv = points.get_texture_coordinates();
  for (size_t i = 0; i < points.size(); i++) {
    int temp = uv[i].u * depth.get_width();
    u.push_back(temp);
    temp = uv[i].v * depth.get_height();
    v.push_back(temp);
  };

  for (size_t i = 0; i < points.size(); i++) {
    RGB_tuple current_color = get_texcolor(color, uv[i]);
    cloudColor.push_back(current_color);
  }
}

RGB_tuple RealsenseServer::get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords) {
  //SOURCE - https://github.com/Resays/xyz_rgb_realsense/blob/master/xyz_rgb_realsense.cpp
	const int w = texture.get_width(), h = texture.get_height();
	int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);
	int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
	return std::tuple<uint8_t, uint8_t, uint8_t>( texture_data[idx], texture_data[idx + 1], texture_data[idx + 2] );
}

void RealsenseServer::publishPointcloud(){
  PointCloud::Ptr pcl_pc = points_to_pcl(points);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(*pcl_pc, pcl_pc2);
  sensor_msgs::PointCloud2 msg_pc2;
  pcl_conversions::fromPCL(pcl_pc2, msg_pc2);
  msg_pc2.header.frame_id = "map";
  msg_pc2.height = 1;
  msg_pc2.width = points.size();
  pubPointCloudGeometryStatic.publish(msg_pc2);
}

PointCloud::Ptr RealsenseServer::points_to_pcl(const rs2::points& points){
  //SOURCE - https://github.com/IntelRealSense/librealsense/blob/master/wrappers/pcl/pcl/rs-pcl.cpp
  PointCloud::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  rs2::depth_frame depth(processed_frame);
  cloud->width = depth.get_width();
  cloud->height = depth.get_height();
  cloud->is_dense = false;
  cloud->points.resize(points.size());
  auto ptr = points.get_vertices();
  for (auto& p : cloud->points)
  {
      p.x = ptr->x;
      p.y = ptr->y;
      p.z = ptr->z;
      ptr++;
  }
  return cloud;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "realsense_service_cpp_node");

  RealsenseServer camera;
  camera.initializeRealsense();
  camera.update();

  ros::Rate loop_rate(60);
  while(ros::ok()){
    camera.publishPointcloud();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
