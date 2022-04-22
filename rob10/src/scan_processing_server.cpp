#include <iostream>
#include <sstream>
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "rob10/requestHumanPose.h"


class ScannerServer{
  public:
    ScannerServer(){
      sub_scan = n.subscribe("temp", 1, &ScannerServer::scanCallback, this);
      serverRequestHumanPose = n.advertiseService("RequestHumanPose", &ScannerServer::getHumanPose, this);
    }
    
  private:
    ros::NodeHandle n;
    ros::ServiceServer serverRequestHumanPose;
    ros::Subscriber sub_scan;
    std::vector<float> scan;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
      std::cout<<"TODO"<<std::endl;
      scan = msg->ranges;
      std::cout<<scan.size()<<std::endl;

    }

    bool getHumanPose(rob10::requestHumanPose::Request& req, rob10::requestHumanPose::Response& res){
      std::cout<<"TODO"<<std::endl;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_processing_server_node");

  ScannerServer server;
  //camera.initializeRealsense();
  //camera.update();

  //ros::Rate loop_rate(30);
  while(ros::ok()){
    //camera.publishPointcloud();
    ros::spinOnce();
    //loop_rate.sleep();
  }

  return 0;
}
