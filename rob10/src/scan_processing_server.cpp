//sudo chmod a+rw /dev/ttyACM0
//rosrun urg_node urg_node

#include <iostream>
#include <sstream>
#include <unistd.h>
#include <algorithm>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "rob10/requestHumanPose.h"


class ScannerServer{
  public:
    ScannerServer(){
      sub_scan = n.subscribe("scan", 1, &ScannerServer::scanCallback, this);
      pub_processed_scan =  n.advertise<sensor_msgs::LaserScan>("processed_scan", 1);
      serverRequestHumanPose = n.advertiseService("RequestHumanPose", &ScannerServer::getHumanPose, this);
    }

  private:
    ros::NodeHandle n;
    ros::ServiceServer serverRequestHumanPose;
    ros::Subscriber sub_scan;
    ros::Publisher pub_processed_scan;
    std::vector<float> scan;
    sensor_msgs::LaserScan processed_scan;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
      scan = msg->ranges;
      std::vector<float> filtered_scan;
      std::vector<float> to_process;
      for (size_t i = 0; i < scan.size(); i++) {
        if ( scan[i]<0.5 || scan[i]>1.0 || std::isnan(scan[i]) ) {
          filtered_scan.push_back(0);
        } else {
          filtered_scan.push_back(scan[i]);
          to_process.push_back(scan[i]);
        }
      }
      std::cout<<to_process.size()<<std::endl;
      float median = calculate_median(to_process);
      std::cout<<"Median "<<median<<std::endl;
      processed_scan = *msg;
      processed_scan.ranges.clear();
      processed_scan.ranges = filtered_scan;
      pub_processed_scan.publish(processed_scan);
    }

    float calculate_median(std::vector<float>& v){
      int size = v.size();
      std::sort(v.begin(), v.end());
      float median;
      if (size % 2 != 0)
        median = v[size/2];
      else{
        median = (v[(size-1)/2] + v[size/2]) / 2.0;
        }
     return median;
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
  std::cout<<"Server is running"<<std::endl;
  while(ros::ok()){
    //camera.publishPointcloud();
    ros::spinOnce();
    //loop_rate.sleep();
  }

  return 0;
}
