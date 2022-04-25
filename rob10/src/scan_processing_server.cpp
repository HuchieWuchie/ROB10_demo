//sudo chmod a+rw /dev/ttyACM0
//rosrun urg_node urg_node

//roslaunch urdf_tutorial display.launch model:=/home/daniel/iiwa_ws/src/ROB10/rob10/urdf/iiwa_setup.urdf.xacro


//SOURCE - https://answers.ros.org/question/304562/x-y-and-z-coordinates-in-laserscan/

#include <iostream>
#include <sstream>
#include <unistd.h>
#include <algorithm>
#include <cmath>

#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"

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
    tf2_ros::StaticTransformBroadcaster tf_broadcaster;
    std::vector<float> scan;
    sensor_msgs::LaserScan processed_scan;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
      scan = msg->ranges;
      std::vector<float> filtered_scan;
      std::vector<float> range;
      std::vector<float> angle_index;
      for (size_t i = 0; i < scan.size(); i++) {
        if ( scan[i]<0.5 || scan[i]>1.0 || std::isnan(scan[i]) ) {
          filtered_scan.push_back(0);
        } else {
          filtered_scan.push_back(scan[i]);
          range.push_back(scan[i]);
          angle_index.push_back(i);
        }
      }
      std::cout<<range.size()<<std::endl;
      if (range.size() == 0){
          std::cout<<"No data to process, quitting. TODO error handling"<<std::endl;
          std::exit(5);
      }
      float median_range = calculate_median(range);
      float median_angle = calculate_median(angle_index);
      std::cout<<"Median_range "<<median_range<<std::endl;
      std::cout<<"Median_angle "<<median_angle<<std::endl;
      float angle = msg->angle_min + (median_angle*msg->angle_increment);
      std::cout<<"angle "<<angle<<std::endl;
      geometry_msgs::Point receiver;
      // POLAR TO CARTESIAN + TRANSFORM TO WORLD FRAME
      //receiver.x = median_range * cos(angle);
      //receiver.y = median_range * sin(angle);
      //receiver.z = 1.3f;
      receiver.x = median_range * cos(angle) + 0.43;
      receiver.y = median_range * sin(angle) - 0.02;
      receiver.z = 1.3f;

      add_giver_frame(receiver);

      processed_scan = *msg;
      processed_scan.ranges.clear();
      processed_scan.ranges = filtered_scan;
      pub_processed_scan.publish(processed_scan);

      //std::exit(3);
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

   void add_giver_frame(geometry_msgs::Point& p){
     // TRANSFORM THE POINT TO THE LINK_0 FRAME
     geometry_msgs::Point p_transformed;
     p_transformed.x = p.x + 0.332;
     p_transformed.y = p.y + 0.179;

     // CALCULATE ANGLE
     float hypotenuse = sqrt(pow(p_transformed.x, 2.0f) + pow(p_transformed.y, 2.0f));
     float rot = asin(p_transformed.y/hypotenuse);
     std::cout<<"rot "<<rot<<std::endl;

     // PUBLISH NEW FRAME
     geometry_msgs::TransformStamped giver_frame;
     giver_frame.header.stamp = ros::Time::now();
     giver_frame.header.frame_id = "iiwa_link_0";
     giver_frame.child_frame_id = "giver";
     giver_frame.transform.translation.x = 0.0f;
     giver_frame.transform.translation.y = 0.0f;
     giver_frame.transform.translation.z = 0.0f;
     tf2::Quaternion quat;
     quat.setRPY(0, 0, rot);
     giver_frame.transform.rotation.x = quat.x();
     giver_frame.transform.rotation.y = quat.y();
     giver_frame.transform.rotation.z = quat.z();
     giver_frame.transform.rotation.w = quat.w();
     tf_broadcaster.sendTransform(giver_frame);
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

/*
rosrun tf tf_echo /world /laser
At time 0.000
- Translation: [0.430, -0.020, 0.975]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
            in RPY (radian) [0.000, -0.000, 0.000]
            in RPY (degree) [0.000, -0.000, 0.000]
========================================================
rosrun tf tf_echo /iiwa_link_0 /world
At time 0.000
- Translation: [0.332, 0.179, -1.038]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
            in RPY (radian) [0.000, -0.000, 0.000]
            in RPY (degree) [0.000, -0.000, 0.000]

========================================================
rosrun tf tf_echo /iiwa_link_0 /laser
At time 0.000
- Translation: [0.762, 0.159, -0.063]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
           in RPY (radian) [0.000, -0.000, 0.000]
           in RPY (degree) [0.000, -0.000, 0.000]

*/
