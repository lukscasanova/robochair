#include "MotionDev.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define __PI__ 3.1415
class ChairController{
private:
  MotionDev *md;
  ros::Subscriber vel_sub;
public:
  void ChairController();
  void vel_cb(const geometry_msgs::Twist::ConstPtr& vel);
  const double RAD_TO_DEG = 180.0/__PI__;
  
};

void ChairController::ChairController(){
 ros::NodeHandle nh;
 
 std::string arduino_address, cmd_vel_topic;
 unsigned short arduino_port;
 nh.getParam("arduino_address", arduino_address);
 nh.getParam("arduino_port", arduino_port);
 md = new MotionDev(arduino_address, arduino_port);
 
 nh.param("cmd_vel_topic", cmd_vel_topic, "cmd_vel");
 vel_sub = nh.subscribe(cmd_vel_topic, 10, &ChairController::vel_cb, this);
}

void vel_cb(const geometry_msgs::Twist::ConstPtr& vel){
  md->setVel(vel->linear.x*1000);
  md->setRotVel(vel->angular.z*RAD_TO_DEG);
}


int main (int argc, char ** argv){
 ros::init(argc, argv, "robochair_vel");
 MotionDev md(arduino_address, arduino_port);
 ros::spin();
 return 0; 
}