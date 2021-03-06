#include "MotionDev.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define __PI__ 3.1415
class VelocityController{
private:
  MotionDev *md;
  ros::Subscriber vel_sub, joy_sub;
public:
  void VelocityController();
  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy);
  void vel_cb(const geometry_msgs::Twist::ConstPtr& vel);
  const double RAD_TO_DEG = 180.0/__PI__;
  
};

void VelocityController::ChairController(){
 ros::NodeHandle nh;
 
 std::string arduino_address, cmd_vel_topic, joy_topic;
 unsigned short arduino_port;
 nh.getParam("arduino_address", arduino_address);
 nh.getParam("arduino_port", arduino_port);
 md = new MotionDev(arduino_address, arduino_port);
 
 nh.param<std::string>("joy_topic", joy_topic, "joy");
 nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "cmd_vel");
 vel_sub = nh.subscribe(cmd_vel_topic, 1, &ChairController::vel_cb, this);
 joy_sub = nh.subscribe(joy_topic, 5, &ChairController::joy_cb, this);
}

void ChairController::joy_cb(const sensor_msgs::Joy::ConstPtr& joy){
  if(joy->buttons[2]==1) md->release();
  if(joy->buttons[3]==1) md->stop();
}


void ChairController::vel_cb(const geometry_msgs::Twist::ConstPtr& vel){
  if( (linvel_==vel->linear.x*1000.0) && (rotvel_==vel->angular.z*1000.0) )return;
  linvel_=vel->linear.x*1000.0;
  rotvel_=vel->angular.z*1000.0;
  //md->setVel2(linvel_+rotvel_*radius/2.0, linvel_+rotvel_*radius/2.0));
  ROS_INFO_STREAM("SET VEL = " << md->setVel(linvel_));
  ROS_INFO_STREAM("SET ROT VEL= " << md->setRotVel(rotvel_));
  
}
int main (int argc, char ** argv){
 ros::init(argc, argv, "robochair_vel");
 VelocityController vc;
 ros::spin();
 return 0; 
}