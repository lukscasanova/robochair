#include "MotionDev.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define __PI__ 3.1415
const double RAD_TO_DEG=180.0/__PI__;

class VelocityController{
private:
  MotionDev *md;
  ros::Subscriber vel_sub, joy_sub;
  double linvel_, rotvel_;
  double lin_scale, rot_scale;
  double radius;
public:
  VelocityController();
  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy);
  void vel_cb(const geometry_msgs::Twist::ConstPtr& vel);
//   const double RAD_TO_DEG = 180.0/__PI__;
  
};

VelocityController::VelocityController(){
 ros::NodeHandle nh;
 
 std::string arduino_address, cmd_vel_topic, joy_topic;
 int arduino_port;
 nh.getParam("arduino_address", arduino_address);
 nh.getParam("arduino_port", arduino_port);
 md = new MotionDev(arduino_address, arduino_port);
 
 std::string status;
 md->getStatus(&status);
 ROS_INFO_STREAM("status: " << status);
 while(!md->release());
 ROS_INFO("Released");
 nh.param<std::string>("joy_topic", joy_topic, "joy");
 nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "cmd_vel");
 vel_sub = nh.subscribe(cmd_vel_topic, 1, &VelocityController::vel_cb, this);
 joy_sub = nh.subscribe(joy_topic, 5, &VelocityController::joy_cb, this);
 
 
 nh.param<double>("linear_velocity_scale", lin_scale, 0.1);
 nh.param<double>("angular_velocity_scale", rot_scale, 0.1);
 nh.param<double>("radius", radius, 20.0);
}

void VelocityController::joy_cb(const sensor_msgs::Joy::ConstPtr& joy){
  if(joy->buttons[2]==1) md->release();
  if(joy->buttons[3]==1) md->stop();
}


void VelocityController::vel_cb(const geometry_msgs::Twist::ConstPtr& vel){
  //if( (linvel_==vel->linear.x*1000.0) && (rotvel_==vel->angular.z*1000.0) )return;
  linvel_=vel->linear.x*1000.0*lin_scale;
  rotvel_=vel->angular.z*1000.0*rot_scale;
   md->setVel2(linvel_-rotvel_*radius, linvel_+rotvel_*radius);
   //md->setVel(linvel_);
   //md->setRotVel(rotvel_);
  //ROS_INFO_STREAM("SET VEL = " << md->setVel(linvel_));
  //ROS_INFO_STREAM("SET ROT VEL= " << );
  
}
int main (int argc, char ** argv){
 ros::init(argc, argv, "robochair_vel");
 VelocityController vc;
 ros::spin();
 return 0; 
}