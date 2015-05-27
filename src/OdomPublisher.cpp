#include "MotionDev.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#define __PI__ 3.1415

class OdomPublisher{
private:
  MotionDev *md;
  bool publish_tf;
  ros::Publisher odom_pub;
  std::string odom_frame, chair_frame;
  double radius;
public:
  OdomPublisher();  
  //const double RAD_TO_DEG = 180.0/__PI__;
  void run();
  void pubOdom();
  
};

OdomPublisher::OdomPublisher(){
 ros::NodeHandle nh;
 
 std::string arduino_address, odom_topic;
 int arduino_port;
 nh.getParam("arduino_address", arduino_address);
 nh.getParam("arduino_port", arduino_port);
 nh.param<bool>("publish_tf", publish_tf, true);
 nh.param<std::string>("odom_topic", odom_topic, "odom");
 nh.param<std::string>("odom_frame", odom_frame, "odom");
 nh.param<std::string>("chair_frame", chair_frame, "base_footprint");
 nh.param<double>("radius", radius, 20.0);
 md = new MotionDev(arduino_address, arduino_port);
 
 std::string status;
 md->getStatus(&status);
 ROS_INFO_STREAM("status: " << status);
 
 odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
 
}


void OdomPublisher::pubOdom(){
  ROS_INFO("Entered PubOdom");
  double RAD_TO_DEG=180.0/__PI__;
  nav_msgs::Odometry odom;
  Pose pose;
  md->getPose(&pose);
  ROS_INFO("Got Pose");
  odom.header.stamp= ros::Time::now();
  odom.header.frame_id=odom_frame;
  odom.child_frame_id=chair_frame;
  odom.pose.pose.position.x=pose.getX()/1000.0;
  odom.pose.pose.position.y=pose.getY()/1000.0;
  odom.pose.pose.position.z=0.0;//TEMPORARY
  //ROS_INFO_STREAM("Theta: "<< pose.getTh());
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.getTh()*1/RAD_TO_DEG);
  double velR, velL;
  double linVel, rotVel;
 // ROS_INFO_STREAM("GET VEL= " << 
  md->getVel2(&velL, &velR);
  ROS_INFO("Got Vel");
  odom.twist.twist.linear.x=(velR+velL)/2000.0;
  odom.twist.twist.linear.y=0.0;
  odom.twist.twist.linear.z=0.0;
  odom.twist.twist.angular.x=0.0;
  odom.twist.twist.angular.y=0.0;
  md->getRotVel(&rotVel);
  //ROS_INFO("Right: %f, Left: %f, Rot: %f", velR, velL, (velR-velL)/(2000.0*radius));
  ROS_INFO("Got Rot Vel");
  odom.twist.twist.angular.z=(velR-velL)/(2000.0*radius);
  odom_pub.publish(odom);
  ROS_INFO("Published Odom");
  //ROS_INFO_STREAM_THROTTLE(3, "ODOM PUB");
  
  //TF
  if(publish_tf){
    ROS_INFO("Entered pubTF");
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.getX()/1000.0, pose.getY()/1000.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, pose.getTh()/RAD_TO_DEG);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame, chair_frame));
    ROS_INFO("published transform");
  }
  return;
}
void OdomPublisher::run(){
  ros::NodeHandle nh;
  double odom_rate;
  nh.param<double>("odom_rate",odom_rate, 1.0); 
  ros::Rate r(odom_rate);
   
   while(ros::ok()){
     pubOdom();
     r.sleep();
   }
}

int main (int argc, char ** argv){
 ros::init(argc, argv, "robochair_vel");
 OdomPublisher op;
 op.run();
 return 0; 
}