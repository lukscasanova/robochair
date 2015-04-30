/*
 *   RestThru is a mobile robotics framework developed at the
 *   School of Electrical and Computer Engineering, University
 *   of Campinas, Brazil by Eleri Cardozo and collaborators.
 *   eleri@dca.fee.unicamp.br
 *
 *   Copyright (C) 2011-2014 Eleri Cardozo
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef INCLUDE_MOTIONDEV_H
#define INCLUDE_MOTIONDEV_H


#include <string>
#include <ros/console.h>

using namespace std;

// Aria-like operations for the wheelchair
class Pose{
public:
  void setX(double x){x_=x;}
  void setY(double y){y_=y;}
  void setTh(double th){th_=th;}
  double getX(){return x_;}
  double getY(){return y_;}
  double getTh(){return th_;}
  Pose(double x, double y, double th):x_(x), y_(y), th_(th){}
  Pose():x_(0.0), y_(0.0), th_(0.0){  }
  
private:
  double x_, y_, th_;
  
};

class MotionDev {

  string serverAddr;
  unsigned short serverPort;
  int sockfd;
  Pose* pose;
  // send UDP message to Arduino and get reply
  bool sendMessage(const char* request, char* reply);

public:

  // ctor & dtor
  MotionDev(string addr, unsigned short port);
  ~MotionDev();

  // methods
  bool brake();    // activate electric brakes
  bool release();    // release electric brakes
  bool stop();    // set linear speed to zero
  bool setVel(double vel);    // mm/s
  bool setVel2(double velLeft, double velRight);   // mm/s
  bool setRotVel(double rotVel);     // degrees/s
  bool setHeading(double heading);   // degrees (absolute)
  bool setDeltaHeading(double heading);   // degrees (relative)
  bool move(double distance);  // mm
  //bool setPose(double x, double y, double th);  // mm , degrees
  bool setPose(Pose pos);
  bool resetTrip();
  bool getVel(double* vel);
  bool getVel2(double* velLeft, double* velRight);
  bool getRotVel(double* rotVel);
  bool getHeading(double* heading);
  //bool getPose(double* x, double* y, double* th);x
  bool getPose(Pose* pose);
  bool getTrip(double* dist, double* angle);   // mm , degrees
  bool getStatus(string* status);
  bool isMoveDone();
  bool isHeadingDone();

  //
};


#endif  
