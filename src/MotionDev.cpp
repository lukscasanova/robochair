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


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include "MotionDev.h"
/*
#include <log4cplus/logger.h>
#include <log4cplus/configurator.h>
#include <log4cplus/loggingmacros.h>*/


using namespace std;

// command issued by upper level
#define BRAKE 1
#define RELEASE 2
#define STOP 3
#define SETVELS 4
#define SETROTVEL 5
#define HEADING 6
#define TURN 7
#define DISLOCATE 8
#define SETPOSE 9
#define RESETTRIP 10
#define GETVELS 11
#define GETROTVEL 12
#define GETPOSE 13
#define GETTRIP 14
#define ISMOVEDONE 15
#define ISTURNINGDONE 16
#define GETSTATUS 17
#define RECOVER 18
#define SETLIMITS 19

// send & reply
bool MotionDev::sendMessage(const char* request, char* reply) {
//   log4cplus::Logger logger = log4cplus::Logger::getInstance("restthru");
   struct sockaddr_in servaddr, cliaddr;
   struct timeval tv;
   fd_set fds;
   int n, nfd;
   char msg[256];

//    LOG4CPLUS_TRACE(logger, "Sending request to arduino: " << request);

   bzero(&servaddr,sizeof(servaddr));
   servaddr.sin_family = AF_INET;
   servaddr.sin_addr.s_addr = inet_addr(serverAddr.c_str());
   servaddr.sin_port = htons(serverPort);
   if(sendto(sockfd, request, strlen(request)+1,0,
             (struct sockaddr *)&servaddr,sizeof(servaddr)) <= 0)
     return false;
   // get reply  (wait 5 secs)
   tv.tv_sec = 5;
   tv.tv_usec = 0;
   FD_ZERO(&fds);
   FD_SET(sockfd, &fds);
   do {
       nfd = select(sockfd+1, &fds, NULL, NULL, &tv);
   } while(nfd == -1 && errno == EINTR);
   if(nfd != 1) return false;
   n = recvfrom(sockfd, reply, 255, 0, NULL, NULL);
   reply[n] = '\0';
   if(n <= 0) return false;
//    LOG4CPLUS_TRACE(logger, "Response from Dev received: "<< reply);
   return true;
}


// ctor
MotionDev::MotionDev(string addr, unsigned short port) {
  serverAddr = addr;
  serverPort = port;
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
}

// dtor
MotionDev::~MotionDev() {
  if(sockfd > 0) close(sockfd);
}

// methods

// activate electric brakes
bool MotionDev::brake() {
  const char* request = "1 0 0 0 0 0";
  char reply[256];
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "1 0", 3) == 0) return true;
  return false;
}


// release electric brakes
bool MotionDev::release() {
  const char* request = "2 0 0 0 0 0";
  char reply[256];
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "2 0", 3) == 0) return true;
  return false;
}


// stop robot (set speed to zero)
bool MotionDev::stop() {
  const char* request = "3 0 0 0 0 0";
  char reply[256];
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "3 0", 3) == 0) return true;
  return false;
}


// set a linear speed
bool MotionDev::setVel(double vel) {
  char request[256];
  char reply[256];
  sprintf(request, "21 %ld 0 0 0 0", (long)vel);
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "21 0", 3) == 0) return true;
  return false;
}



// set linear speeds in each wheel
bool MotionDev::setVel2(double leftVel, double rightVel) {
  char request[256];
  char reply[256];
  sprintf(request, "4 %ld %ld 0 0 0", (long)leftVel, (long)rightVel);
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "4 0", 3) == 0) return true;
  return false;
}


// set rotational speed
bool MotionDev::setRotVel(double rotVel) {
  char request[256];
  char reply[256];
  sprintf(request, "5 %ld 0 0 0 0", (long)(rotVel*1000));
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "5 0", 3) == 0) return true;
  return false;
}


// set an absolute orientation 
bool MotionDev::setHeading(double heading) {
  char request[256];
  char reply[256];
  long h = (long)(heading*1000) % 360000;
  sprintf(request, "6 %ld 0 0 0 0", h);
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "6 0", 3) == 0) return true;
  return false;
}


// set a relative  orientation 
bool MotionDev::setDeltaHeading(double heading) {
  char request[256];
  char reply[256];
  long h = (long)(heading*1000) % 360000;
  sprintf(request, "7 %ld 0 0 0 0", h);
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "7 0", 3) == 0) return true;
  return false;
}


// dislocate to a given distance
bool MotionDev::move(double distance) {
  char request[256];
  char reply[256];
  sprintf(request, "8 %ld 0 0 0 0", (long)distance);
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "8 0", 3) == 0) return true;
  return false;
}


// correct the pose of the robot
//bool MotionDev::setPose(double x, double y, double th) {
bool MotionDev::setPose(Pose pose){
  char request[256];
  char reply[256];
  sprintf(request, "9 %ld %ld %ld 0 0", 
	  (long)pose.getX(), (long)pose.getY(), (long)(pose.getTh()*1000));
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "9 0", 3) == 0) return true;
  return false;
}

// reset odometry trip
bool MotionDev::resetTrip() {
  const char* request = "10 0 0 0 0 0";
  char reply[256];
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "10 0", 4) == 0) return true;
  return false;
}


// get linear speed
bool MotionDev::getVel(double* vel) {
  double vl, vr;
  if(getVel2(&vl, &vr)) {
    *vel = (vl + vr)/2;
    return true;
  }
  return false;
}


// get speed in each wheel
bool MotionDev::getVel2(double* velLeft, double* velRight) {
  const char* request = "11 0 0 0 0 0";
  char reply[256];
  int comm;
  long int vl, vr;
  if(! sendMessage(request, reply)) return false;
  if(sscanf(reply, "%d %ld %ld", &comm, &vl, &vr) == 3 && 
     comm == 11) {
    *velLeft = (double) vl;
    *velRight = (double) vr;
    return true;
  }
  return false;
}


// get rotational speed
bool MotionDev::getRotVel(double* rotVel) {
  const char* request = "12 0 0 0 0 0";
  char reply[256];
  long v;
  int comm;
  if(! sendMessage(request, reply)) return false;
  if(sscanf(reply, "%d %ld", &comm, &v) == 2 && comm == 12) {
    *rotVel = (double)(v)/1000;
    return true;
  }
  return false;
}


// get current orientation
bool MotionDev::getHeading(double* heading) {
  Pose pose;
  //if(! getPose(&x, &y, &th)) return false;
  if(! getPose(&pose)) return false;
  //*heading = th;
  *heading = pose.getTh();
  return true;
}


// get current pose
//bool MotionDev::getPose(double* x, double* y, double* th) {
bool MotionDev::getPose(Pose* pose){
  const char* request = "13 0 0 0 0 0";
  char reply[256];
  long px, py, pth;
  int comm;
  if(! sendMessage(request, reply)) return false;
  if(sscanf(reply, "%d %ld %ld %ld", &comm, &px, &py, &pth) == 4 
     && comm == 13) {
    pose->setX((double)px);
    pose->setY((double)py);
    pose->setTh((double)(pth)/1000.0);
    return true;
  }
  return false;
}

// get status string
bool MotionDev::getStatus(string* status){
  const char* request = "17 0 0 0 0 0";
  string ret;
  string delimiter = " ";
  char reply[256];
  int comm;
  if(!sendMessage(request, reply)) return false;
  ret = reply;
  size_t pos = ret.find(delimiter);
  comm = atoi(ret.substr(0, pos).c_str());
  if(comm == 17){
    ret.erase(0, pos + delimiter.length());
    *status = ret;
    return true;
  }
  return false;
}


// get odometry trip
bool MotionDev::getTrip(double* dist, double* angle) {
  const char* request = "14 0 0 0 0 0";
  char reply[256];
  long d, a;
  int comm;
  if(! sendMessage(request, reply)) return false;
  if(sscanf(reply, "%d %ld %ld", &comm, &d, &a) == 3 && comm == 14) {
    *dist = (double)d;
    *angle = (double)a;
    return true;
  }
  return false;
}


// check if a move has completed
// returns true if the displacement controller is still acting on the robot
bool MotionDev::isMoveDone() {
  const char* request = "15 0 0 0 0 0";
  char reply[256];
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "15 1", 4) == 0) return true;
  return false;
}


// check if an orientation has completed
// returns true if the heading controller is still acting on the robot
bool MotionDev::isHeadingDone() {
  const char* request = "16 0 0 0 0 0";
  char reply[256];
  if(! sendMessage(request, reply)) return false;
  if(strncmp(reply, "16 1", 4) == 0) return true;
  return false;
}
  
