/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdint>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

// async module for multithreading
#include <quanergy/pipelines/async.h>

//#include <quanergy_client_ros/client_node.cpp>

#include <quanergy/common/point_xyzir.h>

using namespace std;
using namespace message_filters;
using namespace sensor_msgs;
ofstream myfile;
std::string first_arg;
std::string fileName;

// ROS
ros::WallTime start_, end_;
int ini_FLAG=0;
double  ini_k_x=0;
double  ini_k_y=0;
double  ini_k_z=0;
long    scan_id=0;

/**
 * This tutorial demonstrates subscribing to a topic using a class method as the callback.
 */

// %Tag(CLASS_WITH_DECLARATION)%
// %EndTag(CLASS_WITH_DECLARATION)%

void createFile(){
  myfile.open(fileName.c_str());
  myfile << "#NODE\t" << "Scan id\t" << "scan delta_time\t" << "qx\t" << "qy\t" << "qz\t" << "qw\t" << "X\t" << "Y\t" << "Z\t" << "latitude\t" << "longitude\t" << "altitude\t" <<"cloudX\t" << "cloudY\t" << "cloudZ\t" << "intensity\t" << "ring\t" << "#mode=1\n";
  myfile.close();
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg_1, const nav_msgs::Odometry::ConstPtr& msg_2, const sensor_msgs::NavSatFix::ConstPtr& msg_3)
{
  //end_ = ros::WallTime::now();
  //scan_id++;
  printf("Aqui\n" );
  ROS_INFO_STREAM("Exectution time (ms):");
  // VN300
  if (ini_FLAG==0){
  ini_k_x=msg_2->pose.pose.position.x;
  ini_k_y=msg_2->pose.pose.position.y;
  ini_k_z=msg_2->pose.pose.position.z;
  myfile.open(fileName.c_str(),fstream::app);
  myfile << "#ECEF_init_x:" << ini_k_x << "\t" << "ECEF_init_y:" << ini_k_y << "\t" << "Ecef_init_z:" << ini_k_z << "\n";
  myfile.close();
  ini_FLAG=1;
  }
  double k_x = msg_2->pose.pose.position.x;
  double k_y = msg_2->pose.pose.position.y;
  double k_z = msg_2->pose.pose.position.z;
  float qx  = msg_2->pose.pose.orientation.x;
  float qy  = msg_2->pose.pose.orientation.y;
  float qz  = msg_2->pose.pose.orientation.z;
  float qw  = msg_2->pose.pose.orientation.w;
  //
  double latitude =msg_3->latitude;
  double longitude=msg_3->longitude;
  double altitude =msg_3->altitude;
  //
  //pcl::PointCloud<pcl::PointXYZI> depth;
  pcl::PointCloud<quanergy::PointXYZIR> depth;
  pcl::fromROSMsg( *msg_1, depth);
  int width = depth.width;
  int height = depth.height;
  int L = depth.points.size();
  float x = 0, y = 0, z=0, intensity=0, ring=0;
  //ROS_INFO("publisher: [%s]", pcl::getFieldsList(*msg).c_str());
  myfile.open(fileName.c_str(),fstream::app);
  float stamp_delta = (end_ - start_).toNSec()*1e-6;
  start_ = ros::WallTime::now();
  for (size_t i =0; i < L; i++){
    x = depth.points[i].x;
    z = depth.points[i].y;
    y = depth.points[i].z;
    intensity = depth.points[i].intensity;
    ring = depth.points[i].ring;
    if (x==x)// ask for a nan value
    {
    myfile << setprecision(12) << scan_id << "\t"<< stamp_delta << "\t" << qx << "\t" << qy << "\t" << qz << "\t" << qw << "\t" << k_x << "\t" << k_y << "\t" << k_z << "\t" << latitude << "\t" << longitude << "\t" << altitude  << "\t"  << x << "\t" << y << "\t" << z << "\t" << intensity << "\t" << ring << "\n";
    }
  }
  myfile.close();
  //ROS_INFO_STREAM("Exectution time (ms): " << stamp_delta);

}

void callback_1(const sensor_msgs::PointCloud2::ConstPtr& msg_1, const nav_msgs::Odometry::ConstPtr& msg_2)
{
  printf("Aqui\n" );
  

}


int main(int argc, char **argv)
{
  first_arg= argv[1];
  fileName = first_arg;
  printf("dataFile Name: %s\n", fileName.c_str() );
  createFile();
  ros::init(argc, argv, "listener_class");
  ros::NodeHandle nh;
  // %Tag(SUBSCRIBER)%
  //ros::Subscriber sub_1 = nh.subscribe("/quanergy/points", 1, callback_1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/quanergy/points", 1);
  message_filters::Subscriber<nav_msgs::Odometry> vn300_sub(nh, "/vectornav/Odom", 1);
  //message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "/vectornav/GPS", 1);
  //typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry, sensor_msgs::NavSatFix> MySyncPolicy;
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, vn300_sub);
  sync.registerCallback(boost::bind( &callback_1, _1, _2));
// %EndTag(SUBSCRIBER)%

  ros::spin();

  return 0;
}
