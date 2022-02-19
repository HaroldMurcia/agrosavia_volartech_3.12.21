#include <iostream>
#include <Eigen/Dense>
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
#include <quanergy/common/point_xyzir.h>
//
using namespace std;
using Eigen::MatrixXd;
//
using namespace message_filters;
using namespace sensor_msgs;
ofstream myfile;
std::string first_arg;
std::string fileName;

// ROS CoefARIABLES
ros::WallTime start_, end_;
float stamp_delta;
long    scan_id=0;

// GLOBAL CoefARIABLES  ///////////////////////////////////////////////////////////
ros::WallTime zero_time;
ros::Time zero_time_stamp;
int zero_time_FLAG=0;

// QUANERGY
int ini_QUANERGY_FLAG=0;
ros::Time zero_QUANERGY_time_stamp;
ros::Duration QUANERGY_time_stamp;

// IMU
double pos_x_hold=-1;
double pos_y_hold=-1;
double pos_z_hold=-1;
double vel_lin_x_hold=-1;
double vel_lin_y_hold=-1;
double vel_lin_z_hold=-1;
double vel_ang_x_hold=-1;
double vel_ang_y_hold=-1;
double vel_ang_z_hold=-1;
float qx_hold=0;
float qy_hold=0;
float qz_hold=0;
float qw_hold=0;
double pos_x_ini=-1;
double pos_y_ini=-1;
double pos_z_ini=-1;
double IMU_historic_time;
ros::WallTime start_imu, end_imu_lag;
ros::Duration ODOM_time_stamp;
int ini_ODOM_FLAG=0;
// GPS
double latitude_hold=0;
double longitude_hold=0;
double altitude_hold=0;
ros::Duration GPS_time_stamp;
int ini_GPS_FLAG=0;
///////////////////////////////////////////////////////////////////////////////

// STRUCTURES /////////////////////////////////////////////////////////////////
struct predictions {
  double x,y,z;
  double qx,qy,qz,qw;
  double latitude,longitude,altitude;
} my_predictions;

class data_object {
  public:
    //{data[k],data[k-1],data[k-2]}
    double X[3]={0,0,0};
    double timestamps[3]={0,0,0};
    double estimation=0;
    double Coef[2]={0,0};
    int data_FLAG=0;

    void data_object_update(double input_value, double input_time){
      X[2]=X[1];
      X[1]=X[0];
      X[0]=input_value;
      timestamps[2]=timestamps[1];
      timestamps[1]=timestamps[0];
      timestamps[0]=input_time;
      data_FLAG+=1;
      if (data_FLAG>=3){
        data_FLAG=3;
        LeastSquare();
      }
    }

    void LeastSquare(){
      MatrixXd M(3,2); //where 3 is the number of samples
      MatrixXd Y(3,1);
      MatrixXd W(2,1);
      MatrixXd A(2,2);
      //
      W(0,0)=0;
      W(1,0)=0;
      //
      M(0,0)=timestamps[2]; //[k-2]
      M(1,0)=timestamps[1]; //[k-1]
      M(2,0)=timestamps[0]; //[k]
      M(0,1)=1;
      M(1,1)=1;
      M(2,1)=1;
      //
      Y(0,0)=X[2]; //[k-2]
      Y(1,0)=X[1]; //[k-1]
      Y(2,0)=X[0]; //[K]
      //
      A=((M.transpose()*M).inverse());
      W = A*M.transpose()*Y;
      Coef[0] = W(0,0);
      Coef[1] = W(1,0);
    }

    double predict(double timeStamp_to_pred){
      double prediction = timeStamp_to_pred*Coef[0]+Coef[1];
      return prediction;
    }

} latitude_estimation, longitude_estimation, altitude_estimation, qx_estimation, qy_estimation, qz_estimation,qw_estimation,x_estimation,y_estimation,z_estimation;
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////// Functions

void createFile(){
  myfile.open(fileName.c_str());
  //myfile << "#NODE\t" << "Scan_id\t" << "scan_time\t" << "qx\t" << "qy\t" << "qz\t" << "qw\t" << "X\t" << "Y\t" << "Z\t" << "latitude\t" << "longitude\t" << "altitude\t" <<"cloudX\t" << "cloudY\t" << "cloudZ\t" << "intensity\t" << "ring\t" << "#mode=1\n";
  myfile << "#NODE" << "scan_id\t" << "scan_time\t" << "odomX\t" << "odomY\t" << "odomZ\t" << "qx\t" << "qy\t" << "qz\t" << "qw\t" << "lat\t" << "long\t" << "alt\t" << "\n";
  myfile.close();
}

void callback_quanergy(const sensor_msgs::PointCloud2::ConstPtr& msg_1) //10HZ
{
  if (ini_QUANERGY_FLAG==0){
    ini_QUANERGY_FLAG=1;
  }
  //
  scan_id++;
  end_ = ros::WallTime::now();
  stamp_delta = (end_ - start_).toNSec()*1e-9;
  pcl::PointCloud<quanergy::PointXYZIR> depth;
  pcl::fromROSMsg( *msg_1, depth);
  //int width = depth.width;
  //int height = depth.height;
  int L = depth.points.size();
  float x = 0, y = 0, z=0, intensity=0, ring=0;
  //ROS_INFO("publisher: [%s]", pcl::getFieldsList(*msg).c_str());

  if (ini_ODOM_FLAG==1 and ini_GPS_FLAG==1 and ini_QUANERGY_FLAG==1){
    myfile.open(fileName.c_str(),fstream::app);
    if (zero_time_FLAG==0){
      // zero time for time synchronizer
      zero_time_stamp = ros::Time::now();
      zero_QUANERGY_time_stamp = msg_1->header.stamp;
      zero_time_FLAG=1;
    }else{
      QUANERGY_time_stamp = msg_1->header.stamp - zero_QUANERGY_time_stamp;
      //ROS_INFO_STREAM("Q timeStamp: " << QUANERGY_time_stamp );
      //ROS_INFO_STREAM("ODOM_time_stamp : " << ODOM_time_stamp );
      //ROS_INFO_STREAM("GPS_time_stamp : " << GPS_time_stamp );
      // prediction with interpolation
      qx_estimation.estimation = qx_estimation.predict(QUANERGY_time_stamp.toNSec()*1e-9);
      qy_estimation.estimation = qy_estimation.predict(QUANERGY_time_stamp.toNSec()*1e-9);
      qz_estimation.estimation = qz_estimation.predict(QUANERGY_time_stamp.toNSec()*1e-9);
      qw_estimation.estimation = qw_estimation.predict(QUANERGY_time_stamp.toNSec()*1e-9);
      x_estimation.estimation  =  x_estimation.predict(QUANERGY_time_stamp.toNSec()*1e-9);
      y_estimation.estimation  = y_estimation.predict(QUANERGY_time_stamp.toNSec()*1e-9);
      z_estimation.estimation  = z_estimation.predict(QUANERGY_time_stamp.toNSec()*1e-9);
      latitude_estimation.estimation  = latitude_estimation.predict(QUANERGY_time_stamp.toNSec()*1e-9);
      longitude_estimation.estimation = longitude_estimation.predict(QUANERGY_time_stamp.toNSec()*1e-9);
      altitude_estimation.estimation  = altitude_estimation.predict(QUANERGY_time_stamp.toNSec()*1e-9);
      for (size_t i =0; i < L; i++)
      {
        x = depth.points[i].x;
        z = depth.points[i].y;
        y = depth.points[i].z;
        intensity = depth.points[i].intensity;
        ring = depth.points[i].ring;
        int echo_layer = 0;
        if (x==x)// ask and jump for a nan value
        {
          // for  testing:
          myfile << setprecision(12) << scan_id << "\t"<< stamp_delta << "\t" << x_estimation.estimation << "\t" << y_estimation.estimation << "\t" << z_estimation.estimation << "\t" << qx_estimation.estimation << "\t" << qy_estimation.estimation  << "\t" << qz_estimation.estimation  << "\t" << qw_estimation.estimation  << "\t" << latitude_estimation.estimation << "\t" << longitude_estimation.estimation << "\t" << altitude_estimation.estimation  << "\t" << x << "\t" << y << "\t" << z << "\t" << intensity << "\t" << ring << "\t" << echo_layer << "\n";
          //myfile << setprecision(12) << scan_id << "\t"<< stamp_delta << "\t" << pos_x_hold << "\t" << pos_y_hold << "\t" << pos_z_hold << "\t" << my_predictions.qx << "\t" << my_predictions.qy << "\t" << my_predictions.qz << "\t" << my_predictions.qw << "\t" << latitude_hold << "\t" << longitude_hold << "\t" << altitude_hold  << "\n";
        }
      }
      myfile.close();
    } //else
  } // Flags
} // void


void callback_Odom(const nav_msgs::Odometry::ConstPtr& msg_2) //20HZ
{
  double stop_imu=(ros::WallTime::now()-start_imu ).toNSec()*1e-9;
  start_imu = ros::WallTime::now();
  IMU_historic_time = (ros::WallTime::now()-zero_time ).toNSec()*1e-9;
  if (ini_ODOM_FLAG==0)
  {
    start_ = ros::WallTime::now();
    pos_x_ini=msg_2->pose.pose.position.x;
	  pos_y_ini=msg_2->pose.pose.position.y;
	  pos_z_ini=msg_2->pose.pose.position.z;
    // Saving initial coordinate in file header
    myfile.open(fileName.c_str(),fstream::app);
    myfile << setprecision(12) << "#NODE ECEF_init_x:" << pos_x_ini << "\t" << "ECEF_init_y:" << pos_y_ini << "\t" << "Ecef_init_z:" << pos_z_ini << "\n";
  	myfile.close();
	  ini_ODOM_FLAG=1;
	  ROS_INFO_STREAM("pos_x_ini: " << pos_x_ini);
	  ROS_INFO_STREAM("pos_y_ini: " << pos_y_ini);
	  ROS_INFO_STREAM("pos_z_ini: " << pos_z_ini);
  }
  pos_x_hold = msg_2->pose.pose.position.x;
  pos_y_hold = msg_2->pose.pose.position.y;
  pos_z_hold = msg_2->pose.pose.position.z;
  qx_hold  = msg_2->pose.pose.orientation.x;
  qy_hold  = msg_2->pose.pose.orientation.y;
  qz_hold  = msg_2->pose.pose.orientation.z;
  qw_hold  = msg_2->pose.pose.orientation.w;
  vel_lin_x_hold=msg_2->twist.twist.linear.x;
  vel_lin_y_hold=msg_2->twist.twist.linear.y;
  vel_lin_z_hold=msg_2->twist.twist.linear.z;
  vel_ang_x_hold=msg_2->twist.twist.angular.x;
  vel_ang_y_hold=msg_2->twist.twist.angular.y;
  vel_ang_z_hold=msg_2->twist.twist.angular.z;
  //
  if (zero_time_FLAG==1){
    ODOM_time_stamp = msg_2->header.stamp-zero_time_stamp;
  }
  qx_estimation.data_object_update(qx_hold,ODOM_time_stamp.toNSec()*1e-9);
  qy_estimation.data_object_update(qy_hold,ODOM_time_stamp.toNSec()*1e-9);
  qz_estimation.data_object_update(qz_hold,ODOM_time_stamp.toNSec()*1e-9);
  qw_estimation.data_object_update(qw_hold,ODOM_time_stamp.toNSec()*1e-9);
  x_estimation.data_object_update(pos_x_hold, ODOM_time_stamp.toNSec()*1e-9);
  y_estimation.data_object_update(pos_y_hold, ODOM_time_stamp.toNSec()*1e-9);
  z_estimation.data_object_update(pos_z_hold, ODOM_time_stamp.toNSec()*1e-9);
}



void callback_GPS(const sensor_msgs::NavSatFix::ConstPtr& msg_3){ //20HZ
  if (ini_GPS_FLAG==0){
    ini_GPS_FLAG=1;
  }
  if (zero_time_FLAG==1){
    GPS_time_stamp = msg_3->header.stamp- zero_time_stamp;
  }
  //Latitude uptate
  latitude_hold=msg_3->latitude;
  latitude_estimation.data_object_update(latitude_hold, GPS_time_stamp.toNSec()*1e-9);
  //Longitude uptate
  longitude_hold=msg_3->longitude;
  longitude_estimation.data_object_update(longitude_hold, GPS_time_stamp.toNSec()*1e-9);
  //Altitude uptate
  altitude_hold=msg_3->altitude;
  altitude_estimation.data_object_update(altitude_hold, GPS_time_stamp.toNSec()*1e-9);
}



// MAIN /////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    first_arg= argv[1];
    fileName = first_arg;
    printf("dataFile Name: %s\n", fileName.c_str() );
    createFile();
    //
    ros::init(argc, argv, "signal2RAW");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();
    // subscriptions
    ros::Subscriber sub_1 = nh.subscribe("/quanergy/points0", 2, callback_quanergy); // 10HZ
    ros::Subscriber sub_2 = nh.subscribe("/vectornav/Odom", 4,callback_Odom); // 20HZ
    ros::Subscriber sub_3 = nh.subscribe("/vectornav/GPS", 4,callback_GPS); // 20Hz
    //
    ros::waitForShutdown();
    //ros::spin();
    return 0;
}
