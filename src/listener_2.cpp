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
using namespace std;
using namespace message_filters;
using namespace sensor_msgs;
ofstream myfile;
std::string first_arg;
std::string fileName;

// ROS
ros::WallTime start_, end_;
float stamp_delta;
int ini_FLAG=0;
double  ini_k_x=0;
double  ini_k_y=0;
double  ini_k_z=0;
long    scan_id=0;

// Global variables
// IMU
double k_x_hold=-1;
double k_y_hold=-1;
double k_z_hold=-1;
float qx_hold=0;
float qy_hold=0;
float qz_hold=0;
float qw_hold=0;
double k_x_ini=0;
double k_y_ini=0;
double k_z_ini=0;
ros::WallTime start_imu, end_imu;
// GPS
double latitude_hold=0;
double longitude_hold=0;
double altitude_hold=0;
ros::WallTime start_gps, end_gps;

struct predictions {
  double x,y,z;
  double qx,qy,qz,qw;
  double lati,longi,alti;
} my_predictions;

struct data_interpolation {
  double X[3]={0,0,0};
  double V[2]={0,0};
  double times[2]={0,0};
  double V_mean=0;
  int data_FLAG=-1;
} latitude_interpola, longitude_interpola, altitude_interpola, qx_interpola, qy_interpola, qz_interpola,qw_interpola,x_interpola,y_interpola,z_interpola;

void data_update(data_interpolation* data, double input_value, double input_time){
  data->X[2]=data->X[1];
  data->X[1]=data->X[0];
  data->X[0]=input_value;
  data->times[1]=data->times[0];
  data->times[0]=input_time;
  data->data_FLAG+=1;
  if (data->data_FLAG>=3){
    data->data_FLAG=3;
    data->V[1]=(data->X[1]-data->X[2])/(data->times[1]+1e-9);
    data->V[0]=(data->X[0]-data->X[1])/(data->times[0]+1e-9);
  }
  data->V_mean=(data->V[0]+data->V[1])*0.5;
}

double prediction(data_interpolation* data,double delta_time){
  double pred=data->X[0]+data->V_mean*delta_time;
  return pred;
}

// Functions
void createFile(){
  myfile.open(fileName.c_str());
  myfile << "#NODE\t" << "Scan_id\t" << "scan_time\t" << "qx\t" << "qy\t" << "qz\t" << "qw\t" << "X\t" << "Y\t" << "Z\t" << "latitude\t" << "longitude\t" << "altitude\t" <<"cloudX\t" << "cloudY\t" << "cloudZ\t" << "intensity\t" << "ring\t" << "#mode=1\n";
  myfile.close();
}

void callback_1(const sensor_msgs::PointCloud2::ConstPtr& msg_1)
{
  // interpolation data
  end_imu = ros::WallTime::now();
  double imu_time = (end_imu-start_imu ).toNSec()*1e-9;
  end_gps = ros::WallTime::now();
  double gps_time = (end_gps-start_gps ).toNSec()*1e-9;
  my_predictions.qx=prediction(&qx_interpola,imu_time);
  my_predictions.qy=prediction(&qy_interpola,imu_time);
  my_predictions.qz=prediction(&qz_interpola,imu_time);
  my_predictions.qw=prediction(&qw_interpola,imu_time);
  my_predictions.x=prediction(&x_interpola,imu_time);
  my_predictions.y=prediction(&y_interpola,imu_time);
  my_predictions.z=prediction(&z_interpola,imu_time);
  my_predictions.lati=prediction(&latitude_interpola,gps_time);
  my_predictions.longi=prediction(&longitude_interpola,gps_time);
  my_predictions.alti=prediction(&altitude_interpola,gps_time);
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

  if (ini_FLAG==1 && latitude_interpola.data_FLAG>=3 && qx_interpola.data_FLAG>=3)
  {
    myfile.open(fileName.c_str(),fstream::app);
    for (size_t i =0; i < L; i++)
    {
      x = depth.points[i].x;
      z = depth.points[i].y;
      y = depth.points[i].z;
      intensity = depth.points[i].intensity;
      ring = depth.points[i].ring;
      if (x==x)// ask for a nan value
      {
        myfile << setprecision(12) << scan_id << "\t"<< stamp_delta << "\t" << my_predictions.qx << "\t" << my_predictions.qy << "\t" << my_predictions.qz << "\t" << my_predictions.qw << "\t" << my_predictions.x << "\t" << my_predictions.y << "\t" << my_predictions.z << "\t" << latitude_hold << "\t" << longitude_hold << "\t" << altitude_hold  << "\t"  << x << "\t" << y << "\t" << z << "\t" << intensity << "\t" << ring  << "\n";
      }
    }
    myfile.close();
    double execution_time = (ros::WallTime::now()-end_ ).toNSec()*1e-3;
    ROS_INFO_STREAM("Exectution time (s): " << execution_time);
  }
}


void callback_2(const nav_msgs::Odometry::ConstPtr& msg_2)
{
  double stop_imu=(ros::WallTime::now()-start_imu ).toNSec()*1e-9;
  if (ini_FLAG==0)
  {
    k_x_ini=msg_2->pose.pose.position.x;
	  k_y_ini=msg_2->pose.pose.position.y;
	  k_z_ini=msg_2->pose.pose.position.z;
	  myfile.open(fileName.c_str(),fstream::app);
  	myfile << "#ECEF_init_x:" << ini_k_x << "\t" << "ECEF_init_y:" << ini_k_y << "\t" << "Ecef_init_z:" << ini_k_z << "\n";
  	myfile.close();
	  ini_FLAG=1;
	  ROS_INFO_STREAM("k_x_ini: " << k_x_ini);
	  ROS_INFO_STREAM("k_y_ini: " << k_y_ini);
	  ROS_INFO_STREAM("k_z_ini: " << k_z_ini);
    start_ = ros::WallTime::now();
  }
  k_x_hold = msg_2->pose.pose.position.x;
  k_y_hold = msg_2->pose.pose.position.y;
  k_z_hold = msg_2->pose.pose.position.z;
  qx_hold  = msg_2->pose.pose.orientation.x;
  qy_hold  = msg_2->pose.pose.orientation.y;
  qz_hold  = msg_2->pose.pose.orientation.z;
  qw_hold  = msg_2->pose.pose.orientation.w;
  data_update(&qx_interpola,qx_hold,stop_imu);
  data_update(&qy_interpola,qy_hold,stop_imu);
  data_update(&qz_interpola,qz_hold,stop_imu);
  data_update(&qw_interpola,qw_hold,stop_imu);
  data_update(&x_interpola,k_x_hold,stop_imu);
  data_update(&y_interpola,k_y_hold,stop_imu);
  data_update(&z_interpola,k_z_hold,stop_imu);
  start_imu = ros::WallTime::now();
}



void callback_3(const sensor_msgs::NavSatFix::ConstPtr& msg_3){
   //Latitude uptate
   double stop_gps=(ros::WallTime::now()-start_gps ).toNSec()*1e-9;
   latitude_hold=msg_3->latitude;
   data_update(&latitude_interpola,latitude_hold,stop_gps);
   //Longitude uptate
   longitude_hold=msg_3->longitude;
   data_update(&longitude_interpola,longitude_hold,stop_gps);
   //Altitude uptate
   altitude_hold=msg_3->altitude;
   data_update(&altitude_interpola,altitude_hold,stop_gps);
   start_gps = ros::WallTime::now();
}




int main(int argc, char **argv)
{
    first_arg= argv[1];
    fileName = first_arg;
    printf("dataFile Name: %s\n", fileName.c_str() );
    createFile();
    //
    ros::init(argc, argv, "signal2RAW");
    ros::NodeHandle nh;
    ros::Subscriber sub_1 = nh.subscribe("/quanergy/points", 1, callback_1);
    ros::Subscriber sub_2 = nh.subscribe("/vectornav/Odom", 1,callback_2);
    ros::Subscriber sub_3 = nh.subscribe("/vectornav/GPS", 1,callback_3);
    ros::spin();

    return 0;
}
