#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

geometry_msgs::PointStamped gps_data;
geometry_msgs::PoseWithCovarianceStamped gps_data_cov;
nav_msgs::Odometry li_odom;
sensor_msgs::Imu imu_data;

ros::Publisher gps_pub;
ros::Publisher imu_pub;
int count = 1;

std::string filename = "/home/icmems/Desktop/SDC/catkin_ws/test.csv";

void getGPSCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    gps_data = *msg;
    gps_data_cov.header = gps_data.header;
    gps_data_cov.pose.pose.position = gps_data.point;
    gps_data_cov.pose.covariance = {10,0,0,0,0,0,
                                    0,10,0,0,0,0,
                                    0,0,10,0,0,0,
                                    0,0,0,0,0,0,
                                    0,0,0,0,0,0,
                                    0,0,0,0,0,0,};
    gps_pub.publish(gps_data_cov);
}

void getImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_data = *msg;
    imu_data.header.frame_id = "imu";
    imu_pub.publish(imu_data);
}

void getLidarCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    li_odom = *msg;

    // angle conversion
    tf2::Quaternion myQuaternion(li_odom.pose.pose.orientation.x,li_odom.pose.pose.orientation.y
    ,li_odom.pose.pose.orientation.z,li_odom.pose.pose.orientation.w);
    tf2::Matrix3x3 mat;
    mat.setRotation(myQuaternion);
    double r,p,y;
    mat.getRPY(r, p, y);

    std::ofstream myfile;
    myfile.open(filename,std::ofstream::app);
    // id x y z yaw pitch roll
    myfile << count << "," 
           << li_odom.pose.pose.position.x << ","  
           << li_odom.pose.pose.position.y << ","  
           << li_odom.pose.pose.position.z << ","  
           << y << ","  
           << p << ","  
           << r << "," << "\n";

    myfile.close();
    count += 1;
}

int main(int argc, char** argv){
    std::ofstream myfile;
    myfile.open(filename,std::ofstream::out);
    // id x y z yaw pitch roll
    myfile << "ID" << "," 
            << "x" << ","  
            << "y" << ","  
            << "z" << ","  
            << "yaw" << ","  
            << "pitch" << ","  
            << "roll" << "," << "\n";

    myfile.close();

  // initial node
  ros::init(argc, argv, "gps_and_output");
  ros::NodeHandle node;

  // initial subscriber
  ros::Subscriber sub_gps = node.subscribe("gps", 5, getGPSCallback);
  ros::Subscriber sub_lidar_odom = node.subscribe("lidar_odom", 5, getLidarCallback);
  ros::Subscriber sub_IMU = node.subscribe("imu/data", 5, getImuCallback);

  // initial publisher
  gps_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps_cov", 1);
  imu_pub = node.advertise<sensor_msgs::Imu>("imu_data", 1);

  while (node.ok()){
    ros::spinOnce ();
  }
  return 0;
};
