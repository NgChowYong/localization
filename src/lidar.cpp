#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include "pcl_ros/transforms.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// some global param
PointCloud::Ptr lidar_cloud (new PointCloud);
geometry_msgs::PointStamped gps_data;
int first_guess = 0;
double initial_angle = -2.2370340344819; // in rad
int get_lidar = 0;

void getGPSCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (first_guess == 0){
      first_guess = 1;
      gps_data = *msg;
    }
    //std::cout<<"get gps data"<<"\n";
}

void getLidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{   
  pcl::PCLPointCloud2 pcl_pc2;  
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  //lidar_cloud = temp_cloud;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (temp_cloud);
  sor.setLeafSize (0.7f, 0.7f, 0.7f);
  sor.filter (*lidar_cloud);
  std::cout<<"get lidar data"<<"\n";
  get_lidar = 1;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "lidar_process");
  ros::NodeHandle node;

  ros::Subscriber sub_gps = node.subscribe("gps", 5, getGPSCallback);
  ros::Subscriber sub_lidar = node.subscribe("lidar_points", 5, getLidarCallback);

  ros::Publisher pcl_map_pub = node.advertise<sensor_msgs::PointCloud2>("map", 1);
  ros::Publisher pcl_match_pub = node.advertise<sensor_msgs::PointCloud2>("match", 1);
  ros::Publisher pcl_loc_pub = node.advertise<nav_msgs::Odometry>("lidar_odom", 1);
  ros::Rate rate(1.0);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster br;


  // loda PCD map
  PointCloud::Ptr cloud (new PointCloud);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/icmems/Desktop/SDC/hwk_localization_midterm/SDC_data/SDC_Data/map/itri_map.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

  //cloud->header.frame_id = "world";

  // down sampled
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2 (*cloud,*cloud2);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud2);
  //sor.setLeafSize (0.5f, 0.5f, 0.5f);
  sor.setLeafSize (0.5f, 0.5f, 0.5f);
  sor.filter (*cloud_filtered);
  pcl::fromPCLPointCloud2 (*cloud_filtered,*cloud);
  cloud->header.frame_id = "world";

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;


  while(first_guess == 0 && node.ok()){
    ros::spinOnce ();
    rate.sleep ();
  }
  sub_gps.shutdown();

  
  // first guess transform
  Eigen::Matrix<float, 4, 4> m4;
  Eigen::Matrix<float, 4, 4> m4_new;
  tf2::Matrix3x3 m3;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  while (node.ok()){
    pcl_conversions::toPCL(ros::Time(0), cloud->header.stamp);//ros::Time::now()
    pcl_map_pub.publish (cloud);
    
    // transformation
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("velodyne", "base_link",
                               ros::Time(0));
      //ROS_INFO("found transform");
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      //ros::Duration(1.0).sleep();
      //continue;
    }
    
    PointCloud::Ptr lidar_cloud_tf (new PointCloud);  
    if (get_lidar == 1){
      pcl_ros::transformPointCloud( *lidar_cloud, *lidar_cloud_tf, transformStamped.transform);

      if (first_guess == 1){
        first_guess = 2;
        
        tf2::Quaternion tfq (0,0,sin(initial_angle/2),cos(initial_angle/2));
        tf2::Vector3 tfv (gps_data.point.x,gps_data.point.y,gps_data.point.z);
        tf2::Transform tftransform (tfq, tfv);
        m3 = tftransform.getBasis();
        
        //std::cout << m3[0][0] <<" " << m3[0][1] <<" " << m3[0][2] <<"\n";
        m4 << m3[0][0] ,m3[0][1] ,m3[0][2] ,gps_data.point.x ,
              m3[1][0] ,m3[1][1] ,m3[1][2] ,gps_data.point.y ,
              m3[2][0] ,m3[2][1] ,m3[2][2] ,gps_data.point.z ,
              0 ,0 ,0 ,1;
        std::cout << m4;


        geometry_msgs::TransformStamped transformStamped2;
        
        transformStamped2.header.stamp = ros::Time(0);//ros::Time::now();
        transformStamped2.header.frame_id = "world";
        transformStamped2.child_frame_id = "base_link";
        transformStamped2.transform.translation.x = gps_data.point.x;
        transformStamped2.transform.translation.y = gps_data.point.y;
        transformStamped2.transform.translation.z = gps_data.point.z;
        br.sendTransform(transformStamped2);


        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (5); // unit in m
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (1000);
        // Set the transformation epsilon (criterion 2) -> diff between previoud and current is smaller than a value
        icp.setTransformationEpsilon (1e-10); // 1e-8 // closeness to converge
        // Set the euclidean distance difference epsilon (criterion 3) -> sum of Euclidean squared errors is smaller than a user defined threshold
        icp.setEuclideanFitnessEpsilon (0.01);
        //icp.setRANSACOutlierRejectionThreshold(0);
      }else{
        
        m4 = m4_new;
        
        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (5); // unit in m
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (1000);
        // Set the transformation epsilon (criterion 2) -> diff between previoud and current is smaller than a value
        icp.setTransformationEpsilon (1e-10); // 1e-8 // closeness to converge
        // Set the euclidean distance difference epsilon (criterion 3) -> sum of Euclidean squared errors is smaller than a user defined threshold
        icp.setEuclideanFitnessEpsilon (0.01);
        //icp.setRANSACOutlierRejectionThreshold(0);

      }

      icp.setInputSource(lidar_cloud_tf);
      icp.setInputTarget(cloud);
      PointCloud Final;
      icp.align(Final, m4); // initial_guess is type of Eigen::Matrix4f

      pcl::PCLPointCloud2::Ptr cloudFinal2 (new pcl::PCLPointCloud2 ());
      pcl::toPCLPointCloud2 (Final,*cloudFinal2);
      cloudFinal2->header.frame_id = "world";
      pcl_conversions::toPCL(ros::Time(0), cloudFinal2->header.stamp); //ros::Time::now()
      pcl_match_pub.publish(cloudFinal2);

      std::cout << "\nhas converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
      m4_new = icp.getFinalTransformation();
      std::cout << m4_new << std::endl;

      nav_msgs::Odometry odom;
      odom.header.frame_id = "world";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = m4_new(0,3);
      odom.pose.pose.position.y = m4_new(1,3);
      odom.pose.pose.position.z = m4_new(2,3);
      m3.setValue(m4_new(0,0) ,m4_new(0,1) ,m4_new(0,2) ,
                  m4_new(1,0) ,m4_new(1,1) ,m4_new(1,2) ,
                  m4_new(2,0) ,m4_new(2,1) ,m4_new(2,2));

      tf2::Quaternion tfq2;
      m3.getRotation(tfq2);

      odom.pose.pose.orientation.x = tfq2[0];
      odom.pose.pose.orientation.y = tfq2[1];
      odom.pose.pose.orientation.z = tfq2[2];
      odom.pose.pose.orientation.w = tfq2[3];

      odom.pose.covariance = [
                              0.05, 0, 0, 0, 0, 0,
                              0, 0.05, 0, 0, 0, 0,
                              0, 0, 0.05, 0, 0, 0,
                              0, 0, 0, 0.05, 0, 0,
                              0, 0, 0, 0, 0.05, 0,
                              0, 0, 0, 0, 0, 0.05,
                            ];
    
      pcl_loc_pub.publish(odom);

      geometry_msgs::TransformStamped transformStamped2;
      
      transformStamped2.header.stamp = ros::Time(0);//ros::Time::now();
      transformStamped2.header.frame_id = "world";
      transformStamped2.child_frame_id = "base_link";
      transformStamped2.transform.translation.x = odom.pose.pose.position.x;
      transformStamped2.transform.translation.y = odom.pose.pose.position.y;
      transformStamped2.transform.translation.z = odom.pose.pose.position.z;
      transformStamped2.transform.rotation.x = odom.pose.pose.orientation.x;
      transformStamped2.transform.rotation.y = odom.pose.pose.orientation.y;
      transformStamped2.transform.rotation.z = odom.pose.pose.orientation.z;
      transformStamped2.transform.rotation.w = odom.pose.pose.orientation.w;

      br.sendTransform(transformStamped2);
      get_lidar = 0;
    }
    ros::spinOnce ();
    //rate.sleep ();
  }
  return 0;
};
