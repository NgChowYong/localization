#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>   
#include "tf2/transform_datatypes.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

// initial position guessing for yaw angle
int first_guess = 0;
double initial_angle = 2.37; // in rad // by trial and error
int get_lidar = 0;

void getGPSCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  // take first first gps data for initial position for icp used
  if (first_guess == 0){
    first_guess = 1;
    gps_data = *msg;
  }
}

void getLidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{   
  pcl::PCLPointCloud2 pcl_pc2;    
  PointCloud::Ptr temp_cloud(new PointCloud);
  // convert message to point cloud 2
  pcl_conversions::toPCL(*msg, pcl_pc2);
  // convert from point cloud 2 to point cloud XYZ
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  // downsample for uniform sample and store in global variable lidar_cloud
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (temp_cloud);
  sor.setLeafSize (0.5f, 0.5f, 0.5f);
  sor.filter (*lidar_cloud);
  get_lidar = 1;

}


int main(int argc, char** argv){

  // initial node
  ros::init(argc, argv, "lidar_process");
  ros::NodeHandle node;

  // initial subscriber
  ros::Subscriber sub_gps = node.subscribe("gps", 5, getGPSCallback);
  ros::Subscriber sub_lidar = node.subscribe("lidar_points", 5, getLidarCallback);

  // initial publisher
  ros::Publisher pcl_map_pub = node.advertise<sensor_msgs::PointCloud2>("map", 1);
  ros::Publisher pcl_match_pub = node.advertise<sensor_msgs::PointCloud2>("match", 1);
  ros::Publisher pcl_loc_pub = node.advertise<nav_msgs::Odometry>("lidar_odom", 1);
  ros::Rate rate(0.5);

  // transformation variable
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

  // convert PCD to Point Cloud 2 for down sample
  // save result in cloud_filtered then save to cloud again
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2 (*cloud,*cloud2);
  
  // down sampled PCD
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud2);
  sor.setLeafSize (0.5f, 0.5f, 0.5f);
  sor.filter (*cloud_filtered);
  // save to cloud data
  pcl::fromPCLPointCloud2 (*cloud_filtered,*cloud);
  cloud->header.frame_id = "world";

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;


  // wait for GPS data first
  while(first_guess == 0 && node.ok()){
    pcl_conversions::toPCL(ros::Time(0), cloud->header.stamp);//ros::Time::now()
    pcl_map_pub.publish (cloud);
    ros::spinOnce ();
    rate.sleep ();
  }
  // after obtain GPS data, shutdown GPS subscriber
  sub_gps.shutdown();

  // first guess transform
  // m4 is for icp used
  // m3 is for converting rotation matrix to quaternion
  Eigen::Matrix<float, 4, 4> m4;
  Eigen::Matrix<float, 4, 4> m4_new;
  tf2::Matrix3x3 m3;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  ros::Time last_time = ros::Time(0);

  while (node.ok()){

    if((ros::Time(0) - last_time) > ros::Duration(0.08)){
      // publish map
      pcl_conversions::toPCL(ros::Time(0), cloud->header.stamp);//ros::Time::now()
      pcl_map_pub.publish (cloud);
    }

    // process once get lidar data
    if (get_lidar == 1){
      
      // store lidar cloud (down sampled) in lidar cloud tf
      PointCloud::Ptr lidar_cloud_tf (new PointCloud);  
      lidar_cloud_tf = lidar_cloud;

      // for first GPS data, do initial guesss of position
      if (first_guess == 1){
        first_guess = 2;
        
        // compute quaternion and rotation matrix from inigital angle guess
        tf2::Quaternion tfq (0,0,sin(initial_angle/2),cos(initial_angle/2));
        tf2::Vector3 tfv (gps_data.point.x,gps_data.point.y,gps_data.point.z);
        tf2::Transform tftransform (tfq, tfv);
        m3 = tftransform.getBasis();
        // compute homogeneous transformation matrix for initial point
        m4 << m3[0][0] ,m3[0][1] ,m3[0][2] ,gps_data.point.x ,
              m3[1][0] ,m3[1][1] ,m3[1][2] ,gps_data.point.y ,
              m3[2][0] ,m3[2][1] ,m3[2][2] ,gps_data.point.z ,
              0 ,0 ,0 ,1;
        std::cout << m4;

        // broadcast world to base link frame
        geometry_msgs::TransformStamped transformStamped2;
        transformStamped2.header.stamp = ros::Time(0);//ros::Time::now();
        transformStamped2.header.frame_id = "world";
        transformStamped2.child_frame_id = "base_link";
        transformStamped2.transform.translation.x = gps_data.point.x;
        transformStamped2.transform.translation.y = gps_data.point.y;
        transformStamped2.transform.translation.z = gps_data.point.z;
        transformStamped2.transform.rotation.x = tfq[0];
        transformStamped2.transform.rotation.y = tfq[1];
        transformStamped2.transform.rotation.z = tfq[2];
        transformStamped2.transform.rotation.w = tfq[3];
        br.sendTransform(transformStamped2);

        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (5); // unit in m
      }else{
        // for new matrix measure
        icp.setMaxCorrespondenceDistance (2); // unit in m
      }

      // Set the maximum number of iterations (criterion 1)
      icp.setMaximumIterations (1000);
      // Set the transformation epsilon (criterion 2) -> diff between previoud and current is smaller than a value
      icp.setTransformationEpsilon (1e-8); // 1e-8 // closeness to converge
      // Set the euclidean distance difference epsilon (criterion 3) -> sum of Euclidean squared errors is smaller than a user defined threshold
      icp.setEuclideanFitnessEpsilon (0.01);

      // do icp for lidar to map and store in final
      icp.setInputSource(lidar_cloud_tf);
      icp.setInputTarget(cloud);
      PointCloud Final;
      icp.align(Final, m4); // initial_guess is type of Eigen::Matrix4f

      // convert point cloud XYZ to point cloud 2
      pcl::PCLPointCloud2::Ptr cloudFinal2 (new pcl::PCLPointCloud2 ());
      pcl::toPCLPointCloud2 (Final,*cloudFinal2);      
      cloudFinal2->header.frame_id = "world";
      // publish point cloud      
      pcl_conversions::toPCL(ros::Time(0), cloudFinal2->header.stamp); //ros::Time::now()
      pcl_match_pub.publish(cloudFinal2);

      // check for icp result, if matched will have a small fitness score
      std::cout << "\nhas converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
      m4_new = icp.getFinalTransformation();
      m4 = m4_new;

      std::cout << m4_new << std::endl;

      // transformation from world to base link
      // get transform from world to velodyne
      m3.setValue(m4_new(0,0) ,m4_new(0,1) ,m4_new(0,2) ,
                  m4_new(1,0) ,m4_new(1,1) ,m4_new(1,2) ,
                  m4_new(2,0) ,m4_new(2,1) ,m4_new(2,2));
      tf2::Transform tffrom;
      tf2::Vector3 v3;      
      tffrom.setBasis(m3);
      v3.setValue(m4_new(0,3) ,m4_new(1,3) ,m4_new(2,3));
      tffrom.setOrigin(v3);

      //get transform from velodyne to base link
      geometry_msgs::TransformStamped transformStamped;
      try{
          transformStamped = tfBuffer.lookupTransform("velodyne", "base_link", ros::Time(0));
          // transform from world to base link
          tf2::Transform tfout;
          geometry_msgs::TransformStamped A;
          geometry_msgs::TransformStamped B;
          A.transform = tf2::toMsg(tffrom);
          B.transform = tf2::toMsg(tfout);
          tf2::doTransform(transformStamped,B,A);
          tf2::fromMsg(B.transform,tfout);          
          m3 = tfout.getBasis();
          v3 = tfout.getOrigin();
          m4_new << m3[0][0] ,m3[0][1] ,m3[0][2] , v3[0],
                    m3[1][0] ,m3[1][1] ,m3[1][2] , v3[1], 
                    m3[2][0] ,m3[2][1] ,m3[2][2] , v3[2],
                    0 ,0 ,0 ,1;
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        // ros::Duration(1.0).sleep();
        // continue;
      }

      // from icp data publish to final result output
      // publish lidar_odom result
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
      odom.pose.covariance = {0.05, 0, 0, 0, 0, 0,
                              0, 0.05, 0, 0, 0, 0,
                              0, 0, 0.05, 0, 0, 0,
                              0, 0, 0, 0.05, 0, 0,
                              0, 0, 0, 0, 0.05, 0,
                              0, 0, 0, 0, 0, 0.05
                            };
      pcl_loc_pub.publish(odom);
      get_lidar = 0;
    }
    ros::spinOnce ();
    //rate.sleep ();
  }
  return 0;
};
