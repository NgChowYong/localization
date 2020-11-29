#include <ros/ros.h>
#include <iostream>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>   
#include "tf2/transform_datatypes.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/icp.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// some global param
PointCloud::Ptr lidar_cloud (new PointCloud);
geometry_msgs::PointStamped gps_data;
nav_msgs::Odometry odom_data;
// initial position guessing for yaw angle
int first_guess = 0;
int get_lidar = 0;
int get_odom = 0;
int difference_flag = 1;

// to publish comparable result after filtering
ros::Publisher pcl_inter_pub;

// data get from ros param
std::string file_name1,file_name2,file_name3,tf_lidar,tf_car;
double initial_angle = 2.37; // in rad // by trial and error

void getGPSCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  // take first first gps data for initial position for icp used
  if (first_guess == 0){
    first_guess = 1;
  }
  gps_data = *msg;
}

void getLidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{   
  // data transformation
  pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr filter_pcl_pc2(new pcl::PCLPointCloud2 ());
  PointCloud::Ptr temp_cloud(new PointCloud);
  // convert message to point cloud 2
  pcl_conversions::toPCL(*msg, *pcl_pc2);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor2;
  sor2.setLeafSize (0.1f, 0.1f, 0.1f);
  sor2.setInputCloud (pcl_pc2);
  sor2.filter (*filter_pcl_pc2);
  pcl_pc2 = filter_pcl_pc2;
  // convert from point cloud 2 to point cloud XYZ
  pcl::fromPCLPointCloud2(*pcl_pc2,*temp_cloud);

  // statistical removal (remove noisy data)
  PointCloud::Ptr lidar_cloud_filter (new PointCloud);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(temp_cloud);
  sor.setMeanK(30);
  sor.setStddevMulThresh(1.5);
  sor.filter(*lidar_cloud_filter);
  temp_cloud = lidar_cloud_filter;

  // estimate normal
  pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
  ne.setInputCloud(temp_cloud);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.5);
  ne.compute(*cloud_normals);

  // sample by using normal
  std::cout << "normal size: " << cloud_normals->size() << "\n";
  pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nsmp;
  nsmp.setInputCloud(temp_cloud);
  nsmp.setNormals(cloud_normals);
  nsmp.setBins(2,2,2);
  nsmp.setSeed(0);
  nsmp.setSample(cloud_normals->size()/2);
  nsmp.filter(*lidar_cloud);

  // remove outlier (in dataset 2)
  // remove ground
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(lidar_cloud);
  pass.setFilterLimits(-2.0,30);
  pass.setFilterFieldName("z");
  pass.filter(*lidar_cloud);

  // convert point cloud XYZ to point cloud 2
  pcl::PCLPointCloud2::Ptr cloudFinal3 (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2 (*lidar_cloud,*cloudFinal3);      
  cloudFinal3->header.frame_id = tf_lidar;
  // publish point cloud      
  pcl_conversions::toPCL(ros::Time(0), cloudFinal3->header.stamp); //ros::Time(0)
  pcl_inter_pub.publish(cloudFinal3);

  get_lidar = 1;
}

void getOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  odom_data = *msg;
  get_odom = 1;
}

int main(int argc, char** argv){

  // initial node
  ros::init(argc, argv, "new_lidar_process");
  ros::NodeHandle node;

  // initial subscriber
  ros::Subscriber sub_gps = node.subscribe("gps", 5, getGPSCallback);
  ros::Subscriber sub_lidar = node.subscribe("lidar_points", 5, getLidarCallback);
  ros::Subscriber sub_ekf = node.subscribe("odom", 5, getOdomCallback);

  // initial publisher
  ros::Publisher init_pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("set_pose", 1);
  ros::Publisher lidar_ekf_pose = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("lidar_cov", 1);
  ros::Publisher pcl_map_pub = node.advertise<sensor_msgs::PointCloud2>("map", 2);
  ros::Publisher pcl_match_pub = node.advertise<sensor_msgs::PointCloud2>("match", 1);
  ros::Publisher pcl_loc_pub = node.advertise<nav_msgs::Odometry>("lidar_odom", 1);
  pcl_inter_pub = node.advertise<sensor_msgs::PointCloud2>("interpolate", 1);

  // transformation variable
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster br;

  // read from rosparam
  node.getParam("file1", file_name1);
  node.getParam("file2", file_name2);
  node.getParam("file3", file_name3);
  node.getParam("init_angle", initial_angle);
  node.getParam("tf_lidar", tf_lidar);
  node.getParam("tf_car", tf_car);
  std::cout << "\nread car tf:" << tf_car <<"\n";
  std::cout << "read file:" << file_name1 <<"\n";
  std::cout << "read file:" << file_name2 <<"\n";
  std::cout << "read file:" << file_name3 <<"\n";
  std::cout << "read angle:" << initial_angle <<"\n";
  std::cout << "read lidar tf:" << tf_lidar <<"\n";

  // load PCD map
  PointCloud::Ptr cloud (new PointCloud);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name1, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded 1 "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

  int flag_ = 0;
  // load PCD map
  PointCloud::Ptr nextcloud (new PointCloud);
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/icmems/Desktop/SDC/hwk_localization_midterm/SDC_data/SDC_Data/map/nuscenes_maps/map_1800_800.pcd", *nextcloud) == -1) //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name2, *nextcloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    //return (-1);
    flag_ = 1;
  }
  std::cout << "Loaded 2 "
            << nextcloud->width * nextcloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;// load PCD map

  PointCloud::Ptr next2cloud (new PointCloud);
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/icmems/Desktop/SDC/hwk_localization_midterm/SDC_data/SDC_Data/map/nuscenes_maps/map_1900_800.pcd", *next2cloud) == -1) //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name3, *next2cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    //return (-1);
    flag_ = 1;
  }
  std::cout << "Loaded 3 "
            << next2cloud->width * next2cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

  if (flag_ == 0){
    // combine of 3 map
    PointCloud::Ptr p_totalcloud (new PointCloud);
    *p_totalcloud += *cloud;
    *p_totalcloud += *nextcloud;
    *p_totalcloud += *next2cloud;
    *cloud = *p_totalcloud;
  }

  // convert PCD to Point Cloud 2 for down sample
  // save result in cloud_filtered then save to cloud again
  pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2 (*cloud,*cloud2);
 
  // down sampled PCD  
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setLeafSize (0.5f, 0.5f, 0.5f);
  sor.setInputCloud (cloud2);
  sor.filter (*cloud_filtered);
  
  // save to cloud data
  pcl::fromPCLPointCloud2 (*cloud_filtered,*cloud);
  cloud->header.frame_id = "world";
  // output data
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
  

  // broadcast initial world to base link frame
  geometry_msgs::TransformStamped transformStamped2;
  transformStamped2.header.stamp = ros::Time::now();
  transformStamped2.header.frame_id = "world";
  transformStamped2.child_frame_id = tf_car;
  transformStamped2.transform.translation.x = 0;
  transformStamped2.transform.translation.y = 0;
  transformStamped2.transform.translation.z = 0;
  transformStamped2.transform.rotation.x = 0;
  transformStamped2.transform.rotation.y = 0;
  transformStamped2.transform.rotation.z = 0;
  transformStamped2.transform.rotation.w = 1;
  br.sendTransform(transformStamped2);

  // publish cloud
  std::cout << "publish cloud with time: " << ros::Time::now() << "\n";
  pcl_conversions::toPCL(ros::Time(0), cloud->header.stamp);
  sensor_msgs::PointCloud2 cmsg;
  pcl::toROSMsg (*cloud, cmsg);
  pcl_map_pub.publish (cmsg);
  ros::Rate rate_(100); // loop hertz

  // wait for gps data
  while((first_guess == 0) && node.ok()){
    ros::spinOnce();
    rate_.sleep();
  }
  std::cout << "after data once\n";

// first guess transform
// m4 is for icp used
// m3 is for converting rotation matrix to quaternion
Eigen::Matrix<float, 4, 4> m4;
Eigen::Matrix<float, 4, 4> m4_new;
tf2::Matrix3x3 m3;
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

ros::Time last_time = ros::Time::now();
std::cout << last_time << "\n";
while (node.ok()){

  // output cloud at constant period
  if((ros::Time::now().toNSec() - last_time.toNSec()) > 0.1*100000000){
    // publish map
    cloud->header.frame_id = "world";
    std::cout << "publish cloud with time loop: " << ros::Time::now() << "\n";
    pcl_conversions::toPCL(ros::Time(0), cloud->header.stamp);//ros::Time(0)
    pcl_map_pub.publish (*cloud);
    last_time = ros::Time::now();
  }

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
      // std::cout << m4;

      // broadcast world to base link frame
      geometry_msgs::TransformStamped transformStamped2;
      transformStamped2.header.stamp = ros::Time::now();
      transformStamped2.header.frame_id = "world";
      transformStamped2.child_frame_id = tf_car;
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
      std::cout << "bef: " << m4(0,3) << "," << m4(1,3) << "\n";
      if (difference_flag == 0){
        m4 << m4(0,0) ,m4(0,1) ,m4(0,2) , gps_data.point.x,
            m4(1,0) ,m4(1,1) ,m4(1,2) , gps_data.point.y,
            m4(2,0) ,m4(2,1) ,m4(2,2) , gps_data.point.z,
            0, 0, 0, 1;
        icp.setMaxCorrespondenceDistance (5); // unit in m
      }else{
        icp.setMaxCorrespondenceDistance (1.0); // unit in m // 1.1 for first
      }
      std::cout << "aft: " << m4(0,3) << "," << m4(1,3) << "\n";
    }

    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (1000);
    // Set the transformation epsilon (criterion 2) -> diff between previoud and current is smaller than a value
    icp.setTransformationEpsilon (1e-9); // 1e-8 // closeness to converge
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
    pcl_conversions::toPCL(ros::Time(0), cloudFinal2->header.stamp); //ros::Time(0)
    pcl_match_pub.publish(cloudFinal2);

    // check for icp result, if matched will have a small fitness score
    std::cout << "\nhas converged:" << icp.hasConverged() << " score: " <<
            icp.getFitnessScore() << std::endl;
    m4_new = icp.getFinalTransformation();

    difference_flag = 1;
    if ((m4(0,3) - m4_new(0,3)) < 0.1 && (m4(1,3) - m4_new(1,3)) < 0.1 ){
      difference_flag = 0;
    }
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
        transformStamped = tfBuffer.lookupTransform(tf_lidar, tf_car,ros::Time(0));
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

    // get icp result
    m3.setValue(m4_new(0,0) ,m4_new(0,1) ,m4_new(0,2) ,
                m4_new(1,0) ,m4_new(1,1) ,m4_new(1,2) ,
                m4_new(2,0) ,m4_new(2,1) ,m4_new(2,2));
                                                                                                                                                                                                                                                                                                                                                                                                                
    tf2::Quaternion tfq2;
    m3.getRotation(tfq2);

    // broadcast world to base link frame
    geometry_msgs::TransformStamped transformStamped2;
    transformStamped2.header.stamp = ros::Time::now();
    transformStamped2.header.frame_id = "world";
    transformStamped2.child_frame_id = tf_car;
    transformStamped2.transform.translation.x = m4_new(0,3);
    transformStamped2.transform.translation.y = m4_new(1,3);
    transformStamped2.transform.translation.z = m4_new(2,3);
    transformStamped2.transform.rotation.x = tfq2[0];
    transformStamped2.transform.rotation.y = tfq2[1];
    transformStamped2.transform.rotation.z = tfq2[2];
    transformStamped2.transform.rotation.w = tfq2[3];
    br.sendTransform(transformStamped2);

    // from icp data publish to final result output
    // publish lidar_odom result
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time(0);
    odom.header.frame_id = "world";
    odom.child_frame_id = tf_car;
    odom.pose.pose.position.x = m4_new(0,3);
    odom.pose.pose.position.y = m4_new(1,3);
    odom.pose.pose.position.z = m4_new(2,3);
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
    pcl_loc_pub.publish(odom_data);

    if (first_guess == 2){
      geometry_msgs::PoseWithCovarianceStamped init_pose;
      init_pose.header.stamp = ros::Time::now();
      init_pose.header.frame_id = "world";
      init_pose.pose.pose.position.x = m4_new(0,3);
      init_pose.pose.pose.position.y = m4_new(1,3);
      init_pose.pose.pose.position.z = m4_new(2,3);
      init_pose.pose.pose.orientation.x = tfq2[0];
      init_pose.pose.pose.orientation.y = tfq2[1];
      init_pose.pose.pose.orientation.z = tfq2[2];
      init_pose.pose.pose.orientation.w = tfq2[3];
      init_pose.pose.covariance = {0.1, 0, 0, 0, 0, 0,
                                    0, 0.1, 0, 0, 0, 0,
                                    0, 0, 0.1, 0, 0, 0,
                                    0, 0, 0, 0.1, 0, 0,
                                    0, 0, 0, 0, 0.1, 0,
                                    0, 0, 0, 0, 0, 0.1
                                  };
      init_pose_pub.publish(init_pose);
      first_guess = 3 ;
    }else{        
      if (difference_flag == 1){
        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";
        pose.pose.pose.position.x = m4_new(0,3);
        pose.pose.pose.position.y = m4_new(1,3);
        pose.pose.pose.position.z = m4_new(2,3);
        pose.pose.pose.orientation.x = tfq2[0];
        pose.pose.pose.orientation.y = tfq2[1];
        pose.pose.pose.orientation.z = tfq2[2];
        pose.pose.pose.orientation.w = tfq2[3];
        pose.pose.covariance = {0.1, 0, 0, 0, 0, 0,
                                0, 0.1, 0, 0, 0, 0,
                                0, 0, 0.1, 0, 0, 0,
                                0, 0, 0, 0.1, 0, 0,
                                0, 0, 0, 0, 0.1, 0,
                                0, 0, 0, 0, 0, 0.1
                              };
        lidar_ekf_pose.publish(pose);
      }
    }
    get_lidar = 0;
  }
  ros::spinOnce ();
}
  return 0;
};
