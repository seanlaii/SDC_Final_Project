#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <string>
#include <ros/ros.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <pcl/features/fpfh.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <iomanip>
#include <time.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common.h>

#include <pcl/surface/mls.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "icp_tracking/Points.h"

using namespace boost::filesystem;

int WIDTH = 1280, HEIGHT = 720;
Eigen::Matrix4f l_t_c;
cv::Mat INTRINSIC_MAT(3, 3, cv::DataType<double>::type); // Intrinsics
cv::Mat DIST_COEFFS(5, 1, cv::DataType<double>::type); // Distortion vector

ros::Publisher plane_filtered_pub;
ros::Publisher cluster_pub;
ros::Publisher yuan_pub;
ros::Publisher centroid_pub;
ros::Publisher cluster_centroid_pub;
ros::Publisher answer_pub;
visualization_msgs::Marker points;
//// 3d bbox ////
ros::Publisher markerPub;
visualization_msgs::MarkerArray markerArray;
/////////////////
pcl::PassThrough<pcl::PointXYZI> pass;
pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;

ros::ServiceClient client;

void initializeGlobalParams() {
  l_t_c << 0.84592974185943604, 0.53328412771224976, -0.0033089336939156055, 0.092240132391452789,
           0.045996580272912979, -0.079141519963741302, -0.99580162763595581, -0.35709697008132935, 
           -0.53130710124969482, 0.84222602844238281, -0.091477409005165100, -0.16055910289287567,
           0, 0, 0, 1;

  INTRINSIC_MAT.at<double>(0, 0) = 698.939;
  INTRINSIC_MAT.at<double>(1, 0) = 0;
  INTRINSIC_MAT.at<double>(2, 0) = 0;

  INTRINSIC_MAT.at<double>(0, 1) = 0;
  INTRINSIC_MAT.at<double>(1, 1) = 698.939;
  INTRINSIC_MAT.at<double>(2, 1) = 0;

  INTRINSIC_MAT.at<double>(0, 2) = 641.868;
  INTRINSIC_MAT.at<double>(1, 2) = 385.788;
  INTRINSIC_MAT.at<double>(2, 2) = 1.0;

  DIST_COEFFS.at<double>(0) = -0.171466;
  DIST_COEFFS.at<double>(1) = 0.0246144;
  DIST_COEFFS.at<double>(2) = 0;
  DIST_COEFFS.at<double>(3) = 0;
  DIST_COEFFS.at<double>(4) = 0;

  points.header.frame_id = "velodyne";
  points.header.stamp = ros::Time::now();
  //points.lifetime = ros::Duration(0.01);
  points.ns = "points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 4;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.scale.x = 0.5;
  points.scale.y = 0.5;
  points.color.g = 1.0f;
  points.color.a = 1.0;

  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.5, 100);
  pass.setFilterLimitsNegative (true);

  sor.setMeanK (50);
  sor.setStddevMulThresh (0.5);
}


void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  icp_tracking::Points srv;
  //// 3d bbox ////
  int count = 0;
  markerArray.markers.clear();
  /////////////////
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  fromROSMsg(*msg, *cloud);
  std::cout<< "original size:" << cloud->size()<<std::endl;
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.08f, 0.08f, 0.08f);
  vg.filter (*cloud_filtered);
  
  //pass.setInputCloud (cloud);
  //pass.filter (*cloud_filtered); 
  sor.setInputCloud(cloud_filtered);
  sor.filter(*cloud_filtered);
  std::cout<< "outlier size:" << cloud_filtered->size()<<std::endl;
  //*cloud_filtered = *cloud;
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  //pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::PCDWriter writer;



  pcl::PointIndicesPtr ground(new pcl::PointIndices);
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
  pmf.setInputCloud (cloud_filtered);
  pmf.setMaxWindowSize (1);//20
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (3.0f);
  pmf.extract (ground->indices);

  /*seg.setOptimizeCoefficients(true);
  //seg.setModelType (pcl::SACMODEL_PLANE);
  //seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setMaxIterations (100);
  //seg.setDistanceThreshold (1);

  
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);  // pcl::SACMODEL_PLANE   SACMODEL_PERPENDICULAR_PLANE
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (200);								
  seg.setDistanceThreshold (0.35); 	// 0.5
  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); 	// xy-plane 
  seg.setAxis(axis);
  seg.setEpsAngle(  10.0f * (3.141592653589793238463/180.0f) );   				// Range degree of plane 
  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0) {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }
*/
  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (ground);
  //extract.setNegative (false);

  // Get the points associated with the planar surface
  extract.filter (*cloud_plane);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_f);
  *cloud_filtered = *cloud_f;
  
  //// KD tree ////
  // std::vector<int> pointIdxRadiusSearch;
  // std::vector<float> pointRadiusSquaredDistance;
  // pcl::PointXYZI origin;
  // pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  // kdtree.setInputCloud(cloud_filtered);
  // kdtree.radiusSearch(origin, 15, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  // boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (pointIdxRadiusSearch)); // Convert to Point Indices
  // pcl::ExtractIndices<pcl::PointXYZI> indiceFilter(true);
  // indiceFilter.setInputCloud(cloud_filtered);
  // indiceFilter.setIndices(indicesptr);
  // indiceFilter.filter(*cloud_filtered);
  ///////////////////

  //// Remove outlier ////
  // pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
  // outrem.setInputCloud(cloud_filtered);
  // outrem.setRadiusSearch(0.35);
  // outrem.setMinNeighborsInRadius (4);
  // outrem.filter (*cloud_filtered);
  ///////////////////////

  //// upsample ////
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  // copyPointCloud(*cloud_filtered, *cloud_xyz);
  // cloud_xyz->points.resize(cloud_filtered->size());
  // cloud_xyz->width = 1;
  // cloud_xyz->height = cloud_filtered->size();
  // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
  // filter.setInputCloud(cloud_xyz);
  // pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
  // filter.setSearchMethod(kdtree);
  // filter.setSearchRadius(3);
  // filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
  // filter.setUpsamplingRadius(3);
  // filter.setUpsamplingStepSize(2);
  // filter.process(*cloud_xyz);
  // copyPointCloud(*cloud_xyz, *cloud_filtered);
  // cloud_filtered->points.resize(cloud_xyz->size());
  // cloud_filtered->width = 1;
  // cloud_filtered->height = cloud_xyz->size();
  //////////////////

  std::cout<< "segment size:" << cloud_filtered->size()<<std::endl;

  sensor_msgs::PointCloud2 filtered_cloud;
  pcl::toROSMsg(*cloud_filtered, filtered_cloud);
  filtered_cloud.header.frame_id = "velodyne";
  plane_filtered_pub.publish(filtered_cloud);
  srv.request.point_cloud = *msg;
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance (0.49); // 2cm
  ec.setMinClusterSize (5);
  ec.setMaxClusterSize (2000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  std::cout << "Cluster size: " << cluster_indices.size() << std::endl;


  int j = 50;
  int k = 0;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr center_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_centers(new pcl::PointCloud<pcl::PointXYZI>);

  points.points.clear();

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    float max_z = INT_MIN, min_z =INT_MAX, max_y = INT_MIN, min_y =INT_MAX, max_x = INT_MIN, min_x =INT_MAX;
    // extract clusters and save as a single point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      if(cloud_filtered->points[*pit].z > max_z)
      	max_z = cloud_filtered->points[*pit].z;
      if(cloud_filtered->points[*pit].z < min_z)
      	min_z = cloud_filtered->points[*pit].z;
      if(cloud_filtered->points[*pit].y > max_y)
      	max_y = cloud_filtered->points[*pit].y;
      if(cloud_filtered->points[*pit].y < min_y)
      	min_y = cloud_filtered->points[*pit].y;
      if(cloud_filtered->points[*pit].x > max_x)
      	max_x = cloud_filtered->points[*pit].x;
      if(cloud_filtered->points[*pit].x < min_x)
      	min_x = cloud_filtered->points[*pit].x;
    }

   
    if(max_z - min_z > 3)
    	continue;
    if(max_z > 0)
      continue;
    //else if(max_z - min_z < 0.1)
    //	continue;
    
    // if(sqrt(pow(max_x - min_x, 2)+pow(max_y - min_y, 2)) > 9)
    // 	continue;
    //else if(sqrt(pow(max_x - min_x, 2)+pow(max_y - min_y, 2)) < 1)
    	//continue;
/*
    if(max_x - min_x > 3.8)
    	continue;
    
    
    if(max_y - min_y > 3.8)
    	continue;*/
    //if(max_x -min_x < 1 && max_y - min_y < 1)
    //	continue;
    
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      cloud_filtered->points[*pit].intensity = j;
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    /////// 3d bbox //////
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud (cloud_cluster);
    feature_extractor.compute ();

    visualization_msgs::Marker marker;
    geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8;

    marker.header.frame_id = "/velodyne";
    marker.header.stamp = ros::Time::now();
    marker.ns = "lines";
    marker.id = count;
    count ++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    pcl::PointXYZI minPt, maxPt, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    feature_extractor.getOBB (minPt, maxPt, position_OBB, rotational_matrix_OBB);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    float height_obb = maxPt.z - minPt.z;
    if (height_obb < 0.1)
    	continue;

    //if (maxPt.z > 0)
    //	continue;
    // std::cout << height_obb << std::endl;		 // Height 
    // Eigen::Quaternionf quat (rotational_matrix_OBB);

    // pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
    Eigen::Vector3f a(minPt.x, minPt.y, minPt.z);
    a = rotational_matrix_OBB * a + position;
    p1.x = a(0);
    p1.y = a(1);
    p1.z = a(2);
    Eigen::Vector3f b(minPt.x, maxPt.y, minPt.z);
    b = rotational_matrix_OBB * b + position;
    p2.x = b(0);
    p2.y = b(1);
    p2.z = b(2);
    Eigen::Vector3f c(maxPt.x, maxPt.y, minPt.z);
    c = rotational_matrix_OBB * c + position;
    p3.x = c(0);
    p3.y = c(1);
    p3.z = c(2);
    Eigen::Vector3f d(maxPt.x, minPt.y, minPt.z);
    d = rotational_matrix_OBB * d + position;
    p4.x = d(0);
    p4.y = d(1);
    p4.z = d(2);
    Eigen::Vector3f e(minPt.x, minPt.y, maxPt.z);
    e = rotational_matrix_OBB * e + position;
    p5.x = e(0);
    p5.y = e(1);
    p5.z = e(2);
    Eigen::Vector3f f(minPt.x, maxPt.y, maxPt.z);
    f = rotational_matrix_OBB * f + position;
    p6.x = f(0);
    p6.y = f(1);
    p6.z = f(2);
    Eigen::Vector3f g(maxPt.x, maxPt.y, maxPt.z);
    g = rotational_matrix_OBB * g + position;
    p7.x = g(0);
    p7.y = g(1);
    p7.z = g(2);
    Eigen::Vector3f h(maxPt.x, minPt.y, maxPt.z);
    h = rotational_matrix_OBB * h + position;
    p8.x = h(0);
    p8.y = h(1);
    p8.z = h(2);

    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p4);
    marker.points.push_back(p1);
    marker.points.push_back(p5);
    marker.points.push_back(p6);
    marker.points.push_back(p7);
    marker.points.push_back(p8);
    marker.points.push_back(p5);
    marker.points.push_back(p6);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p7);
    marker.points.push_back(p8);
    marker.points.push_back(p4);

    marker.lifetime = ros::Duration(1);
    markerArray.markers.push_back(marker); 
    //////////////////////

    *cloud_clusters += *cloud_cluster;

    // compute cluster centroid and publish
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    pcl::PointXYZI center_point;
    center_point.x = position_OBB.x;
    center_point.y = position_OBB.y;
    center_point.z = position_OBB.z;
    center_point_cloud->points.push_back(center_point);

    geometry_msgs::Point c_p;
    c_p.x = center_point.x;
    c_p.y = center_point.y;
    c_p.z = center_point.z;
    srv.request.points.push_back(c_p);
    
    geometry_msgs::Point p;
    p.x = position_OBB.x;
    p.y = position_OBB.y;
    p.z = position_OBB.z;
    points.points.push_back(p);

    j+=2;
    nav_msgs::Odometry p_odom;
    p_odom.header = msg->header;
	p_odom.pose.pose.position.x = p.x;
	p_odom.pose.pose.position.y = p.y;
	p_odom.pose.pose.position.z = p.z;
	answer_pub.publish(p_odom);

  }
  centroid_pub.publish(points);
  //// 3d bbox ////
  markerPub.publish(markerArray);
  /////////////////
  for (size_t i = 0 ; i < center_point_cloud->size() ; ++i)
      {

       //center_point_cloud.points[i].r = 255;
       //center_point_cloud.points[i].b = 0;
       //center_point_cloud.points[i].g = 0;
       //std::cout << center_point_cloud->points[i] <<std::endl;
      }
  
  for (size_t i = 0 ; i < cloud_clusters->size() ; ++i)
      {
       //cloud.points[i].r = 255;
       //cloud.points[i].b = 0;
       //cloud.points[i].g = 0;
       //std::cout << cloud_clusters->points[i] <<std::endl;
      }
  std::cout<< "cloud_filtered size:" << cloud_clusters->size()<<std::endl;
  sensor_msgs::PointCloud2 cluster_cloud, centers_point_cloud;
  pcl::toROSMsg(*cloud_clusters, cluster_cloud);
  pcl::toROSMsg(*center_point_cloud, centers_point_cloud);
  cluster_cloud.header.frame_id = "velodyne";
  cluster_pub.publish(cluster_cloud);

  centers_point_cloud.header.frame_id = "velodyne";
  cluster_centroid_pub.publish(centers_point_cloud);
  yuan_pub.publish(msg);

  if (client.call(srv)){
    ROS_INFO("Calling service success.");
  }
  else{
    ROS_ERROR("Failed to call service icp_tracker.");
  }

}




int main(int argc, char **argv) {
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;
  //// 3d bbox ////
  markerPub = nh.advertise<visualization_msgs::MarkerArray>("bbox", 10);
  /////////////////
  plane_filtered_pub = nh.advertise<sensor_msgs::PointCloud2 >("plane_filtered_pub_points", 1);
  cluster_pub = nh.advertise<sensor_msgs::PointCloud2 >("cluster_cloud", 1);
  cluster_centroid_pub = nh.advertise<sensor_msgs::PointCloud2 >("points_cluster", 1);
  yuan_pub = nh.advertise<sensor_msgs::PointCloud2 >("points_yuan", 1);
  centroid_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Subscriber lidar_sub = nh.subscribe("points_raw", 1000, lidar_callback);
  answer_pub = nh.advertise<nav_msgs::Odometry> ("/cluster_answer", 10);
  client = nh.serviceClient<icp_tracking::Points>("Points");
  initializeGlobalParams();
  // ros::Rate loop_rate(10);
  // while (ros::ok()) {
  //   loop_rate.sleep();
  ros::spin();
  //}
}

