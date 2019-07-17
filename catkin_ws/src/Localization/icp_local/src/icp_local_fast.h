#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <math.h>
// TF
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
// Ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
// Pcl load and ros
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
// Pcl icp
#include <pcl/registration/icp.h>
// Pcl passthrough
#include <pcl/filters/passthrough.h>
// Pcl outlier removal
#include <pcl/filters/statistical_outlier_removal.h>
// Pcl downsampling
#include <pcl/filters/voxel_grid.h>
// Pcl plane filter
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pthread.h>
using namespace ros;
using namespace pcl;
using namespace std;
const double PI  =3.141592653589793238463;

class icp_local{
  public:
	icp_local();
	void load_map();
	void thread_icp();
	void process_node(const sensor_msgs::PointCloud2 msg);    // sensor_msgs::PointCloud2
	PointCloud<PointXYZ> from_pcd(string);
	void save_ptr(icp_local*);
	void get_initial_guess();
	icp_local *cls_ptr;

  	int len;
  	int count;
  	int thres_count;
  	int initial_acc;
  	int score_threshold;
  	int updater;
  	int score_thres;
  	bool param_check;
  	bool icp_set;

  	string path;
	Publisher pc_map_pub;
	Publisher pc_filter_pub;
	Publisher pc_map_loc_pub;
	Publisher pose_pub;
	Subscriber pc_sub;
	pthread_t thread_id;

	PointCloud<PointXYZ>::Ptr map;
	PointCloud<PointXYZ>::Ptr map_process;
	PointCloud<PointXYZ>::Ptr pc_input;
	PointCloud<PointXYZ>::Ptr pc_filter;
	PointCloud<PointXYZ>::Ptr pc_target;
	PointCloud<PointXYZ>::Ptr pc_pass;
	PointCloud<PointXYZ>::Ptr result;
	PointCloud<PointXYZ>::Ptr pc_input_thread;
	PointCloud<PointXYZ>::Ptr pc_target_thread;
	vector < std_msgs::Header > header_list;
	vector < PointCloud<PointXYZ> > pc_buffer;
	vector < Eigen::Matrix4f > pc_transform;

	vector < PointCloud<PointXYZ> > pc_thread_src;
	vector < PointCloud<PointXYZ> > pc_thread_tar;

	IterativeClosestPoint<PointXYZ, PointXYZ> icp;
	PassThrough<PointXYZ> pass;
	StatisticalOutlierRemoval<PointXYZ> sor;
	VoxelGrid<PointXYZ> downsample;
	VoxelGrid<PointXYZ> downsample_map;
	VoxelGrid<PointXYZ> downsample_pass;

	Eigen::Matrix4f initial_guess;

	sensor_msgs::PointCloud2 origin_map;
	sensor_msgs::PointCloud2 ros_cloud_msg;

	pcl::SACSegmentation<PointXYZ> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers;
    pcl::ModelCoefficients::Ptr coefficients;

	Publisher marker_pub;
	uint32_t shape;
	visualization_msgs::Marker marker;
};