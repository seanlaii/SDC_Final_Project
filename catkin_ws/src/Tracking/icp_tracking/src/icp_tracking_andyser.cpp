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
#include <visualization_msgs/MarkerArray.h>
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
using namespace ros;
using namespace pcl;
using namespace std;
const double PI  =3.141592653589793238463;
#include <sstream>
#include "icp_tracking/Points.h"
#include "icp_tracking/Objects.h"

struct object_point{
	int id;
	std_msgs::Header header;
	Eigen::Vector4d p;
	Eigen::Vector4d velocity;
	int count_pre;
	int count_post;
	int count_dis;
};
class icp_tracker{
  public:
	icp_tracker();
	bool process_node(icp_tracking::Points::Request &req, icp_tracking::Points::Response &res);    // sensor_msgs::PointCloud2
	void assign_marker(object_point point_new);
	visualization_msgs::MarkerArray objects;
	Publisher test_pc_pub;
	Publisher marker_pub;
	Publisher markerarray_pub;
	Publisher pc_map_pub;
	int c;
	double threshhold;
	bool assign;
	int id;
	Publisher objects_pub;
	PointCloud<PointXYZ>::Ptr pc_input;
	PointCloud<PointXYZ>::Ptr pc_filter;
	PointCloud<PointXYZ>::Ptr pc_target;
	PointCloud<PointXYZ>::Ptr pc_raw;
	Eigen::Matrix4f rotation_matrix;
	Eigen::Matrix4f initial_guess;
	vector<object_point> object_list;
	icp_tracking::Objects output_object;
	PointCloud<PointXYZ>::Ptr result;

	// PointCloud<PointXYZ>::Ptr pc_cluster;
	
	// PointCloud<PointXYZ>::Ptr pc_total;

	//// icp ////
	vector < PointCloud<PointXYZ> > pc_buffer;
	PointCloud<PointXYZ>::Ptr map;
	Eigen::Vector3f delta_pos;
	visualization_msgs::Marker marker;
	sensor_msgs::PointCloud2 origin_map;
	/////////////
	IterativeClosestPoint<PointXYZ, PointXYZ> icp;
	VoxelGrid<PointXYZ> downsample;

	
	// Eigen::Matrix4f initial_guess;

	ServiceServer service;
};
icp_tracker::icp_tracker(){
	NodeHandle nh;
	map.reset(new PointCloud<PointXYZ>());
	pc_input.reset(new PointCloud<PointXYZ>());
	pc_filter.reset(new PointCloud<PointXYZ>());
	pc_raw.reset(new PointCloud<PointXYZ>());
	pc_target.reset(new PointCloud<PointXYZ>());
	result.reset(new PointCloud<PointXYZ>());


	// pc_total.reset(new PointCloud<PointXYZ>());
	// pc_filter.reset(new PointCloud<PointXYZ>());

	// 
	//// icp ////
	marker.header.frame_id = "/velodyne";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker_pub = nh.advertise<visualization_msgs::Marker>("path_vis", 1);
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;
	downsample.setLeafSize (0.5, 0.5, 0.5);
	icp.setMaxCorrespondenceDistance(100);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1e-8);
	icp.setMaximumIterations(500); 
	initial_guess.setIdentity(4,4);
	delta_pos.setZero(3);
	/////////////
	c = 0;
	threshhold = 50;
	assign = false;
	id = 0;
	
	rotation_matrix.setIdentity(4,4);
	// initial_guess.setIdentity(4,4);

	//objects_pub = nh.advertise<icp_tracking::Objects>("tracking_result", 10);
	objects_pub = nh.advertise<icp_tracking::Objects>("tracking_result", 10);
	markerarray_pub = nh.advertise<visualization_msgs::MarkerArray>("object_markers", 10);
	test_pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("test_pc", 10);
 	service = nh.advertiseService("Points", &icp_tracker::process_node, this);
 	pc_map_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_pc", 10);
 	cout << "---Server established---" << endl;

}
void icp_tracker::assign_marker(object_point point_new){
	visualization_msgs::Marker marker_text;
	marker_text.header.frame_id = "velodyne";
	marker_text.header.stamp = point_new.header.stamp;
	marker_text.ns = "ID";
	marker_text.action = visualization_msgs::Marker::ADD;
	marker_text.pose.orientation.w = 1.0;
	marker_text.id = point_new.id;
	marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker_text.scale.z = 1;
	marker_text.color.a = 1.0;
	marker_text.color.r = 1.0;
	marker_text.color.g = 0;
	marker_text.color.b = 0;
	marker_text.lifetime = ros::Duration(1);
	marker_text.pose.position.x = point_new.p[0];
	marker_text.pose.position.y = point_new.p[1];
	marker_text.pose.position.z = point_new.p[2];
	stringstream ss;
	ss << point_new.id;
	string str = ss.str();
	marker_text.text = "ID: " + str;
	objects.markers.push_back(marker_text);

	visualization_msgs::Marker marker_sphere;
	marker_sphere.header.frame_id = "velodyne";
	marker_sphere.header.stamp = point_new.header.stamp;
	marker_sphere.ns = "SPHERE";
	marker_sphere.action = visualization_msgs::Marker::ADD;
	marker_sphere.pose.orientation.w = 1.0;
	marker_sphere.id =  point_new.id;
	marker_sphere.type = visualization_msgs::Marker::SPHERE;
	marker_sphere.scale.x = 1;
	marker_sphere.scale.y = 1;
	marker_sphere.scale.z = 1;
	marker_sphere.color.a = 1.0;
	marker_sphere.color.r = 0;
	marker_sphere.color.g = 1.0;
	marker_sphere.color.b = 0;
	marker_sphere.lifetime = ros::Duration(1);
	marker_sphere.pose.position.x = point_new.p[0];
	marker_sphere.pose.position.y = point_new.p[1];
	marker_sphere.pose.position.z = point_new.p[2];
	objects.markers.push_back(marker_sphere);
}
bool icp_tracker::process_node(icp_tracking::Points::Request &req, icp_tracking::Points::Response &res){

	fromROSMsg (req.point_cloud, *pc_raw);

	test_pc_pub.publish(pc_raw);

	std_msgs::Header header = std_msgs::Header();
	header.stamp = req.point_cloud.header.stamp;

	if(c != 0){
		//transformPointCloud (*pc_input, *pc_filter, initial_guess);
		downsample.setInputCloud (pc_raw);
		downsample.filter (*pc_filter);
		icp.setInputSource(pc_filter);
		icp.setInputTarget(pc_target);
		cout << "Number PC of Source: "<< pc_filter->size() << endl;
		cout << "Number PC of Target: "<< pc_target->size() << endl;
		icp.align(*result);
		Eigen::Matrix4f icp_mat = icp.getFinalTransformation() * initial_guess;
		delta_pos(0) = initial_guess(0,3) - icp_mat(0,3);
		delta_pos(1) = initial_guess(1,3) - icp_mat(1,3);
		delta_pos(2) = initial_guess(2,3) - icp_mat(2,3);
		initial_guess = icp_mat;
		cout << "trans: " << delta_pos << endl;
		cout << " score: " << icp.getFitnessScore() << endl;
		cout << initial_guess << endl;
	}
	*pc_target = *pc_raw;
	geometry_msgs::Point p;
	p.x = initial_guess(0,3);
	p.y = initial_guess(1,3);
	p.z = initial_guess(2,3);
	marker.points.push_back(p);
	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);

	transformPointCloud (*pc_raw, *pc_raw, initial_guess);
	*map += (*pc_raw);
	toROSMsg(*map, origin_map);
	origin_map.header.frame_id = "velodyne";
	pc_map_pub.publish(origin_map);

	int i = 0;
	int j = 0;
	int k = 0;
	if(c == 0){
		for(i = 0; i < req.points.size(); i ++){

			object_point initial_point;
			initial_point.p[0] = req.points[i].x;
			initial_point.p[1] = req.points[i].y;
			initial_point.p[2] = req.points[i].z;
			initial_point.p[3] = 1.0;
			initial_point.id = i;
			id ++;
			initial_point.header = req.point_cloud.header;
			initial_point.count_pre = 0;
			initial_point.count_post = 0;
			initial_point.count_dis = 0;
			object_list.push_back(initial_point);

			// output_object.headers.push_back(req.point_cloud.header);
			// output_object.ids.push_back(initial_point.id);
			// geometry_msgs::Point temp_p;
			// temp_p.x = req.points[i].x;
			// temp_p.y = req.points[i].y;
			// temp_p.z = req.points[i].z;
			// output_object.points.push_back(temp_p);
			//assign_marker(initial_point);
		}	
		c++;
		cout << "Initialization complete." << endl;
	}
	else{
		//---------------- doing ICP--------------------
		// 	icp.setInputSource(pc_filter);
		// 	icp.setInputTarget(pc_target);
		// 	icp.align(result);
		//-----------------doing ICP--------------------
		// rotation_matrix = icp.getFinalTransformation();
		// for(i = 0; i < object_list->size(); i ++){
		// 	object_list[k].p = rotation_matrix * object_list[k].p;
	 // 	}

	 	for(int j = 0; j < req.points.size(); j ++){
			Eigen::Vector4d temp;
			for(int k = 0; k < object_list.size(); k ++){
				temp[0] = req.points[j].x;
				temp[1] = req.points[j].y;
				temp[2] = req.points[j].z;
				temp[3] = 1.0;
				if((object_list[k].p-temp).norm() < threshhold){
					
					object_point newpoint;
					newpoint.id = object_list[k].id;
					newpoint.p[0] = req.points[j].x;
					newpoint.p[1] = req.points[j].y;
					newpoint.p[2] = req.points[j].z;
					newpoint.p[3] = 1.0;
					newpoint.header = req.point_cloud.header;
					newpoint.count_pre = object_list[k].count_post;
					newpoint.count_post = object_list[k].count_post + 1;
					newpoint.count_dis = 0;
					object_list.erase(object_list.begin()+k);

					object_list.push_back(newpoint);
					assign = true;

					//assign_marker(newpoint);
				}
			}
			if(assign != true){
				object_point newpoint;
				newpoint.id = id;
				id ++;
				newpoint.p[0] = req.points[j].x;
				newpoint.p[1] = req.points[j].y;
				newpoint.p[2] = req.points[j].z;
				newpoint.p[3] = 1.0;
				newpoint.header = req.point_cloud.header;
				newpoint.count_pre = 0;
				newpoint.count_post = 0;
				newpoint.count_dis = 0;
				object_list.push_back(newpoint);
				//assign_marker(newpoint);

				output_object.headers.push_back(req.point_cloud.header);
				output_object.ids.push_back(id);
				geometry_msgs::Point temp_p;
				temp_p.x = req.points[j].x;
				temp_p.y = req.points[j].y;
				temp_p.z = req.points[j].z;
				output_object.points.push_back(temp_p);
				
			}
			assign = false;
		}
	}
	int z = 0;
	for(i = 0; i < object_list.size(); i ++){	
		if(object_list[i].count_pre == object_list[i].count_post){
			object_list[i].count_dis ++;
		}
		if(object_list[i].count_dis > 3){
			break;
		}
		else if(object_list[i].count_dis <= 3){
			object_point newpoint;
			newpoint.id = object_list[i].id;
			newpoint.p[0] = object_list[i].p[0];
			newpoint.p[1] = object_list[i].p[1];
			newpoint.p[2] = object_list[i].p[2];
			newpoint.p[3] = 1.0;
			newpoint.header = object_list[i].header;
			object_list.push_back(newpoint);
			assign_marker(newpoint);
			z++;
		}
		 
	}	
	cout << "The number of objects: " << z << endl;
	pc_input = pc_target;

	markerarray_pub.publish(objects);
	objects.markers.clear();
	return true;
}

int main(int argc, char** argv){
	init(argc, argv, "icp_tracking");
	icp_tracker icp_tracker;
	ros::spin();
	return 0;
}
