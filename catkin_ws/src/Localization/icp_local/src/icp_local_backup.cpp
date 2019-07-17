#include "icp_local.h"

icp_local::icp_local(){
	NodeHandle nh;
	map.reset(new PointCloud<PointXYZ>());
	map_process.reset(new PointCloud<PointXYZ>());
	pc_input.reset(new PointCloud<PointXYZ>());
	pc_filter.reset(new PointCloud<PointXYZ>());

	pc_target.reset(new PointCloud<PointXYZ>());
	result.reset(new PointCloud<PointXYZ>());

	path = package::getPath("icp_local");


	// cov = {0.001, 0.0, 0.0, 0.0, 0.0, 0.0,\
	// 		0.0, 0.001, 0.0, 0.0, 0.0, 0.0,\
	// 		 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,\
	// 		  0.0, 0.0, 0.0, 0.001, 0.0, 0.0,\
	// 		   0.0, 0.0, 0.0, 0.0, 0.001, 0.0,\
	// 		    0.0, 0.0, 0.0, 0.0, 0.0, 0.03};

	len = 1;
	count = 0;
	pc_thread_src = vector< PointCloud<PointXYZ> >(0);
	pc_thread_tar = vector< PointCloud<PointXYZ> >(0);
	pc_buffer = vector< PointCloud<PointXYZ> >(0);



	pc_transform = vector< Eigen::Matrix4f >(0);

	coefficients.reset(new pcl::ModelCoefficients);
	inliers.reset(new pcl::PointIndices);

	shape = visualization_msgs::Marker::LINE_STRIP;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker_pub = nh.advertise<visualization_msgs::Marker>("icp_marker", 1);
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	// initial_guess.setIdentity(4,4);
	// bag 01				// 225 degree 
	initial_guess  << -0.7071, 0.7071, 0, -1.84,
					  -0.7071, -0.7071, 0, 225.2,
					  0, 0, 1, -1.02,
					  0, 0, 0, 1;
	// bag 02
	// initial_guess  << 0.866, 0.5, 0, 323.69,
	// 				  -0.5, 0.866, 0, -34.1962,
	// 				  0, 0, 1, -12.0175,
	// 				  0, 0, 0, 1;

	icp.setMaxCorrespondenceDistance(100);
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.setMaximumIterations(100); 

	sor.setMeanK (50);
	sor.setStddevMulThresh (0.4);
	
	downsample_map.setLeafSize (0.6, 0.6, 0.6); 	// for making map
	// Optional // plane filter 
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);  // pcl::SACMODEL_PLANE   SACMODEL_PERPENDICULAR_PLANE
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (50);								
	seg.setDistanceThreshold (0.5);
	Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); 	// xy-plane 
	seg.setAxis(axis);
	seg.setEpsAngle(  50.0f * (PI/180.0f) );   				// Range degree of plane 
	extract.setNegative (true);

	// ROS

	pc_map_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_pc_all", 10);
	pc_filter_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_loc", 10);
	pc_map_loc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pc_loc", 10);
	pose_pub = nh.advertise<geometry_msgs::Point> ("/pose_icp", 10);

	if (pcl::io::loadPCDFile<PointXYZ> (path + "/map.pcd" , *map) == -1){
		cout << path + "Couldn't read " << path + "/map.pcd" << endl;
		PCL_ERROR ("Couldn't read file pcd \n");
	}	
	cout << map->points.size() << endl;
	// load_map();			// if you already have map, doesn't need to execute this method

	toROSMsg(*map, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "map";
	pc_map_pub.publish(ros_cloud_msg);

	downsample.setLeafSize (0.5, 0.5, 0.5);
	downsample_map.setLeafSize (1, 1, 1);
	pc_sub = nh.subscribe("/points_raw", 1, &icp_local::process_node, this);

	int rate_b = 1;
	boost::thread thread_b(&icp_local::thread_icp, this);
}
PointCloud<PointXYZ> icp_local::from_pcd(string path){
	PointCloud<PointXYZ> p;
	if (pcl::io::loadPCDFile<PointXYZ> (path, p) == -1){
		cout << path + "Couldn't read " << path << endl;
		PCL_ERROR ("Couldn't read file pcd \n");
	}	
	return p;
}
void icp_local::thread_icp(){
	while(1){
		cout << "thread" << count << endl;
	}
}

void icp_local::load_map(){
	int i;
	stringstream ss("");
	for (i = 0; i < 15; i++){
		ss.str("");
		ss << i;
		*map += from_pcd(path + "/map/first-" + ss.str() + ".pcd");	
	}
	for (i = 0; i < 21 ; i++){
		ss.str("");
		ss << i;
		*map += from_pcd(path + "/map/submap_" + ss.str() + ".pcd");	
	}
	for (i = 0; i < 3 ; i++){
		ss.str("");
		ss << i;
		*map += from_pcd(path + "/map/second-" + ss.str() + ".pcd");	
	}

	cout << "Point size before filter : "<< map->points.size() << endl;
	downsample_map.setInputCloud (map);
	downsample_map.filter (*map);
	cout << "Point size after downsampling : "<< map->points.size() << endl;
	sor.setInputCloud (map);
	sor.filter (*map);
	cout << "Point size after outlier removal : "<< map->points.size() << endl;
	
	pcl::io::savePCDFileASCII (path + "/map.pcd", *map);
}
void icp_local::process_node(const sensor_msgs::PointCloud2 msg){
	fromROSMsg (msg, *pc_input);
	geometry_msgs::Point::ConstPtr  gps_msg = ros::topic::waitForMessage<geometry_msgs::Point>("/pose_gps",ros::Duration());
	pc_target->clear();
	*pc_target = *map;
	ros::Time begin = ros::Time::now();
	pass.setInputCloud (pc_target);

	pass.setFilterFieldName ("y");
	pass.setFilterLimits (gps_msg->y - 60,gps_msg->y + 60);
	pass.filter (*pc_target);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (gps_msg->x - 60,gps_msg->x + 60);
	pass.filter (*pc_target);
	float z_sum = 0;
	for (int i = 0; i < pc_target->points.size(); i++){
		z_sum += pc_target->points[i].z;
	}
	z_sum = z_sum / pc_target->points.size();
	cout << "Z average = " << z_sum << endl;
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (z_sum + 0, z_sum + 6);
	pass.filter (*pc_target);

	downsample_map.setInputCloud (pc_target);
	downsample_map.filter (*pc_target);
	
	for (int i = 0; i < pc_buffer.size(); i++){
		*pc_target += pc_buffer[i];
	}

	toROSMsg(*pc_target, origin_map);
	origin_map.header.frame_id = "map";
	pc_filter_pub.publish(origin_map);

	cout << "Pass filter Process time : " <<  ros::Time::now() - begin << endl;

	// sor.setInputCloud (pc_input);
	// sor.filter (*pc_filter);

	seg.setInputCloud (pc_input);
	seg.segment (*inliers, *coefficients);
	extract.setInputCloud (pc_input);
	extract.setIndices (inliers);
	extract.filter (*pc_input);
	z_sum = 0;
	for (int i = 0; i < pc_input->points.size(); i++){
		z_sum += pc_input->points[i].z;
	}
	z_sum = z_sum / pc_input->points.size();

	pass.setInputCloud (pc_input);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (z_sum + 1, z_sum + 5);		// -1 ,2 
	pass.filter (*pc_input);

	downsample.setInputCloud (pc_input);
	downsample.filter (*pc_input);

	////
	// icp
	begin = ros::Time::now();
	pcl::transformPointCloud (*pc_input, *pc_filter, initial_guess);
	toROSMsg(*pc_filter, origin_map);
	origin_map.header.frame_id = "map";
	pc_map_loc_pub.publish(origin_map);

	icp.setInputSource(pc_filter);
	icp.setInputTarget(pc_target);
	cout << "Number PC of Source: "<< pc_filter->size() << endl;
	cout << "Number PC of Target: "<< pc_target->size() << endl;
	icp.align(*result);
	initial_guess = icp.getFinalTransformation() * initial_guess;
	pcl::transformPointCloud (*pc_input, *pc_input, initial_guess);



	cout << "ICP Process time : " <<  ros::Time::now() - begin << endl;

	////
	pc_buffer.push_back(*pc_input);
	pc_transform.push_back(initial_guess);
	count ++;
	if (count > len){
		pc_buffer.erase(pc_buffer.begin());
		pc_transform.erase(pc_transform.begin());
	}


	// publish trajectory of car 
	geometry_msgs::Point p;
	p.x = initial_guess(0,3);
	p.y = initial_guess(1,3);
	p.z = initial_guess(2,3) - 0.4;
	pose_pub.publish(p);
	marker.points.push_back(p);
	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);


	toROSMsg(*map, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "map";
	pc_map_pub.publish(ros_cloud_msg);
}

int main(int argc, char** argv){
	init(argc, argv, "icp_local");
	icp_local icp_local;
	spin();
	return 0;
}