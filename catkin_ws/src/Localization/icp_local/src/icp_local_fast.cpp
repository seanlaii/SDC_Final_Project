#include "icp_local_fast.h"


void *thread(void *);
icp_local::icp_local(){
	NodeHandle nh;
	map.reset(new PointCloud<PointXYZ>());
	map_process.reset(new PointCloud<PointXYZ>());
	pc_input.reset(new PointCloud<PointXYZ>());
	pc_filter.reset(new PointCloud<PointXYZ>());
	pc_pass.reset(new PointCloud<PointXYZ>());
	pc_target.reset(new PointCloud<PointXYZ>());
	result.reset(new PointCloud<PointXYZ>());

	pc_input_thread.reset(new PointCloud<PointXYZ>());
	pc_target_thread.reset(new PointCloud<PointXYZ>());

	path = package::getPath("icp_local");

	len = 1;
	thres_count = 0;
	count = 0;
	initial_acc = 5;
	score_threshold = 15;
	updater = 2000;
	score_thres = 0.8;
	param_check = true;
	icp_set = true;
	pc_thread_src = vector< PointCloud<PointXYZ> >(0);
	pc_thread_tar = vector< PointCloud<PointXYZ> >(0);
	pc_buffer = vector< PointCloud<PointXYZ> >(0);
	header_list = vector< std_msgs::Header >(0);


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
	// initial_guess  << -0.7071, 0.7071, 0, -1.84,
	// 				  -0.7071, -0.7071, 0, 225.2,
	// 				  0, 0, 1, -1.02,
	// 				  0, 0, 0, 1;
	// bag 02
	// initial_guess  << 0.866, 0.5, 0, 323.69,
	// 				  -0.5, 0.866, 0, -34.1962,
	// 				  0, 0, 1, -12.0175,
	// 				  0, 0, 0, 1;

	icp.setMaxCorrespondenceDistance(100);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1e-8);
	icp.setMaximumIterations(500); 

	sor.setMeanK (50);
	sor.setStddevMulThresh (0.4);
	
	downsample_map.setLeafSize (0.6, 0.6, 0.6); 	// for making map
	// Optional // plane filter 
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);  // pcl::SACMODEL_PLANE   SACMODEL_PERPENDICULAR_PLANE
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (50);								
	seg.setDistanceThreshold (0.2); 	// 0.5
	Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); 	// xy-plane 
	seg.setAxis(axis);
	seg.setEpsAngle(  50.0f * (PI/180.0f) );   				// Range degree of plane 
	extract.setNegative (true);

	// ROS

	pc_map_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_pc_all", 10);
	pc_filter_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_loc", 10);
	pc_map_loc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pc_loc", 10);
	pose_pub = nh.advertise<nav_msgs::Odometry> ("/pose_icp", 10);

	if (pcl::io::loadPCDFile<PointXYZ> (path + "/map_0.6.pcd" , *map) == -1){
		cout << path + "Couldn't read " << path + "/map_0.6.pcd" << endl;
		PCL_ERROR ("Couldn't read file pcd \n");
	}	
	cout << "Map point size : " << map->points.size() << endl;
	cout << "Now you can rosbag play XXX.bag\n";
	// load_map();			// if you already have map, doesn't need to execute this method

	toROSMsg(*map, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "map";
	pc_map_pub.publish(ros_cloud_msg);

	downsample.setLeafSize (0.8, 0.8, 0.8);
	downsample_map.setLeafSize (1.5, 1.5, 1.5);

	downsample_pass.setLeafSize (1.6, 1.6, 1.6);
	// for (int i = 0; i < 10; i++){
	// 	geometry_msgs::Point::ConstPtr  gps_msg1 = ros::topic::waitForMessage<geometry_msgs::Point>("/pose_gps",ros::Duration());
	// }
	get_initial_guess();
	pc_sub = nh.subscribe("/points_raw", 1, &icp_local::process_node, this);

	// boost::thread thread_b(&icp_local::thread_icp, this);
}
PointCloud<PointXYZ> icp_local::from_pcd(string path){
	PointCloud<PointXYZ> p;
	if (pcl::io::loadPCDFile<PointXYZ> (path, p) == -1){
		cout << path + "Couldn't read " << path << endl;
		PCL_ERROR ("Couldn't read file pcd \n");
	}	
	return p;
}
void icp_local::get_initial_guess(){
	geometry_msgs::Point::ConstPtr  gps_msg1 = ros::topic::waitForMessage<geometry_msgs::Point>("/pose_gps",ros::Duration());
	gps_msg1 = ros::topic::waitForMessage<geometry_msgs::Point>("/pose_gps",ros::Duration());
	geometry_msgs::Point::ConstPtr  gps_msg2 = ros::topic::waitForMessage<geometry_msgs::Point>("/pose_gps",ros::Duration());
	gps_msg2 = ros::topic::waitForMessage<geometry_msgs::Point>("/pose_gps",ros::Duration());
	float x_del = gps_msg2->x - gps_msg1->x;
	float y_del = gps_msg2->y - gps_msg1->y;
	float delta = atan2(y_del, x_del) / PI * 180. + 360;
	delta = delta - 119.358;
	cout << gps_msg1->x << " "  << gps_msg1->y << endl;
	cout << "delta : " << delta << endl;
	delta  = delta / 180. * PI;

	initial_guess << cos(delta), -sin(delta), 0, gps_msg2->x,
					sin(delta), cos(delta), 0, gps_msg2->y,
					0, 0, 1, 0,
					0, 0, 0, 1;

}
void *thread(void *conn){

	icp_local *ptr = (icp_local*)conn;
	while(1){
	// icp
		while(ptr->pc_thread_tar.size() != 0){


			cout << "============== size : " << ptr->pc_thread_tar.size() << endl;
			ros::Time begin = ros::Time::now();
			*(ptr->pc_input_thread) = ptr->pc_thread_src[0];
			*(ptr->pc_target_thread) = ptr->pc_thread_tar[0];


			if(ptr->thres_count > ptr->initial_acc && ptr->thres_count % ptr->updater != ptr->initial_acc
				 && (ptr->thres_count % ptr->updater - ptr->initial_acc ) >= ptr->len){
				for (int i = 0; i < ptr->pc_buffer.size(); i++){
					*(ptr->pc_target_thread) += ptr->pc_buffer[i];
				}
			}

			if (ptr->thres_count != 0){
				toROSMsg(*(ptr->pc_target_thread), ptr->origin_map);
				ptr->origin_map.header.frame_id = "map";
				ptr->pc_filter_pub.publish(ptr->origin_map);

				pcl::transformPointCloud (*(ptr->pc_input_thread), *(ptr->pc_filter), ptr->initial_guess);

				toROSMsg(*(ptr->pc_filter), ptr->origin_map);
				ptr->origin_map.header.frame_id = "map";
				ptr->pc_map_loc_pub.publish(ptr->origin_map);

				ptr->icp.setInputSource(ptr->pc_filter);
				ptr->icp.setInputTarget(ptr->pc_target_thread);
				cout << "Number PC of Source: "<< ptr->pc_filter->size() << endl;
				cout << "Number PC of Target: "<< ptr->pc_target_thread->size() << endl;
				ptr->icp.align(*(ptr->result));
				ptr->initial_guess = ptr->icp.getFinalTransformation() * ptr->initial_guess;
				cout << " score: " << ptr->icp.getFitnessScore() << endl;
				if ( ptr->icp.getFitnessScore() < ptr->score_threshold && ptr->icp_set){
					ptr->icp_set = false;
					cout << "=========================== SWITCH ICP SET STATIC =========================\n";
					ptr->icp.setMaxCorrespondenceDistance(100);
					ptr->icp.setTransformationEpsilon(1e-5);
					ptr->icp.setEuclideanFitnessEpsilon(1e-4);
					ptr->icp.setMaximumIterations(250); 	
				}
				else if(ptr->icp.getFitnessScore() > 3 && !ptr->icp_set){
					ptr->icp_set = true;
					cout << "=========================== SWITCH ICP SET BAD ============================\n";
					ptr->icp.setMaxCorrespondenceDistance(100);
					ptr->icp.setTransformationEpsilon(1e-8);
					ptr->icp.setEuclideanFitnessEpsilon(1e-8);
					ptr->icp.setMaximumIterations(500); 
				}
			}
			pcl::transformPointCloud (*(ptr->pc_input_thread), *(ptr->pc_input_thread), ptr->initial_guess);

			cout << "Thread ICP Process time : " <<  ros::Time::now() - begin << endl;

			////
			ptr->pc_buffer.push_back(*(ptr->pc_input_thread));
			ptr->pc_transform.push_back(ptr->initial_guess);
			ptr->thres_count ++;
			if (ptr->thres_count > ptr->len){
				ptr->pc_buffer.erase(ptr->pc_buffer.begin());
				ptr->pc_transform.erase(ptr->pc_transform.begin());
			}


			// publish trajectory of car 
			nav_msgs::Odometry p_odom;
			
			geometry_msgs::Point p;
			p.x = ptr->initial_guess(0,3);
			p.y = ptr->initial_guess(1,3);
			p.z = ptr->initial_guess(2,3);
			if (ptr->thres_count > ptr->score_threshold){
				p_odom.header = ptr->header_list[0];
				p_odom.pose.pose.position.x = ptr->initial_guess(0,3);
				p_odom.pose.pose.position.y = ptr->initial_guess(1,3);
				p_odom.pose.pose.position.z = ptr->initial_guess(2,3);
				ptr->pose_pub.publish(p_odom);
			}
			ptr->marker.points.push_back(p);
			ptr->marker.lifetime = ros::Duration();
			ptr->marker_pub.publish(ptr->marker);

			ptr->header_list.erase(ptr->header_list.begin());
			ptr->pc_thread_src.erase(ptr->pc_thread_src.begin());
			ptr->pc_thread_tar.erase(ptr->pc_thread_tar.begin());
		}
	}
}
void icp_local::save_ptr(icp_local *p){
    cls_ptr = p;
    // cout <<  " " << cls_ptr << endl;

    if( pthread_create( &thread_id, NULL ,  thread , (void*) cls_ptr) < 0){
        perror("could not create thread");
        return;
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
	
	pcl::io::savePCDFileASCII (path + "/map_0.6.pcd", *map);
}
void icp_local::process_node(const sensor_msgs::PointCloud2 msg){
	std_msgs::Header header = std_msgs::Header();
	header.stamp = msg.header.stamp;
	header_list.push_back(header);
	fromROSMsg (msg, *pc_input);
	geometry_msgs::Point::ConstPtr  gps_msg = ros::topic::waitForMessage<geometry_msgs::Point>("/pose_gps",ros::Duration());

	toROSMsg(*map, ros_cloud_msg);
	ros_cloud_msg.header.frame_id = "map";
	pc_map_pub.publish(ros_cloud_msg);

	*pc_target = *map;
	ros::Time begin = ros::Time::now();
	pass.setInputCloud (pc_target);

	pass.setFilterFieldName ("y");
	pass.setFilterLimits (gps_msg->y - 100,gps_msg->y + 100);
	pass.filter (*pc_target);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (gps_msg->x - 100,gps_msg->x + 100);
	pass.filter (*pc_target);

	float z_sum = 0;

	if(count >= initial_acc && param_check){
		param_check = false;
		// icp.setMaximumIterations(125); 
		// seg.setDistanceThreshold (0.5); 
		downsample.setLeafSize (0.5, 0.5, 0.5);
		downsample_map.setLeafSize (1, 1, 1);
	}
	else if(count % updater == initial_acc){
		param_check = true;
		icp.setMaximumIterations(250); 
		// seg.setDistanceThreshold (0.2); 
		downsample.setLeafSize (0.2, 0.2, 0.2);
		downsample_map.setLeafSize (0.5, 0.5, 0.5);	
	}

	for (int i = 0; i < pc_target->points.size(); i++){
		z_sum += pc_target->points[i].z;
	}
	//// initial position z
	z_sum = z_sum / pc_target->points.size();
	cout << "Z average = " << z_sum << endl;
	if (count == 0){
		initial_guess(0,3) = gps_msg->x;
		initial_guess(1,3) = gps_msg->y;
		initial_guess(2,3) = z_sum;
	}
	if (count > initial_acc && count % updater != initial_acc){


		pass.setFilterFieldName ("z");
		pass.setFilterLimits (z_sum - 3.5, z_sum - 1.5);
		pass.filter (*pc_pass);
		downsample_pass.setInputCloud (pc_pass);
		downsample_pass.filter (*pc_pass);

		pass.setFilterFieldName ("z");
		pass.setFilterLimits (z_sum - 1.5, z_sum + 10);
		pass.filter (*pc_target);
		*pc_target += *pc_pass;


	}
	else{
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (z_sum - 3.5, z_sum - 1.5);
		pass.filter (*pc_pass);
		downsample_pass.setInputCloud (pc_pass);
		downsample_pass.filter (*pc_pass);

		pass.setFilterFieldName ("z");
		pass.setFilterLimits (z_sum - 1.5, z_sum + 5);
		pass.filter (*pc_target);
		*pc_target += *pc_pass;		
	}
	downsample_map.setInputCloud (pc_target);
	downsample_map.filter (*pc_target);
	z_sum = 0;

	if(count > initial_acc && count % updater != initial_acc){
		seg.setDistanceThreshold (0.1); 
		seg.setInputCloud (pc_input);
		seg.segment (*inliers, *coefficients);
		extract.setInputCloud (pc_input);
		extract.setIndices (inliers);
		extract.filter (*pc_input);
		for (int i = 0; i < pc_input->points.size(); i++){
			z_sum += pc_input->points[i].z;
		}
		z_sum = z_sum / pc_input->points.size();
		pass.setInputCloud (pc_input);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (z_sum - 2, z_sum + 10);		// -1 ,2 
		pass.filter (*pc_input);

	}
	else{
		seg.setDistanceThreshold (0.1); 
		seg.setInputCloud (pc_input);
		seg.segment (*inliers, *coefficients);
		extract.setInputCloud (pc_input);
		extract.setIndices (inliers);
		extract.filter (*pc_input);
	}
	downsample.setInputCloud (pc_input);
	downsample.filter (*pc_input);

	pc_thread_src.push_back(*pc_input);
	pc_thread_tar.push_back(*pc_target);
	pc_input->clear();
	pc_target->clear();
	cout << "Pass filter Process time : " <<  ros::Time::now() - begin << endl;
	
	count ++; 
	////


}

int main(int argc, char** argv){
	init(argc, argv, "icp_local");
	icp_local icp_local;
	icp_local.save_ptr(&icp_local);
	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();
	return 0;
}
