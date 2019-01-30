#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/pca.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <autoware_msgs/CloudCluster.h>
#include <autoware_msgs/CloudClusterArray.h>

#include <interactive_markers/interactive_marker_server.h>

#include <cmath>

#include <fstream>
#include <string>
#include <sstream>
#include <vector>


boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::vector<tf::Vector3> positions;

template <typename T>
std::string To_String(T val){
	std::stringstream stream;
	stream << val;
	return stream.str();
}

class put_multi_obstacle{

	private:
		ros::Publisher pub_pointcloud;
		ros::Publisher pub_cloud;
		geometry_msgs::Pose obstacle_pose;
		std::vector<jsk_recognition_msgs::BoundingBox> out_jsk_msgs;

	public :
		put_multi_obstacle();
		void sync_jsk_box(uint32_t obstacle_label);
		void read_obstacle_pose(char& file_name);

	private :
		void make_cube(uint32_t obstacle_label);
		visualization_msgs::InteractiveMarkerControl& make_box_control( visualization_msgs::InteractiveMarker &msg);
		void shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
};


put_multi_obstacle::put_multi_obstacle(){

	ros::NodeHandle n;

	pub_pointcloud = n.advertise<sensor_msgs::PointCloud2>("/int_pointcloud", 50);
	pub_cloud = n.advertise<autoware_msgs::CloudClusterArray>("/int_cluster", 50);

}


void put_multi_obstacle::read_obstacle_pose(char& file_name){

	std::ifstream ifs(&file_name);

	std::string line, field;
	std::vector<std::string> strvec;

	uint32_t obstacle_label = 0;

	while(std::getline(ifs, line)){

		std::istringstream stream(line);
		for(unsigned int i = 0; i < 7; i++){
			std::getline(stream, field, ',');
			switch(i){
				case 0:
					obstacle_pose.position.x = std::stod(field);
					break;
				case 1:
					obstacle_pose.position.y = std::stod(field);
					break;
				case 2:
					obstacle_pose.position.z = std::stod(field);
					break;
				case 3:
					obstacle_pose.orientation.x = std::stod(field);
					break;
				case 4:
					obstacle_pose.orientation.y = std::stod(field);
					break;
				case 5:
					obstacle_pose.orientation.z = std::stod(field);
					break;
				case 6:
					obstacle_pose.orientation.w = std::stod(field);
			}
		}


		out_jsk_msgs.push_back(obstacle_pose);
		out_jsk_msgs.back().dimensions.x = 1.0;
		out_jsk_msgs.back().dimensions.y = 6.0;
		out_jsk_msgs.back().dimensions.z = 2.0;
		out_jsk_msgs.back().value = 1.0;
		out_jsk_msgs.back().label = obstacle_label;
		out_jsk_msgs.back().header.frame_id = "world";

		make_cube(obstacle_label);
		obstacle_label++;
	}

}


void put_multi_obstacle::make_cube(uint32_t obstacle_label){

	visualization_msgs::InteractiveMarker int_marker;

	int_marker.header.frame_id = "world";
	int_marker.name = To_String(obstacle_label);
	int_marker.scale = 1.0;
	int_marker.pose = out_jsk_msgs.at(obstacle_label).pose;

	make_box_control(int_marker);
	sync_jsk_box(obstacle_label);
	//ROS_INFO_STREAM(out_jsk_msgs.pose);
	server->insert(int_marker);

	server->setCallback(int_marker.name, boost::bind(&put_multi_obstacle::shift_feedback, this, _1));
	server->applyChanges();


}


visualization_msgs::InteractiveMarkerControl& put_multi_obstacle::make_box_control( visualization_msgs::InteractiveMarker &msg){

	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible  = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	//control.orientation = out_jsk_msgs.pose.orientation;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.orientation.w = 1;

	//ROS_INFO_STREAM(control.orientation);
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.scale.x = msg.scale;
	marker.scale.y = msg.scale*6;
	marker.scale.z = msg.scale*3;
	marker.color.r = 0;
	marker.color.g = 1;
	marker.color.b = 0;
	marker.color.a = 0.5;

	control.markers.push_back(marker);
	msg.controls.push_back(control);

	return msg.controls.back();
}


void put_multi_obstacle::sync_jsk_box(uint32_t obstacle_label){

	autoware_msgs::CloudClusterArray cluster_array;
	autoware_msgs::CloudCluster cluster;

	jsk_recognition_msgs::BoundingBox& sync_jsk_msgs = out_jsk_msgs.at(obstacle_label);

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mono(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;

	double roll, pitch, yaw;
	tf::Quaternion tf_quat(sync_jsk_msgs.pose.orientation.x, sync_jsk_msgs.pose.orientation.y, sync_jsk_msgs.pose.orientation.z, sync_jsk_msgs.pose.orientation.w);
	tf::Matrix3x3(tf_quat).getRPY( roll, pitch, yaw);

	//ROS_INFO("%lf, %lf, %lf",roll/M_PI*180, pitch/M_PI*180, yaw/M_PI*180);

	sync_jsk_msgs.header.stamp = ros::Time::now();

	cluster_array.header = sync_jsk_msgs.header;
	cluster.header = sync_jsk_msgs.header;
	pcl_conversions::toPCL(sync_jsk_msgs.header, cloud.header);

	cluster.id = 0;
	cluster.score = 0.0;
	cloud.width = 20;
	cloud.height = 5;
	cloud.points.resize(100);

	double x, y;
	unsigned int count = 0;

	for (unsigned int row = 0; row < 5; row++){
		for (unsigned int col = 0; col < 20; col++){

			pcl::PointXYZRGB &point = cloud.points[count];
			x = -0.5 + row * 0.2;
			y = -3.0 + col * 0.3;
			point.x = x*cos(2*M_PI+yaw) - y*sin(2*M_PI+yaw) + sync_jsk_msgs.pose.position.x;
			point.y = x*sin(2*M_PI+yaw) + y*cos(2*M_PI+yaw) + sync_jsk_msgs.pose.position.y ;
			point.z = sync_jsk_msgs.pose.position.z;
			point.r = point.g = point.b = 0.3;

			count++;
		}
	}

	pcl::toROSMsg(cloud, cluster.cloud);

	pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZ>(cloud, *cloud_mono);

	cluster.min_point.header = cluster.max_point.header = cluster.avg_point.header = sync_jsk_msgs.header;
	cluster.min_point.point.x = cloud_mono->points[0].x;
	cluster.min_point.point.y = cloud_mono->points[0].y;
	cluster.min_point.point.z = cloud_mono->points[0].z;
	cluster.max_point.point.x = cloud_mono->points[99].x;
	cluster.max_point.point.y = cloud_mono->points[99].y;
	cluster.max_point.point.z = cloud_mono->points[99].z;
	cluster.avg_point.point.x = cloud_mono->points[50].x;
	cluster.avg_point.point.y = cloud_mono->points[50].y;
	cluster.avg_point.point.z = cloud_mono->points[50].z;
	cluster.centroid_point = cluster.avg_point;
	cluster.estimated_angle = 0.0;


	cluster.dimensions.x = 6.0;
	cluster.dimensions.y = 1.0;
	cluster.dimensions.z = 0.2;


	pca.setInputCloud(cloud_mono);

	Eigen::Vector3f eigen_values = pca.getEigenValues();
	cluster.eigen_values.x = eigen_values.x();
	cluster.eigen_values.y = eigen_values.y();
	cluster.eigen_values.z = eigen_values.z();


	Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
	for (unsigned int i = 0; i < 3; i++){

		geometry_msgs::Vector3 eigen_vector;
		eigen_vector.x = eigen_vectors(i, 0);
		eigen_vector.y = eigen_vectors(i, 1);
		eigen_vector.z = eigen_vectors(i, 2);
		cluster.eigen_vectors.push_back(eigen_vector);
	}


	cluster.bounding_box = sync_jsk_msgs;

	//cluster.convex_hull
	cluster_array.clusters.push_back(cluster);
	pub_cloud.publish(cluster_array);
	pub_pointcloud.publish(cluster.cloud);

}


void put_multi_obstacle::shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){

	uint32_t obstacle_label = stoi(feedback->marker_name);
	out_jsk_msgs.at(obstacle_label).pose = feedback->pose;
	//ROS_INFO_STREAM(out_jsk_msgs);
	//ROS_INFO_STREAM(feedback->pose);
	sync_jsk_box(obstacle_label);
}



int main(int argc, char **argv){

	ros::init(argc, argv, "put_multi_obstacle_node");
	server.reset(new interactive_markers::InteractiveMarkerServer("put_multi_obstacle_node"));
	ros::Duration(0.1).sleep();
	ROS_INFO("Initializing...");

	put_multi_obstacle put_multi_obstacle;
	if (argc == 2){
		put_multi_obstacle.read_obstacle_pose(*argv[1]);
	}else{
		std::cout << "file name ERROR" << std::endl;
	}
	ROS_INFO("Ready...");
	//server->applyChanges();

	ros::spin();
	server.reset();

	return 0;

}
