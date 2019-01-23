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

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::vector<tf::Vector3> positions;


class add_obj{

	private:
		ros::Publisher pub_pointcloud;
		ros::Publisher pub_cloud;
		float shift;
		geometry_msgs obstacle_pose;
		jsk_recognition_msgs out_jsk_msgs;

	public :
		add_obstacle();
		void sync_jsk_box();

	private :
		void make_cube();
		visualization_msgs::InteractiveMarkerControl& make_box_control( visualization_msgs::InteractiveMarker &msg);
		void shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
		void calc_boxpose();
};


int_detection::int_detection(): shift(0){

	ros::NodeHandle n;

	pub_pointcloud = n.advertise<sensor_msgs::PointCloud2>("/add_obstacle_box", 50);
	pub_cloud = n.advertise<autoware_msgs::CloudClusterArray>("/add_obstacle_cluster", 50);

	out_jsk_msgs.dimensions.x = 1.0;
	out_jsk_msgs.dimensions.y = 6.0;
	out_jsk_msgs.dimensions.z = 2.0;
	out_jsk_msgs.value = 1;
}


void int_detection::sync_jsk_box(){

	autoware_msgs::CloudClusterArray cluster_array;
	autoware_msgs::CloudCluster cluster;

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mono(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;


	std_msgs::Header out_header = obstacle_pose.header;
	out_header.frame_id = "world";

	cluster_array.header = out_header;
	cluster.header = out_header;
	pcl_conversions::toPCL(out_header, cloud.header);

	cluster.id = 0;
	cluster.score = 0.0;
	cloud.width = 20;
	cloud.height = 5;
	cloud.points.resize(100);

	unsigned int count = 0;

	for (unsigned int row = 0; row < 5; row++){
		for (unsigned int col = 0; col < 20; col++){

			pcl::PointXYZRGB &point = cloud.points[count];
			point.x = out_jsk_msgs.pose.position.x - 0.5 + row * 0.2;
			point.y = out_jsk_msgs.pose.position.y - 3.0 + col * 0.3;
			point.z = out_jsk_msgs.pose.position.z;
			point.r = point.g = point.b = 0.3;

			count++;
		}
	}
	pcl::toROSMsg(cloud, cluster.cloud);

	pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZ>(cloud, *cloud_mono);

	cluster.min_point.header = cluster.max_point.header = cluster.avg_point.header = out_header;
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


	cluster.bounding_box = out_jsk_msgs;

	//cluster.convex_hull
	cluster_array.clusters.push_back(cluster);
	pub_cloud.publish(cluster_array);
	pub_pointcloud.publish(cluster.cloud);

}


void int_detection::shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){

	float permanet_shift;
	permanet_shift = std::sqrt(std::pow(feedback->pose.position.x - out_jsk_msgs.pose.position.x, 2.0) + std::pow(feedback->pose.position.y - out_jsk_msgs.pose.position.y, 2.0));
	if ((feedback->pose.position.y - out_jsk_msgs.pose.position.y) < 0){
		permanet_shift = -permanet_shift;
	}
	shift += permanet_shift;
	calc_boxpose();
	sync_jsk_box();
	ROS_INFO_STREAM(shift);
}


visualization_msgs::InteractiveMarkerControl& int_detection::make_box_control( visualization_msgs::InteractiveMarker &msg){

	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible  = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1.0;
	control.orientation.w = 1;

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


void int_detection::calc_boxpose(){

	geometry_msgs::Pose box_pose;
	tf::Pose world_to_velodyne;
	tf::Pose req_to_velodyne;
	tf::StampedTransform req_to_world;


	float theta = std::atan(obstacle_pose.position.y / obstacle_pose.position.x);

	box_pose.position.x = obstacle_pose.position.x + shift * std::sin(theta);
	box_pose.position.y = obstacle_pose.position.y + shift * std::cos(theta);
	box_pose.position.z = obstacle_pose.position.z;
	box_pose.orientation.x = 0;
	box_pose.orientation.y = 0;
	box_pose.orientation.z = 0;
	box_pose.orientation.w = 1;

}


void int_detection::make_cube(){

	calc_boxpose();

	visualization_msgs::InteractiveMarker int_marker;

	int_marker.header.frame_id = "map";
	int_marker.name = "No.1";
	int_marker.scale = 1.0;
	int_marker.pose = out_jsk_msgs.pose;

	make_box_control(int_marker);
	sync_jsk_box();

	server->insert(int_marker);

	server->setCallback(int_marker.name, boost::bind(&int_detection::shift_feedback, this, _1));
	server->applyChanges();


}


int main(int argc, char **argv){

	ros::init(argc, argv, "add_obj_node");
	server.reset(new interactive_markers::InteractiveMarkerServer("int_detection_node"));
	ros::Duration(0.1).sleep();
	ROS_INFO("Initializing...");

	int_detection int_detect;
	ROS_INFO("Ready...");
	//server->applyChanges();

	ros::spin();
	server.reset();

	return 0;

}
