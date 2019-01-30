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


class int_detection_camera{

	private:
		ros::Subscriber sub_detection;
		ros::Subscriber sub_camera_info;
		ros::Publisher pub_pointcloud;
		//ros::Publisher pub_jsk_box;
		ros::Publisher pub_cloud;
		float shift;
		jsk_recognition_msgs::BoundingBox in_jsk_msgs;
		jsk_recognition_msgs::BoundingBox out_jsk_msgs;

		//std_msgs::Header jsk_array_header;
		tf::TransformListener tf_listener;

	public :
		int_detection_camera();
		void sync_jsk_box();

	private :
		void detection_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &msgs);
		void camera_info_callback(const  &msgs);
		void make_cube();
		visualization_msgs::InteractiveMarkerControl& make_box_control( visualization_msgs::InteractiveMarker &msg);
		void shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
		void calc_boxpose();
};


int_detection_camera::int_detection_camera()): shift(0){

	ros::NodeHandle n;

	sub_detection = n.subscribe("/detection/tracked_objects", 5, &int_detection_camera::detection_callback, this);
	sub_detection = n.subscribe("/camera_info", 5, &int_detection_camera::camera_info_callback, this);
	pub_pointcloud = n.advertise<sensor_msgs::PointCloud2>("/int_pointcloud", 50);
	pub_cloud = n.advertise<autoware_msgs::CloudClusterArray>("/int_cluster", 50);
	//pub_jsk_box = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/int_boundingbox", 5);



	out_jsk_msgs.dimensions.x = 1.0;
	out_jsk_msgs.dimensions.y = 6.0;
	out_jsk_msgs.dimensions.z = 2.0;
	out_jsk_msgs.value = 1;
}


void int_detection_camera::sync_jsk_box(){

	autoware_msgs::CloudClusterArray cluster_array;
	autoware_msgs::CloudCluster cluster;

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mono(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;

	//jsk_recognition_msgs::BoundingBoxArray out_jsk_msgs_array;
	//out_jsk_msgs_array.header = in_jsk_msgs.header;
	//out_jsk_msgs_array.boxes.push_back(out_jsk_msgs);

	std_msgs::Header out_header = in_jsk_msgs.header;
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
	//pub_jsk_box.publish(out_jsk_msgs_array);

}


void int_detection_camera::detection_callback(const autoware_msgs::DetectedObjectArray::ConstPtr &msgs){


	for (size_t i = 0; i < msgs->boxes.size(); i++){
		make_cube();
		in_jsk_msgs = msgs->boxes[i];
		out_jsk_msgs.header = msgs->boxes[i].header;
		out_jsk_msgs.label = 1;
	}
}

void int_detection_camera::camera_info_callback(const ){

}
void int_detection_camera::shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){

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


visualization_msgs::InteractiveMarkerControl& int_detection_camera::make_box_control( visualization_msgs::InteractiveMarker &msg){

	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible  = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
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


void int_detection_camera::calc_boxpose(){

	geometry_msgs::Pose box_pose;
	tf::Pose world_to_velodyne;
	tf::Pose req_to_velodyne;
	tf::StampedTransform req_to_world;
	float theta = 0;
	if (in_jsk_msgs.pose.position.x != 0){
		theta = std::atan(out_jsk_msgs.pose.position.y / out_jsk_msgs.pose.position.x);;
	}

	box_pose.position.x = in_jsk_msgs.pose.position.x + shift * std::sin(theta);
	box_pose.position.y = in_jsk_msgs.pose.position.y + shift * std::cos(theta);
	box_pose.position.z = in_jsk_msgs.pose.position.z;
	box_pose.orientation.x = 0;
	box_pose.orientation.y = 0;
	box_pose.orientation.z = 0;
	box_pose.orientation.w = 1;

	try{
		tf_listener.waitForTransform("velodyne", "world", ros::Time(0), ros::Duration(1.0));
		tf_listener.lookupTransform("world", "velodyne", ros::Time(0), req_to_world);

	}catch(...){
		ROS_INFO("velodyne to world transform ERROR");
	}
	tf::poseMsgToTF(box_pose, world_to_velodyne);
	req_to_velodyne = req_to_world * world_to_velodyne;
	tf::poseTFToMsg(req_to_velodyne, out_jsk_msgs.pose);

	//ROS_INFO("x:%03f -> %03f, y:%03f -> %03f", velodyne_pose.position.x, world_pose.position.x, velodyne_pose.position.y, world_pose.position.y);

}


void int_detection_camera::make_cube(){

	calc_boxpose();

	visualization_msgs::InteractiveMarker int_marker;

	int_marker.header.frame_id = "world";
	int_marker.name = "No.1";
	int_marker.scale = 1.0;
	int_marker.pose = out_jsk_msgs.pose;

	make_box_control(int_marker);
	sync_jsk_box();

	server->insert(int_marker);

	server->setCallback(int_marker.name, boost::bind(&int_detection_camera::shift_feedback, this, _1));
	server->applyChanges();


}


int main(int argc, char **argv){

	ros::init(argc, argv, "int_detection_camera_node");
	server.reset(new interactive_markers::InteractiveMarkerServer("int_detection_camera_node"));
	ros::Duration(0.1).sleep();
	ROS_INFO("Initializing...");

	int_detection_camera int_detect;
	ROS_INFO("Ready...");
	//server->applyChanges();

	ros::spin();
	server.reset();

	return 0;

}
