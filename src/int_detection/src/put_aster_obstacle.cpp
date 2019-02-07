#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <autoware_msgs/CloudCluster.h>
#include <autoware_msgs/CloudClusterArray.h>

#include <interactive_markers/interactive_marker_server.h>

#include <cmath>

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::vector<tf::Vector3> positions;


class put_aster_obstacle{

	private:
		ros::Publisher pub_pointcloud;
		geometry_msgs::Pose obstacle_pose;
		pcl::PointCloud<pcl::PointXYZRGB> out_cloud;
		ros::Timer timer;
		tf::TransformListener tf_listener;


	public :
		put_aster_obstacle();
		void sync_jsk_box();
		void get_obstacle_pose();

	private :
		void make_cube();
		visualization_msgs::InteractiveMarkerControl& make_box_control( visualization_msgs::InteractiveMarker &msg);
		void shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
		void pub_pointcloud_Nhz(const ros::TimerEvent&);
};


put_aster_obstacle::put_aster_obstacle(){

	ros::NodeHandle n;

	pub_pointcloud = n.advertise<sensor_msgs::PointCloud2>("/int_pointcloud", 1);

	get_obstacle_pose();

	make_cube();
	ros::Duration(0.5).sleep();
	timer = n.createTimer(ros::Duration(0.2), &put_aster_obstacle::pub_pointcloud_Nhz, this);


}


void put_aster_obstacle::get_obstacle_pose(){

	//obstacle_pose.position.x = -10.5868;//nu_garden_route1
	obstacle_pose.position.x = 6.53186;//takeda_lab
	//obstacle_pose.position.x = -41.8812;//nu_garden_route2
	// obstacle_pose.position.y = 38.6797;//nu_garden_route1
	obstacle_pose.position.y = 0.86863;//takeda_lab
	//obstacle_pose.position.y = 28.1681;//nu_garden_route2
	obstacle_pose.position.z = 0.0;
	obstacle_pose.orientation.x = 0.0;
	obstacle_pose.orientation.y = 0.0;
	// obstacle_pose.orientation.z = sin(M_PI/4);//nu_garden_route1
	obstacle_pose.orientation.z = 0;//takeda_lab
	//obstacle_pose.orientation.z = sin(M_PI/7);//nu_garden_route2
	// obstacle_pose.orientation.w = cos(M_PI/4);//nu_garden_route1
	obstacle_pose.orientation.w = 0;//takeda_lab
	//obstacle_pose.orientation.w = cos(M_PI/7);//nu_garden_route2


}

void put_aster_obstacle::make_cube(){

	visualization_msgs::InteractiveMarker int_marker;

	int_marker.header.frame_id = "world";
	int_marker.name = "No.1";
	int_marker.scale = 1.0;
	int_marker.pose = obstacle_pose;

	make_box_control(int_marker);
	sync_jsk_box();

	server->insert(int_marker);

	server->setCallback(int_marker.name, boost::bind(&put_aster_obstacle::shift_feedback, this, _1));
	server->applyChanges();


}


visualization_msgs::InteractiveMarkerControl& put_aster_obstacle::make_box_control( visualization_msgs::InteractiveMarker &msg){

	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible  = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	//control.orientation = obstacle_pose.orientation;
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


void put_aster_obstacle::sync_jsk_box(){

	double roll, pitch, yaw;
	tf::Quaternion tf_quat(obstacle_pose.orientation.x, obstacle_pose.orientation.y, obstacle_pose.orientation.z, obstacle_pose.orientation.w);
	tf::Matrix3x3(tf_quat).getRPY( roll, pitch, yaw);

	out_cloud.width = 20;
	out_cloud.height = 5;
	out_cloud.points.resize(100);

	double x, y;
	unsigned int count = 0;

	for (unsigned int row = 0; row < 5; row++){
		for (unsigned int col = 0; col < 20; col++){

			pcl::PointXYZRGB &point = out_cloud.points[count];
			x = -0.5 + row * 0.2;
			y = -3.0 + col * 0.3;
			point.x = x*cos(2*M_PI+yaw) - y*sin(2*M_PI+yaw) + obstacle_pose.position.x;
			point.y = x*sin(2*M_PI+yaw) + y*cos(2*M_PI+yaw) + obstacle_pose.position.y ;
			point.z = obstacle_pose.position.z;
			point.r = point.g = point.b = 0.3;

			count++;
		}
	}

}


void put_aster_obstacle::pub_pointcloud_Nhz(const ros::TimerEvent&){

	sensor_msgs::PointCloud2 in_scan, out_scan;

	geometry_msgs::Pose velodyne_frame_pose;
	// geometry_msgs::Pose &world_frame_pose = out_cloud;

	pcl::toROSMsg(out_cloud, in_scan);
	in_scan.header.stamp = ros::Time::now();

	in_scan.header.frame_id = "world";
	out_scan.header.frame_id = "velodyne";

	if(!out_cloud.points.empty()){

		try{
			tf_listener.waitForTransform("world", "velodyne", in_scan.header.stamp, ros::Duration(0.2));

		}catch(...){
			ROS_INFO("world to velodyne transform ERROR");
		}

		pcl_ros::transformPointCloud("velodyne", in_scan, out_scan, tf_listener);

		out_scan.header.stamp = in_scan.header.stamp;
		pub_pointcloud.publish(out_scan);
		std::cout << "published" <<std::endl;
	}


}

void put_aster_obstacle::shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){

	obstacle_pose = feedback->pose;
	sync_jsk_box();
}


int main(int argc, char **argv){

	ros::init(argc, argv, "put_aster_obstacle_node");
	server.reset(new interactive_markers::InteractiveMarkerServer("put_aster_obstacle_node"));
	ros::Duration(0.1).sleep();
	ROS_INFO("Initializing...");

	put_aster_obstacle put_aster_obstacle;
	ROS_INFO("Ready...");
	//server->applyChanges();

	ros::spin();
	server.reset();

	return 0;

}
