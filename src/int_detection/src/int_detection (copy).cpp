#include <ros/ros.h>
//#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <interactive_markers/interactive_marker_server.h>

#include <cmath>

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::vector<tf::Vector3> positions;


class int_detection{

	private:
		ros::Subscriber sub_detection;
		ros::Publisher pub_jsk_box;
		float shift;
		jsk_recognition_msgs::BoundingBox jsk_msgs;
		std_msgs::Header jsk_array_header;
		tf::TransformListener tf_listener;

	public :
		int_detection();
		void sync_jsk_box(const geometry_msgs::Pose &pose);

	private :
		void detection_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &msgs);
		void make_cube();
		visualization_msgs::InteractiveMarkerControl& make_box_control( visualization_msgs::InteractiveMarker &msg);
		void shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
};


int_detection::int_detection(): shift(0){

	ros::NodeHandle n;

	sub_detection = n.subscribe("/detection/combined_objects_boxes", 5, &int_detection::detection_callback, this);
	pub_jsk_box = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detection/interactive_object", 5);

}


void int_detection::sync_jsk_box(const geometry_msgs::Pose &pose){

	jsk_recognition_msgs::BoundingBoxArray out_jsk_box_array;
	jsk_recognition_msgs::BoundingBox out_jsk_box = jsk_msgs;

	out_jsk_box_array.header = jsk_array_header;

	out_jsk_box.header = jsk_array_header;
	out_jsk_box.header.frame_id = "world";
	out_jsk_box.label = 1;

	out_jsk_box.pose = pose;
	out_jsk_box.dimensions.x = 1.0;
	out_jsk_box.dimensions.y = 6.0;
	out_jsk_box.dimensions.z = 1.0;
	out_jsk_box.value = 1;

	out_jsk_box_array.boxes.push_back(out_jsk_box);

	pub_jsk_box.publish(out_jsk_box_array);

}


void int_detection::detection_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &msgs){


	for (size_t i = 0; i < msgs->boxes.size(); i++){
		make_cube();
		jsk_msgs = msgs->boxes[i];
		jsk_array_header = msgs->header;
	}
}


void int_detection::shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){

	shift = std::sqrt(std::pow(feedback->pose.position.x - jsk_msgs.pose.position.x, 2.0) + std::pow(feedback->pose.position.y - jsk_msgs.pose.position.y, 2.0));
	if ((feedback->pose.position.y - jsk_msgs.pose.position.y) < 0){
		shift = -shift;
	}

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
	marker.scale.z = msg.scale;
	marker.color.r = 0;
	marker.color.g = 1;
	marker.color.b = 0;
	marker.color.a = 0.5;

	control.markers.push_back(marker);
	msg.controls.push_back(control);



	return msg.controls.back();
}



void int_detection::make_cube(){

	geometry_msgs::Pose velodyne_pose;
	geometry_msgs::Pose world_pose;
	tf::Pose world_to_velodyne;
	tf::Pose req_to_velodyne;
	tf::StampedTransform req_to_world;

	std::stringstream s;
	s << jsk_msgs.label;

	float theta = std::atan(jsk_msgs.pose.position.y / jsk_msgs.pose.position.x);

	velodyne_pose.position.x = jsk_msgs.pose.position.x + shift * std::sin(theta);
	velodyne_pose.position.y = jsk_msgs.pose.position.y + shift * std::cos(theta);
	velodyne_pose.position.z = jsk_msgs.pose.position.z;
	velodyne_pose.orientation.x = 0;
	velodyne_pose.orientation.y = 0;
	velodyne_pose.orientation.z = 0;
	velodyne_pose.orientation.w = 1;

	try{
		tf_listener.waitForTransform("velodyne", "world", ros::Time(0), ros::Duration(1.0));
		tf_listener.lookupTransform("world", "velodyne", ros::Time(0), req_to_world);

	}catch(...){
		ROS_INFO("velodyne to world transform ERROR");
	}

	tf::poseMsgToTF(velodyne_pose, world_to_velodyne);
	req_to_velodyne = req_to_world * world_to_velodyne;
	tf::poseTFToMsg(req_to_velodyne, world_pose);
	//ROS_INFO("x:%03f -> %03f, y:%03f -> %03f", velodyne_pose.position.x, world_pose.position.x, velodyne_pose.position.y, world_pose.position.y);

	visualization_msgs::InteractiveMarker int_marker;

	int_marker.header.frame_id = "world";
	int_marker.name = "No.1";
	int_marker.scale = 1.0;

	int_marker.pose = world_pose;

	make_box_control(int_marker);
	sync_jsk_box(world_pose);

	server->insert(int_marker);

	server->setCallback(int_marker.name, boost::bind(&int_detection::shift_feedback, this, _1));
	server->applyChanges();


}


int main(int argc, char **argv){

	ros::init(argc, argv, "int_detection_node");
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
