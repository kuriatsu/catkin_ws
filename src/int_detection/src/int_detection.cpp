#include <ros/ros.h>
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
		jsk_recognition_msgs::BoundingBox in_jsk_msgs;
		jsk_recognition_msgs::BoundingBox out_jsk_msgs;

		std_msgs::Header jsk_array_header;
		tf::TransformListener tf_listener;

	public :
		int_detection();
		void sync_jsk_box();

	private :
		void detection_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &msgs);
		void make_cube();
		visualization_msgs::InteractiveMarkerControl& make_box_control( visualization_msgs::InteractiveMarker &msg);
		void shift_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
		void calc_boxpose();
};


int_detection::int_detection(): shift(0){

	ros::NodeHandle n;

	sub_detection = n.subscribe("/detection/combined_objects_boxes", 5, &int_detection::detection_callback, this);
	pub_jsk_box = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detection/interactive_object", 5);

	out_jsk_msgs.dimensions.x = 1.0;
	out_jsk_msgs.dimensions.y = 6.0;
	out_jsk_msgs.dimensions.z = 2.0;
	out_jsk_msgs.value = 1;
}


void int_detection::sync_jsk_box(){

	jsk_recognition_msgs::BoundingBoxArray out_jsk_msgs_array;
	out_jsk_msgs_array.header = in_jsk_msgs.header;
	out_jsk_msgs.header.frame_id = "world";
	out_jsk_msgs.pose.position.z = -0.2;
	out_jsk_msgs_array.boxes.push_back(out_jsk_msgs);
	pub_jsk_box.publish(out_jsk_msgs_array);
}


void int_detection::detection_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &msgs){


	for (size_t i = 0; i < msgs->boxes.size(); i++){
		make_cube();
		in_jsk_msgs = msgs->boxes[i];
		out_jsk_msgs.header = msgs->boxes[i].header;
		out_jsk_msgs.label = 1;
	}
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


	float theta = std::atan(in_jsk_msgs.pose.position.y / in_jsk_msgs.pose.position.x);

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


void int_detection::make_cube(){

	calc_boxpose();

	visualization_msgs::InteractiveMarker int_marker;

	int_marker.header.frame_id = "world";
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
