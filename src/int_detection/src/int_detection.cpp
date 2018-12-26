#include <ros/ros.h>
#include <tf/tf.h>
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

	jsk_recognition_msgs::BoundingBox out_jsk_box = jsk_msgs;
	jsk_recognition_msgs::BoundingBoxArray out_jsk_box_array;

	out_jsk_box.pose = pose;
	out_jsk_box.dimensions.y = 6.0;

	out_jsk_box_array.boxes[0] = out_jsk_box;
	out_jsk_box_array.header = jsk_array_header;

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

	std::stringstream s;
	s << jsk_msgs.label;

	float theta = std::atan(jsk_msgs.pose.position.y / jsk_msgs.pose.position.x);
	geometry_msgs::Pose pose;
	pose.position.x = jsk_msgs.pose.position.x + shift * std::sin(theta);
	pose.position.y = jsk_msgs.pose.position.y + shift * std::cos(theta);
	pose.position.z = jsk_msgs.pose.position.z;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1;


	//server->clear();

	visualization_msgs::InteractiveMarker int_marker;

	int_marker.header.frame_id = "velodyne";
	int_marker.name = "No.1";
	int_marker.scale = 1.0;

	int_marker.pose = pose;

	make_box_control(int_marker);
	//sync_jsk_box(pose);

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
