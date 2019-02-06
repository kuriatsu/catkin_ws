#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>

#include <interactive_markers/interactive_marker_server.h>

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;


class select_obstacle{
	private:
		 ros::Subscriber sub_points_no_ground;
		 ros::Subscriber sub_clustered_points;
		 std::vector<geometry_msgs::Pose> cloud_point_positions;
		 std_msgs::Header cloud_point_header;
		 std::vector<geometry_msgs::Pose> cluster_point_positions;
		 unsigned int subscribe_flag;
		 tf::TransformListener tf_listener;

	public:
		select_obstacle();

	private:
		void no_ground_callback(const sensor_msgs::PointCloud2 msgs);
//		void cluster_callback(const sensor_msgs::PointCloud2 &msgs);
		pcl::PointCloud<pcl::PointXYZI> pc2converter(sensor_msgs::PointCloud2 in_points);
		geometry_msgs::Pose tf_converter(geometry_msgs::Pose in_pose);
		void make_cube(geometry_msgs::Pose in_pose, unsigned int mode, std::string box_id);
		visualization_msgs::InteractiveMarkerControl& make_box_control(visualization_msgs::InteractiveMarker &msg, unsigned int mode);
		//void push_feedback_0(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
		//void push_feedback_1(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);


};

select_obstacle::select_obstacle(): subscribe_flag(1){

	ros::NodeHandle n;
	cloud_point_positions.reserve(15000);
	ROS_INFO("Ready...");

	sub_points_no_ground = n.subscribe("/points_no_ground", 2000, &select_obstacle::no_ground_callback, this);
//	sub_clustered_points = n.subscribe("/points_cluster", 50, &select_obstacle::cluster_callback, this);

};

// void select_obstacle::push_feedback_0(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
//
// 	make_cube(feedback->pose, 2, feedback->marker_name);
//
// }
//
// void select_obstacle::push_feedback_1(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
//
// 	ROS_INFO("Already added!!");
//
// }

void select_obstacle::no_ground_callback(const sensor_msgs::PointCloud2 msgs){

	pcl::PointCloud<pcl::PointXYZI> in_points;


	if(subscribe_flag == 1){

		cloud_point_positions.clear();
		geometry_msgs::Pose pose, last_pose;

		last_pose.position.x = 0;
		last_pose.position.y = 0;

		in_points = pc2converter(msgs);
		pcl_conversions::fromPCL(in_points.header, cloud_point_header);
		ROS_INFO("received");


		for (size_t i=0; i<in_points.points.size(); ++i){

			pcl::PointXYZI point = in_points[i];

			if(point.x > 1.0 && point.x < 10.0 && point.y > -3.0 && point.y < 3.0 && point.z < 2.0 ){

				if(point.x < (last_pose.position.x - 0.2) || point.x > (last_pose.position.x + 0.2)){

				 	if(point.y < (last_pose.position.y - 0.2) || point.y > (last_pose.position.y + 0.2)){

						std::stringstream s; s << i;

						pose.position.x = point.x;
						pose.position.y = point.y;
						pose.position.z = point.z;
						pose = tf_converter(pose);
						cloud_point_positions.push_back(pose);
						last_pose = pose;
						make_cube(pose, 0, s.str());
					}
				}
			}
		}
		ROS_INFO_STREAM(cloud_point_positions.size());
	}
}


// void select_obstacle::cluster_callback(const sensor_msgs::PointCloud2 &msgs){
//
// 	pcl::PointCloud<pcl::PointXYZI> in_points;
//
// 	subscribe_flag = 0;
//
// 	if(msgs.header.stamp == cloud_point_header.stamp){
//
// 		in_points = pc2converter(msgs);
//
// 		for (size_t i=0; i<in_points.points.size(); ++i){
//
// 			pcl::PointXYZI point = in_points[i];
// 			geometry_msgs::Pose pose;
// 			std::stringstream s; s << i;
//
// 			pose.position.x = point.x;
// 			pose.position.y = point.y;
// 			pose.position.z = point.z;
// 			pose = tf_converter(pose);
//
// 			make_cube(pose, 1, s.str());
//
// 		}
// 	}
// 	subscribe_flag = 1;
// }

void select_obstacle::make_cube(geometry_msgs::Pose in_pose, unsigned int mode, std::string box_id){

	visualization_msgs::InteractiveMarker int_marker;

	int_marker.header.frame_id = "world";
	int_marker.name = box_id;
	int_marker.scale = 1.0;
	int_marker.pose = in_pose;

	int_marker.pose.orientation.x = 0;
	int_marker.pose.orientation.y = 0;
	int_marker.pose.orientation.z = 0;
	int_marker.pose.orientation.w = 1;


	make_box_control(int_marker, mode);

	server->insert(int_marker);

	// switch(mode){
	// 	case 0:{
	// 		server->setCallback(int_marker.name, boost::bind(&select_obstacle::push_feedback_0, this, _1));
	// 	}break;
	// 	case 1:{
	// 		server->setCallback(int_marker.name, boost::bind(&select_obstacle::push_feedback_1, this, _1));
	// 	}break;
	//
	// }

	server->applyChanges();
}

visualization_msgs::InteractiveMarkerControl& select_obstacle::make_box_control(visualization_msgs::InteractiveMarker &msg, unsigned int mode){

	visualization_msgs::InteractiveMarkerControl control;

	switch(mode){
		case 0:{
			control.always_visible = true;
			control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
			control.name = "button";

			visualization_msgs::Marker marker;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.scale.x = msg.scale*0.2;
			marker.scale.y = msg.scale*0.2;
			marker.scale.z = msg.scale*0.2;
			marker.color.r = 1;
			marker.color.g = 0;
			marker.color.b = 0;
			marker.color.a = 0.5;

			control.markers.push_back(marker);
			msg.controls.push_back(control);
			//ROS_INFO("created");

		}break;

		case 1:{
			control.always_visible = true;
			control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
			control.name = "button";

			visualization_msgs::Marker marker;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.scale.x = msg.scale;
			marker.scale.y = msg.scale;
			marker.scale.z = msg.scale * msg.pose.position.z;
			marker.color.r = 0;
			marker.color.g = 0;
			marker.color.b = 1;
			marker.color.a = 0.5;

			control.markers.push_back(marker);
			msg.controls.push_back(control);

		}break;

		case 2:{
			control.always_visible = true;
			control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
			control.name = "button";

			visualization_msgs::Marker marker;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.scale.x = msg.scale;
			marker.scale.y = msg.scale;
			marker.scale.z = msg.scale * msg.pose.position.z;
			marker.color.r = 0;
			marker.color.g = 1;
			marker.color.b = 0;
			marker.color.a = 0.5;

			control.markers.push_back(marker);
			msg.controls.push_back(control);

		}break;


	}
	control.always_visible = true;

}

pcl::PointCloud<pcl::PointXYZI> select_obstacle::pc2converter(sensor_msgs::PointCloud2 in_points){

	pcl::PointCloud<pcl::PointXYZI> out_points;

	pcl::fromROSMsg(in_points, out_points);

	return out_points;
}


geometry_msgs::Pose select_obstacle::tf_converter(geometry_msgs::Pose in_pose){

	geometry_msgs::Pose out_pose;
	tf::Pose world_to_velodyne;
	tf::Pose req_to_velodyne;
	tf::StampedTransform req_to_world;

	try{
		tf_listener.waitForTransform("velodyne", "world", ros::Time(0), ros::Duration(1.0));
		tf_listener.lookupTransform("world", "velodyne", ros::Time(0), req_to_world);

	}catch(...){
		ROS_INFO("velodyne to world transform ERROR");
	}

	tf::poseMsgToTF(in_pose, world_to_velodyne);
	req_to_velodyne = req_to_world * world_to_velodyne;
	tf::poseTFToMsg(req_to_velodyne, out_pose);

	return out_pose;
}


int main(int argc, char **argv){

	ros::init(argc, argv, "select_obstacle_node");

	server.reset(new interactive_markers::InteractiveMarkerServer("select_obstacle_node"));
	ros::Duration(0.1).sleep();

	ROS_INFO("Initializing...");

	select_obstacle select_obstacle;

	ROS_INFO("Ready...");

	ros::spin();
	server.reset();
	return 0;

}
