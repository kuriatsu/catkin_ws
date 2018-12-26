#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/CameraInfo.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>



class viz_yolo
{
private :
	ros::Subscriber sub_camera_info;
	ros::Subscriber sub_coordinarate;

public :
	viz_yolo();

private :
	void camera_callback(const sensor_msgs::CameraInfo &msgs);
	void detection_callback(const autoware_msgs::DetectedObjectArray::ConstPtr &msgs);

};

viz_yolo::viz_yolo()
{
	ros::NodeHandle n;
	sub_camera_info = n.subscribe("/camera_info", 5, &viz_yolo::camera_callback, this);
	sub_coordinarate = n.subscribe("/detection/tracked_objects", 5, &viz_yolo::detection_callback, this);

}


void viz_yolo::camera_callback(const sensor_msgs::CameraInfo &msgs)
{
	//std::cout << msgs.K[0] << std::endl;
}

void viz_yolo::detection_callback(const autoware_msgs::DetectedObjectArray::ConstPtr &msgs)
{
	//autoware_msgs::DetectedObject object;
	//object = msgs[1];
	std::cout << "hello" << std::endl;
	for (size_t i = 0; i < msgs->objects.size(); i++)
	{
		std::cout << msgs->objects[i].x << std::endl;
	}

}


int main(int argc, char **argv){

	ros::init(argc, argv, "viz_yolo");
	viz_yolo vizyolo;

	ros::spin();

	return 0;

}
