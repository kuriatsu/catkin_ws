#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>

ros::Publisher pub;


void cloud_cb (const sensor_msgs::PointCloud2 input)
{
	//PointCloud2 to PCL
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::PointCloud<pcl::PointXYZ> buf_cloud;

	//add intensity;
	pcl::fromROSMsg (input, buf_cloud);
	pcl::copyPointCloud(buf_cloud, cloud);

	for (size_t i = 0; i < buf_cloud.points.size(); ++i){
		cloud.points[i].intensity = 1;
	}

	//PCL to PointCloud2
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg (cloud, output);
	output.header = input.header;
	output.header.frame_id = "velodyne";

	// Publish the PointCloud2
	pub.publish (output);
}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pc2_to_pcl_node");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/lidar_0", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("/points_raw", 1);

	// Spin
	ros::spin ();
}
