/*
This code is written for reading the point cloud data from a generated PCD file
*/

#include <ros/ros.h>

#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "pcl_read");
	ROS_INFO("Started PCL read node");

	ros::NodeHandle nh;
	//create a publisher to see the pointcloud on rviz
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);

	sensor_msgs::PointCloud2 output;
	
	pcl::PointCloud<pcl::PointXYZ> cloud;

	//Load our_cloud.pcd file (read te cloud from pcd)
	pcl::io::loadPCDFile ("our_cloud.pcd", cloud);

	//convert the data into ROS message
	pcl::toROSMsg(cloud, output);

	//set the frame id for rviz simulation
	output.header.frame_id = "camera_link";

	ros::Rate loop_rate(1);
	
	while (ros::ok())
	{
		//Publishing the cloud inside pcd file
		pcl_pub.publish(output);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
