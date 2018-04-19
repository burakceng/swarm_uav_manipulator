/*
This code is written to generate point cloud by subscribing the related topic and generate a pcd file for needed usage
*/

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/point_field_conversion.h>
#include <sensor_msgs/PointCloud.h>

#include <cstdlib>
#include <iostream>

ros::Publisher pcl_pub;
ros::Subscriber pcl_sub;


//Creating cloud callback
void depthCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PCLPointCloud2 cloud;
	
	pcl_conversions::toPCL(*input,cloud);
	
	pcl::PointCloud<pcl::PointXYZ> new_cloud;
	
	pcl::fromPCLPointCloud2(cloud,new_cloud);
	
	// If you need a voxelGrid object you can use the codes below
	//Creating VoxelGrid object
	/*pcl::VoxelGrid<pcl::PointXYZ> vox_obj;

	//Set input to voxel object
	vox_obj.setInputCloud (cloud.makeShared());

	//Setting parameters of filter such as leaf size
	vox_obj.setLeafSize (0.1f, 0.1f, 0.1f);
	
	//Performing filtering and copy to cloud_filtered variable
	vox_obj.filter(cloud_filtered);*/

	// save the point cloud in to the PCD file
	pcl::io::savePCDFileASCII ("our_cloud.pcd", new_cloud);

	//Publish the cloud as a message
	pcl_pub.publish(input);	
}


int main(int argc, char **argv)
{
	ros::init(argc,argv,"point_cloud");
	ros::NodeHandle node;

	ros::Rate loop_rate(1);

	//get the data from camera/depth/points topic 
	pcl_sub = node.subscribe("/camera/depth/points",10, &depthCallback);

	//publish the data with the created topic 
	pcl_pub = node.advertise<sensor_msgs::PointCloud2>("output",1);
	ros::spin();
	return 0;
}