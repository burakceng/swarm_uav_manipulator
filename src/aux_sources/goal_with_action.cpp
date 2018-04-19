#include <ros/ros.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/PoseActionGoal.h>

int main(int argc,char** argv){
	ros::init(argc,argv,"goal_setter");
	ros::NodeHandle nh;
	ros::Publisher pub_vel = nh.advertise<hector_uav_msgs::PoseActionGoal>("/action/pose/goal",1000);
	hector_uav_msgs::PoseActionGoal goal;
	
	goal.goal.target_pose.pose.position.z = 15;
	goal.goal.target_pose.pose.position.y = 10;
	goal.goal.target_pose.pose.position.x = 3;

	goal.goal.target_pose.pose.orientation.x = 0;
	goal.goal.target_pose.pose.orientation.y = 0;
	goal.goal.target_pose.pose.orientation.z = 0;
	goal.goal.target_pose.pose.orientation.w = 1;
	/* ---------------------------- Enabling motors handled by pose_action node ---------------------*/
	ros::ServiceClient enable_motors = nh.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
	hector_uav_msgs::EnableMotors srv;
	srv.request.enable = true;
	if(enable_motors.call(srv)){
		if(srv.response.success){
			ROS_INFO("Motors are enabled");		
		}
	}
	
	ros::Rate rate(10.0);
	while(ros::ok()){
		goal.goal.target_pose.header.stamp = ros::Time::now();
		goal.goal.target_pose.header.frame_id = "world";
		pub_vel.publish(goal);
				
		rate.sleep();
	}
	return 0;
}

