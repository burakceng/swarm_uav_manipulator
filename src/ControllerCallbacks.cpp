#include "../include/swarm_uav_manipulator/QuadController.h"

// Indicates the radius of the goal region by meters
#define EQUAL_CONST 0.4

// If this was not declared here, yo will probably get "undefined reference to pid_" error when linker phase runs in compiling.
extern struct PID pid_;

//	This function is to assign a virtual "carrot" position for the UAV, if the UAV is close enough to its goal
//	or sub-goal positions.
bool isEqual(double a, double b) { return ( fabs(a - b) > EQUAL_CONST ) ? false : true; }


/*---------------------------------------------- CALLBACK FUNCTIONS FOR QUAD_MANIPULATOR --------------------------*/

// If a collision risk is detected, this callback reverts the "slowing" boolean to true, hence enforces the UAV to slow.
void Wrapper::QuadController::collisionCallback(const std::string& robot_frame, const std_msgs::Bool::ConstPtr& msg)
{
	slowing = msg->data; 
}


// This callback updates the UAV's velocity according to the number of waypoints in the path. This method is embraced, because 
// as the waypoints decreases, distance between them increases. Implemented function paces up to higher speeds as distance
// grows, so we have determined to use a variable velocity factor.
void Wrapper::QuadController::UpdateVelocity(const std::string& robot_frame, const std_msgs::Int32::ConstPtr& msg)
{
	velocity_factor = 0.15 * msg->data;
	if(velocity_factor > 1.0)
		velocity_factor = 1.0;
}


// This callback updates the UAV's goal position that is published to specific-to-UAV /move_base_simple/goal topic in the 
// geometry_msgs/PoseStamped message form. There is a boolean variable "operating" in order to disable the Done (or myDone) messages
// that come from somewhere unknown and prevent them to be processed in quadPoseCallback.
void Wrapper::QuadController::goalPoseCallback(const std::string& robot_frame, const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
	// Set the operating to true, indicating that the UAV has received its ultimate goal and will start to plan and move.
	operating = true;
	QuadController* correspondingQuad = activeQuadrotors[robot_frame];

	double x = msg->pose.position.x;
	double y = msg->pose.position.y;
	double z = msg->pose.position.z;

	ROS_INFO("%s : Step goal is (%f,%f,%f)",robot_frame.c_str(),x,y,z); // For simple debugging purposes

	double temp;
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, 
		msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3(q).getRPY(temp, temp, correspondingQuad->yaw_command);

	if((correspondingQuad->goalPose.pose.position.x == x) 
		&& (correspondingQuad->goalPose.pose.position.y == y) 
		&& (correspondingQuad->goalPose.pose.position.z == z))
		return;

	correspondingQuad->goalPose.pose.position.x = x;
	correspondingQuad->goalPose.pose.position.y = y;
	correspondingQuad->goalPose.pose.position.z = z;
	correspondingQuad->desired_updated = true;
	correspondingQuad->desired_achived = false;
}


//Quadrotor's own position callback. It not only updates the position of quadrotor; 
//but also computes the necessary velocity vectors in order to reach to the goal. 
void Wrapper::QuadController::quadPoseCallback(const std::string& robot_frame, const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
	// This callback adjusts linear velocity and adjusts the yaw rate. The roll and pitch rates might be added as the next step.

	Wrapper::QuadController* correspondingQuad = Wrapper::activeQuadrotors[robot_frame];

	double goalX = correspondingQuad->goalPose.pose.position.x;
	double goalY = correspondingQuad->goalPose.pose.position.y;
	double goalZ = correspondingQuad->goalPose.pose.position.z;
	correspondingQuad->quadPose.pose.position.x = msg->pose.position.x;
	correspondingQuad->quadPose.pose.position.y = msg->pose.position.y;
	correspondingQuad->quadPose.pose.position.z = msg->pose.position.z;

	geometry_msgs::Twist vel_msg;
	ros::Duration period(0.1);

	if(correspondingQuad->shouldWait() == false)
	{
		double x = pid_.x.computeCommand(goalX - correspondingQuad->quadPose.pose.position.x, period);
		double y = pid_.y.computeCommand(goalY - correspondingQuad->quadPose.pose.position.y, period);
		double z = pid_.z.computeCommand(goalZ - correspondingQuad->quadPose.pose.position.z, period);

		tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
			msg->pose.orientation.z, msg->pose.orientation.w);
		double yaw = tf::getYaw(q);
		double yaw_error = correspondingQuad->yaw_command - yaw;
	    // detect wrap around pi and compensate

		if (yaw_error > 3.14) 
			yaw_error -= 2 * 3.14;
		else if (yaw_error < -3.14) 
			yaw_error += 2 * 3.14;

		// If UAV is in the correct position it will publish a Done message and inform the program that goal is satisfied.
		if(operating && isEqual(goalX, correspondingQuad->quadPose.pose.position.x) 
			&& isEqual(goalY, correspondingQuad->quadPose.pose.position.y) 
			&& isEqual(goalZ, correspondingQuad->quadPose.pose.position.z) )
		{
			hector_uav_msgs::Done done_send;

			done_send.commandDone = true;
			done_send.position.x = goalX;
			done_send.position.y = goalY;
			done_send.position.z = goalZ;
			done_send.orientation.x = 0;
			done_send.orientation.y = 0;
			done_send.orientation.z = 0;

			correspondingQuad->quad_done.publish(done_send);
			operating = false;
		}

		// The following block of code makes final adjustments to velocity command before publishing. The velocities are scaled down
		// by some constant, since the world(s) used in simulation has little space to move for UAVs and high speeds can cause overshooting
		// the step/ultimate goals, worse it can cause collisions. The "velocity_factor" and "slowing_factor" variables are used for this
		// issue.

		// corrects the position of the quadrotor by giving velocity
		if(!slowing && operating)
		{
			vel_msg.linear.x = x * velocity_factor;
			vel_msg.linear.y = y * velocity_factor;
			vel_msg.linear.z = z * velocity_factor;
			vel_msg.angular.z = 2 * pid_.yaw.computeCommand(yaw_error, period);
		}
		else if (slowing && operating)
		{
			// In this context, slowing is arrived when there is a risk of collision.
			// All UAVs will receive this message except the highest priority.
			// For further analysis, check the collision_checker.cpp

			vel_msg.linear.x = x * slowing_factor;
			vel_msg.linear.y = y * slowing_factor;
			
			slowing_factor = slowing_factor - 0.05;
			if(slowing_factor < 0)
				slowing_factor = 0;

			vel_msg.angular.z = 2 * pid_.yaw.computeCommand(yaw_error, period);
		}
		
		correspondingQuad->quad_vel.publish(vel_msg);
	}
}

/*-----------------------------------------------------------------------------------------------------------------*/
