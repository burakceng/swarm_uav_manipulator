#ifndef QUAD_CONTROLLER_H_
#define QUAD_CONTROLLER_H_

#include <ros/ros.h>

#include <control_toolbox/pid.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <hector_uav_msgs/EnableMotors.h>
#include <hector_uav_msgs/Done.h>

#include <boost/bind.hpp>
#include <stdlib.h>
#include <fstream>
#include <map>
#include <cmath>

#include <tf/transform_listener.h>

// For PID control; however, PID control is not optimal or not applied to the whole path of a UAV to its goal.
// On the other hand, they are changing from one velocity command to another, since there can be a problem about turning
// velocity (either it turns with a wide angle, returning its original path with a waste of time and possibly colliding, or 
// overshoots its goal position and similar risks exist).
struct PID
{
	control_toolbox::Pid x, y, z, yaw;
};

bool isEqual(double a, double b);

class Wrapper{
public:
	class QuadController {

		private:
			bool shouldWait() { return (!desired_updated) && (desired_achived); }

		public:
			// Node handle in order to create the Subscriber and Publisher objects.
			ros::NodeHandle node;
			// The motors are shut down initially, to move a UAV, they should be started.
			ros::ServiceClient engage_motors;
			// In declaration order: obtains UAV's goal position, obtains UAV's position at a certain time instance,
			// obtains a boolean indicating if there is a collision risk or not, obtains which step the UAV is in
			ros::Subscriber goal_sub, quad_sub, collision_sub, waypoint_number_sub;
			// In declaration order: publishes both linear and angular velocity commands, publishes a Done (or myDone)
			// message indicating that the UAV has reached its step position or it ultimate goal position
			ros::Publisher quad_vel, quad_done;
			// Variables holding ultimate goal position and UAV's position at a certain time instance
			geometry_msgs::PoseStamped goalPose, quadPose;
			// Name of the UAV
			std::string quad_name;
			// If some collision risk exists, UAV will slow down at a rate determined by these factors.
			float velocity_factor, slowing_factor;
			// Plays role in how sharp the UAV will turn when it has non-zero angular velocity.
			double yaw_command;
			// ...
			bool desired_updated, desired_achived;
			// Indicates that the UAV is on the move to its goal / sub-goal.
			bool operating;
			// Indicates that the UAV is facing a risk of collision and slowing down for safety.
			bool slowing;

			QuadController(ros::NodeHandle _node, std::string uav_name) 
			{
				activeQuadrotors.insert(std::pair<std::string, Wrapper::QuadController*>(uav_name, this));

				quad_name = uav_name;
				node = _node;
				desired_updated = false;
				desired_achived = false;
				operating = false;
				slowing = false;
				yaw_command = 0;
				velocity_factor = 0.15; //It will be multiplied by number of waypoints in the path.
				slowing_factor = 0.15;

				engage_motors = node.serviceClient<hector_uav_msgs::EnableMotors>("/" + quad_name + "/enable_motors");
				hector_uav_msgs::EnableMotors srv;
				srv.request.enable = true;
				if(engage_motors.call(srv))
				{
					if(srv.response.success)
					{
						ROS_INFO("%s motors are enabled",quad_name.c_str());		
					}
				}

				goal_sub = node.subscribe<geometry_msgs::PoseStamped>("/" + quad_name + "/move_base_simple/goal", 10, 
					boost::bind(&QuadController::goalPoseCallback, this, quad_name, _1));
				quad_sub = node.subscribe<geometry_msgs::PoseStamped>("/" + quad_name + "/ground_truth_to_tf/pose", 10, 
					boost::bind(&QuadController::quadPoseCallback, this, quad_name, _1));
				collision_sub = node.subscribe<std_msgs::Bool>("/" + quad_name + "/check_collision",1,
					boost::bind(&QuadController::collisionCallback, this, quad_name, _1));

				waypoint_number_sub = node.subscribe<std_msgs::Int32>("/" + quad_name + "/path_number",1,
					boost::bind(&QuadController::UpdateVelocity, this, quad_name, _1));
				quad_vel = node.advertise<geometry_msgs::Twist>("/" + quad_name + "/cmd_vel", 10);
				quad_done = node.advertise<hector_uav_msgs::Done>("/" + quad_name + "/Done", 10);
			}

			// If a collision risk is detected, this callback reverts the "slowing" boolean to true, hence enforces the UAV to slow.
			void collisionCallback(const std::string& robot_frame, const std_msgs::Bool::ConstPtr& msg);

			/* This callback updates the UAV's velocity according to the number of waypoints in the path. 
			This method is embraced, because as the waypoints decreases, distance between them increases. 
			Implemented function paces up to higher speeds as distance grows, so we have determined to use a variable velocity factor.*/
			void UpdateVelocity(const std::string& robot_frame, const std_msgs::Int32::ConstPtr& msg);

			// Goal callback. Updates the goal point.
			void goalPoseCallback(const std::string& robot_frame, const geometry_msgs::PoseStamped::ConstPtr& msg);

			// Quadrotor's own position callback. It not only updates the position of quadrotor; 
			// but also computes the necessary velocity vectors in order to reach to the goal. 
			void quadPoseCallback(const std::string& robot_frame, const geometry_msgs::PoseStamped::ConstPtr& msg);
			
			~QuadController() {}
		};

	// The container that holds a mapping from the names of UAVs that are active in the simulation to their controller instances
	// in order to respond position updates and velocity adjustments asynchronnously.
	static std::map<std::string, QuadController*> activeQuadrotors;
};

#endif
