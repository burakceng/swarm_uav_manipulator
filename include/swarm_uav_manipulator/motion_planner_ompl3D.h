#ifndef MOTION_PLANNER_OMPL3D_H_
#define MOTION_PLANNER_OMPL3D_H_

/* -------------------- INCLUDES -------------------- */

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <hector_uav_msgs/Altimeter.h>
#include <hector_uav_msgs/Done.h>


#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <gazebo_msgs/ModelStates.h>

#include <fstream>
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <boost/bind.hpp>

#include <vector>
#include <utility>
#include <cmath>
#include <tuple>
#include <map>

#include <mutex>
#include <thread>
#include <condition_variable>

#include "../include/Semaphore.h"

#define SEQUENTIAL_PLANNER_TIME 2.5 /*This is the time variable for planning. Planning can be useful down to 0.5 seconds.
                                      It should be designated to meet the purpose of application. */

namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef std::tuple<float, float, float> Coord3D;

/* -------------------- HELPERS -------------------- */

// In order to print the trajectories of active UAVs.
void printPaths();

// A simple formula of distance between 2 known points in space.
float euclidean_distance(float x1, float y1, float x2, float y2, float z1, float z2);

// Test some state/position given by (sx, sy, sz) if it falls into directly an obstacle's region, or the region inflated
// by robots radius - to consider the UAV as point robot. This function considers each UAV is an obstacle to other UAVs as well.
bool notObstacle(float sx, float sy, float sz);

// The state validity checker method whose reference is given to planner object of WrapperPlanner class to check the sampled states
// are valid/invalid in terms of the ability of the UAV to move there without any collision.
bool isStateValid(const std::string& quad_name, const ob::State* state);

// Includes paths as obstacles. Thus, no intersection between UAV paths occurs.
bool check_paths_as_obstacles(float sx,float sy,float sz);

/* -------------------- CLASS -------------------- */

class WrapperPlanner{
	public:
		// Node handle instance to initialize Publisher and Subscriber objects.
		ros::NodeHandle node;
		// In order: Publishes a step to ultimate goal, publishes remaining path length,
		// publishes if the UAV has arrived or not, publishes the number of the step on which the UAV currently is 
		ros::Publisher /*samp_pub,*/ step_cmd, rem_pub, arrived_pub, waypoint_pub;
		// In order: Subscribes to Done messages in order to generate next step if any (else stop!), obtain the ultimate goal,
		// obtain UAV's position at a certain time instance, 
		ros::Subscriber done_sub, mainGoal_sub, rloc_sub, update_goal_sub;
		// The path from initial position to ultimate goal as 3D points in the space.
		std::vector< Coord3D > paths;
		size_t p_total_model_count;
		// UAV's most recent position, ultimate goal position, next step position
		Coord3D robot_pos, goal_pos, step_pos;
		// UAV's name
		std::string quad_name;
		// Starting time to compute elapsed time in terminalCondition method
		clock_t timeBeforePlanning;
		// In order: Erase is allowed when there is no risk of collision, otherwise, the risk is avoided by sending 
		// current position of robot until risk is diminished to a feasible level; indicates that next step has taken
		bool erase_flag, step_acquired;
		// In order: A value that determines the priority of the UAV in case of multiple UAVs has a risk of collision at a
		// close region, another value in order to reinforce this idea
		float total_path_length, step_path_distance;

		// Constructor of the essentials and the planner context of the motion planning process of UAVs
		WrapperPlanner(ros::NodeHandle _nh, std::string uav_name);

		~WrapperPlanner() {}

		// The callback function that handles the publishes to actual_uav_goal topic which is related to ultimate goals of UAVs.
		void MainGoal(const std::string& robot_frame, const geometry_msgs::Point::ConstPtr& goal);

		// The callback function that handles the messages sent when the UAV arrived its step location - the path to the ultimate goal
		// may contain several of these steps.
		void StepDone(const std::string& robot_frame, const hector_uav_msgs::Done::ConstPtr& msg);

		// A stimuli that dictates to change our path has arrived. Check that if goal is in a safe area.
		void UpdateGoal(const std::string& robot_frame,const std_msgs::Bool::ConstPtr& msg);

		// The callback function that updates the current position of the UAV.
		void rloc_callback(const std::string& robot_frame, const gazebo_msgs::ModelStates::ConstPtr& msg);

		/* The function that checks if the time limit for the planner algorithm has passed or not. When RRT* is used as planner, the OMPL library did not include the function of duration as terminal condition by default; therefore, there is a new one which utilizes <ctime> header's clock() function to compute the time just before the planning starts and let RRT* check it whenever required while sampling states.*/
		bool terminalCondition();

		// This method sets up the necessary/general constraints of the space that the planner will perform, the problem that the planner
		// will be solving.
		ob::ProblemDefinitionPtr planner_setup(ob::StateSpacePtr& space, ob::SpaceInformationPtr& si);

		/* This method will set the planner's algorithm when performing state sampling - different algorithms have different approaches of sampling states. The function seems to have a loop of tries, if planner could not find a solution in its first try, it will have the chance to plan it again, until timeout counter reaches a certain value.*/
		void autoPlanning();

		/* When the planner successfully finds a path, it will contain a number of steps to reach the ultimate goal. This function will publish the steps in the given path order to /move_base_simple topic. Of course, there can be paths with only 1 step.*/
		void publishStep();
};

void initiate_UAV(ros::NodeHandle& nh, const std::string& uav_name);

void loadModels(ros::NodeHandle& nh, const gazebo_msgs::ModelStates::ConstPtr& msg);

#endif
