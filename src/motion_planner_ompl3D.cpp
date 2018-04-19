#include "../include/swarm_uav_manipulator/motion_planner_ompl3D.h"

/* -------------------- GLOBALS -------------------- */

// The callbacks implemented in PlannerCallbacks.cpp will refer to these variables when performing
// essential planner, collision avoidance tasks.

// In this simple simulation the obstacles are unit spheres. Therefore, the radius is known to be 0.5m and this vector
// is to hold the centers of the obstacles only.
std::vector< Coord3D > obstacle_list;
// A mapping from name of the UAV's to which step they're taking currently in terms of step's start position and end position
std::map<std::string, std::vector< Coord3D > > all_paths;
// The vector holding centers of UAV's - UAVs are considered as they have a sphere wrapping around them, for the sake of safety
std::map<std::string, Coord3D > uav_list;
// The vector of thread pointers that run the planner instances
std::vector<std::thread*> thread_refs;
// Mutex is used for mutually exclusive planning and motion
std::mutex mtx;

// Semaphore in order to make sure that all UAVs completed their planning (or spent their chances)
Semaphore sem(0);

// gazebo_msgs/ModelStates messages are published frequently; so, to avoid redundant computation, if the total_model_count is different from
// the actual count in the simulation, we process the incoming message in loadModels callback function.
size_t total_model_count = 2;
// The timing constraint for RRT* (or some other planning algorithm) for UAVs' planners
float waiting_duration = 1.0;
// Holds the number of UAVs, incremented only in the autoPlanning() method which is accessible via locking a mutex; so, the variable
// is mutually exclusive in terms of accessing.
int uav_count = 0;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_planner_ompl");
    ros::NodeHandle node;
   
    ros::Subscriber model_sub = node.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, 
    	boost::bind(&loadModels, std::ref(node), _1));
    	
    // ROS requires to use MultiThreadedSpinner object when user-defined threads will play a role in execution of some task.
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
