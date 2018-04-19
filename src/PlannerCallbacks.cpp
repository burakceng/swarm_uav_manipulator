#include "../include/swarm_uav_manipulator/motion_planner_ompl3D.h"

/*---------------------------------------- CALLBACK FUNCTIONS FOR MOTION PLANNER OMPL 3D --------------------------*/


/* -------------------- GLOBALS -------------------- */
// If this was not declared here, yo will probably get "undefined reference to <variable_name>" error when linker phase runs in compiling.

// In this simple simulation the obstacles are unit spheres. Therefore, the radius is known to be 0.5m and this vector
// is to hold the centers of the obstacles only.
extern std::vector< Coord3D > obstacle_list;
// A mapping from name of the UAV's to which step they're taking currently in terms of step's start position and end position
extern std::map<std::string, std::vector< Coord3D > > all_paths;
// The vector holding centers of UAV's - UAVs are considered as they have a sphere wrapping around them, for the sake of safety
extern std::map<std::string, Coord3D > uav_list;
// The vector of thread pointers that run the planner instances
extern std::vector<std::thread*> thread_refs;
// Mutex is used for mutually exclusive planning and motion
extern std::mutex mtx;

// Semaphore in order to make sure that all UAVs completed their planning (or spent their chances)
extern Semaphore sem;

// gazebo_msgs/ModelStates messages are published frequently; so, to avoid redundant computation, if the total_model_count is different from
// the actual count in the simulation, we process the incoming message in loadModels callback function.
extern size_t total_model_count;// = 2;
// The timing constraint for RRT* (or some other planning algorithm) for UAVs' planners
extern float waiting_duration;// = 1.5;
// Holds the number of UAVs, incremented only in the autoPlanning() method which is accessible via locking a mutex; so, the variable
// is mutually exclusive in terms of accessing.
extern int uav_count;// = 0;


// In order to print the trajectories of active UAVs.
void printPaths()
{
    std::map<std::string, std::vector< Coord3D > >::iterator map_itr = all_paths.begin();
    for(; map_itr != all_paths.end(); map_itr++)
    {
        std::cout << map_itr->first << " PATH" << std::endl;
        for(unsigned int i = 0; i < (map_itr->second).size(); i++)
        {
        	std::cout << "(" << std::get<0>(map_itr->second[i]) << ", " << std::get<1>(map_itr->second[i]) << ", "
        		<< std::get<2>(map_itr->second[i]) << ")" << std::endl;
        }

        std::cout << "---------------------" << std::endl;
    }
}


// A simple formula of distance between 2 known points in space.
float euclidean_distance(float x1, float y1, float x2, float y2, float z1, float z2)
{
    return sqrt(pow(x1 - y1, 2) + pow(x2 - y2, 2) + pow(z1 - z2, 2));
}


// Test some state/position given by (sx, sy, sz) if it falls into directly an obstacle's region, or the region inflated
// by robots radius - to consider the UAV as point robot. This function considers each UAV is an obstacle to other UAVs as well.
bool notObstacle(float sx, float sy, float sz)
{
    std::vector< Coord3D >::iterator obs = obstacle_list.begin();
    for(; obs != obstacle_list.end(); obs++)
    {
        // obstacles are unit spheres (unit circles when projected on 2D) with 0.5 m as their radius
        // inflation radius (the radius that wraps the quadrotor properly) is at least 0.5 m. But for more safe-sought
        // implementations, it can be increased.
        
        float result = pow(std::get<0>(*obs) - sx, 2) + pow(std::get<1>(*obs) - sy, 2) + pow(std::get<2>(*obs) - sz, 2) - 2.0;
        if (result < 0) // The state falls into either an inner region of an obstacle or an inflation area
            return false;
    }

    std::map<std::string, Coord3D >::iterator it = uav_list.begin();
    for(; it != uav_list.end(); it++){
        
        float result = pow(std::get<0>(it->second) - sx,2) + pow(std::get<1>(it->second) - sy,2) + pow(std::get<2>(it->second) - sz,2) - 1.2;
        if(result<0)
            return false;
    }

    // It was considered at first as that the UAV's current position to its current step position will be taken as a cylindrical obstacle.
    // If there was any other UAVs trying to perform motion planning, they should consider these dynamically shrinking cylinrical obstacles.
    

    return true;
}


// The state validity checker method whose reference is given to planner object of WrapperPlanner class to check the sampled states
// are valid/invalid in terms of the ability of the UAV to move there without any collision.
bool isStateValid(const std::string& quad_name, const ob::State* state)
{
    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
    //const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
    //const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // Check state if its in any of the occupied obstacle areas or the ones inflated with robot's radius
    float sx = se3state->getX();
    float sy = se3state->getY();
    float sz = se3state->getZ();
    return notObstacle(sx,sy,sz);
}


// Constructor of the essentials and the planner context of the motion planning process of UAVs
WrapperPlanner::WrapperPlanner(ros::NodeHandle _nh, std::string uav_name)
{
    node = _nh;
    quad_name = uav_name;

    done_sub = node.subscribe<hector_uav_msgs::Done>("/" + quad_name + "/Done", 10, 
    	boost::bind(&WrapperPlanner::StepDone, this, quad_name, _1));
    mainGoal_sub = node.subscribe<geometry_msgs::Point>("/" + quad_name + "/actual_uav_goal", 1, 
    	boost::bind(&WrapperPlanner::MainGoal, this, quad_name, _1));

    // ---------------- It might be useful when further plannings are considered as meaningfully costly. ---------------
    update_goal_sub = node.subscribe<std_msgs::Bool>("/" + quad_name + "/update_goal", 1, 
    	boost::bind(&WrapperPlanner::UpdateGoal, this, quad_name, _1));
    // -----------------------------------------------------------------------------------------------------------------

    step_cmd = node.advertise<geometry_msgs::PoseStamped>("/" + quad_name + "/move_base_simple/goal", 1);
    //samp_pub = node.advertise<geometry_msgs::Point>("/" + quad_name + "/sampled_point", 1000);
    rem_pub = node.advertise<std_msgs::Float64>("/" + quad_name + "/remaining_step", 1);
    arrived_pub = node.advertise<std_msgs::String>("/" + quad_name + "/arrival", 1);
    waypoint_pub = node.advertise<std_msgs::Int32>("/" + quad_name + "/path_number", 1);

    all_paths[quad_name] = std::vector<Coord3D>(2);
    rloc_sub = node.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, 
    	boost::bind(&WrapperPlanner::rloc_callback, this, quad_name, _1));
    
    p_total_model_count = 2;
    erase_flag = false;
    total_path_length = 0;
    step_path_distance = 0;
    step_acquired = false;
}


// The callback functon that handles the publishes to actual_uav_goal topic which is related to ultimate goals of UAVs.
void WrapperPlanner::MainGoal(const std::string& robot_frame, const geometry_msgs::Point::ConstPtr& goal)
{
	// Set the goal position for the UAV
    std::get<0>(goal_pos) = goal->x;
    std::get<1>(goal_pos) = goal->y;
    std::get<2>(goal_pos) = goal->z;
    ROS_INFO("Subscription is successful. %.2f,%.2f,%.2f",goal->x,goal->y,goal->z);
    // Every UAV will have its exclusive order to perform planning
    mtx.lock();
    timeBeforePlanning = clock();
    autoPlanning();
    mtx.unlock();

    // After planning completed, wait for other UAVs to complete their planning.
    sem.wait();
    publishStep();
    // To enable all UAVs notify the semaphore again
    sem.notify();
}


// The callback function that handles the messages sent when the UAV arrived its step location - the path to the ultimate goal
// may contain several of these steps.
void WrapperPlanner::StepDone(const std::string& robot_frame, const hector_uav_msgs::Done::ConstPtr& msg)
{
    mtx.lock();
    if (!paths.empty())
    {
    	// UAV has steps to go to reach ultimate goal.

        all_paths[quad_name][1] = paths[0];
        total_path_length -= step_path_distance;
        
        ROS_INFO("%s total_path_length = %.2f", robot_frame.c_str(), total_path_length);
        publishStep();
    }
    else
    {
    	// The UAV is already on the previous ultimate goal position.

        all_paths.erase(quad_name);
        erase_flag = true;
        ROS_INFO("%s arrived!", robot_frame.c_str());

        std_msgs::String msg;
        msg.data = "SUCCESS";
        arrived_pub.publish(msg);

        //Update current position of robot as an obstacle.
        uav_list[robot_frame] = robot_pos;
        ROS_INFO("%s position (%.2f,%.2f,%.2f) is added as an obstacle", robot_frame.c_str(), std::get<0>(robot_pos), 
        	std::get<1>(robot_pos), std::get<2>(robot_pos));
    }
    mtx.unlock();
}


// A stimuli that dictates to change our path has arrived. Check that if goal is in a safe area.
void WrapperPlanner::UpdateGoal(const std::string& robot_frame, const std_msgs::Bool::ConstPtr& msg)
{   
    ros::Duration(1.0).sleep();
    //This sleep is necessary for UAV to stop its movement.

    mtx.lock();
    //This if block is necessary when the UAVs send to same or close goal points.
    if(notObstacle(std::get<0>(goal_pos), std::get<1>(goal_pos), std::get<2>(goal_pos)) == false)
    {
        std::srand(std::time(0));
        while(1)
        {
        	// This while loop generates random (x, y, z) positions in order to attach a temporary goal, 
        	// if a risk of collision is the case.

            float x_addee, y_addee, z_addee;
            float temp_goalX, temp_goalY, temp_goalZ;

            x_addee = (float)((std::rand() % 200) - 100) / 100;
            y_addee = (float)((std::rand() % 200) - 100) / 100;
            z_addee = (float)((std::rand() % 200) - 100) / 100;
            temp_goalX = std::get<0>(goal_pos) + x_addee;
            temp_goalY = std::get<1>(goal_pos) + y_addee;
            temp_goalZ = std::get<2>(goal_pos) + z_addee;

            ROS_INFO("Checking %.2f, %.2f, %.2f as possible goal", temp_goalX, temp_goalY, temp_goalZ);
            if(notObstacle(temp_goalX, temp_goalY, temp_goalZ))
            {
                std::get<0>(goal_pos) = temp_goalX;
                std::get<1>(goal_pos) = temp_goalY;
                std::get<2>(goal_pos) = temp_goalZ;

                ROS_INFO("New goal_point is (%.2f, %.2f, %.2f)", std::get<0>(goal_pos), std::get<1>(goal_pos), std::get<2>(goal_pos));
                autoPlanning(); // Plan again.
                break;
            }
        }
    }
    else // No need to update goal. Check left path considering finished UAVs
        autoPlanning();
    mtx.unlock();
}


// The callback function that updates the current position of the UAV.
void WrapperPlanner::rloc_callback(const std::string& robot_frame, const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	// This callback routine may be transformed into one that accepts a geometry_msgs::Point message published from another 
	// callback routine that updates the position of the UAV, Wrapper::QuadController::quadPoseCallback(...). In that way, we
	// could avoid the unnecessary loop below to find the related UAV.

    size_t model_count = msg->name.size();
    for (unsigned int i = 1; i < model_count; i++)
    {
        size_t found = msg->name[i].find(quad_name);
        if(found!=std::string::npos)
        {
            robot_pos = std::make_tuple(msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z);
            if(!step_acquired)
                step_pos = robot_pos;

            float current_distance_to_step = euclidean_distance(std::get<0>(step_pos), std::get<0>(robot_pos),
                std::get<1>(step_pos), std::get<1>(robot_pos), std::get<2>(step_pos), std::get<2>(robot_pos)); 

            total_path_length -= step_path_distance - current_distance_to_step;
            step_path_distance = current_distance_to_step;
            break;
        }
    }

    if(erase_flag)
        return;
    else
        all_paths[quad_name][0] = robot_pos;
}


// The function that checks if the time limit for the planner algorithm has passed or not. When RRT* is used as planner,
// the OMPL library did not include the function of duration as terminal condition by default; therefore, there is a new 
// one which utilizes <ctime> header's clock() function to compute the time just before the planning starts and let RRT*
// check it whenever required while sampling states.
bool WrapperPlanner::terminalCondition()
{
    clock_t checkTime = clock() - timeBeforePlanning;
    float seconds_passed = ((float)checkTime) / CLOCKS_PER_SEC;
    if (seconds_passed >= waiting_duration)
    {
        ROS_INFO("Termination achieved.");
        waiting_duration += SEQUENTIAL_PLANNER_TIME;
        ROS_INFO("waiting_duration for %s is %.2f",quad_name.c_str(),waiting_duration);
        return true;
    }
    else
        return false;
}


// This method sets up the necessary/general constraints of the space that the planner will perform, the problem that the planner
// will be solving.
ob::ProblemDefinitionPtr WrapperPlanner::planner_setup(ob::StateSpacePtr& space, ob::SpaceInformationPtr& si)
{
    ob::RealVectorBounds bounds(3);
    // Bounds of X in 3D space
    bounds.setLow(0, -10);
    bounds.setHigh(0, 10);
    // Bounds of Y in 3D space
    bounds.setLow(1, -10);
    bounds.setHigh(1, 10);
    // Bounds of Z in 3D space
    bounds.setLow(2, 0);
    bounds.setHigh(2, 4);

    space->as<ob::SE3StateSpace>()->setBounds(bounds);
    
    si->setStateValidityChecker(boost::bind(&isStateValid, quad_name, _1));

    si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
    // This resolution value determines the rate at which the MotionValidator check the points between 2 formerly sampled points.
    // In other words, MotionValidator will check the line piece between those points and move from one to the other with small steps.
    // If it meets an obstacle, then that line piece will not be added to the RRT tree. If this value is too small, the MotionValidator will
    // take long time to finish checking a line piece; if its too big, then validator might miss a small obstacle.
    si->setStateValidityCheckingResolution(0.05);

    ob::ScopedState<ob::SE3StateSpace> robot(space);
    robot->setX(std::get<0>(robot_pos));
    robot->setY(std::get<1>(robot_pos));
    robot->setZ(std::get<2>(robot_pos));
    // If there is no valid orientation set, the sampled states will be invalid.
    robot->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

    ob::ScopedState<ob::SE3StateSpace> goal(space);
    goal->setX(std::get<0>(goal_pos));
    goal->setY(std::get<1>(goal_pos));
    goal->setZ(std::get<2>(goal_pos));
    // If there is no valid orientation set, the sampled states will be invalid.
    goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

    ROS_INFO("Manual planning for %s : start point(%.2f,%.2f,%.2f) | goal point(%.2f,%.2f,%.2f)", quad_name.c_str(),
        std::get<0>(robot_pos), std::get<1>(robot_pos), std::get<2>(robot_pos), std::get<0>(goal_pos),
        std::get<1>(goal_pos), std::get<2>(goal_pos));
    
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(robot, goal);
    return pdef;
}



// This method will set the planner's algorithm when performing state sampling - different algorithms have different approaches
// of sampling states. The function seems to have a loop of tries, if planner could not find a solution in its first try, it will
// have the chance to plan it again, until timeout counter reaches a certain value.
void WrapperPlanner::autoPlanning()
{
    int solverTimeout = 0;
    ob::StateSpacePtr space(new ob::SE3StateSpace());
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    
    ob::ProblemDefinitionPtr pdef = planner_setup(space,si);
    // OMPL library offers many planner algorithms, some are variations of some. We chose RRT* here.
    // If you want to use another planner algorithm, then you will have to add its header file.
    auto planner(std::make_shared<og::RRTstar>(si));
    planner->setProblemDefinition(pdef);
    planner->setup(); 
    
    // Some planners offer built-in terminal condition functions based on the elapsed time. However, RRT* algorithm
    // does not offer this feature; so, we added our terminal condition by checking the elapsed clock() between start of the planning
    // and some certain time at when the function is called.
    std::function<bool()> f = std::bind(&WrapperPlanner::terminalCondition, this);
    ob::PlannerTerminationCondition ptc(f);
    ob::PlannerStatus solved;// = planner->solve(ptc);

    do
    {
    	// This do-while loop will run either until a solution is found or the solverTimeout has reached a certain value 
    	// which indicates planner has failed to find a valid path to goal (If this function is called because of a collsion risk,
    	// then no valid path means using formerly planned path).

        ROS_INFO("Trying to solve for %s",quad_name.c_str());
        solved = planner->solve(ptc);
        solverTimeout++;

        if(!solved)
        {
        	// Clears the planner object
            planner->clear();

            // This block re-initializes essential planner components
            ob::ProblemDefinitionPtr pdef = planner_setup(space,si);
            auto planner(std::make_shared<og::RRTstar>(si));
            planner->setProblemDefinition(pdef);
            planner->setup();                    
            
            std::function<bool()> f = std::bind(&WrapperPlanner::terminalCondition,this);
            ob::PlannerTerminationCondition ptc(f);
        }

        if(solverTimeout == 5)
            break; // Proceed with the former plan.

    } while(!solved);

    if(solved)
    {   
        ROS_INFO("Found a solution for %s!",quad_name.c_str());
        step_acquired = true;
        ob::PathPtr p = pdef->getSolutionPath();

        if(!paths.empty())
        {
        	// If this is not the first planning of UAV, then clear old path.
        	// This line is added, because this UAV's motion was previously planned.
            uav_count--;
            paths.clear();
        }

        og::PathGeometric* path = (*p).as<og::PathGeometric> ();
        
        std::vector<ob::State*> path_states = path->getStates();
        float previousX = std::get<0>(robot_pos);
        float previousY = std::get<1>(robot_pos);
        float previousZ = std::get<2>(robot_pos);

        total_path_length = 0;
        for(unsigned int i = 0, j = 0; i < path_states.size(); i++, j++)
        {
            const ob::State* state = path_states[i];
            const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
            
            float x = se3state->getX();
            float y = se3state->getY();
            float z = se3state->getZ();

            paths.emplace_back(std::make_tuple(x, y, z));
            total_path_length += euclidean_distance(x, previousX, y, previousY, z, previousZ);

            previousX = x;
            previousY = y;
            previousZ = z;
        }

        // This code block, sends the number of waypoints in the path, to the quad_manipulator.

        std_msgs::Int32 msg;
        msg.data = path_states.size();
        waypoint_pub.publish(msg);
        all_paths[quad_name][1] = paths[0];

        /* It, then adjusts the velocity of quadrotor. This method is embraced, because more waypoints tend to have 
           short distances between them. Short distances between waypoints result in low natural speeds (computed by PID unit) 
           compared to longer distances. Therefore, we multiply this natural speed by the waypoint number.
           --------------------------------------------------------------------------- 
           To illustrate, assume the natural velocity factor is n. If the path consists of just 2 waypoints, then
           velocity factor will become 2n; and if its computed velocity is vf, the final velocity would be 2n * vf.
           On the contrary, if there are 8 waypoints in the path, velocity factor will become 8n; and if its computed velocity
           is vs, the final velocity would be 8n * vs. These two terms (2n*vf, 8n*vs) would produce roughly the same velocity magnitudes.
           Term "n" can be adjusted experimentally.
        */
    }

    // When all the UAVs have completed their planning, semaphore "sem" will be notified in order for UAVs to start moving.
    uav_count++;
    if(uav_count == 3)
        sem.notify();
}


// When the planner successfully finds a path, it will contain a number of steps to reach the ultimate goal. This function will
// publish the steps in the given path order to /move_base_simple topic. Of course, there can be paths with only 1 step.
void WrapperPlanner::publishStep()
{
    if (!paths.empty())
    {
    	// The "paths" variable of class is a vector that is treated a queue.
        std::vector< Coord3D >::iterator step_itr = paths.begin();

        // Construct the message to be published to UAV-specific /move_base_simple/goal topic
        geometry_msgs::PoseStamped step;
        step.header.stamp = ros::Time(0);
        step.header.frame_id = "world";

        step.pose.position.x = std::get<0>(*step_itr);
        step.pose.position.y = std::get<1>(*step_itr);
        step.pose.position.z = std::get<2>(*step_itr);

        step.pose.orientation.x = 0;
        step.pose.orientation.y = 0;
        step.pose.orientation.z = 0;
        step.pose.orientation.w = 1;

        step_pos = std::make_tuple(std::get<0>(*step_itr), std::get<1>(*step_itr), std::get<2>(*step_itr));
        paths.erase(step_itr);

        if(!paths.empty())
            all_paths[quad_name][1] = paths[0]; // Next element is loaded to all_paths.
        
        // This block of code is to determine the priority of UAVs to move when there is risk of collision.
        std_msgs::Float64 distance_msg;
        distance_msg.data = total_path_length;
        // Publish the message holding priority information for UAV
        rem_pub.publish(distance_msg);
        // Publish the step position
        step_cmd.publish(step);

        step_path_distance = euclidean_distance(std::get<0>(step_pos), std::get<0>(robot_pos), 
        	std::get<1>(step_pos), std::get<1>(robot_pos), std::get<2>(step_pos), std::get<2>(robot_pos));
    }
    else // If this function called the first time and enters here, then there might be error in the planning or no valid path is found.
        ROS_INFO("Either %s arrived or no path found!",quad_name.c_str());
}


// The routine that threads will run independently (in detached state). It just creates a pointer to WrapperPlanner 
// instance to initialize essential ROS communication with Gazebo and OMPL components.
void initiate_UAV(ros::NodeHandle& nh, const std::string& uav_name)
{
    WrapperPlanner* planner = new WrapperPlanner(nh,uav_name);
}


// This callback routine will handle the messages that is published to /gazebo/model_states topic to
// load any models that is added to simulation (by loading it will be treated as pure obstacle or UAV etc.)
void loadModels(ros::NodeHandle& nh, const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    size_t model_count = msg->name.size();
    if (model_count > total_model_count)
    {
        for (unsigned int i = 1; i < model_count; i++)
        {
            size_t found = msg->name[i].find("uav");
            if(found!=std::string::npos)
            {
            	// The launch files initiate a spawning sequence of UAVs with their names starting the prefix "uav".
            	// Therefore, if the current model has "uav" at the front of its name, then it will represent a UAV.
                std::thread *uav_thread = new std::thread(&initiate_UAV, std::ref(nh), msg->name[i]);
                uav_thread->detach();
                thread_refs.push_back(uav_thread);
            }
            else // Otherwise, it will represent pure obstacle (dynamic or static) and be added to obstacle_list.
                obstacle_list.push_back(std::make_tuple(msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z));
        }

        // Set the total_model_count in order to prevent unnecessary computation of this callback routine if there are no
        // new models added to simulation.
        total_model_count = model_count;
    }
}
