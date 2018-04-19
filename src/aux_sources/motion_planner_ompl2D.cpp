#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>


#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/Done.h>
#include <hector_uav_msgs/Vector.h>
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
#include <map>

#include <mutex>
#include <thread>
#include <condition_variable>

#include "../include/Semaphore.h"

#define SEQUENTIAL_PLANNER_FACTOR 2.5

namespace ob = ompl::base;
namespace og = ompl::geometric;

std::vector<std::pair<float, float> > obstacle_list;
std::map<std::string, std::vector<std::pair<float, float> > > all_paths;
std::map<std::string, std::pair<float,float> > uav_list;

std::vector<std::thread*> thread_refs;
std::mutex mtx;

std::condition_variable planning_cnd; 
Semaphore sem(0);
size_t total_model_count = 2;
float waiting_duration = 1.5;
int uav_count = 0;

bool notObstacle(float sx,float sy){
    std::vector<std::pair<float, float> >::iterator obs = obstacle_list.begin();
    for(; obs != obstacle_list.end(); obs++)
    {
        // obstacles are unit spheres (unit circles when projected on 2D) with 0.5 m as their radius
        // inflation radius (the radius that wraps the quadrotor properly) is at least 0.5 m
        
        float result = pow(obs->first - sx, 2) + pow(obs->second - sy, 2) - 2.0;
        if (result < 0) // The state falls into either an inner region of an obstacle or an inflation area
            return false;
    }

    std::map<std::string, std::pair<float,float> >::iterator it = uav_list.begin();
    for(;it!=uav_list.end();it++){
        
        float result = pow(it->second.first - sx,2) + pow(it->second.second - sy,2) - 1.2;
        if(result<0)
            return false;
    }
    
    /*if(!all_paths.empty()){
        std::map<std::string, std::vector<std::pair<float, float> > >::iterator map_itr = all_paths.begin();
        float a_square = 0.49; //Elipse's short radius
        for(;map_itr!=all_paths.end();map_itr++){
            //ROS_INFO("Path of %s is registered!",(map_itr->first).c_str());
            for(unsigned int i=0;i<(map_itr->second).size() - 1;i++){
                std::pair<float, float> next_step, previous_step;
                previous_step = (map_itr->second)[i];
                next_step = (map_itr->second)[i+1];
                float distance_between_step_goals = pow(next_step.first - previous_step.first, 2) + pow(next_step.second - previous_step.second, 2);
                float b_square =pow(sqrt(distance_between_step_goals) + 0.5, 2); //Ellipse's long radius
                float ellipse_center_x =  (next_step.first + previous_step.first) / 2;
                float ellipse_center_y =  (next_step.second + previous_step.second) / 2;
                float result = (b_square * pow(sx - ellipse_center_x,2)) + (a_square * pow(sy - ellipse_center_y,2)) - (a_square * b_square);
                if(result < 0)                    
                    return false;
            }     
        }
    }*/
    return true;
}

bool isStateValid(const std::string& quad_name,const ob::State* state)
{
    const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();

    // Check state if its in any of the occupied obstacle areas or the ones inflated with robot's radius
    float sx = se2state->getX();
    float sy = se2state->getY();
    return notObstacle(sx,sy);
}
void printPaths(){
    std::map<std::string, std::vector<std::pair<float, float> > >::iterator map_itr = all_paths.begin();
    for(;map_itr!=all_paths.end();map_itr++){
        std::cout<<map_itr->first<<" PATH"<<std::endl;
        for(unsigned int i=0;i<(map_itr->second).size();i++)
            std::cout<<"("<<map_itr->second[i].first<<","<<map_itr->second[i].second<<")"<<std::endl;
        std::cout<<"---------------------"<<std::endl;
    }
}


class WrapperPlanner{
    public:
      ros::NodeHandle node;
      ros::Publisher samp_pub, step_cmd,rem_pub,arrived_pub,waypoint_pub;
      ros::Subscriber done_sub, mainGoal_sub, rloc_sub, update_goal_sub;
      std::vector< std::pair<float,float> > paths;
      size_t p_total_model_count;
      
      std::pair<float, float> robot_pos, goal_pos,step_pos;

      std::string quad_name;
      clock_t timeBeforePlanning;
      bool erase_flag,step_acquired;
      float total_path_length,step_path_distance;

      WrapperPlanner(ros::NodeHandle _nh, std::string uav_name){
            node = _nh;
            quad_name = uav_name;
            done_sub = node.subscribe<hector_uav_msgs::Done>("/"+quad_name+"/Done", 10, boost::bind(&WrapperPlanner::StepDone,this,quad_name,_1));
            mainGoal_sub = node.subscribe<hector_uav_msgs::Vector>("/"+quad_name+"/actual_uav_goal", 1, boost::bind(&WrapperPlanner::MainGoal,this,quad_name,_1));
            /* ---------------- It might be useful when further plannings are considered as meaningfully costly. ---------------*/
            update_goal_sub = node.subscribe<std_msgs::Bool>("/"+quad_name+"/update_goal",1, boost::bind(&WrapperPlanner::UpdateGoal,this,quad_name,_1));
            /* -----------------------------------------------------------------------------------------------------------------*/
            

            step_cmd = node.advertise<geometry_msgs::PoseStamped>("/"+quad_name+"/move_base_simple/goal", 1);
            samp_pub = node.advertise<hector_uav_msgs::Vector>("/"+quad_name+"/sampled_point", 1000);
            rem_pub = node.advertise<std_msgs::Float64>("/"+quad_name+"/remaining_step",1);
            arrived_pub = node.advertise<std_msgs::String>("/"+quad_name+"/arrival",1);
            waypoint_pub = node.advertise<std_msgs::Int32>("/"+quad_name+"/path_number",1);

            timeBeforePlanning = clock();
            all_paths[quad_name] = std::vector<std::pair<float, float> >(2);
            rloc_sub = node.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10,boost::bind(&WrapperPlanner::rloc_callback,this,quad_name,_1));
            
            p_total_model_count = 2;
            erase_flag = false;
            total_path_length = 0;
            step_path_distance = 0;
            step_acquired = false;
      }
      ~WrapperPlanner() {}
      void MainGoal(const std::string& robot_frame, const hector_uav_msgs::Vector::ConstPtr& goal)
      {
            goal_pos.first = goal->x;
            goal_pos.second = goal->y;
            
            mtx.lock();
            autoPlanning();
            mtx.unlock();
            
           
            sem.wait();
            publishStep();
            sem.notify();
            /*planning_cnd.notify_one();
            mtx.unlock();*/
      }
      void StepDone(const std::string& robot_frame,const hector_uav_msgs::Done::ConstPtr& msg)
      {
            mtx.lock();
            if (!paths.empty()){
                all_paths[quad_name][1] = paths[0];
                total_path_length -= step_path_distance;
                
                ROS_INFO("%s total_path_length = %.2f",robot_frame.c_str(),total_path_length);
                publishStep();
                //planning_cnd.notify_one();
            }
            else{
                    all_paths.erase(quad_name);
                    erase_flag = true;
                    ROS_INFO("%s arrived!",robot_frame.c_str());
                    std_msgs::String msg;
                    msg.data = "SUCCESS";
                    arrived_pub.publish(msg);

                    //Update current position of robot as an obstacle.
                    uav_list[robot_frame] = robot_pos;
                    ROS_INFO("%s position (%.2f,%.2f) is added as an obstacle",robot_frame.c_str(),robot_pos.first,robot_pos.second);
            }
            mtx.unlock();
      }

      void UpdateGoal(const std::string& robot_frame,const std_msgs::Bool::ConstPtr& msg){
            //A stimuli that dictates to change our path has arrived. Check that if goal is in a safe area.
            
            ros::Duration(1.0).sleep();
            //This sleep is necessary for UAV to stop its movement.
            mtx.lock();
            if(notObstacle(goal_pos.first,goal_pos.second) == false){
                std::srand(std::time(0));
                while(1){
                        
                        float x_addee,y_addee;
                        float temp_goalX,temp_goalY;
                        x_addee = (float)((std::rand()%200) - 100) / 100;
                        y_addee = (float)((std::rand()%200) - 100) / 100;
                        temp_goalX = goal_pos.first + x_addee;
                        temp_goalY = goal_pos.second + y_addee;
                        ROS_INFO("Checking %.2f,%.2f as possible goal",temp_goalX,temp_goalY);
                        if(notObstacle(temp_goalX,temp_goalY))
                        {
                            goal_pos.first = temp_goalX;
                            goal_pos.second = temp_goalY;
                            ROS_INFO("New goal_point is (%.2f,%.2f)",goal_pos.first,goal_pos.second);
                            autoPlanning(); //Plan again.
                            break;
                        }
                }
            }
            else //No need to update goal. Check left path considering finished UAVs
                autoPlanning();
            mtx.unlock();
      }
      
      void rloc_callback(const std::string& robot_frame, const gazebo_msgs::ModelStates::ConstPtr& msg){
            
            size_t model_count = msg->name.size();
            for (unsigned int i = 1; i < model_count; i++){
                size_t found = msg->name[i].find(quad_name);
                if(found!=std::string::npos){
                    robot_pos.first = msg->pose[i].position.x;
                    robot_pos.second = msg->pose[i].position.y;
                    if(!step_acquired)
                        step_pos = robot_pos;
                    float current_distance_to_step = euclidean_distance(step_pos.first,robot_pos.first,step_pos.second,robot_pos.second); 
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

      float euclidean_distance(float x1,float y1,float x2,float y2){

        return sqrt(pow(x1 - y1, 2) + pow(x2 - y2, 2));
      }
      bool terminalCondition()
      {
            clock_t checkTime = clock() - timeBeforePlanning;
            float seconds_passed = ((float)checkTime) / CLOCKS_PER_SEC;
            //ROS_INFO("%s seconds_passed: %f ",quad_name.c_str(),seconds_passed);
            if (seconds_passed >= waiting_duration)
            {
                ROS_INFO("Termination achieved.");
                waiting_duration += SEQUENTIAL_PLANNER_FACTOR;
                ROS_INFO("waiting_duration for %s is %.2f",quad_name.c_str(),waiting_duration);
                return true;

            }
            else
                return false;
      }

      ob::ProblemDefinitionPtr planner_setup(ob::StateSpacePtr& space,ob::SpaceInformationPtr& si){

            ob::RealVectorBounds bounds(2);
            bounds.setLow(-10);
            bounds.setHigh(10);
            space->as<ob::SE2StateSpace>()->setBounds(bounds);
            
            si->setStateValidityChecker(boost::bind(&isStateValid,quad_name,_1));

            si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
            si->setStateValidityCheckingResolution(0.05);

            ob::ScopedState<ob::SE2StateSpace> robot(space);
            robot->setX(robot_pos.first);
            robot->setY(robot_pos.second);
            robot->setYaw(0.0);

            ob::ScopedState<ob::SE2StateSpace> goal(space);
            goal->setX(goal_pos.first);
            goal->setY(goal_pos.second);
            goal->setYaw(0.0);
            ROS_INFO("Manual planning for %s : start point(%.2f,%.2f) | goal point(%.2f,%.2f)",quad_name.c_str(),robot_pos.first,robot_pos.second,goal_pos.first,goal_pos.second);
            
            ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
            pdef->setStartAndGoalStates(robot, goal);
            return pdef;
      }

      void autoPlanning()
      {
            //std::unique_lock<std::mutex> lock(mtx); //It gets automatically unlocked when goes out of scope.
            int solverTimeout = 0;
            ob::StateSpacePtr space(new ob::SE2StateSpace());
            ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
            
            ob::ProblemDefinitionPtr pdef = planner_setup(space,si);
            auto planner(std::make_shared<og::RRTstar>(si));
            planner->setProblemDefinition(pdef);
            planner->setup(); 

            
            std::function<bool()> f = std::bind(&WrapperPlanner::terminalCondition,this);
            ob::PlannerTerminationCondition ptc(f);
            ob::PlannerStatus solved;// = planner->solve(ptc);

            do
            {
                ROS_INFO("Trying to solve for %s",quad_name.c_str());
                solved = planner->solve(ptc);
                solverTimeout++;
                if(!solved){
                    //mtx.unlock();
                    //planning_cnd.wait(lock);
                    planner->clear();

                    ob::ProblemDefinitionPtr pdef = planner_setup(space,si);
                    auto planner(std::make_shared<og::RRTstar>(si));
                    planner->setProblemDefinition(pdef);
                    planner->setup();                    
                    
                    std::function<bool()> f = std::bind(&WrapperPlanner::terminalCondition,this);
                    ob::PlannerTerminationCondition ptc(f);
                }
                if(solverTimeout == 5)
                    break; //Proceed with the former plan.
            }while(!solved);
            if(solved){   
                ROS_INFO("Found a solution for %s!",quad_name.c_str());
                step_acquired = true;
                ob::PathPtr p = pdef->getSolutionPath();

                if(!paths.empty()){//If this is not the first planning of UAV, then clear old path.
                    uav_count--; // This line is added, because this UAV's motion was previously planned.
                    paths.clear();
                }
                og::PathGeometric* path = (*p).as<og::PathGeometric> ();
                
                
                std::vector<ob::State*> path_states = path->getStates();
                float previousX = robot_pos.first;
                float previousY = robot_pos.second;
                total_path_length = 0;
                for(unsigned int i=0,j=0;i<path_states.size();i++,j++){
                    const ob::State* state = path_states[i];
                    const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();
                    float x = se2state->getX();
                    float y = se2state->getY();

                    hector_uav_msgs::Vector p;
                    p.x = x;
                    p.y = y;
                    if(j==0)
                        p.z = 99;
                    samp_pub.publish(p);
                    paths.emplace_back(std::make_pair(x,y));

                    total_path_length += euclidean_distance(x,previousX,y,previousY);

                    previousX = x;
                    previousY = y;
                }
                std_msgs::Int32 msg;
                msg.data = path_states.size();
                waypoint_pub.publish(msg);
                all_paths[quad_name][1] = paths[0];

                //all_paths[quad_name]= paths;
            }
            uav_count++;
            if(uav_count == 3)
                sem.notify();
      }

      void publishStep()
      {
            if (!paths.empty())
            {
                std::vector< std::pair<float,float> >::iterator step_itr = paths.begin();
                        
                geometry_msgs::PoseStamped step;
                step.header.stamp = ros::Time(0);
                step.header.frame_id = "world";

                step.pose.position.x = step_itr->first;
                step.pose.position.y = step_itr->second;
                

                step.pose.position.z = 0.5;
                step.pose.orientation.x = 0;
                step.pose.orientation.y = 0;
                step.pose.orientation.z = 0;
                step.pose.orientation.w = 1;
                step_pos = std::make_pair(step_itr->first,step_itr->second);
                paths.erase(step_itr);
                if(!paths.empty())
                    all_paths[quad_name][1] = paths[0]; //Next element is loaded to all_paths.
                
                std_msgs::Float64 distance_msg;
                distance_msg.data = total_path_length;
               
               
                rem_pub.publish(distance_msg);

                step_cmd.publish(step);

                step_path_distance = euclidean_distance(step_pos.first,robot_pos.first,step_pos.second,robot_pos.second);
             } 
            else
                ROS_INFO("Either %s arrived or no path found!",quad_name.c_str());
      }


};


void initiate_UAV(ros::NodeHandle& nh, const std::string& uav_name){
    WrapperPlanner* planner = new WrapperPlanner(nh,uav_name);
}

void loadModels(ros::NodeHandle& nh, const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    size_t model_count = msg->name.size();
    if (model_count > total_model_count)
    {
        for (unsigned int i = 1; i < model_count; i++){
            size_t found = msg->name[i].find("uav");
            if(found!=std::string::npos){
                std::thread *uav_thread = new std::thread(&initiate_UAV, std::ref(nh),msg->name[i]);
                uav_thread->detach();
                thread_refs.push_back(uav_thread);
            }
            else
                obstacle_list.push_back(std::make_pair(msg->pose[i].position.x, msg->pose[i].position.y));
        }
        total_model_count = model_count;
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_planner_ompl");
    ros::NodeHandle node;

    ros::Subscriber model_sub = node.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10,
                                boost::bind(&loadModels,std::ref(node),_1));
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
