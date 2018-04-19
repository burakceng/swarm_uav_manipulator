#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include "Eigen/Dense"

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <limits.h>
#define UAV_COUNT 3
size_t total_model_count = 2;
ros::Publisher uavs_pub[UAV_COUNT];
ros::Publisher update_uav_path[UAV_COUNT];

using Eigen::MatrixXf;
using Eigen::MatrixXi;

MatrixXf uavPoseMatrix(3,3);
/* Distance and Check matrices are lower triangular matrices. This is designed in order to avoid duplicates. */
MatrixXf uavDistanceMatrix(3,3);
MatrixXi checkMatrix(3,3);
/* ------------------------------------------------------------------------------------------------------- */
ros::Subscriber uavs_remaining_sub[UAV_COUNT];
ros::Subscriber uavs_arrival_sub[UAV_COUNT];
std::map<std::string, float> remaining;
std::map<std::string, bool> arrived;

/* Receives the remaining path length to goal as float */
void remainingStepCallback(const std::string& _qname, const std_msgs::Float64::ConstPtr& msg)
{ 
    remaining[_qname] = msg->data; 
}

/* When a UAV arrives, this callback receives the SUCCESS message. The messages are sent from QuadController objects.
   If more state is desired to be added, it can be implemented there. E.g. a UAV may lost communication, and just before the comm
   is lost, UAV may send a LOST message.
*/   
void arrivalCallback(const std::string& _qname, const std_msgs::String::ConstPtr& msg){
    if(msg->data.compare("SUCCESS") == 0){
        //ROS_INFO("qname = %s",_qname.c_str());
        arrived[_qname] = true;
    }
}

// Only checks collision risk, and sends slowing data for lower priority UAVs.
void check_collision(){
    std_msgs::Bool uavs[UAV_COUNT];
    for(int i=0;i<UAV_COUNT;i++)
        uavs[i].data = false;
    
    for(int i=1;i<UAV_COUNT;i++){
        for(int j=0;j<i;j++){
            float collision_check_parameter = 2.0;
            std::string uav_i = "uav", uav_j = "uav";
            uav_i.push_back(i + 49);
            uav_j.push_back(j + 49);
            if(arrived[uav_i] || arrived[uav_j]) //One of them has arrived
                collision_check_parameter = 1.5; //So reduce the collision check_parameter
            if(uavDistanceMatrix(i,j) < collision_check_parameter){ //Check whether they have the possibility of "near miss"
                if (remaining[uav_i] <= remaining[uav_j])
                    uavs[j].data = true;
                else
                    uavs[i].data = true;
                   
            }
        }
    }
    
    for(int i=0;i<UAV_COUNT;i++)
        uavs_pub[i].publish(uavs[i]);
}

/*
    Checks the collision risk. In addition, if any UAV is stopped, it is added as the static obstacle. 
    So a new planning might be necessary. Sends the corresponding message for replanning.
*/
void check_collision_with_replanning(){

	std_msgs::Bool uavs[UAV_COUNT];
	for(int i=0;i<UAV_COUNT;i++)
        uavs[i].data = false;
	
    for(int i=1;i<UAV_COUNT;i++){
        for(int j=0;j<i;j++){
            float collision_check_parameter = 2.0;
            std::string uav_i = "uav", uav_j = "uav";
            uav_i.push_back(i + 49);
            uav_j.push_back(j + 49);
            if(arrived[uav_i] || arrived[uav_j]){ //One of them has arrived
                collision_check_parameter = 4.0; //This is needed to update other uavs' paths safely.
                std_msgs::Bool msg;
                msg.data = true;

                /* ------------------------------ This block contains the mechanism of replanning stimuli, --------------- *
                 * ------------------------------  considering the other UAVs final positions ---------------------------- */
                if(arrived[uav_i] && !arrived[uav_j]){
                    remaining[uav_i] = INT_MAX;
                    
                    if(!checkMatrix(i,j)){
                        checkMatrix(i,j) = 1;
                        ROS_INFO("%s is arrived. Update plan of %s",uav_i.c_str(),uav_j.c_str());
                        update_uav_path[j].publish(msg);
                        uavs[j].data = true;
                    }
                }
                else if(arrived[uav_j] && !arrived[uav_i]){
                    remaining[uav_j] = INT_MAX;
                    if(!checkMatrix(i,j)){
                        checkMatrix(i,j) = 1;
                        ROS_INFO("%s is arrived. Update plan of %s",uav_j.c_str(),uav_i.c_str());
                        update_uav_path[i].publish(msg);
                        uavs[j].data = true;
                    }
                }

                /*-------------------------------------------------------------------------------------------------------- */
            }
            if(uavDistanceMatrix(i,j) < collision_check_parameter){ //Check whether they have the possibility of "near miss"
                
                if (remaining[uav_i] <= remaining[uav_j])
                    uavs[j].data = true;
                else
                    uavs[i].data = true;
            }
        }
    }
	// Publish if_slowing messages.
    for(int i=0;i<UAV_COUNT;i++)
        uavs_pub[i].publish(uavs[i]);
}

/* This callback function, not only records the current positions of UAVs,
   but also calculates distances of each of them in pairs.
*/
void loadUAVs(const gazebo_msgs::ModelStates::ConstPtr& msg)
{

    //uavPoseMatrix accumulates the origins of UAVs in real-time.
	size_t model_count = msg->name.size();
	for (unsigned int i = 1; i < model_count; i++){
        size_t found = msg->name[i].find("uav");
        if(found!=std::string::npos){
            if(msg->name[i].compare("uav1") == 0){
                uavPoseMatrix(0,0) = msg->pose[i].position.x;
                uavPoseMatrix(0,1) = msg->pose[i].position.y;
                uavPoseMatrix(0,2) = 0.5; //Will be scaled to 3D later.
            }
            else if(msg->name[i].compare("uav2") == 0){
                uavPoseMatrix(1,0) = msg->pose[i].position.x;
                uavPoseMatrix(1,1) = msg->pose[i].position.y;
                uavPoseMatrix(1,2) = 0.5; //Will be scaled to 3D later.
            }
            else if(msg->name[i].compare("uav3") == 0){
                uavPoseMatrix(2,0) = msg->pose[i].position.x;
                uavPoseMatrix(2,1) = msg->pose[i].position.y;
                uavPoseMatrix(2,2) = 0.5; //Will be scaled to 3D later.                      
            }
        }
    }
    
    //uavDistanceMatrix calculates the distances between UAVs in pairs.
    for(int i=1;i<UAV_COUNT;i++)
        for(int j=0;j<i;j++)
            uavDistanceMatrix(i,j) = sqrt((uavPoseMatrix.row(i) - uavPoseMatrix.row(j)).squaredNorm());
         

    check_collision_with_replanning();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "collision_checker");
	ros::NodeHandle node;
	ros::Subscriber model_sub = node.subscribe("/gazebo/model_states", 10, &loadUAVs);
    //Euclidean distances to themselves always equal to 0.
    uavDistanceMatrix(0,0) = 0; 
    uavDistanceMatrix(1,1) = 0;
    uavDistanceMatrix(2,2) = 0;

    for(int i=0;i<UAV_COUNT;i++){
        for(int j=0;j<UAV_COUNT;j++)
            checkMatrix(i,j) = 0;
    }

    uavs_pub[0] = node.advertise<std_msgs::Bool>("/uav1/check_collision",1);
    uavs_pub[1] = node.advertise<std_msgs::Bool>("/uav2/check_collision",1);
    uavs_pub[2] = node.advertise<std_msgs::Bool>("/uav3/check_collision",1);

    update_uav_path[0] = node.advertise<std_msgs::Bool>("/uav1/update_goal",1);
    update_uav_path[1] = node.advertise<std_msgs::Bool>("/uav2/update_goal",1);
    update_uav_path[2] = node.advertise<std_msgs::Bool>("/uav3/update_goal",1);

    uavs_remaining_sub[0] = node.subscribe<std_msgs::Float64>("/uav1/remaining_step", 1, boost::bind(&remainingStepCallback, "uav1", _1));
    uavs_remaining_sub[1] = node.subscribe<std_msgs::Float64>("/uav2/remaining_step", 1, boost::bind(&remainingStepCallback, "uav2", _1));
    uavs_remaining_sub[2] = node.subscribe<std_msgs::Float64>("/uav3/remaining_step", 1, boost::bind(&remainingStepCallback, "uav3", _1));
	
    uavs_arrival_sub[0] = node.subscribe<std_msgs::String>("/uav1/arrival", 1, boost::bind(&arrivalCallback, "uav1", _1));
    uavs_arrival_sub[1] = node.subscribe<std_msgs::String>("/uav2/arrival", 1, boost::bind(&arrivalCallback, "uav2", _1));
    uavs_arrival_sub[2] = node.subscribe<std_msgs::String>("/uav3/arrival", 1, boost::bind(&arrivalCallback, "uav3", _1));
    

    ros::spin();
	return 0;
}
