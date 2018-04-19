#include "../include/swarm_uav_manipulator/QuadController.h"

// Recall that, static member variables of a class must be properly initialized/constructed outside of the class.
std::map<std::string, Wrapper::QuadController*> Wrapper::activeQuadrotors;

// The callbacks implemented in Callbacks.cpp will refer to this variable when computing velocity commands.
struct PID pid_;

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "quad_manipulator");
	ros::NodeHandle root_node;

	// Initialize the pid_ parameters (proportional - integral - derivative gain)
	pid_.x.init(ros::NodeHandle(root_node, "x"));
	pid_.y.init(ros::NodeHandle(root_node, "y"));
	pid_.z.init(ros::NodeHandle(root_node, "z"));
	pid_.yaw.init(ros::NodeHandle(root_node, "yaw"));
	
    Wrapper::QuadController* new_quad_controller1 = new Wrapper::QuadController(root_node,"uav1");
    Wrapper::QuadController* new_quad_controller2 = new Wrapper::QuadController(root_node,"uav2");
    Wrapper::QuadController* new_quad_controller3 = new Wrapper::QuadController(root_node,"uav3");
    Wrapper::QuadController* new_quad_controller4 = new Wrapper::QuadController(root_node,"uav4");

	ros::Rate rate(1000);
	while(ros::ok()) {
		ros::spinOnce();
	}

	return 0;
}
