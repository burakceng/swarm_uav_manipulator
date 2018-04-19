#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <control_msgs/PidState.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>

#include <hector_uav_msgs/AttitudeCommand.h>
#include <hector_uav_msgs/YawrateCommand.h>
#include <hector_uav_msgs/ThrustCommand.h>

#define GRAVITY 		  9.8065
#define MASS 			  1.477000
#define LOAD_FACTOR_LIMIT 1.5

void x_callBack(const control_msgs::PidStateConstPtr& state);
void y_callBack(const control_msgs::PidStateConstPtr& state);
void z_callBack(const control_msgs::PidStateConstPtr& state);
void cmd_velCommandCallback(const geometry_msgs::TwistConstPtr &command);
void groundCallback(const geometry_msgs::PoseStampedConstPtr &ground_data);

class QuadVelocity {
	public:
		struct
		{
		   control_toolbox::Pid x, y, z, yaw;
		} pid_;

		double velocity_x, velocity_y, velocity_z;
		double command_lin_x, command_lin_y, command_lin_z, command_ang_z;
		double qX, qY, qZ, qW;

		ros::NodeHandle controller_node;
		ros::Subscriber x_output_sub, y_output_sub, z_output_sub, cmd_vel_sub, ground_sub;
		ros::Publisher attitude_pub, yawrate_pub, thrust_pub;

		QuadVelocity(ros::NodeHandle _node) 
		{
			controller_node = _node;
			x_output_sub = controller_node.subscribe("/controller/velocity/x/state", 10, &x_callBack);
			y_output_sub = controller_node.subscribe("/controller/velocity/y/state", 10, &y_callBack);
			z_output_sub = controller_node.subscribe("/controller/velocity/z/state", 10, &z_callBack);
			cmd_vel_sub = controller_node.subscribe("/cmd_vel", 10, &cmd_velCommandCallback);
			ground_sub = controller_node.subscribe("/ground_truth_to_tf/pose", 10, &groundCallback);

			attitude_pub = controller_node.advertise<hector_uav_msgs::AttitudeCommand>("/command/attitude",1);
			yawrate_pub = controller_node.advertise<hector_uav_msgs::YawrateCommand>("/command/yawrate",1);
			thrust_pub = controller_node.advertise<hector_uav_msgs::ThrustCommand>("/command/thrust",1);

			pid_.x.init(ros::NodeHandle(controller_node, "x"));
			pid_.y.init(ros::NodeHandle(controller_node, "y"));
			pid_.z.init(ros::NodeHandle(controller_node, "z"));
			pid_.yaw.init(ros::NodeHandle(controller_node, "yaw"));
		}

		~QuadVelocity() {}
};

QuadVelocity* quad_vel_controller;

void x_callBack(const control_msgs::PidStateConstPtr& state) { quad_vel_controller->velocity_x = state->output; }
void y_callBack(const control_msgs::PidStateConstPtr& state) { quad_vel_controller->velocity_y = state->output; }
void z_callBack(const control_msgs::PidStateConstPtr& state) { quad_vel_controller->velocity_z = state->output; }	

void cmd_velCommandCallback(const geometry_msgs::TwistConstPtr &command)
{
	quad_vel_controller->command_lin_x = command->linear.x;
	quad_vel_controller->command_lin_y = command->linear.y;
	quad_vel_controller->command_lin_z = command->linear.z;
	quad_vel_controller->command_ang_z = command->angular.z;
}

void groundCallback(const geometry_msgs::PoseStampedConstPtr &ground_data)
{
	quad_vel_controller->qX = ground_data->pose.orientation.x;
	quad_vel_controller->qY = ground_data->pose.orientation.y;
	quad_vel_controller->qZ = ground_data->pose.orientation.z;
	quad_vel_controller->qW = ground_data->pose.orientation.w;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "quad_velocity_controller");
	ros::NodeHandle node;

	quad_vel_controller = new QuadVelocity(node);

	ros::Duration period(0.1);
	ros::Rate r(10);

	while(quad_vel_controller->controller_node.ok())
	{
		tf::Quaternion q(quad_vel_controller->qX, quad_vel_controller->qY, 
			quad_vel_controller->qZ, quad_vel_controller->qW);
		double yaw = tf::getYaw(q), sin_yaw, cos_yaw;
		sincos(yaw, &sin_yaw, &cos_yaw);
		
		double w_square = quad_vel_controller->qW * quad_vel_controller->qW;
		double x_square = quad_vel_controller->qX * quad_vel_controller->qX;
		double y_square = quad_vel_controller->qY * quad_vel_controller->qY;
		double z_square = quad_vel_controller->qZ * quad_vel_controller->qZ;
		double load_factor = 1. / ( w_square - x_square - y_square + z_square );
		if(load_factor > LOAD_FACTOR_LIMIT) load_factor = LOAD_FACTOR_LIMIT;
		
		double acceleration_commandX = quad_vel_controller->pid_.x.computeCommand(quad_vel_controller->command_lin_x - quad_vel_controller->velocity_x, 
			period);
		double acceleration_commandY = quad_vel_controller->pid_.y.computeCommand(quad_vel_controller->command_lin_y - quad_vel_controller->velocity_y, 
			period);
		double acceleration_commandZ = quad_vel_controller->pid_.z.computeCommand(quad_vel_controller->command_lin_z - quad_vel_controller->velocity_z, 
			period);
	
		double acceleration_commandX_base_stabilized = cos_yaw * acceleration_commandX + sin_yaw * acceleration_commandY;
		double acceleration_commandY_base_stabilized = -sin_yaw * acceleration_commandX  + cos_yaw * acceleration_commandY;
		double acceleration_commandZ_base_stabilized = acceleration_commandZ;

		hector_uav_msgs::AttitudeCommand attitude_control;
		hector_uav_msgs::YawrateCommand yawrate_control;
		hector_uav_msgs::ThrustCommand thrust_control;

		attitude_control.roll    = -asin(std::min(std::max(acceleration_commandY_base_stabilized / GRAVITY, -1.0), 1.0));
		attitude_control.pitch   =  asin(std::min(std::max(acceleration_commandX_base_stabilized / GRAVITY, -1.0), 1.0));
		yawrate_control.turnrate = quad_vel_controller->command_ang_z;
		thrust_control.thrust    = MASS * ((acceleration_commandZ_base_stabilized - GRAVITY) * load_factor + GRAVITY);
		
		quad_vel_controller->attitude_pub.publish(attitude_control);
		quad_vel_controller->yawrate_pub.publish(yawrate_control);
		quad_vel_controller->thrust_pub.publish(thrust_control);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}