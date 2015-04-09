/*
 * \dc_motor_controller.cpp
 * \controls the motor based on PID
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date February 14, 2015
 */

#include "actuation_module/dc_motor_controller.h"

dc_motor_controller::dc_motor_controller(){
	//the main node handle
	ros::NodeHandle nh;

	//grab the parameters
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param<double>("time_step_s", time_step_s, 0.01);
	private_node_handle_.param<int>("ticks_per_rev", ticks_per_rev, 3200);
	private_node_handle_.param<double>("wheel_radius_m", wheel_radius_m, 0.0619125);
	private_node_handle_.param<double>("max_vel_mps", max_vel_mps, 1.0);
	private_node_handle_.param<double>("min_vel_mps", min_vel_mps, -1.0);
	private_node_handle_.param<double>("timeout_s", timeout_s, 1.0);
	private_node_handle_.param<std::string>("eqep_path", eqep_path, "/sys/devices/ocp.3/48304000.epwmss/48304180.eqep");
	
	//initialize the publishers and subscribers
	node_name = ros::this_node::getName();
	std::string joint(node_name + "/joint");
	std::string feedback(node_name + "/feedback");
	std::string command(node_name + "/command");

	joint_pub = nh.advertise<sensor_msgs::JointState>(joint, 1000);
	feedback_pub = nh.advertise<oddbot_msgs::ActuationFeedback>(feedback, 1000);
	command_sub = nh.subscribe(command, 1000, &dc_motor_controller::update_controller, this);
        ROS_INFO("SUBS AND PUBS INITIALIZED!");	
	//intialize state of the controller
	// desired control 
	des_ang_pos_rad = 0.0;
	des_vel_mps = 0.0;

	//initialize watch dog 
	timeout_time_s = ros::Time::now().toSec();

	// current control
	cur_vel_mps = 0.0;
	cur_ang_vel_rps = 0.0;
	cur_pos_m = 0.0;
	cur_ang_pos_rad = 0.0;
	cur_cur_amp = 0.0;
	
	ROS_INFO("DC_MOTOR_CONTROLLER_INIT");
}

/**
 * A function to generate a value to set the motor
 */
void dc_motor_controller::update_motor(float * setMotor, int deltaEncoder_ticks){
	static int prev_deltaEncoder_ticks = 0;	

	//reset the motor command value
	*setMotor = 0;
	
	//check for the special condition
	if(deltaEncoder_ticks == -1 && prev_deltaEncoder_ticks == -1)
	{
		deltaEncoder_ticks = 0;
	}

	//calculate angular position
	float deltaAngPosition_rad = (((float)deltaEncoder_ticks) / this->ticks_per_rev) * (2 * M_PI);
	this->cur_ang_pos_rad += deltaAngPosition_rad;

	// calculate the position
	float deltaPosition_m =  deltaAngPosition_rad * this->wheel_radius_m; 
	this->cur_pos_m += deltaPosition_m;
	prev_deltaEncoder_ticks = deltaEncoder_ticks;
	
	// calculate the velocity
	this->cur_vel_mps = deltaPosition_m/this->time_step_s;    
	this->cur_ang_vel_rps = deltaAngPosition_rad/this->time_step_s;;
	
        //calcuate output
        this->getMotorCommand(setMotor);  
}

void dc_motor_controller::update_controller(const oddbot_msgs::ActuationCommand::ConstPtr& msg) {
	ROS_INFO("RECEIVED");
	//update the timeout for time the message was received
	ros::Time timeout_time = ros::Time::now() + ros::Duration(this->timeout_s);
	this->timeout_time_s = timeout_time.toSec();
	this->des_vel_mps = msg->des_ctrl;
	this->des_ang_pos_rad = msg->des_ctrl;
}

void dc_motor_controller::update_feedback(){
	
	//publish the joint state
	sensor_msgs::JointState joint_msg;
	joint_msg.header.stamp = ros::Time::now();
	joint_msg.name.push_back(this->node_name);
	joint_msg.position.push_back(this->cur_ang_pos_rad);
	joint_msg.velocity.push_back(this->cur_ang_vel_rps);
	joint_pub.publish(joint_msg);

	//publish all of the feedback
	oddbot_msgs::ActuationFeedback fdbk_msg;
	fdbk_msg.header.stamp = ros::Time::now();
	fdbk_msg.item.push_back("cur_pos_m");
	fdbk_msg.value.push_back(this->cur_pos_m);
	fdbk_msg.item.push_back("cur_vel_mps");
	fdbk_msg.value.push_back(this->cur_vel_mps);
	fdbk_msg.item.push_back("wheel_radius_m");
	fdbk_msg.value.push_back(this->wheel_radius_m);
	fdbk_msg.item.push_back("max_vel_mps");
	fdbk_msg.value.push_back(this->max_vel_mps);
	fdbk_msg.item.push_back("min_vel_mps");
        fdbk_msg.value.push_back(this->min_vel_mps);
	fdbk_msg.item.push_back("cur_cur_amp");
	fdbk_msg.value.push_back(this->cur_cur_amp);
	fdbk_msg.item.push_back("time_step_s");
	fdbk_msg.value.push_back(this->time_step_s);
	fdbk_msg.item.push_back("mode");
	fdbk_msg.value.push_back(this->mode);
	fdbk_msg.item.push_back("timeout");
	fdbk_msg.value.push_back(this->timeout_s);
	fdbk_msg.item.push_back("des_ang_pos_m");
	fdbk_msg.value.push_back(this->des_ang_pos_rad);
	fdbk_msg.item.push_back("des_vel_mps");
	fdbk_msg.value.push_back(this->des_vel_mps);
	fdbk_msg.item.push_back("ticks_per_rev");
	fdbk_msg.value.push_back(this->ticks_per_rev);
	feedback_pub.publish(fdbk_msg);
}

