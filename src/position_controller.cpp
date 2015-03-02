/*
 * \position_controller.cpp
 * \controls the motor based on PID
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date February 14, 2015
 */

#include "actuation_controller/position_controller.h"

/** 
 * the constructor for the class motor_controller
 * it is used to initialize the ros node 
 */
position_controller::position_controller(){

	//the main node handle
	ros::NodeHandle nh;

	//grab the parameters
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param<double>("kp", kp, 1.0);
	private_node_handle_.param<double>("ki", ki, 1.0);
        private_node_handle_.param<double>("kd", kd, 1.0);
	private_node_handle_.param<double>("rate_s", time_step_s, 0.01);
	private_node_handle_.param<int>("ticks_per_rev", ticks_per_rev, 3200);
	private_node_handle_.param<double>("wheel_radius_m", wheel_radius_m, 0.0619125);
	private_node_handle_.param<double>("max_vel_mps", max_vel_mps, 2.0);
	private_node_handle_.param<double>("min_output", min_output, -100.0);
	private_node_handle_.param<double>("max_output", max_output, 100.0);
	private_node_handle_.param<std::string>("eqep_path", eqep_path, "/sys/devices/ocp.3/48304000.epwmss/48304180.eqep");
	
	// initialize PID controller
	pos_PID = PID(kp, ki, kd, min_output, max_output);
  
	//initialize the publishers and subscribers
	node_name = ros::this_node::getNamespace();
	
	joint_pub = nh.advertise<sensor_msgs::JointState>("/joint", 1000);
	feedback_pub = nh.advertise<oddbot_msgs::ServoFeedback>("/feedback", 1000);
	command_sub = nh.subscribe("/command", 1000, &motor_controller::update_controller, this);
	
	//intialize state of the controller
	// desired control 
	double des_pos_m = 0.0;

	// current control
	double cur_vel_mps = 0.0;
	double cur_pos_m = 0.0;
	double cur_cur_amp = 0.0;
}

/**
 * A function to generate a value to set the motor
 */
float motor_controller::update_motor(int deltaEncoder_ticks){
	
	static int prev_deltaEncoder_ticks = 0;	
	
	//check for the special condition
	if(deltaEncoder_ticks == -1 && prev_deltaEncoder_ticks == -1)
	{
		deltaEncoder_ticks = 0;
	}

	float setMotor;

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
	
	// calculate the velocity
	this->cur_vel_mps = deltaPosition_m/this->time_step_s;    

        //calcuate PID output
        setMotor = this->pos_PID.getValue(this->cur_pos_m,this->des_pos_m,this->time_step_s);  
	
	return setMotor;
}

/**
 * A function to publish the current state of the controller
 */
void motor_controller::update_feedback(){
	
	//publish the joint state
	sensor_msgs::JointState joint_msg;
	joint_msg.header.stamp = ros::Time::now();
	joint_msg.name.push_back(this->node_name);
	joint_msg.position.push_back(this->cur_ang_pos_rad);
	joint_msg.velocity.push_back(this->cur_ang_vel_rps);
	joint_pub.publish(joint_msg);
	

	//publish all of the feedback
	oddbot_msgs::ServoFeedback fdbk_msg;
	fdbk_msg.header.stamp = ros::Time::now();
	fdbk_msg.item.push_back("cur_pos_m");
	fdbk_msg.value.push_back(this->cur_pos_m);
	fdbk_msg.item.push_back("cur_vel_mps");
        fdbk_msg.value.push_back(this->cur_vel_mps);
	fdbk_msg.item.push_back("wheel_radius_m");
        fdbk_msg.value.push_back(this->wheel_radius_m);
	fdbk_msg.item.push_back("max_vel_mps");
	fdbk_msg.value.push_back(this->max_vel_mps);
	fdbk_msg.item.push_back("cur_cur_amp");
        fdbk_msg.value.push_back(this->cur_cur_amp);
	fdbk_msg.item.push_back("time_step_s");
        fdbk_msg.value.push_back(this->time_step_s);
	fdbk_msg.item.push_back("mode");
        fdbk_msg.value.push_back(this->mode);
	fdbk_msg.item.push_back("timeout");
        fdbk_msg.value.push_back(this->timeout);
	fdbk_msg.item.push_back("des_pos_m");
        fdbk_msg.value.push_back(this->des_pos_m);
	fdbk_msg.item.push_back("cur_vel_mps");
        fdbk_msg.value.push_back(this->cur_vel_mps);
	fdbk_msg.item.push_back("ticks_per_rev");
        fdbk_msg.value.push_back(this->ticks_per_rev);
	feedback_pub.publish(fdbk_msg);
}


