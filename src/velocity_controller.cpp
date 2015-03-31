/*
 * \velocity_controller.cpp
 * \controls the motor based on PID
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date February 14, 2015
 */

#include "actuation_module/velocity_controller.h"

/** 
 * the constructor for the class motor_controller
 * it is used to initialize the ros node 
 */
velocity_controller::velocity_controller() : dc_motor_controller(){

	//grab the parameters
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param<double>("kp", kp, 400.0);
	private_node_handle_.param<double>("ki", ki, 5.5);
        private_node_handle_.param<double>("kd", kd, 0.01);
	private_node_handle_.param<double>("min_output", min_output, -100.0);
	private_node_handle_.param<double>("max_output", max_output, 100.0);
	
	// initialize PID controller
	vel_PID = PID(kp, ki, kd, min_output, max_output);  
	ROS_INFO("VELOCITY CONTROLLER INIT");
}

/**
 * A function to generate a value to set the motor
 */
void velocity_controller::getMotorCommand(float * setMotor){
	//calcuate output
	//turn off the motor if it has not received a command after the timeout period
	if((ros::Time::now().toSec() - this->timeout_time_s) > 0){
		this->setDesVelToZero();
	} else {
		if(this->des_vel_mps > this->max_vel_mps){
                	this->des_vel_mps = this->max_vel_mps;
        	} else if(this->des_vel_mps < this->min_vel_mps){ 
                	this->des_vel_mps = this->min_vel_mps;
        	}
	}
//	ROS_INFO("DES VEL: %f ::: CUR VEL %f ",this->des_vel_mps, this->cur_vel_mps);
	this->des_ang_pos_rad = 0.0;
        *setMotor = this->vel_PID.getValue(this->cur_vel_mps,this->des_vel_mps,this->time_step_s);
//	ROS_INFO("SET: %f", *setMotor); 
}


