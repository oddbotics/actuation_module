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
	vel_PID = PID(kp, ki, kd, min_output, max_output)
  
}

/**
 * A function to generate a value to set the motor
 */
position_controller::getMotorCommand(float * setMotor){
	//calcuate output
        *setMotor = this->vel_PID.getValue(this->cur_pos_m,this->des_pos_m,this->time_step_s); 
}


