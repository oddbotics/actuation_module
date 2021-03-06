/*
 * DC motor controller class that will allow a controller to interface with a dc motor connected 
 * to a Beaglebone Black
 *
 * By: Christopher Dunkers, cmdunkers@cmu.edu
 * March 1, 2015
 */
#ifndef DC_MOTOR_CONTROLLER_H_
#define DC_MOTOR_CONTROLLER_H_

#include <ros/ros.h>
#include "oddbot_msgs/ActuationCommand.h"
#include "oddbot_msgs/ActuationFeedback.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include <cmath>

class dc_motor_controller 
{
    public: 	
	dc_motor_controller();
	void update_motor(float * setMotor, int deltaEncoder_ticks);
	void update_feedback();
	double getTimeStepS(){return this->time_step_s;}
	std::string getEqepPath(){return this->eqep_path;}
	void setDesVelToZero(){this->des_vel_mps = 0.0;}
	void setCurAmp(float value){this->cur_cur_amp = value;}
	int getMode(){return this->mode;}
	//void update_controller(const oddbot_msgs::ActuationCommand::ConstPtr& msg);
	
    protected:
	//function to call the correct control 
	virtual void getMotorCommand(float * setMotor){ROS_INFO("SETTING MOTOR TO ZERO"); *setMotor = 0.0;}

	//call back function for subscriber
	void update_controller(const oddbot_msgs::ActuationCommand::ConstPtr& msg);

	//the control loop time interval
	double time_step_s;
	
	// motor parameters
	double wheel_radius_m;
	int ticks_per_rev;
	double max_vel_mps;	
	double min_vel_mps;	

	//control variables
	int mode;
	double timeout_s;
	double timeout_time_s;

	// desired control 
	double des_vel_mps;
	double des_ang_pos_rad;

	// current control
	double cur_vel_mps;
	double cur_ang_vel_rps;
	double cur_pos_m;
	double cur_ang_pos_rad;
	double cur_cur_amp;

	//eQEP
	std::string eqep_path;

	//Subscribers and publishers
	ros::Subscriber command_sub;
	ros::Publisher feedback_pub,joint_pub;

	//node name to use for joint state
	std::string node_name;
};

#endif
 
