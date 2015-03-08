/*
 * \dc_motor_controller.cpp
 * \controls the motor based on PID
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date February 14, 2015
 */

#include "actuation_controller/velocity_controller.h"
#include "actuation_controller/position_controller.h"

dc_motor_controller::dc_motor_controller(){
	//the main node handle
	ros::NodeHandle nh;

	//grab the parameters
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param<double>("time_step_s", time_step_s, 0.01);
	private_node_handle_.param<int>("ticks_per_rev", ticks_per_rev, 3200);
	private_node_handle_.param<double>("wheel_radius_m", wheel_radius_m, 0.0619125);
	private_node_handle_.param<double>("max_vel_mps", max_vel_mps, 2.0);
	private_node_handle_.param<double>("min_vel_mps", min_vel_mps, -2.0);
	private_node_handle_.param<double>("timeout_s", timeout_s, 1.0);
	private_node_handle_.param<std::string>("eqep_path", eqep_path, "/sys/devices/ocp.3/48304000.epwmss/48304180.eqep");
	
	//initialize the publishers and subscribers
	node_name = ros::this_node::getNamespace();
	
	joint_pub = nh.advertise<sensor_msgs::JointState>("/joint", 1000);
	feedback_pub = nh.advertise<oddbot_msgs::ServoFeedback>("/feedback", 1000);
	command_sub = nh.subscribe("/command", 1000, &motor_controller::update_controller, this);
	
	//intialize state of the controller
	// desired control 
	double des_ang_pos_rad = 0.0;
	double des_vel_mps = 0.0;

	// current control
	double cur_vel_mps = 0.0;
	double cur_ang_vel_rps = 0.0;
	double cur_pos_m = 0.0;
	double cur_ang_pos_rad = 0.0;
	double cur_cur_amp = 0.0;
}

/**
 * A function to generate a value to set the motor
 */
void motor_controller::update_motor(float * setMotor, int deltaEncoder_ticks){
	
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

        //calcuate output
        this->pos_PID.getMotorCommand(setMotor);  
}

void dc_motor_controller::update_controller(const oddbot_msgs::DCMotorCommand::ConstPtr& msg) {
	//update the timeout for time the message was received
	ros::Time timeout_time_s = ros::Time::now() + ros::Duration(this->timeout_s);
	if(msg->des_ctrl > this->max_vel_mps){
		this->des_vel_mps = this->max_vel_mps;
	} else if(msg->des_ctrl < this->min_vel_mps){ 
		this->des_vel_mps = this->min_vel_mps;
	}else {
		this->des_vel_mps = msg->des_ctrl;
	}
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_controller");
	
	if(control == position)
	    position_controller ctrl = position_controller();
	else {
	    velocity_controller ctrl = velocity_controller();   
  	}

	ROS_INFO("laser scanner test node started!");	
	
	// Initialize DIGITAL OUTPUT
	BlackLib::BlackGPIO ENA(BlackLib::GPIO_39,BlackLib::output, BlackLib::SecureMode);   
	BlackLib::BlackGPIO ENB(BlackLib::GPIO_38,BlackLib::output, BlackLib::SecureMode);
	BlackLib::BlackGPIO INA(BlackLib::GPIO_35,BlackLib::output, BlackLib::SecureMode);
	BlackLib::BlackGPIO INB(BlackLib::GPIO_34,BlackLib::output, BlackLib::SecureMode);

	// Initialize ANALOG INPUT
	BlackLib::BlackGPIO CS(BlackLib::GPIO_51,BlackLib::input, BlackLib::SecureMode);
	
	// INA - Motor direction input A (CW)
	// INB - Motor direction input B (CCW)
	// ENA - enable half bridge A (HIGH enabled)
	// ENB - enable half bridge B (HIGH enabled)
	// CS  -  Current Sense output
	
	// INA---INB---ENA---ENB---OUTA---OUTB---Operating Mode
	//  1     1     1     1     H       H     Brake to Vcc
	//  1     0     1     1     H       L     CW
	//  0     1     1     1     L       H     CCW
	//  0     0     1     1     L       L     Brake to GND
	
	ENA.setValue(BlackLib::high);          // enable half bridge A
	ENB.setValue(BlackLib::high);          // enable half bridge B
	INA.setValue(BlackLib::low);          // Turn on to set into brake 
	INB.setValue(BlackLib::low);          // turn on to set to Brake 
	
	//Initialize PWM
	BlackLib::BlackPWM pwmMotor(BlackLib::EHRPWM2A);
	pwmMotor.setDutyPercent(0.0); 
        pwmMotor.setPeriodTime(0.001 * pow(10,12), BlackLib::picosecond); 
    
	// Allocate an instane of eqep
	eQEP eqep(ctrl.getEqepPath(), eQEP::eQEP_Mode_Relative);

	// Set the unit time period on eqep
	eqep.set_period(ctrl.getTimeStepS() * pow(10,9));
	
	//initialize other params
	int deltaEncoder_ticks;
	float setMotor;

	while (ros::ok())
	{
		// check for updates
		ros::spinOnce();

		//turn off the motor if it has not received a command after the timeout period
		if((ros::Time::now().toSec() - ctrl.getTimeoutTime()) > 0){
			ctrl.setDesVelToZero();
		} 
		
		//get the change in encoder ticks
		deltaEncoder_ticks = eqep.get_position();
		
		//get the value to set the motor at
		ctrl.update_motor(setMotor&, deltaEncoder_ticks);
		
		//set the Motor Speed
		if(setMotor < 0)
		{
			INA.setValue(BlackLib::high);
			INB.setValue(BlackLib::low);         
			setMotor = -1*setMotor;
		} else if(setMotor > 0) 
		{
			INA.setValue(BlackLib::low);
			INB.setValue(BlackLib::high);          
		} else if(setMotor == 0){
			INA.setValue(BlackLib::low);
			INB.setValue(BlackLib::low);          
		}

		//set the motor PWM value
		pwmMotor.setDutyPercent(setMotor);
		
		//get the current drawn
		ctrl.setCurAmp(CS.getValue());
		
		//Publish the update
		ctrl.update_feedback();
	}

	// turn off the motor;
	INA.setValue(BlackLib::low);
	INB.setValue(BlackLib::low); 
	pwmMotor.setDutyPercent(0.0);

	return 0;
}

