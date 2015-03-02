/*
 * \dc_motor_controller.cpp
 * \controls the motor based on PID
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date February 14, 2015
 */

#include "actuation_controller/velocity_controller.h"
#include "actuation_controller/position_controller.h"

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
	BlackLib::BlackGPIO   CS(BlackLib::GPIO_51,BlackLib::input, BlackLib::SecureMode);
	
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
	eQEP eqep(vel_ctrl.getEqepPath(), eQEP::eQEP_Mode_Relative);

	// Set the unit time period on eqep
	eqep.set_period(mc.getTimeStepS() * pow(10,9));
	
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
		setMotor = ctrl.update_motor(deltaEncoder_ticks);
		
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

