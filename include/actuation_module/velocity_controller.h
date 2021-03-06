/*
 * DC motor controller using PC controlling velocity 
 *
 * By: Christopher Dunkers, cmdunkers@cmu.edu
 * March 1, 2015
 */

#ifndef VELOCITY_CONTROLLER_H_
#define VELOCITY_CONTROLLER_H_

#include "actuation_module/dc_motor_controller.h"
#include "actuation_module/PID.hpp"
#include <chrono>

class velocity_controller : public dc_motor_controller
{
	public:
		velocity_controller();
		void getMotorCommand(float * setMotor);

	private:
		// PID Parameters for the specific motor
		double kp;
		double ki;
		double kd;
		double min_output;
		double max_output;	

		//PID handles
		PID vel_PID;
};

#endif
