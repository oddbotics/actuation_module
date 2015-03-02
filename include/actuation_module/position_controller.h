#ifndef POSITION_CONTROLLER_H_
#define POSITION_CONTROLLER_H_

#include "actuation_controller/dc_motor_controller.h"
#include "actuation_controller/PID.hpp"
#include <chrono>

class position_controller : public dc_motor_controller
{
	public:
		position_controller();
		float update_motor(int deltaEncoder_ticks);
		void update_feedback();
		
	private:
		// PID Parameters for the specific motor
		double kp;
		double ki;
		double kd;
		double min_output;
		double max_output;	

		//PID handles
		PID pos_PID;
};

#endif
