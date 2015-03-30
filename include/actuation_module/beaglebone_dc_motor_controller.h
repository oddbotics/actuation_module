/*
 * DC motor controller class that will allow a controller to interface with a dc motor connected 
 * to a Beaglebone Black
 *
 * By: Christopher Dunkers, cmdunkers@cmu.edu
 * March 1, 2015
 */

#ifndef BEAGLEBONE_DC_MOTOR_CONTROLLER_H_
#define BEAGLEBONE_DC_MOTOR_CONTROLLER_H_

#include <ros/ros.h>
#include "actuation_module/dc_motor_controller.h"
#include "actuation_module/velocity_controller.h"
#include "actuation_module/position_controller.h"
#include "beaglebone_blacklib/BlackPWM.h"
#include "beaglebone_blacklib/BlackGPIO.h"
#include "beaglebone_eqep/eqep.h"
#include <string>

#endif
