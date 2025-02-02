#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intakeMotor(12);
inline pros::MotorGroup wallStake({7,-2});
inline pros::adi::DigitalOut Clamper('H');
inline pros::adi::DigitalOut intake('B');
inline pros::adi::DigitalOut doinker('A'); 
inline pros::adi::DigitalOut ringRush('C');                                                                                                          
inline pros::Optical OP(5);
inline pros::Distance wallSense(1);
