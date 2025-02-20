#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intakeMotor(16);
inline pros::MotorGroup wallStake({8,-2});
inline pros::MotorGroup drive({-11, -3, -4, 6, 10, 19});
inline pros::adi::DigitalOut Clamper('H');
inline pros::adi::DigitalOut intake('B');
inline pros::adi::DigitalOut doinker('A'); 
inline pros::adi::DigitalOut secondDoinker('F');                                                                                                          
inline pros::Optical OP(18);
inline int color = 0;
inline pros::Distance wallSense(1);
inline pros::Distance sideSense(13);