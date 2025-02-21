#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 120;
const int TURN_SPEED = 120;
const int SWING_SPEED = 120;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void sensingRed(){
  while (true) { // Infinite loop to continuously check the color sensor
    OP.set_led_pwm(100);
    // Get the detected color hue from the color sensor
    int hue = OP.get_hue();

    // Check if the detected color is red (typical red hue is around 8 degrees)
    if (hue >= 0 && hue <= 10 && OP.get_proximity() == 255){
      pros::delay(217); // Small delay before de-activating intake
      intakeMotor.move_velocity(-600);
      pros::delay(175); // Delay to keep sorter active for a longer duration
    }
    else
    {
      intakeMotor.move_velocity(600);
    }

    // Small delay to prevent overwhelming the CPU with constant checks
    pros::delay(20);
    }
}

void sensingBlue(){
  while (true) { // Infinite loop to continuously check the color sensor
    OP.set_led_pwm(100);
    // Get the detected color hue from the color sensor
    int hue = OP.get_hue();

    // Check if the detected color is red (typical red hue is around 8 degrees)
    if (hue >= 220 && hue <= 230 && OP.get_proximity() == 255){
      pros::delay(217); // Small delay before activating piston
      intakeMotor.move_velocity(-600);
      pros::delay(175); // Delay to keep sorter active for a longer duration
    }
    else
    {
      intakeMotor.move_velocity(600);
    }

    // Small delay to prevent overwhelming the CPU with constant checks
    pros::delay(20);
    }
}

void stoppingBlue(){
  while (true) { // Infinite loop to continuously check the color sensor
    OP.set_led_pwm(100);
    // Get the detected color hue from the color sensor
    int hue = OP.get_hue();

    // Check if the detected color is red (typical red hue is around 8 degrees)
    if (!(hue >= 220 && hue <= 230 && OP.get_proximity() == 255)){
      intakeMotor.move_velocity(600);
    }
    else
    {
      intakeMotor.move_velocity(0);
    }

    // Small delay to prevent overwhelming the CPU with constant checks
    pros::delay(20);
    }
}

void stoppingRed(){
  while (true) { // Infinite loop to continuously check the color sensor
    OP.set_led_pwm(100);
    // Get the detected color hue from the color sensor
    int hue = OP.get_hue();

    // Check if the detected color is red (typical red hue is around 8 degrees)
    if (!(hue >= 0 && hue <= 10 && OP.get_proximity() == 255)){
      intakeMotor.move_velocity(600);
    }
    else
    {
      intakeMotor.move_velocity(0);
    }

    // Small delay to prevent overwhelming the CPU with constant checks
    pros::delay(20);
    }
}
///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .


void elimsAutoRedLeft() {
//   chassis.moveTo(0, 0, 5000);
// chassis.moveTo(42.011, 9.258, 5000);
// chassis.moveTo(41.774, 35.183, 5000);
// chassis.moveTo(8.951, 40.177, 5000);
// chassis.moveTo(-1.514, 52.07, 5000);
// chassis.moveTo(17.276, 32.804, 5000);
// chassis.moveTo(44.152, 6.641, 5000);

  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(-65.502_in, 17.458_in, 216_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistencey
  
  pros::Task intaking(sensingBlue);
  intaking.suspend();
  wallStake.move_absolute(1300, 600);
  pros::delay(500);
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();
  wallStake.move_absolute(-130, 600);
  chassis.pid_odom_pp_set({{{-45.523, 22.453}, rev, DRIVE_SPEED},
                           {{-24.355, 23.88}, rev, 70}});
chassis.pid_wait();
  pros::delay(200);
  Clamper.set_value(1);
  pros::delay(200);
  intaking.resume();
  chassis.pid_odom_pp_set({{{-23.88, 50.043}, fwd, DRIVE_SPEED},
                           {{-66.692, 67.405}, fwd, DRIVE_SPEED}});
  chassis.pid_wait();
  pros::delay(1000);
  chassis.pid_odom_set({{-46.475, 46.951}, rev, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({{-17.933, 17.696}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  wallStake.move_absolute(1050, 600);
  intaking.suspend();  


  // color = 1;
  // pros::delay(2000);
  // pros::Task intaking(stoppingBlue);
  // wallStake.move_absolute(1300, 600);
  // pros::delay(500);
  // chassis.pid_drive_set(-14_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(55_deg, TURN_SPEED);
  // chassis.pid_wait();
  // wallStake.move_absolute(-127, 600);
  // chassis.pid_drive_set(-23_in, 85);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-8_in, 50);
  // chassis.pid_wait();
  // pros::delay(200);
  // Clamper.set_value(1);
  // pros::delay(200);
  // chassis.pid_turn_set(145_deg, TURN_SPEED);
  // chassis.pid_wait();
  // intaking.resume();
  // chassis.pid_drive_set(25_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(85_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(52_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // pros::delay(400);
  // chassis.pid_drive_set(-5_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(-255_deg, TURN_SPEED, ez::cw);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(-75_deg, TURN_SPEED, ez::ccw);
  // chassis.pid_wait();
  // chassis.pid_drive_set(30_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(200_deg, TURN_SPEED, ez::ccw);
  // chassis.pid_wait();
  // chassis.pid_drive_set(17_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(185_deg, TURN_SPEED, ez::ccw);
  // chassis.pid_wait();
  // chassis.pid_drive_set(10_in, DRIVE_SPEED);
  // chassis.pid_wait();
  
}

void redAutoRightElims(){

  chassis.pid_drive_set(12_in, DRIVE_SPEED);
  chassis.pid_wait();
  wallStake.move_velocity(2300);
  pros::delay(500);
  wallStake.move_velocity(-2300);
  pros::delay(500);
  wallStake.move_velocity(0);
  chassis.pid_drive_set(-42_in, 60);
  chassis.pid_wait();
  pros::delay(500);
  Clamper.set_value(1);
  pros::delay(500);
  chassis.pid_wait();
  chassis.pid_turn_set(-115_deg, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(23_in, 100);
  intakeMotor.move_velocity(3000);
  pros::delay(2100);
  intakeMotor.move_velocity(0);
  intakeMotor.move_velocity(-3000);
  pros::delay(200);
  intakeMotor.move_velocity(0);
  chassis.pid_drive_set(-25_in, DRIVE_SPEED);
  chassis.pid_wait();
  Clamper.set_value(0);
  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-10_deg, TURN_SPEED);
  chassis.pid_wait();
  intake.set_value(1);
  chassis.pid_drive_set(48_in, DRIVE_SPEED);
  chassis.pid_wait();
  intake.set_value(0);
  intakeMotor.move_velocity(3000);
  pros::delay(900);
  chassis.pid_drive_set(-25_in, DRIVE_SPEED);
  pros::delay(50);
  intakeMotor.move_velocity(0);
  intakeMotor.move_velocity(-3000);
  pros::delay(300);
  intakeMotor.move_velocity(0);
  chassis.pid_turn_set(120_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(24.5_in, DRIVE_SPEED);
  chassis.pid_wait();
  wallStake.move_absolute(400,100);
}

void elimsAutoBlueRight() {
  color = 2;
  wallStake.move_absolute(600,3000);
  chassis.pid_wait();
  chassis.pid_drive_set(-41_in, 60);
  chassis.pid_wait();
  pros::delay(500);
  Clamper.set_value(1);
  pros::delay(500);
  chassis.pid_turn_set(-115_deg, 75);
  chassis.pid_wait();
  intakeMotor.move_velocity(6000);
  chassis.pid_drive_set(22_in, 75);
  chassis.pid_wait();
  chassis.pid_turn_set(-70_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(5.5_in, DRIVE_SPEED);
  pros::delay(3400);
  intakeMotor.move_velocity(0);
  chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-30_deg, 75);
  chassis.pid_wait();
  intakeMotor.move_velocity(6000);
  chassis.pid_drive_set(23_in, 75);
  chassis.pid_wait();
  chassis.pid_turn_set(-107_deg, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(21_in, 75);
  chassis.pid_wait();
  intakeMotor.move_velocity(0);
  wallStake.move_absolute(1625,3000);
  chassis.pid_wait();
  pros::delay(1500);
  chassis.pid_drive_set(-25_in, 75);
  pros::delay(2500);
  wallStake.move_absolute(20,3000);
  chassis.pid_wait();
  
}

void blue_Left_Side() {
  color = 2;
  // pros::delay(2000);
  pros::Task intaking(sensingRed);
  intaking.suspend();
  // wallStake.move_absolute(1300, 600);
  // pros::delay(500);
  chassis.pid_drive_set(-14_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(55_deg, TURN_SPEED);
  chassis.pid_wait();
  wallStake.move_absolute(-127, 600);
  chassis.pid_drive_set(-23_in, 85);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8_in, 50);
  chassis.pid_wait();
  pros::delay(200);
  Clamper.set_value(1);
  pros::delay(200);
  chassis.pid_turn_set(145_deg, TURN_SPEED);
  chassis.pid_wait();
  intaking.resume();
  chassis.pid_drive_set(25_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(85_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(52_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(400);
  chassis.pid_drive_set(-7_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(9_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-255_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-75_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(33_in, DRIVE_SPEED);
  chassis.pid_wait();
  wallStake.move_absolute(1050, 600);
  intaking.suspend();
}

void blue_Right_Side() {
  color = 2;
  // pros::delay(2000);
  pros::Task intaking(sensingRed);
  intaking.suspend();
  wallStake.move_absolute(1300, 600);
  pros::delay(500);
  chassis.pid_drive_set(-14_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-55_deg, TURN_SPEED);
  chassis.pid_wait();
  wallStake.move_absolute(-127, 600);
  chassis.pid_drive_set(-23_in, 85);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8_in, 50);
  chassis.pid_wait();
  pros::delay(200);
  Clamper.set_value(1);
  pros::delay(200);
  chassis.pid_turn_set(-145_deg, TURN_SPEED);
  chassis.pid_wait();
  intaking.resume();
  chassis.pid_drive_set(25_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-85_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(52_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(400);
  chassis.pid_drive_set(-7_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(9_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(255_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(75_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(33_in, DRIVE_SPEED);
  chassis.pid_wait();
  wallStake.move_absolute(1050, 600);
  intaking.suspend();
}

void red_Left_Side() {
  color = 1;
  // chassis.odom_xyt_set(-63.838_in, 18.647_in, 145_deg);
  // chassis.pid_odom_set({{-22.69, 23.88}, rev, DRIVE_SPEED});
  // chassis.pid_wait();
  // chassis.pid_odom_set({{-23.88, 50.043}, fwd, DRIVE_SPEED});
  // chassis.pid_wait();
  // chassis.pid_odom_set({{-66.692, 67.405}, fwd, DRIVE_SPEED});
  // chassis.pid_wait();
  // chassis.pid_odom_set({{-12.463, 12.939}, fwd, DRIVE_SPEED});
  // chassis.pid_wait();

  
  // pros::delay(2000);
  pros::Task intaking(sensingBlue);
  intaking.suspend();
  wallStake.move_absolute(1300, 600);
  pros::delay(500);
  chassis.pid_drive_set(-14_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(55_deg, TURN_SPEED);
  chassis.pid_wait();
  wallStake.move_absolute(-127, 600);
  chassis.pid_drive_set(-23_in, 85);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8_in, 50);
  chassis.pid_wait();
  pros::delay(200);
  Clamper.set_value(1);
  pros::delay(200);
  chassis.pid_turn_set(145_deg, TURN_SPEED);
  chassis.pid_wait();
  intaking.resume();
  chassis.pid_drive_set(25_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(85_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(52_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(400);
  chassis.pid_drive_set(-7_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(9_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-255_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-75_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(33_in, DRIVE_SPEED);
  chassis.pid_wait();
  wallStake.move_absolute(1050, 600);
  intaking.suspend();
  }

void red_Right_Side_AWP() {
  color = 1;
  // pros::delay(2000);
  pros::Task intaking(sensingBlue);
  intaking.suspend();
  wallStake.move_absolute(1300, 600);
  pros::delay(500);
  chassis.pid_drive_set(-14_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-55_deg, TURN_SPEED);
  chassis.pid_wait();
  wallStake.move_absolute(-127, 600);
  chassis.pid_drive_set(-23_in, 85);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8_in, 50);
  chassis.pid_wait();
  pros::delay(200);
  Clamper.set_value(1);
  pros::delay(200);
  chassis.pid_turn_set(-145_deg, TURN_SPEED);
  chassis.pid_wait();
  intaking.resume();
  chassis.pid_drive_set(25_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-85_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(52_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(400);
  chassis.pid_drive_set(-7_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(9_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(255_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(75_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(33_in, DRIVE_SPEED);
  chassis.pid_wait();
  wallStake.move_absolute(1050, 600);
  intaking.suspend();
  }
void blueRingRush()
{
  color = 2;
  pros::Task intaking(sensingRed);
  pros::Task stopping(stoppingBlue);
  intaking.suspend();
  stopping.suspend();
  secondDoinker.set_value(1);
  stopping.resume();
  pros::delay(200);
  chassis.pid_drive_set(47_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(500);
  stopping.suspend();
  intakeMotor.move_velocity(0);
  chassis.pid_drive_set(-30_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-25_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  secondDoinker.set_value(0);
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(125_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  // chassis.pid_drive_set(-23_in, 70);
  // chassis.pid_wait();
  // Clamper.set_value(1);
  // pros::delay(200);
  // chassis.pid_turn_set(70_deg, TURN_SPEED);
  // chassis.pid_wait();
  // intaking.resume();
  // chassis.pid_drive_set(45_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(160_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(48_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // pros::delay(700);
  // intaking.suspend();
  // chassis.pid_turn_set(140_deg, TURN_SPEED, ez::ccw);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  // chassis.pid_wait();
  // chassis.pid_turn_set(260_deg, TURN_SPEED, ez::cw);
  // chassis.pid_wait();
  // intaking.resume();
  // chassis.pid_drive_set(33_in, DRIVE_SPEED);
  // chassis.pid_wait();
}

void redRingRush()
{
// chassis.moveTo(-58.843, 17.22, 5000);
// chassis.moveTo(-2.474, 47.188, 5000);
// chassis.moveTo(-24.593, 24.117, 5000);
// chassis.moveTo(-23.88, 49.329, 5000);
// chassis.moveTo(-67.643, 67.167, 5000);
// chassis.moveTo(-59.794, 25.545, 5000);
// chassis.moveTo(-46.713, 0.095, 5000);
// chassis.moveTo(-67.405, -0.381, 5000);
// chassis.moveTo(-25.307, 0.333, 5000);

  color = 1;
  pros::Task intaking(sensingBlue);
  pros::Task stopping(stoppingRed);
  intaking.suspend();
  stopping.suspend();
  stopping.resume();
  pros::delay(200);
  chassis.pid_drive_set(46_in, DRIVE_SPEED);
  chassis.pid_wait_until(23_in);
  doinker.set_value(1);
  chassis.pid_wait();
  pros::delay(200);
  stopping.suspend();
  intakeMotor.move_velocity(0);
  chassis.pid_drive_set(-29_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(25_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  doinker.set_value(0);
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-128_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(-25_in, 70);
  chassis.pid_wait();
  Clamper.set_value(1);
  pros::delay(200);
  chassis.pid_turn_set(-70_deg, TURN_SPEED);
  chassis.pid_wait();
  intaking.resume();
  chassis.pid_drive_set(38_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-150_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(45_in, DRIVE_SPEED);
  chassis.pid_wait();
  intaking.suspend();
  chassis.pid_drive_set(-1_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(105_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(33_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void skillsAuto(){
// chassis.moveTo(-62.41, 20.55, 5000);
// chassis.moveTo(-47.188, 24.117, 5000);
// chassis.moveTo(-23.166, 23.88, 5000);
// chassis.moveTo(-10.798, 37.675, 5000);
// chassis.moveTo(23.214, 47.188, 5000);
// chassis.moveTo(-0.095, 40.529, 5000);
// chassis.moveTo(-1.047, 64.551, 5000);
// chassis.moveTo(-23.642, 47.188, 5000);
// chassis.moveTo(-46.951, 47.426, 5000);
// chassis.moveTo(-61.459, 47.426, 5000);
// chassis.moveTo(-46.475, 37.437, 5000);
// chassis.moveTo(-47.426, 61.221, 5000);
// chassis.moveTo(-63.362, 62.886, 5000);
// chassis.moveTo(-47.426, 47.188, 5000);
// chassis.moveTo(-46.951, -24.641, 5000);
// chassis.moveTo(-23.404, -23.689, 5000);
// chassis.moveTo(-4.376, -39.149, 5000);
// chassis.moveTo(23.927, -46.998, 5000);
// chassis.moveTo(0.381, -41.052, 5000);
// chassis.moveTo(-1.047, -64.123, 5000);
// chassis.moveTo(-23.404, -46.76, 5000);
// chassis.moveTo(-47.188, -46.998, 5000);
// chassis.moveTo(-61.459, -46.998, 5000);
// chassis.moveTo(-47.902, -37.484, 5000);
// chassis.moveTo(-47.188, -59.604, 5000);
// chassis.moveTo(-66.216, -63.409, 5000);
// chassis.moveTo(-34.107, -46.047, 5000);
// chassis.moveTo(46.522, -46.522, 5000);
// chassis.moveTo(58.653, -22.262, 5000);
// chassis.moveTo(65.074, -62.458, 5000);
// chassis.moveTo(46.522, -22.5, 5000);
// chassis.moveTo(46.522, 1.284, 5000);
// chassis.moveTo(66.026, 0.809, 5000);
// chassis.moveTo(39.387, -0.143, 5000);
// chassis.moveTo(23.927, -23.689, 5000);
// chassis.moveTo(0.143, 0.095, 5000);
// chassis.moveTo(23.689, 23.642, 5000);
// chassis.moveTo(46.285, 46.475, 5000);
// chassis.moveTo(47.236, 60.27, 5000);
// chassis.moveTo(46.285, 38.864, 5000);
// chassis.moveTo(59.128, 46.713, 5000);
// chassis.moveTo(66.026, 65.74, 5000);
// chassis.moveTo(12.273, 10.798, 5000);

  wallStake.set_brake_mode_all(MOTOR_BRAKE_HOLD);
  wallStake.move_absolute(1350, 600);
  pros::delay(400);
  chassis.pid_drive_set(-13_in, DRIVE_SPEED);
  chassis.pid_wait();
  Clamper.set_value(1);
  pros::delay(100);
  chassis.pid_turn_set(220_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  wallStake.move_absolute(-127, 600);
  intakeMotor.move_velocity(600);
  pros::delay(100);
  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(190_deg, 100, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(25_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(215_deg, 100, ez::cw);
  chassis.pid_wait();
  wallStake.move_absolute(25, 600);
  pros::delay(200);
  chassis.pid_drive_set(34_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(600);
  chassis.pid_drive_set(-22.5_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(137_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  intakeMotor.move_velocity(0);
  wallStake.move_absolute(600, 600);
  pros::delay(100);
  intakeMotor.move_velocity(600);
  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  chassis.pid_wait();
  drive.move_velocity(600);
  pros::delay(100);
  wallStake.move_absolute(1200, 600);
  pros::delay(500);
  wallStake.move_absolute(-127, 600);
  drive.move_velocity(0);
  pros::delay(300);
  while(wallSense.get_distance() <= 305)
  {
    drive.move_velocity(-100);
  }
  drive.move_velocity(0);
  chassis.pid_turn_set(50_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  intakeMotor.move_velocity(600);
  chassis.pid_drive_set(25_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(500);
  chassis.pid_drive_set(35_in, 40);
  chassis.pid_wait();
  pros::delay(800);
  chassis.pid_turn_set(190_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  chassis.pid_drive_set(16_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(300);
  chassis.pid_turn_set(265_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  chassis.pid_drive_set(-13_in, DRIVE_SPEED);
  chassis.pid_wait();
  Clamper.set_value(0);
  intakeMotor.move_velocity(-600);
  pros::delay(300);
  chassis.pid_drive_set(14_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(140_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(-50_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-29_in, 60);
  chassis.pid_wait();
  Clamper.set_value(1);
  pros::delay(100);
  chassis.pid_turn_set(230_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  intakeMotor.move_velocity(600);
  chassis.pid_wait();
  wallStake.move_absolute(-127, 600);
  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(280_deg, 100, ez::cw);
  chassis.pid_wait();
  chassis.pid_drive_set(25_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(255_deg, 100, ez::ccw);
  chassis.pid_wait();
  wallStake.move_absolute(25, 600);
  pros::delay(200);
  chassis.pid_drive_set(35_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-25_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(320_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  intakeMotor.move_velocity(0);
  wallStake.move_absolute(600, 600);
  pros::delay(100);
  intakeMotor.move_velocity(600);
  chassis.pid_drive_set(18_in, DRIVE_SPEED);
  chassis.pid_wait();
  drive.move_velocity(600);
  pros::delay(100);
  wallStake.move_absolute(1200, 600);
  pros::delay(500);
  wallStake.move_absolute(-127, 600);
  drive.move_velocity(0);
  pros::delay(300);
  while(wallSense.get_distance() <= 305)
  {
    drive.move_velocity(-100);
  }
  drive.move_velocity(0);
  chassis.pid_turn_set(50_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  intakeMotor.move_velocity(600);
  chassis.pid_drive_set(25_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(500);
  chassis.pid_drive_set(35_in, 40);
  chassis.pid_wait();
  pros::delay(800);
  chassis.pid_turn_set(280_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(16_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(300);
  chassis.pid_turn_set(210_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(-13_in, DRIVE_SPEED);
  chassis.pid_wait();
  Clamper.set_value(0);
  intakeMotor.move_velocity(-600);
  pros::delay(300);
  chassis.pid_drive_set(11_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(230_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  wallStake.move_absolute(20, 600);
  intakeMotor.move_velocity(600);
  chassis.pid_drive_set(82_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(320_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  chassis.pid_drive_set(-60_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(230_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(30_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(320_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait();
  chassis.pid_drive_set(-100_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(200_in, DRIVE_SPEED);
  chassis.pid_wait();
  }

