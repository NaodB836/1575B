#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
        // These are your drive motors, the first motor is used for sensing!
    {-11, -3, -4},     // Left Chassis Ports (negative port will reverse it!)
    {6, 10, 19},  // Right Chassis Ports (negative port will reverse it!)

    7,      // IMU Port
    2.75,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM
// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
//ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
//ez::tracking_wheel vert_tracker(15, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  //chassis.odom_tracker_front_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  //chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(2.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Left Side Red Elims Auto", elimsAutoRedLeft}, 
      {"Blue Ring Rush", blueRingRush},  
      {" Left Side Red Auton AWP", red_Left_Side},
      {"Red Ring Rush", redRingRush},
      {"Blue Ring Rush", blueRingRush},  
      {" Left Side Blue Auton AWP", blue_Left_Side},
      {" Right Side Blue Auton AWP", blue_Right_Side},
      {" Left Side Red Auton AWP", red_Left_Side},
      {" Right Side Red Auton AWP", red_Right_Side_AWP},
      {"Skills Auto", skillsAuto},      
      // {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      // {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      // {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      // {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      // {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      // {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

// /**
//  * Simplifies printing tracker values to the brain screen
//  */
// void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
//   std::string tracker_value = "", tracker_width = "";
//   // Check if the tracker exists
//   if (tracker != nullptr) {
//     tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
//     tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
//   }
//   ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
// }

// /**
//  * Ez screen task
//  * Adding new pages here will let you view them during user control or autonomous
//  * and will help you debug problems you're having
//  */
// void ez_screen_task() {
//   while (true) {
//     // Only run this when not connected to a competition switch
//     if (!pros::competition::is_connected()) {
//       // Blank page for odom debugging
//       if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
//         // If we're on the first blank page...
//         if (ez::as::page_blank_is_on(0)) {
//           // Display X, Y, and Theta
//           ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
//                                "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
//                                "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
//                            1);  // Don't override the top Page line

//           // Display all trackers that are being used
//           screen_print_tracker(chassis.odom_tracker_left, "l", 4);
//           screen_print_tracker(chassis.odom_tracker_right, "r", 5);
//           screen_print_tracker(chassis.odom_tracker_back, "b", 6);
//           screen_print_tracker(chassis.odom_tracker_front, "f", 7);
//         }
//       }
//     }

//     // Remove all blank pages when connected to a comp switch
//     else {
//       if (ez::as::page_blank_amount() > 0)
//         ez::as::page_blank_remove_all();
//     }

//     pros::delay(ez::util::DELAY_TIME);
//   }
// }
// pros::Task ezScreenTask(ez_screen_task);

// /**
//  * Gives you some extras to run in your opcontrol:
//  * - run your autonomous routine in opcontrol by pressing DOWN and B
//  *   - to prevent this from accidentally happening at a competition, this
//  *     is only enabled when you're not connected to competition control.
//  * - gives you a GUI to change your PID values live by pressing X
//  */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void senseRedOP(){
  while (true) { // Infinite loop to continuously check the color sensor
    OP.set_led_pwm(100);
    // Get the detected color hue from the color sensor
    int hue = OP.get_hue();

    // Check if the detected color is red (typical red hue is around 8 degrees)
    if (hue >= 0 && hue <= 10 && OP.get_proximity() == 255){
      pros::delay(210); // Small delay before activating piston
      intakeMotor.move_velocity(0);
      pros::delay(175); // Delay to keep sorter active for a longer duration
      
    }
    else{ // If the color is neither red nor blue, deactivate the color sorter piston
      if(master.get_digital(DIGITAL_R1))
      {
        intakeMotor.move_velocity(600);
      }
      else if(master.get_digital(DIGITAL_R2))
      {
        intakeMotor.move_velocity(-600);
      }
      else
      {
        intakeMotor.move_velocity(0);
      }
    }

    // Small delay to prevent overwhelming the CPU with constant checks
    pros::delay(20);
    }
}

void senseBlueOP(){
  while (true) { // Infinite loop to continuously check the color sensor
    OP.set_led_pwm(100);
    // Get the detected color hue from the color sensor
    int hue = OP.get_hue();

    // Check if the detected color is blue(typical red hue is around 222 degrees)
    if (hue >= 220 && hue <= 230 && OP.get_proximity() == 255){
      pros::delay(210); // Small delay before activating piston
      intakeMotor.move_velocity(0);
      pros::delay(175); // Delay to keep sorter active for a longer duration
      
    }
    else{ // If the color is neither red nor blue, deactivate the color sorter piston
      if(master.get_digital(DIGITAL_R1))
      {
        intakeMotor.move_velocity(600);
      }
      else if(master.get_digital(DIGITAL_R2))
      {
        intakeMotor.move_velocity(-600);
      }
      else
      {
        intakeMotor.move_velocity(0);
      }
    }

    // Small delay to prevent overwhelming the CPU with constant checks
    pros::delay(20);
    }
}
const int numStates = 2;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {0, 1000};
int currState = 0;
int target = 0;
void nextState() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
    }
    target = states[currState];
    wallStake.move_absolute(target, 600);
}

bool clampToggleEnabled = false; // two-choice toggle, so we use bool for clamp
bool buttonPressed = false; // IGNORE, logic variable

bool wallStakeToggleEnabled = false; // two-choice toggle, so we use bool for wall stake pistons
bool buttonPressed2 = false; // IGNORE, logic variable

bool doinkerToggleEnabled = false; // two-choice toggle, so we use bool for rachet
bool buttonPressed3 = false; // IGNORE, logic variable

bool blueColorSortToggleEnabled = false; // two-choice toggle, so we use bool for rachet
bool buttonPressed4 = false; // IGNORE, logic variable

bool redColorSortToggleEnabled = false; // two-choice toggle, so we use bool for rachet
bool buttonPressed5 = false; // IGNORE, logic variable

void opcontrol() {
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;
  chassis.drive_brake_set(driver_preference_brake);
  drive.set_brake_mode_all(MOTOR_BRAKE_COAST);
  pros::Task colorRed(senseRedOP);
  pros::Task colorBlue(senseBlueOP);
  colorRed.suspend();
  colorBlue.suspend();
  while (true) {
    if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_UP)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
    bool buttonY = master.get_digital(DIGITAL_Y);
    //Toggle Logic
    if (buttonY && !buttonPressed){
      buttonPressed = true; 
      clampToggleEnabled = !clampToggleEnabled;
    }
    else if (!buttonY) buttonPressed = false;

    bool buttonRIGHT = master.get_digital(DIGITAL_RIGHT);
    //Toggle Logic
    if (buttonRIGHT && !buttonPressed2){
      buttonPressed2 = true; 
      wallStakeToggleEnabled = !wallStakeToggleEnabled;
    }
    else if (!buttonRIGHT) buttonPressed2 = false;

    bool buttonUp = master.get_digital(DIGITAL_DOWN);
    //Toggle Logic
    if (buttonUp && !buttonPressed3){
      buttonPressed3 = true; 
      doinkerToggleEnabled = !doinkerToggleEnabled;
    }
    else if (!buttonUp) buttonPressed3 = false;

    bool buttonLeft = master.get_digital(DIGITAL_LEFT);
    //Toggle Logic
    if (buttonLeft && !buttonPressed4){
      buttonPressed4 = true; 
      blueColorSortToggleEnabled = !blueColorSortToggleEnabled;
    }
    else if (!buttonLeft) buttonPressed4 = false;

    bool buttonLeft2 = master.get_digital(DIGITAL_LEFT);
    //Toggle Logic
    if (buttonLeft2 && !buttonPressed5){
      buttonPressed5 = true; 
      redColorSortToggleEnabled = !redColorSortToggleEnabled;
    }
    else if (!buttonLeft2) buttonPressed5 = false;
    
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
        autonomous();
        chassis.drive_brake_set(driver_preference_brake);
      }
      if(color == 1)
      {
        if(blueColorSortToggleEnabled){
          // Do another thing
          colorBlue.suspend();
          if(master.get_digital(DIGITAL_R1))
          {
            intakeMotor.move_velocity(600);
          }
          else if(master.get_digital(DIGITAL_R2))
          {
            intakeMotor.move_velocity(-600);
          }
          else
          {
            intakeMotor.move_velocity(0);
          }

        }
        else{
          // Do initial thing
          colorBlue.resume();
        }
      }
      else if(color == 2)
      {
        if(redColorSortToggleEnabled){
          // Do another thing
          colorRed.suspend();
          if(master.get_digital(DIGITAL_R1))
          {
            intakeMotor.move_velocity(600);
          }
          else if(master.get_digital(DIGITAL_R2))
          {
            intakeMotor.move_velocity(-600);
          }
          else
          {
            intakeMotor.move_velocity(0);
          }

        }
        else{
          // Do initial thing
          colorRed.resume();
        }
      }
      else
      {
        if(master.get_digital(DIGITAL_R1))
          {
            intakeMotor.move_velocity(600);
          }
          else if(master.get_digital(DIGITAL_R2))
          {
            intakeMotor.move_velocity(-600);
          }
          else
          {
            intakeMotor.move_velocity(0);
          }
      }
    wallStake.set_brake_mode_all(MOTOR_BRAKE_HOLD);
      
    // if(master.get_digital(DIGITAL_L1))
    // {
    //   wallStake.move_velocity(600);
    // }    
    // else
    // {
    //   wallStake.move_velocity(0);
    // }
    if(master.get_digital_new_press(DIGITAL_L2))
    {
      wallStake.move_absolute(-130,600);
    }    
    else if(master.get_digital_new_press(DIGITAL_RIGHT))
    {
      nextState();
    }


    if(clampToggleEnabled){
      // Do another thing
      Clamper.set_value(1);
    }
    else{
      // Do initial thing
      Clamper.set_value(0);
    }

    if(doinkerToggleEnabled){
      // Do another thing
      doinker.set_value(1);
    }
    else{
      // Do initial thing
      doinker.set_value(0);
    }

    // if (master.get_digital(DIGITAL_UP) && master.get_digital(DIGITAL_X)) {
    //   while(wallSense.get_distance() <= 350)
    //   {
    //     drive.move_velocity(-100);
    //   }
    //   drive.move_velocity(0);
    //   wallStake.move_absolute(1450, 600);
    // }


    //chassis.opcontrol_tank();  // Tank control
     chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    //chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
     //chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
