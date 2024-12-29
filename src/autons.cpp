#include "main.h"

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  // https://ez-robotics.github.io/EZ-Template/tutorials/tuning_constants
  chassis.pid_drive_constants_set(20, 0.0, 100);          // Fwd/rev constants, used for odom and non odom motions 20, 0, 100
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom 11, 0, 20
  chassis.pid_turn_constants_set(3.0, 0.005, 20.0, 15.0);     // Turn in place constants i = .005 d = 20 3, 19, 15
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants 
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions 6.5 0 52.5
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions 5.8 0 32.5

  // Exit conditions
  // https://ez-robotics.github.io/EZ-Template/tutorials/tuning_exit_conditions
  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 200_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  // https://ez-robotics.github.io/EZ-Template/tutorials/slew_constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.3);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void skills() {
  intake.move(127);
  pros::delay(200);
  intake.move(0);
  chassis.pid_odom_set(13_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set({24_in, 13_in}, rev, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(-15_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  mogo.set(true);
  pros::delay(200);
  set_intake(127);
  chassis.pid_odom_set({{{20_in, 35_in}, fwd, 90},
                      {{42_in, 59_in}, fwd, 110},
                      {{48_in, 83_in}, fwd, 90},
                      {{56_in, 60_in}, fwd, 90},
                      {{60_in, 40_in}, fwd, 90}},
                     false);
  chassis.pid_wait_until_index(3);
  intake.move(0);
  liftPID.target_set(3300);
  lift_wait();
  chassis.pid_wait();
  chassis.pid_odom_set(-3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  
  // chassis.pid_turn_set({71_in, 43_in}, fwd, TURN_SPEED);
 
  set_intake(0);
  pros::delay(100);
  liftPID.target_set(16000);


}

void red_right_rush() {
  chassis.drive_angle_set(0_deg);
  arm.set(true);
  chassis.pid_odom_set({{{-2.75_in, 31.6_in}, fwd, 127}}, true);
  chassis.pid_wait_quick_chain();
  arm.set(false);
  chassis.pid_odom_set(-10_in, DRIVE_SPEED, true);
  //  chassis.pid_odom_set({{{-1.5_in, 28_in}, rev, 127}}, true);
  //  chassis.pid_odom_set({{{-1.5_in, 22_in}, rev, 110},
  //                     {{-6_in, 12_in}, rev, 110},},
  //                    true);
                     
   chassis.pid_wait();
   chassis.pid_turn_set({-3_in, 28_in}, rev, TURN_SPEED);
   chassis.pid_wait();
   chassis.pid_odom_set(-10_in, DRIVE_SPEED, true);
  //  chassis.pid_odom_set({{{-5_in, 28_in}, rev, DRIVE_SPEED}}, true);
  chassis.pid_wait();
}

void red_solo_awp() {
  intake.move(127);
  liftPID.target_set(3300);
  chassis.pid_turn_set({-15_in, 15_in}, fwd, TURN_SPEED);
  chassis.pid_wait();
  intake.move(0);
  chassis.pid_odom_set(3.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  liftPID.target_set(21000);
  pros::delay(400);
  chassis.pid_odom_set({{{6_in, -15_in}, rev, 110},
                      {{11_in, -24_in}, rev, 70},},
                     true);
  chassis.pid_wait_quick_chain(); //e
  mogo.set(true);
  chassis.pid_odom_set(-9_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  liftPID.target_set(0);
  chassis.pid_turn_set(-270_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  set_intake(127);
  chassis.pid_odom_set(24_in, 90, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{{14_in, -5_in}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-450_deg, TURN_SPEED);
  chassis.pid_wait();
  mogo.set(false);
  chassis.drive_angle_set(-90_deg);
  chassis.pid_odom_set(14_in, 50, true);
  chassis.pid_wait(); 
  chassis.pid_odom_set(16_in, 90, true);
  chassis.pid_wait_until(12_in);
  set_intake(0);
  chassis.pid_wait(); 
  chassis.pid_turn_set(23_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(-21_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  mogo.set(true);
  chassis.pid_turn_set(-95_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  set_intake(127);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-25_in, 127, true);
  chassis.pid_wait();
  set_intake(0);
  chassis.pid_turn_set(-180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(17_in, 127, true);
}

void red_left_6ring(){
  set_intake(127);
  chassis.pid_odom_set({{{-1_in, 34_in}, fwd, 127}, // get 1st ring
                      {{-5_in, 44_in}, fwd, 127}, // -5 44
                      {{1_in, 38_in}, fwd, 127},
                      {{4_in, 28.5_in}, rev, 70}}, // mogo 3.5 29
                     false);
  chassis.pid_wait();
  mogo.set(true);
  pros::delay(200);
  chassis.pid_odom_set({{{-5_in, 32_in}, fwd, DRIVE_SPEED},
                      {{-17_in, 45_in}, fwd, DRIVE_SPEED}, // 2nd ring
                      {{6_in, 33_in}, rev, DRIVE_SPEED}, //go back
                      {{-19_in, 32_in}, fwd, 90}, // 3rd ring
                      {{0_in, 25_in}, rev, DRIVE_SPEED}},
                     false);
  chassis.pid_wait();
  chassis.pid_turn_set(-133_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(40_in, 110, false);
  chassis.pid_wait();
  // pros::delay(200);
  chassis.pid_odom_set(-5_in, 40, false);
  chassis.pid_wait();
  intakelift.set(true);
  chassis.pid_odom_set(6_in, 30, false);
  chassis.pid_wait();
  chassis.pid_odom_set(-15_in, 30, false);
  chassis.pid_wait_until(-2_in);
  intakelift.set(false);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(110);
  chassis.pid_wait();  
  chassis.pid_turn_set(-265_deg, TURN_SPEED);
  chassis.pid_wait();
  arm.set(true);
  chassis.pid_odom_set(80_in, 110, false); //80
  chassis.pid_wait_until(50_in);
  set_intake(0);
  chassis.pid_wait();
  set_intake(0); 

  // chassis.pid_turn_set(-40_deg, TURN_SPEED, ez::right_turn);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_odom_set(-13_in, DRIVE_SPEED, false);
  // arm.set(false);
}

void lift_test() {
   liftPID.target_set(500);
  lift_wait();

  pros::delay(1000);

  liftPID.target_set(0);
  lift_wait();
}

void intake_test() {
  optical.set_led_pwm(50);
  set_intake(70);
}

// void red_right_rush() {
//   // chassis.drive_angle_set(0_deg);
//    chassis.pid_odom_set(-25_in, DRIVE_SPEED, true);
//    chassis.pid_wait_quick_chain();
//    chassis.pid_turn_set(-30_deg, TURN_SPEED);
//    chassis.pid_wait_quick_chain();
//    chassis.pid_odom_set(-14.5_in, DRIVE_SPEED, true);
//    chassis.pid_wait_quick_chain();
//    mogo.set(true);
//    intake.move(127);
//    chassis.pid_odom_set(5_in, DRIVE_SPEED, true);
//    chassis.pid_wait();
//    chassis.pid_turn_set(30_deg, TURN_SPEED);
//    chassis.pid_wait();
//    chassis.pid_odom_set(30_in, 70, true);
//    chassis.pid_wait_until(15_in);
//    intake.move(0);
//    chassis.pid_wait();
//    chassis.pid_turn_set(200_deg, TURN_SPEED);
//    chassis.pid_wait();
//    chassis.pid_odom_set(-15_in, DRIVE_SPEED, true);
//    chassis.pid_wait();
//     mogo.set(false);
//     pros::delay(300); 
//    chassis.pid_odom_set(28.25_in, DRIVE_SPEED, true);
//    chassis.pid_wait();
//    chassis.pid_turn_set(270_deg, TURN_SPEED);
//    chassis.pid_wait();
//    chassis.pid_odom_set(-18_in, 60, true);
//    chassis.pid_wait();
//    mogo.set(true);
//    intake.move(127);
//    pros::delay(700);
//    intake.move(0);
//    chassis.pid_turn_set(330_deg, TURN_SPEED);
//    chassis.pid_wait();
//    arm.set(true);
//    chassis.pid_odom_set(38_in, DRIVE_SPEED, true);
// }

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .