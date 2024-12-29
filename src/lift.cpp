#include "main.h"

void set_lift(int input) {
  lift.move(input);
}

void lift_task() {
    pros::delay(2000);  // Optional delay for task initialization
    while (true) {
        double scaled_position = rotation.get_position() / 3.0; 
        set_lift(liftPID.compute(scaled_position)); 

        pros::delay(ez::util::DELAY_TIME);  // Maintain consistent loop timing
    }
}

//auton
void lift_wait() {
  while (liftPID.exit_condition({lift}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}


