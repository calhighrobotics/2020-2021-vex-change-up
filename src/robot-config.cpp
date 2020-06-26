#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT2, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);

motor rightMotorA = motor(PORT9, ratio18_1, true); 
motor rightMotorB = motor(PORT10, ratio18_1, true); 
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);

drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 457.2, 457.2, mm, 1);
controller Controller1 = controller(primary);
motor intake1 = motor(PORT7, ratio6_1, true);
motor intake2 = motor(PORT3, ratio6_1, true);
motor intake3 = motor(PORT4, ratio6_1, true);
motor roller1 = motor(PORT5, ratio6_1, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1RightShoulderControlMotorsStopped = true;
bool Controller1LeftShoulderControlMotorsStopped = true;

bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_callback_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis1
      // right = Axis3 - Axis1
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor next time the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
      //for our roller to shoot the ball. check the button L1/l2 status to controll roller1
      if(Controller1.ButtonL1.pressing()){
        intake3.spin(reverse);
        roller1.spin(reverse); 
        intake3.setVelocity(-600, rpm);
        roller1.setVelocity(-600, rpm); 
        Controller1LeftShoulderControlMotorsStopped = false; 
      }else if (Controller1.ButtonL2.pressing()) {
         intake3.spin(forward);
        roller1.spin(forward); 
        intake3.setVelocity(600, rpm);
        roller1.setVelocity(600, rpm); 
        Controller1LeftShoulderControlMotorsStopped = false; 
      }else {
        intake3.stop(); 
        roller1.stop(); 
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
       Controller1LeftShoulderControlMotorsStopped = true;
      }
      // check the ButtonR1/ButtonR2 status to control intake1
      if (Controller1.ButtonR1.pressing()) {
        intake1.spin(reverse);
        intake2.spin(reverse);
        intake3.spin(reverse);
        intake1.setVelocity(-600, rpm);
        intake2.setVelocity(600, rpm); 
        intake3.setVelocity(600, rpm);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        intake1.spin(forward);
        intake2.spin(forward); 
        intake3.spin(forward); 
        intake1.setVelocity(600, rpm);
        intake2.setVelocity(-600, rpm);
        intake3.setVelocity(-600, rpm); 
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        intake1.stop();
        intake2.stop(); 
        intake3.stop(); 
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_callback_Controller1);
}