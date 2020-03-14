#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
pot LiftPot = pot(Brain.ThreeWirePort.H);
motor LiftMotor = motor(PORT14, ratio18_1, false);
motor BackLeftMotor = motor(PORT8, ratio18_1, false);
motor BackRightMotor = motor(PORT4, ratio18_1, true);
motor FrontRightMotor = motor(PORT2, ratio18_1, true);
motor FrontLeftMotor = motor(PORT6, ratio18_1, false);
motor LeftRollerMotor = motor(PORT20, ratio18_1, false);
motor RightRollerMotor = motor(PORT11, ratio18_1, true);
pot TrayPot = pot(Brain.ThreeWirePort.G);
controller Controller1 = controller(primary);
motor TrayMotor = motor(PORT12, ratio18_1, false);
inertial InternalSensor = inertial(PORT19);
line CubePositioned = line(Brain.ThreeWirePort.C);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}