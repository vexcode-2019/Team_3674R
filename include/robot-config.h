using namespace vex;

extern brain Brain;

// VEXcode devices
extern pot LiftPot;
extern motor LiftMotor;
extern motor BackLeftMotor;
extern motor BackRightMotor;
extern motor FrontRightMotor;
extern motor FrontLeftMotor;
extern motor LeftRollerMotor;
extern motor RightRollerMotor;
extern pot TrayPot;
extern controller Controller1;
extern motor TrayMotor;
extern inertial InternalSensor;
extern line CubePositioned;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );