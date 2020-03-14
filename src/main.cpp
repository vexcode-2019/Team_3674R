/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LiftPot              pot           H               
// LiftMotor            motor         14              
// BackLeftMotor        motor         8               
// BackRightMotor       motor         4               
// FrontRightMotor      motor         2               
// FrontLeftMotor       motor         6               
// LeftRollerMotor      motor         20              
// RightRollerMotor     motor         11              
// TrayPot              pot           G               bool isRed = true;
// Controller1          controller                    
// TrayMotor            motor         12              
// InternalSensor       inertial      19              
// CubePositioned       line          C               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath> //For std::abs

using namespace vex;

//THESE ARE GLOBAL VARIABLES, IGNORE
int driveFError, driveFPrevError, driveFDerivative, driveFTotalError, driveTTotalError, driveFDesired, driveTError, driveTPrevError, driveTDerivative, driveTDesired = 0;
bool resetEncoders = false;

//Autonomous Drivetrain Settings (IGNORE THIS, DEPRICATED)
double driveFkP = 0.1;
double driveFkI = 0.0;
double driveFkD = 0.0;
double driveTkP = 0.1;
double driveTkI = 0.0;
double driveTkD = 0.0;
double driveCap = 6.0;

double lerpFrequency = 0.1; //Lerping is speed smoothening. This allows the ability to transition speeds, similar to motion profiling.
double lerpDeadzone = 0.1; //This is the deadzone of the lerp

///THIS IS WHAT YOU SHOULD NOT IGNORE
bool isRed = true;
bool oneCube = false;
double redTurnTime = 0.9; // THIS IS THE TIME FOR TURNING THE ROBOT TOWARDS THE GOAL ZONE
double blueTurnTime  = 0.9;

//Driver Control Drivetrain Settings
double turnImportance = 0.4;
double driveDeadZone = 4;
int turningCurve = 5; //out of 20 being the most curved
int forwardCurve = 5; //out of 20 being the most curved

//Tray Settings (Down)
double traykPDown = 0.0001;
double traykIDown = 0.0;
double traykDDown = 0.0;
double trayDesiredDown = 2401;

//Tray Settings (Up)
double traykPUp = 0.0000000045;
double traykIUp = 0.000000000002;
double traykDUp = 0.0;
double trayDesiredUp = 30;

//Arm Settings (Down)
double armkPDown = 0.03;
double armkIDown = 0.0;
double armkDDown = 0.0;
double armDesiredDown = 50;

//Arm tower mid
double armkPUp = 0.15;
double armkIUp = 0.0;
double armkDUp = 0.15;

//Arm tower low
double armkPUpLow = 0.14;
double armkIUpLow = 0.0003;
double armkDUpLow = 0.6;

//Arm low tower up
double armLowDesiredUp = 1150;
//Arm low tower down
double armLowDesiredDown = 1030;

//Arm mid tower up
double armMidDesiredUp = 1600;
//Arm mid tower down
double armMidDesireddown = 1450;

//Line follower interval
double lineFollowerAmount = 2800;

//Global Variables
bool autonEnabled, trayEnabled, trayScoring, armScoring = false;
int trayDesired = trayDesiredDown;
int armDesired = armDesiredDown;
double armkP, armkI, armkD, traykP, traykI, traykD = 0.0;
int armError, armTotalError, armDerivative, armPrevError, armToggle, trayError, trayTotalError, trayDerivative, trayPrevError, leftLerp, rightLerp = 0;
bool armDown(){armDesired = armDesiredDown; armkP = armkPDown; armkI = armkIDown; armkD = armkDDown; return 1;}
bool setRollers(double voltageAmt){LeftRollerMotor.spin(forward, voltageAmt, voltageUnits::volt); RightRollerMotor.spin(forward, voltageAmt, voltageUnits::volt); return true; }
bool brakeRollers(){ LeftRollerMotor.stop(brakeType::hold); RightRollerMotor.stop(brakeType::hold); return true;} 
bool hasCube(){ return CubePositioned.value(analogUnits::range12bit) <= lineFollowerAmount;}
bool trayUp(){trayDesired = trayDesiredUp; traykP = traykPUp; traykI = traykIUp;traykD = traykDUp; setRollers(0.0); return 1;}
bool traydown(){ trayDesired = trayDesiredDown; traykP = traykPDown; traykI = traykIDown; traykD = traykDDown; return 1;}
bool pause(double amt) {vex::task::sleep(amt*1000); return true;}
bool armUp() {armkP = armkPUp; armkI = armkIUp; armkD = armkDUp; armDesired = armMidDesiredUp; return 1;}
bool prepareStack() { while(!hasCube()){setRollers(-12.0);pause(0.1);} brakeRollers(); return true; }
double cap(double amount) {if (amount > driveCap) return driveCap; else if (amount < -driveCap) return -driveCap; return 1;}
double resetLEEncoders(){BackLeftMotor.resetPosition();
    BackLeftMotor.resetRotation();
    BackRightMotor.resetPosition();
    BackRightMotor.resetRotation();
    FrontLeftMotor.resetPosition();
    FrontLeftMotor.resetRotation();
    FrontRightMotor.resetPosition();
    FrontRightMotor.resetRotation(); return true;}

//Autonomous code. I removed the auton because I didn't have time to make a legitimate one (3/13/2020)
/////////////////////////////////////////////////////////////////////////
bool stopEnabled, outtakeSwitch, buttonPressed = false;
int autonCode(){

  //leftLerp = 0;//This resets the motor power to 0 instead of a deceleration
  //rightLerp = 0;//This resets the motor power to 0 instead of a deceleration
  //hasCube()//This detects if a cube is detected into the rollers
  //brakeRollers()//This brakes the rollers
  //setRollers(int val)//This sets the rollers to a desired value
  //armDown()//This utilizes the Arm's PID
  //trayUp()//This commands the tray PID to bring the tray up
  //trayDown()//This commands the tray PID to bring the tray down
  //driveFDesired = int num//This commands the drivetrain pid to go forward x amount
  //driveTDesired = int num//This commands the drivetrain pid to turn deg amount
  //resetEncoders = bool amt//This commands the encoders to reset their positions
  //pause(double amt)//This commands the code to wait a specific amount in seconds
  //armUp()//This commands the arms to go up
  //prepareStack()//This command prepares the stack within the robot for scoring.
  
 
  return 1;
}
/////////////////////////////////////////////////////////////////////////

//This caps the motor power
double capPower(double amount) { if (amount > 12.0) amount = 12.0; else if(amount < -12.0) amount = -12.0; return amount;}

//Global variables
double lerpLeftDrivePower(double amount){
  amount = capPower(amount);

  if(std::abs(std::abs(amount) - std::abs(leftLerp)) < lerpDeadzone/2) return amount;
  else if(amount > leftLerp) leftLerp += lerpFrequency;
  else leftLerp -= lerpFrequency;

  return leftLerp;
}

//Global variables
double lerpRightDrivePower(double amount){
  amount = capPower(amount);

  if(std::abs(std::abs(amount) - std::abs(rightLerp)) < lerpDeadzone/2) return amount;
  else if(amount > rightLerp) rightLerp += lerpFrequency;
  else leftLerp -= lerpFrequency;

  return rightLerp;
}


bool setMotorPower(double forwardV, double turnV){//This is a function that can be called, and they must always return something
  FrontLeftMotor.spin(forward, cap(forwardV + turnV), voltageUnits::volt); 
  BackLeftMotor.spin(forward, cap(forwardV + turnV), voltageUnits::volt);
  FrontRightMotor.spin(forward, cap(forwardV - turnV), voltageUnits::volt);
  BackRightMotor.spin(forward, cap(forwardV - turnV), voltageUnits::volt);
  return 1; //I just do this because I dont need anything from returning
}

//Global variable
int fResetPos = 0;
double tResetPos = 0;

//Drivetrain PID
int drivePID(){
  while(autonEnabled){

    if(resetEncoders){
      resetEncoders = false;
      fResetPos = driveFError;
      tResetPos = driveTError;
    }
    
    int averageRotations = (FrontLeftMotor.position(rotationUnits::deg) + BackRightMotor.position(rotationUnits::deg) + BackLeftMotor.position(rotationUnits::deg) + FrontRightMotor.position(rotationUnits::deg))/4;
    driveFError = (driveFDesired - averageRotations) - fResetPos;

    //Get the derivative of the drivetrain
    driveFDerivative = driveFPrevError - driveFPrevError;
    driveFTotalError += driveFError;
    driveFPrevError = driveFError;
    double forwardVoltage = driveFError * driveFkP + driveFDerivative * driveFkD + driveFTotalError * driveFkI;
    //Brain.Screen.clearScreen();
    //Brain.Screen.print(InternalSensor.yaw());
    int turnRotations = (FrontLeftMotor.position(rotationUnits::deg) + BackLeftMotor.position(rotationUnits::deg))/2 - (BackRightMotor.position(rotationUnits::deg) + FrontRightMotor.position(rotationUnits::deg))/2;
    driveTError = ((int)driveTDesired - (int)turnRotations) - (int)tResetPos;
    driveTTotalError += driveTError;
    driveTPrevError = driveTError;
    double turnVoltage = driveTError * driveTkP + driveTDerivative * driveTkD + driveTTotalError * driveTkI;

    setMotorPower(forwardVoltage, turnVoltage);
    vex::task::sleep(20);
  }
  return 1;
}

//Arm Code PID
int armCode(){
  while(trayEnabled){

    //Error, or Potential
    armError = LiftPot.value(analogUnits::range12bit) - armDesired;
    //Deadzone integral
    if (std::abs(armError) < 20) armTotalError = 0;
    else armTotalError += armError;
    //Derivative
    armDerivative = armError - armPrevError;

    //Apply all of the values to the motor
    double liftMotorPower = armTotalError * armkI + armDerivative * armkD + armError * armkP;
    LiftMotor.spin(forward, liftMotorPower, voltageUnits::volt);

    //Arm previous error
    armPrevError = armError;
    vex::task::sleep(20);
  }
  return 1;
}

//Tray Code PID
int trayCode(){
  while(trayEnabled){

    //Error, or Potential
    trayError = pow(TrayPot.value(analogUnits::range12bit) - trayDesired,3);
    //Deadzone integral
    if (std::abs(trayError) < 50) trayTotalError = 0;
    else trayTotalError += trayError;
    //Derivative
    trayDerivative = trayError - trayPrevError;

    //Apply all of the values to the motor
    double trayMotorPower = trayTotalError * traykI + trayDerivative * traykD + trayError * traykP;
    TrayMotor.spin(forward, trayMotorPower, voltageUnits::volt);

    //Tray previous error
    trayPrevError = trayError;
    vex::task::sleep(20);
  }
  return 1;
}

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  trayEnabled = false;
  autonEnabled = false;
  vexcodeInit();
  while (1){
    InternalSensor.calibrate();
    BackLeftMotor.resetPosition();
    BackLeftMotor.resetRotation();
    BackRightMotor.resetPosition();
    BackRightMotor.resetRotation();
    FrontLeftMotor.resetPosition();
    FrontLeftMotor.resetRotation();
    FrontRightMotor.resetPosition();
    FrontRightMotor.resetRotation();
    vex::task::sleep(1000);
    InternalSensor.resetHeading();
    InternalSensor.resetRotation();
  }

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {

  //Enable both the tray and drivetrain code
  trayEnabled = true;
  autonEnabled = true;
  vex::task a(armCode);
  //vex::task d(drivePID);
  vex::task t(trayCode);

  //Run the specified autonomous
  autonCode();
}

//graph of red and blue lines from 5225A here
//https://www.desmos.com/calculator/fatajfuuxu
int curveJoystick(bool red, int input, double t){
  int val = 0;
  if(red){
    val = (std::exp(-t/10)+std::exp((std::abs(input)-100)/10)*(1-std::exp(-t/10))) * input;
  }else{
    //blue
    val = std::exp(((std::abs(input)-100)*t)/1000) * input;
  }
  return val;
}

//Detects when the drive motor power is within the deadzone amoutn
bool valueDeaded(double amount){
  return (std::abs(amount) <= (driveDeadZone/2)*0.12);}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
bool descoreMode = false;

void usercontrol(void) {

  //Enable the tray code, and disable auton's drive
  trayEnabled = true;
  autonEnabled = false;
  vex::task a(armCode);
  vex::task t(trayCode);

  // User control code here, inside the loop
  while (1) {

    //Controller values
    bool trayButton = Controller1.ButtonR1.pressing();
    bool rollerSpeedUpButton = Controller1.ButtonR2.pressing();
    bool rollerReverseButton = Controller1.ButtonL2.pressing();
    double turnVal = curveJoystick(false, Controller1.Axis1.position(percent), turningCurve);
    double forwardVal = curveJoystick(false, Controller1.Axis3.position(percent), forwardCurve);
    bool armButton = Controller1.ButtonDown.pressing();
    bool descoreButton = Controller1.ButtonB.pressing();
    bool debugButton = Controller1.ButtonUp.pressing();

    //Tray Control Code
    /////////////////////////////////////////////////////////////////////////////////////
    if(trayButton){
      trayDesired = trayDesiredUp;
      traykP = traykPUp;
      traykI = traykIUp;
      traykD = traykDUp;
      trayScoring = true;
      stopEnabled = false;
    }
    else{
      trayDesired = trayDesiredDown;
      traykP = traykPDown;
      traykI = traykIDown;
      traykD = traykDDown;
      if(rollerSpeedUpButton || rollerReverseButton) trayScoring = false;
    }

    //Arm Control Code
    /////////////////////////////////////////////////////////////////////////////////////
    if(debugButton){
      armToggle = 0;
      trayScoring = false;
      stopEnabled = false;
      trayScoring = false;
      armkP = armkPDown;
        armkI = armkIDown;
        armkD = armkDDown;
        armDesired = armDesiredDown;
        descoreMode = false; 
        trayScoring = false;
        outtakeSwitch = false;
    }

    if(armButton && !buttonPressed){
      buttonPressed = true;
      armTotalError = 0;
      armToggle += 1;
      if(armToggle > 2) {armToggle = 0; trayScoring = false;}
    }
    else if(Controller1.ButtonB.pressing() && !buttonPressed){
      armTotalError = 0;
      buttonPressed = true;
      if(descoreMode) {
        descoreMode = false; 
        trayScoring = false;
        outtakeSwitch = false;
        armkP = armkPDown;
        armkI = armkIDown;
        armkD = armkDDown;
        armDesired = armDesiredDown;
        }
      else descoreMode = true;
    }
    else if(!armButton && !descoreButton) buttonPressed = false;

    if(Controller1.ButtonA.pressing()){
      armkP = armkPUp;
      armkI = armkIUp;
      armkD = armkDUp;
      armDesired = armMidDesiredUp;
    }
    else if(descoreMode){
        armkP = armkPUpLow;
        armkI = armkIUpLow;
        armkD = armkDUpLow;
        armDesired = 1100;
    }
    else if(armToggle == 0){
      outtakeSwitch = false;
      armkP = armkPDown;
      armkI = armkIDown;
      armkD = armkDDown;
      armDesired = armDesiredDown;
    }
    else if ((armToggle == 1 || armToggle == 2) && outtakeSwitch){
      if (armToggle == 1){
        armkP = armkPUpLow;
        armkI = armkIUpLow;
        armkD = armkDUpLow;
        armDesired = armLowDesiredUp;
      }
      else{
        armkP = armkPUp;
        armkI = armkIUp;
        armkD = armkDUp;
        armDesired = armMidDesiredUp;
      }
      trayScoring = true;
    }
    else if(hasCube()){
      stopEnabled = true;
      outtakeSwitch = true;
    }
    else{
      rollerReverseButton = true;
    }



    //Rollers Control Code
    /////////////////////////////////////////////////////////////////////////////////////
    if(trayScoring == false){
      if(rollerSpeedUpButton) {setRollers(12.0); stopEnabled = false;}
      else if(rollerReverseButton || descoreMode) {
        setRollers(-6.0);
        trayScoring = true;
        stopEnabled = true;
        }
      else setRollers(6.0);
    }
    else {
      if(rollerReverseButton || descoreMode) {setRollers(-6.0); stopEnabled = true;}
      else if(stopEnabled) brakeRollers();
      else setRollers(0.0);
    }
    

    ///////////////////////////
    //Drivetrain control code//
    /////////////////////////////////////////////////////////////////////////////////////

    //Translate the percent values to voltage
    double turnVolts = turnVal * 0.12;
    double forwardVolts = forwardVal * 0.12 * ( 1 - (std::abs(turnVolts) / 12.0) * turnImportance); // The portion after the 0.12 allows more control while moving

    //Add the amount for the sides
    double leftAddedAmout = forwardVolts + turnVolts;
    double rightAddedAmout = forwardVolts - turnVolts;

    //Apply the values to the left motors
    if(valueDeaded(leftAddedAmout)){ //This is for deadzone
      BackLeftMotor.stop(brakeType::brake);
      FrontLeftMotor.stop(brakeType::brake);
    }
    else{ //Allow drivetrain if not in deadzone
      BackLeftMotor.spin(forward, leftAddedAmout, voltageUnits::volt);
      FrontLeftMotor.spin(forward, leftAddedAmout, voltageUnits::volt);
    }

    //Apply the values to the right motors
    if(valueDeaded(rightAddedAmout)){ //This is for deadzone
      BackRightMotor.stop(brakeType::brake);
      FrontRightMotor.stop(brakeType::brake);
    }
    else{ //Allow drivetrain if not in deadzone
      BackRightMotor.spin(forward, rightAddedAmout, voltageUnits::volt);
      FrontRightMotor.spin(forward, rightAddedAmout, voltageUnits::volt);
    }

    /////////////////////////////////////////////////////////////////////////////////////



    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
