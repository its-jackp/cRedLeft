#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// Define your global instances of motors and other devices here
motor leftMotorA = motor(PORT6, ratio18_1, true);
motor leftMotorB = motor(PORT3, ratio18_1, true);
motor rightMotorA = motor(PORT11, ratio18_1, false);
motor rightMotorB = motor(PORT8, ratio18_1, false);
motor_group rightMotors = motor_group(rightMotorA, rightMotorB);
motor_group leftMotors = motor_group(leftMotorA, leftMotorB);
controller controller1 = controller();
brain brain1;
motor intake = motor(PORT14, ratio18_1, false);
pneumatics pneumatic = pneumatics(brain1.ThreeWirePort.H);
pneumatics pneumatic_doink = pneumatics(brain1.ThreeWirePort.G);
float rotations;
float speed;
//bool vex::brain::lcd::drawImageFromFile(const char *filename, int x, int y);

void driveForward(float(rotations), float(speed)) {
leftMotors.spinFor(forward, rotations, rotationUnits::rev, speed, velocityUnits::pct, false);
rightMotors.spinFor(forward, rotations, rotationUnits::rev, speed, velocityUnits::pct, true);
}  

void driveReverse(float(rotations), float(speed)) {
leftMotors.spinFor(reverse, rotations, rotationUnits::rev, speed, velocityUnits::pct, false);
rightMotors.spinFor(reverse, rotations, rotationUnits::rev, speed, velocityUnits::pct, true);
} 

void rightTurn(float(rotations), float(speed)) {
leftMotors.spinFor(forward, rotations, rotationUnits::rev, speed, velocityUnits::pct, false);
rightMotors.spinFor(reverse, rotations, rotationUnits::rev, speed, velocityUnits::pct, true);
} 

void leftTurn(float(rotations), float(speed)) {
leftMotors.spinFor(reverse, rotations, rotationUnits::rev, speed, velocityUnits::pct, false);
rightMotors.spinFor(forward, rotations, rotationUnits::rev, speed, velocityUnits::pct, true);
} 


void driveStop(){
    rightMotors.stop();
    leftMotors.stop();
}

// Function to apply a quadratic input curve
double applyCurve(double input) {
    double scaledInput = input / 100.0; // Scale to range -1.0 to 1.0
    double curved = scaledInput * scaledInput * (scaledInput < 0 ? -1 : 1); // Quadratic curve
    return curved * 100.0; // Scale back to range -100 to 100
}

//Pre-auton
void pre_auton(void) {
  // Initialization tasks before competition starts
    pneumatic_doink.open();
  // Example: clearing encoders, setting servo positions, ...
}

//Auton
void autonomous(void) {
  // Autonomous code here;
    //pneumatic_doink.open();
    intake.setVelocity(100, percent);
    leftMotors.setStopping(brake);
    rightMotors.setStopping(brake);
    driveReverse(1.55, 20);
    pneumatic.open();
    wait(0.5, seconds);
    rightTurn(0.24, 30);
    intake.spin(reverse);
    driveForward(1.1, 30);
    leftTurn(0.76, 30);
    driveForward(1.5, 30);
    //driveForward(0.4, 50);
    wait(0.3, sec);
    //rightTurn(0.5, 30);
    wait(0.25, sec);
    //driveReverse(0.25,30);
    //leftTurn(0.5, 30);
    //driveForward(0.4, 30);


    
    
  


  // Example: move robot forward
}

//Driver control
void usercontrol(void) {
    bool isExtended = false;  // Track the current state of the pneumatics
    bool buttonPressed = false;
    bool buttonR2Pressed;
    bool intakeIn = false;
    bool buttonL2Pressed = false;
    bool intakeOut = false;
    bool buttonL1Pressed = false;
    bool doinkExtended; // Track if the button is currently pressed
    intake.setVelocity(100, percent);
    bool isRunning = false;
    brain1.Screen.clearScreen();


    while (1) {
        // Drive Control (Tank Drive)
        double leftInput = controller1.Axis3.position();  // Left joystick (forward/backward)
        double rightInput = controller1.Axis2.position(); // Right joystick (forward/backward)

        // Apply input curve
        double leftOutput = applyCurve(leftInput);
        double rightOutput = applyCurve(rightInput);

        // Set motor speeds for left and right sides
        leftMotorA.spin(vex::directionType::fwd, leftOutput, vex::velocityUnits::pct);
        leftMotorB.spin(vex::directionType::fwd, leftOutput, vex::velocityUnits::pct);
        rightMotorA.spin(vex::directionType::fwd, rightOutput, vex::velocityUnits::pct);
        rightMotorB.spin(vex::directionType::fwd, rightOutput, vex::velocityUnits::pct);

        // Intake Control
        //if (controller1.ButtonL1.pressing()) {
          //  intake.spin(reverse);
        //} else if (controller1.ButtonL2.pressing()) {
          //  intake.spin(forward);
        //} else {
          //  intake.stop();
        //}

        // Pneumatic Control (ButtonR1 toggles pneumatic state)
        if (controller1.ButtonR1.pressing()) {
            if (!buttonPressed) {
                isExtended = !isExtended;          // Toggle the pneumatic state
                pneumatic.set(isExtended); // Set solenoid to the new state
                buttonPressed = true;             // Mark button as pressed
            }
        } else {
            buttonPressed = false; // Reset button pressed state when the button is released
        }

        if (controller1.ButtonR2.pressing()) {
            if (!buttonR2Pressed) {
                doinkExtended = !doinkExtended;          // Toggle the pneumatic state
                pneumatic_doink.set(doinkExtended); // Set solenoid to the new state
                buttonR2Pressed = true;             // Mark button as pressed
            }
        } else {
            buttonR2Pressed = false; // Reset button pressed state when the button is released
        }
    
        if (controller1.ButtonL1.pressing()) {
            if (!buttonL1Pressed) {
                intakeIn = !intakeIn;          // Toggle the pneumatic state
                buttonL1Pressed = true;             // Mark button as pressed
            }
        } else {
            buttonL1Pressed = false; // Reset button pressed state when the button is released
        }
        
        if (controller1.ButtonL2.pressing()) {
            if (!buttonL2Pressed) {
                intakeOut = !intakeOut;          // Toggle the pneumatic state
                buttonL2Pressed = true;             // Mark button as pressed
            }
        } else {
            buttonL2Pressed = false; // Reset button pressed state when the button is released
        }

        if (intakeIn == true) {
            intake.spin(reverse);
        } else if (intakeOut == true) {
            intake.spin(forward);
        } else {
            intake.stop();
        }
    }

}

//Main
int main() {
    // Set up callbacks for autonomous and driver control periods
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    // Run the pre-autonomous function
    pre_auton();

    // Prevent main from exiting with an infinite loop
    while (true) {
        wait(100, msec);
    }
}