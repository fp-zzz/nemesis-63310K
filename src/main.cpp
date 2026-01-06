//# 63310K-
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <sys/syslimits.h>
#include "pros/apix.h"

/*
===========================================================
VEX V5 Robot Program — PROS + LemLib Template
Author: Bhargav Trivedi
===========================================================
📘 Overview:
This template combines PROS (for low-level robot control)
and LemLib (for odometry & motion control).

Sections include:
  1. Motor Configuration
  2. Sensor Configuration
  3. Pneumatics (ADI Example)
  4. LemLib Drivetrain & Controllers
  5. Initialization / Calibration
  6. Competition Functions
  7. Driver Control
===========================================================
*/

/* 
-----------------------------------------------------------
1️⃣ MOTOR CONFIGURATION
-----------------------------------------------------------
💡 NOTES:
- Port numbers correspond to the V5 Brain labels.
- Use negative ports to reverse motor direction.
- Motors can be grouped using `pros::MotorGroup` or defined individually.
*/

// Example of standalone motor declaration
// pros::Motor exampleMotor(7, pros::MotorGears::blue, false);

//controler delcatiaroin
pros::Controller master(pros::E_CONTROLLER_MASTER);

constexpr auto in = pros::E_CONTROLLER_DIGITAL_L2;
constexpr auto out = pros::E_CONTROLLER_DIGITAL_R2;
constexpr auto intop = pros::E_CONTROLLER_DIGITAL_L1;
constexpr auto outtop = pros::E_CONTROLLER_DIGITAL_R1;
constexpr auto tounge = pros::E_CONTROLLER_DIGITAL_Y;

// Left motor group on ports 1, 2, 3 (1 & 3 reversed)
pros::MotorGroup left_motors({-2,-6,-7},pros::MotorGears::blue);

// Right motor group on ports 4, 5, 6 (5 reversed)
pros::MotorGroup right_motors({8,13,21}, pros::MotorGears::blue);

// Standalone intake motor (port 10)
pros::Motor intake(-5, pros::MotorGears::blue);
pros::Motor top(16, pros::MotorGears::blue);


/*
-----------------------------------------------------------
2️⃣ SENSOR CONFIGURATION
-----------------------------------------------------------
💡 Includes IMU, tracking wheels, encoders, etc.
- Tracking wheels measure distance and direction.
- IMU provides inertial heading tracking.
*/

pros::Imu imu(20); // IMU on port 10

/*
-----------------------------------------------------------
3️⃣ PNEUMATICS (ADI EXAMPLE)
-----------------------------------------------------------
💡 Pneumatics are controlled via ADI digital outputs.
- Setting HIGH opens the solenoid.
- Example button control included in opcontrol().
*/

pros::adi::DigitalOut clamp('A',false);  // Pneumatic clamp on ADI port A
bool clampValue = false;           // Initial state of pneumatic clamp


/*
-----------------------------------------------------------
4️⃣ LEMLIB DRIVETRAIN & CONTROLLERS
-----------------------------------------------------------
💡 LemLib handles odometry, PID control, and autonomous motion.
- Adjust parameters based on robot geometry and wheel setup.
*/

lemlib::Drivetrain drivetrain(
    &left_motors,                // Left motor group
    &right_motors,               // Right motor group
    12,                          // Track width (inches)
    lemlib::Omniwheel::NEW_275,    // Wheel type (4" omni)
    442,                         // Max RPM
    2                            // Drift (measured experimentally)
);

// Odometry sensor setup
lemlib::OdomSensors sensors(
    nullptr,    // Vertical tracking wheel 1
    nullptr,                     // Vertical tracking wheel 2 (none)
    nullptr,  // Horizontal tracking wheel 1
    nullptr,
    &imu                    // Horizontal tracking wheel 2 (none)
                             // IMU
);

// PID Controller settings for forward/backward motion
lemlib::ControllerSettings lateral_controller(
    10, 0, 3, 3,  // kP, kI, kD, anti-windup
    1, 100,        // Small error (inches), timeout (ms)
    3, 500,        // Large error (inches), timeout (ms)
    80             // Max acceleration (slew)
);

// PID Controller settings for turning
lemlib::ControllerSettings angular_controller(
    2, 0, 10, 3,  // kP, kI, kD, anti-windup
    1, 100,        // Small error (degrees), timeout (ms)
    3, 500,        // Large error (degrees), timeout (ms)
    80             // Max acceleration (slew)
);

// Create LemLib chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);


/*
-----------------------------------------------------------
5️⃣ INITIALIZATION & CALIBRATION
-----------------------------------------------------------
💡 Runs on startup to initialize devices and calibrate sensors.
*/

void initialize() {
    chassis.calibrate();  // Calibrate IMU & encoders
    clamp.set_value(false); // Ensure clamp is in initial state
    
}


/*
-----------------------------------------------------------
6️⃣ COMPETITION TEMPLATE FUNCTIONS
-----------------------------------------------------------
💡 These are standard PROS functions for competition control.
*/

void disabled() {}
void competition_initialize() {}

enum class Auto{
    Left,
    Right,
    WinPoint,
    Skills
};

constexpr Auto AutoSelect = Auto::Right;

void AutoLeft()
{
    //** All coordinates are in INCHES and are CLOSE APPROXIMARIONS 
    //Set starting position
    chassis.setPose(-45,10,90);

    //collect first 3 blocks
    chassis.turnToPoint(-40,20,750);
    chassis.moveToPoint(-40,20,900);//move to point
    chassis.turnToHeading(90,750);
    intake.move(127);
    chassis.moveToPoint(-5,20,1750,{.maxSpeed = 55}); 

    //Score in middle goal
    chassis.moveToPoint(-20,20,500,{.forwards = false});
    chassis.turnToHeading(225,750);
    chassis.moveToPoint(-10,10,750,{.forwards = false,.maxSpeed = 50});
    intake.move(63 ) && top.move(63);
    pros::delay(2000);
    top.move(0) && intake.move(127);
    
    //Get to loader
    chassis.moveToPoint(-45,45,900,{.minSpeed = 45});

    //intake from loader
    chassis.turnToHeading(270,500);
    clamp.set_value(true);
    chassis.moveToPoint(-55,45,500,{.maxSpeed = 68});
    pros::delay(1750);//Intakes

    ///Score in long goal
    chassis.moveToPoint(-20,45,750,{.forwards = false,.minSpeed = 40});
    top.move(127);
    pros::delay(1000);//Scores
}
    
void AutoRight()
{
    //** All coordinates are in INCHES and are CLOSE APPROXIMARIONS 
    //Sets position on right edge of the parking zone **Learn how the Theta works**(for Shriyans)
    chassis.setPose(-45,-10, 90); 

    //intaking the first 3 balls
    chassis.turnToPoint(-40,-20,750);//Turns to the balls
    chassis.moveToPoint(-40,-20,900);//Moves to the halfway point really fast
    chassis.turnToHeading(90,900);//Moves to the balls slowly
    intake.move(127);
    chassis.moveToPoint(-5,-20,1750,{.maxSpeed = 50});//Moves to the lower goal

    //Scoring in the low goal
    chassis.moveToPoint(-25,-25,900,{.forwards = false},false);
    chassis.turnToPoint(-10,-10,800);
    chassis.moveToPoint(-10,-10,1750,{.maxSpeed = 50});
    intake.move(-127);
    intake.move(127);
    intake.move(-127);
    pros::delay(2000);
    intake.move(0);
    //chassis.moveToPoint(-5,-5,950,{.forwards = false,.maxSpeed = 20});
   // intake.move(127);

    //Moving to the loader
    chassis.moveToPoint(-45,-40,1000,{.forwards = false,},false);

    //Intake from the loader
    chassis.turnToHeading(270,1000);
    clamp.set_value(true);
    chassis.moveToPoint(-55,-40,900,{.maxSpeed = 68});
    pros::delay(1000);

   //Scoring in the long goal
    chassis.moveToPoint(-20,-41,1000,{.forwards = false},false);
    top.move(127)&&intake.move(127);
    pros::delay(1000);



}
void AutoWinPoint()
{


}
void AutoSkills()
{
    //Sets position on the edge of the parking zone **Learn how the Theta works**(for Shriyans);
    chassis.setPose(-45,-10,360); 

    /*//Clearing the park and moving out of it
    intake.move(127); //Intake on
    chassis.moveToPoint(-65,0,3000,{.minSpeed = 45});
    chassis.moveToPoint(-42,0,750,{.forwards = false,.minSpeed = 45});*/

    //scoring in the low goal
    chassis.turnToPoint(-22.5,-23,750);
    chassis.moveToPoint(-22.5,-23,1750,{.maxSpeed = 60});
    chassis.turnToPoint(-12,-13,750);
    chassis.moveToPoint(-12,-13,750,{.maxSpeed = 50});
    intake.move(-63);
    pros::delay(2000);
    intake.move(0);
    
    //Intaking from first match loader
    chassis.moveToPoint(-43,47,1000,{.forwards = false,.minSpeed = 45});
    chassis.turnToHeading(270,750);
    clamp.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-55,-47,750,{.maxSpeed = 68});
    pros::delay(2000);

    //Get to the other side of the long goal
    chassis.moveToPoint(-43,-59,750,{.forwards = false,.minSpeed = 45});
    chassis.turnToHeading(270,750);
    chassis.moveToPoint(43,-59,1000,{.forwards = false,.minSpeed = 45});
    chassis.swingToPoint(25,-47,DriveSide::RIGHT,750);
    //chassis.moveToPoint(33.5,-47,750,{.forwards = false,.minSpeed = 45});

    //Score in the long goal
    //chassis.turnToHeading(90,750);
    top.move(127);
    pros::delay(1000);
    top.move(0);

    //Intake from second match loader
    chassis.moveToPoint(55,-47,750,{.maxSpeed = 68});
    pros::delay(1000);

    //Score again in the long goal
    chassis.moveToPoint(25,-47,750,{.forwards = false,.minSpeed = 45});\
    top.move(127);
    pros::delay(1000);
    intake.move(0) && top.move(0);
    clamp.set_value(false); 

    //Clear parking zone
    chassis.turnToPoint(61.5,-22,750);
    chassis.moveToPoint(61.5,-22,1000,{.minSpeed = 45});
    chassis.moveToPoint(61.5,22,1750,{.minSpeed = 55});

    //Intake the third match loader
    chassis.turnToPoint(43,47,750);
    chassis.moveToPoint(43,47,750);
    chassis.turnToHeading(90,750);
    clamp.set_value(true);
    intake.move(127);
    chassis.moveToPoint(55,47,750,{.maxSpeed = 68});
    pros::delay(2000);
    
    //Move to the other side of the long goal
    chassis.moveToPoint(43,59,750,{.forwards = false,.minSpeed = 45});
    chassis.turnToHeading(90,750);
    chassis.moveToPoint(-43,59,1000,{.forwards = false,.minSpeed = 45});
    chassis.swingToPoint(-25,47,DriveSide::RIGHT,750);


    //Score in the long goal again
    top.move(127);
    pros::delay(2000);
    top.move(0);

    //Intake from last match loader
    chassis.moveToPoint(-55,47,750,{.maxSpeed = 68});
    pros::delay(1000);
    
    //Final scoring set
    chassis.moveToPoint(25,47,750,{.forwards = false,.minSpeed = 45});
    top.move(127);
    pros::delay(1000);
    top.move(0) && intake.move(0);

    //Park in the parking zone
    chassis.turnToPoint(-42,0,750);
    chassis.moveToPoint(-42,0,1000);
    chassis.turnToHeading(270,750);
    chassis.moveToPoint(-65,0,750,{.minSpeed = 45});


}

void autonomous()
{
    switch(AutoSelect)
    {
        case Auto::Left:
            AutoLeft();
            break;
        case Auto::Right:
            AutoRight();
            break;
        case Auto::WinPoint:
            AutoWinPoint();
            break;
        case Auto::Skills:
            AutoSkills();
            break;
    }
}


/*
-----------------------------------------------------------
7️⃣ DRIVER CONTROL (OPCONTROL)
-----------------------------------------------------------
💡 Handles user control using the VEX Controller.
Includes examples for drive, motor control, and pneumatics.
*/

void opcontrol() {
    

    while (true) {
        // --- Drive Controls ---
        // Arcade drive (single-stick)
        //chassis.arcade(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_X));

        // Example template for clarity (not executable):
        // chassis.arcade(int throttle, int turn);

        // Arcade drive
         chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        // Example template:
      


        // --- Intake Motor Control ---
        // L2 = forward, L1 = reverse
        if (master.get_digital(in)) {
            intake.move(127);  // Full forward
        } 
        else if (master.get_digital(out)) {
            intake.move(-127); // Full reverse
        } 
        else if (master.get_digital(intop))
        {
            intake.move(127);
            top.move(MAX_INPUT);

        }
        else if(master.get_digital(outtop))
        {
            intake.move(-MAX_INPUT);
            top.move(-MAX_INPUT);
        }
        else {
            intake.brake();    // Stop (optional — can replace with .move(0))
            top.brake();
        }


        // --- Pneumatics Toggle ---
        // A button toggles the pneumatic clamp
        if (master.get_digital_new_press(tounge)) {
            clampValue = !clampValue;
            clamp.set_value(clampValue);
        }

        pros::delay(20); // Delay to reduce CPU usage
    }
}