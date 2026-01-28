//# 63310K-Nemesis
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

constexpr auto in = pros::E_CONTROLLER_DIGITAL_L2; // Intake & hold balls
constexpr auto out = pros::E_CONTROLLER_DIGITAL_R2; // Outake balls that are being held
constexpr auto intop = pros::E_CONTROLLER_DIGITAL_L1; // Outakes balls to score
constexpr auto middle = pros::E_CONTROLLER_DIGITAL_R1; // Outake balls that are held; Both motors out
constexpr auto tounge = pros::E_CONTROLLER_DIGITAL_Y; // Piston to control tongue
constexpr auto wing = pros::E_CONTROLLER_DIGITAL_RIGHT; // Piston to control wing/descore


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

// TONGUE PISTON
pros::adi::DigitalOut clamp('A');  // Pneumatic clamp on ADI port A
bool clampValue = false;           // Initial state of pneumatic clamp
bool lockT = false;

//  WING PISTON
pros::adi::DigitalOut clamp2('B'); // Pneumatic clamp on ADI port B
bool clampValue2 = false;
bool lockW = false;

// MIDDLE PISTON
pros::adi::DigitalOut clamp3('C');
bool clampValue3 = true;
bool lockM = true;

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
    450,                         // Max RPM
    2                           // Drift (measured experimentally)
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
    clamp2.set_value(false); // Ensure clamp is in initial state
    clamp3.set_value(true);
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

constexpr Auto AutoSelect = Auto::Skills;

void AutoLeft()
{
    //** All coordinates are in INCHES and are CLOSE APPROXIMARIONS 
    //Set starting position
    chassis.setPose(-45,10,90);

    //collect first 3 blocks
    chassis.turnToPoint(-40,18,750);
    chassis.moveToPoint(-40,18,900);//move to point
    chassis.turnToHeading(90,900);
    intake.move(127);
    chassis.moveToPoint(-5,18,1750,{.maxSpeed = 53}); 

    //twwo blocks
    chassis.turnToPoint(-5,45,900);
    chassis.moveToPoint(-5,45,2000,{.maxSpeed = 65});
    chassis.turnToPoint(-20,20,750);
    
    //Middle goal
    chassis.moveToPoint(-20,20,1750,{.forwards = false,.maxSpeed = 70});
    chassis.turnToPoint(-70,70,750);
    chassis.moveToPoint(-10,10,1750,{.forwards = false, .maxSpeed = 60});
    //intake.move(127);
    pros::delay(1750);

    
    //Get to loader
    chassis.turnToPoint(-40,42,1000);
    chassis.moveToPoint(-40,42,1750,{.maxSpeed = 70});
    //intake from loader
    chassis.turnToPoint(-59.70,45,900);
    clamp.set_value(true);
    chassis.moveToPoint(-59.70,45,2000,{.maxSpeed = 70});
    intake.move(127);
    pros::delay(2000);//Intakes

    ///Score in long goal
    chassis.moveToPoint(-17.75,42.5,2750,{.forwards = false,.maxSpeed = 70});
    pros::delay(1000);
    top.move(127);
    pros::delay(1000);//Scores

    //

}
    
void AutoRight()
{
    //** All coordinates are in INCHES and are CLOSE APPROXIMARIONS 
    //Sets position on right edge of the parking zone **Learn how the Theta works**(for Shriyans)
    chassis.setPose(-45,-10, 90); 

    //intaking the first 3 balls
    chassis.turnToPoint(-40,-18,750);//Turns to the balls
    chassis.moveToPoint(-40,-18,900);//Moves to the halfway point really fast
    chassis.turnToHeading(90,900);//Moves to the balls slowly
    intake.move(127);
    chassis.moveToPoint(-5,-18,1750,{.maxSpeed = 53});//Moves to the lower goal

    //two balls
    chassis.turnToPoint(-5,-45,900);
    chassis.moveToPoint(-5,-45,2000,{.maxSpeed = 65});
    
    //Scoring in the low goal
    chassis.turnToPoint(-20,-20,750);
    chassis.moveToPoint(-20,-20,1750,{.forwards = false,.maxSpeed = 70});
    chassis.turnToPoint(-10,-10,750);
    chassis.moveToPoint(-10,-10,1750,{ .maxSpeed = 60});
    intake.move(127);
    pros::delay(1750);

  

    //Moving to the loader

    chassis.turnToPoint(-40,-42,1000);
    chassis.moveToPoint(-40,-42,1750,{.maxSpeed = 70});




    //Intake from the loader
    chassis.turnToPoint(-55.75,-45,900);
     clamp.set_value(true);
     chassis.moveToPoint(-55.75,-45,2000,{.maxSpeed =70});
    intake.move(127);
    pros::delay(2000);
    chassis.moveToPoint(-17.75,-42.5,2750,{.forwards = false,.maxSpeed = 70});
    pros::delay(1000);
   top.move(127);
    pros::delay(2000);

    chassis.moveToPoint(-25,-42.5,1000,{.maxSpeed = 70});
    pros::delay(9);

   //Scoring in the long goal
   



}
void AutoWinPoint()
{


}
void AutoSkills()
{
    //Sets position on the edge of the parking zone **Learn how the Theta works**(for Shriyans);
    chassis.setPose(-45,10,360); 

    //intake
    chassis.turnToPoint(-45,46,750);
    chassis.moveToPoint(-45,46,2000,{.maxSpeed = 70});
    chassis.turnToPoint(-59,45,1000);
    clamp.set_value(true);
    intake.move(127);
     chassis.moveToPoint(-57,45,2750,{.maxSpeed = 70});
     chassis.moveToPoint(-59,47,750,{.maxSpeed = 75});
     intake.move(127);
    pros::delay(2000);
    chassis.moveToPoint(-59.4,45,2750,{.maxSpeed = 75});

    //Move to score
    chassis.moveToPoint(-45,45,1000,{.forwards = false,.maxSpeed = 70});
    chassis.turnToPoint(-20,20,1000);
    intake.move(0);
    chassis.moveToPoint(-20,20,2000,{.maxSpeed = 70});
    chassis.turnToPoint(45,20,1000);
    intake.move(127);
    chassis.moveToPoint(45,20,2000,{.maxSpeed = 70});
    chassis.turnToPoint(45,39,1000);
    chassis.moveToPoint(45,39,2750,{.maxSpeed = 70});
    chassis.turnToPoint(61,39,1000);;
    chassis.moveToPoint(19,44.3,2000,{.forwards = false,.maxSpeed = 70});
    pros::delay(750);
    intake.move(-127);
    pros::delay(500);
    top.move(127) && intake.move(127);
    pros::delay(3500);

    //intake 
    top.move(0);
    chassis.turnToPoint(45,44,1000);
    chassis.moveToPoint(45,44,1000,{.maxSpeed = 70});
    chassis.turnToPoint(59,45,900);
    clamp.set_value(true);
    intake.move(127);
    chassis.moveToPoint(57,45,2750,{.maxSpeed = 70});
     chassis.moveToPoint(59,45,750,{.maxSpeed = 75});
     intake.move(127);
    pros::delay(2000);
    chassis.moveToPoint(59,45,3000,{.maxSpeed = 75});

    //score
    chassis.moveToPoint(18.5,45,2000,{.forwards = false,.maxSpeed = 70});
    pros::delay(750);
    top.move(-127) && intake.move(-127);
    pros::delay(500);
    top.move(127) && intake.move(127);
    pros::delay(3750);
    chassis.moveToPoint(33,45,1000,{.maxSpeed = 70});
    clamp.set_value(false);

   ///clear park
    top.move(0) && intake.move(0);
    clamp.set_value(false);
    chassis.turnToPoint(55,25,900);
    chassis.moveToPoint(56,25,2000,{.maxSpeed = 70});
    chassis.turnToPoint(56,-25,900);
    intake.move(127);
    chassis.moveToPoint(56,-25,4000,{.minSpeed = MAX_INPUT});
    top.move(127);

    //intake
    chassis.turnToPoint(45,-45,900);
    chassis.moveToPoint(45,-45,2000,{.maxSpeed = 70});
    chassis.turnToPoint(59.,-45,900);
    clamp.set_value(true);
    chassis.moveToPoint(57,-45,2000,{.maxSpeed = 70});
    chassis.moveToPoint(59,-45,2000,{.maxSpeed = 75});
    top.move(0)&&intake.move(127);
    pros::delay(2000);

    //Score
    chassis.moveToPoint(18,-44.5,2000,{.forwards = false,.maxSpeed = 70});
    intake.move(127) && top.move(127);
    pros::delay(3000);


    //move 
    chassis.moveToPoint(45,-45,1000,{.forwards = false,.maxSpeed = 70});
    chassis.turnToPoint(20,-20,900);
    chassis.moveToPoint(20,-20,2000,{.maxSpeed = 70});
    chassis.turnToPoint(-45,-20,900);
    chassis.moveToPoint(-45,-20,2000,{.maxSpeed = 70});
    chassis.turnToPoint(-45,-39,1000);
    chassis.moveToPoint(-45,-39,2000,{.maxSpeed = 70});

    //score
    chassis.turnToPoint(-61,-39,1000);;
    chassis.moveToPoint(-18,-44.5,2000,{.forwards = false,.maxSpeed = 70});
    intake.move(127) && top.move(127);
    pros::delay(3000);

    //intake
    chassis.turnToPoint(-59.70,-45,900);
    clamp.set_value(true);
    chassis.moveToPoint(-59.70,-45,2000,{.maxSpeed = 60});
    top.move(0) && intake.move(127);
    pros::delay(3000);

    //score
    chassis.moveToPoint(-18,-44.5,2000,{.forwards = false,.maxSpeed = 70});
    intake.move(127) && top.move(127);
    pros::delay(2000);
    clamp.set_value(false);

    //clear park and park
    chassis.turnToPoint(-75,-15,900);
    chassis.moveToPoint(-75,-15,1750,{.maxSpeed = 70});
    chassis.turnToPoint(-75,0,900);
    intake.move(127) && top.move(127);
    chassis.moveToPoint(-75,0,2000,{.maxSpeed = 70});
    clamp.set_value(true);
    pros::delay(2000);
    clamp.set_value(false);
   

    /*//clear
    chassis.moveToPoint(45,45,1000,{.forwards = false,.maxSpeed = 70});
    clamp.set_value(false);
    chassis.turnToPoint(50,15,900);
    chassis.moveToPoint(50,15,2000,{.maxSpeed = 70});
    chassis.turnToPoint(50,-15,900);
    chassis.moveToPoint(50,-15,2000,{.maxSpeed = 80});*/




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

        // --- Intake Motor Control ---
        // L2 = forward, L1 = reverse
        if (master.get_digital(in)) {
            intake.move(127);  // Full forward
        } 
        else if (master.get_digital(out)) {
            intake.move(-127); // Full reverse
        } 
        else if (master.get_digital(intop)) {
            intake.move(127);
            top.move(MAX_INPUT);
        }
        else if(master.get_digital(middle)) {   
            intake.move(MAX_INPUT);
            top.move(-MAX_INPUT);
        }
        else {
            intake.brake();    // Stop (optional — can replace with .move(0))
            top.brake();
        }

        // --- Pneumatics Toggle ---
        //  TONGUE PISTON CONTROL
        if (master.get_digital(tounge) && !lockT) {
            clampValue = !clampValue;
            clamp.set_value(clampValue);
            lockT = true;
        }
        else if(!(master.get_digital(tounge)))
        {
            lockT = false;
        }

        //  WING PISTON CONTROL
        if (master.get_digital(wing) && !lockW) {
            clampValue2 = !clampValue2;
            clamp2.set_value(clampValue2);
            lockW = true;
        }
        else if(!(master.get_digital(wing)))
        {
            lockW = false;
        }

        // MIDDLE PISTON CONTROL
        
        if (master.get_digital(middle) && !lockM) {
            clampValue3 = !clampValue3;
            clamp3.set_value(clampValue3);
            lockM = true;
        }
        else if(!(master.get_digital(middle))) {
            clampValue3 = !clampValue3;
            lockM = false;
        }
        pros::delay(20); // Delay to reduce CPU usage
    }
}