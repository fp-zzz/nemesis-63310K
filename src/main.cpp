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
bool clampValue3 = false;
bool lockM = false;

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
    2.5, 0.7, 50, 4,  // kP, kI, kD, anti-windup
    1.5, 600,        // Small error (degrees), timeout (ms)
    4, 900,        // Large error (degrees), timeout (ms)
    20           // Max acceleration (slew)
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
    Skills,
    Test,
    Alliance
};

constexpr Auto AutoSelect = Auto::Right;

void AutoLeft()
{
     //** All coordinates are in INCHES and are CLOSE APPROXIMARIONS 
    //Sets position on right edge of the parking zone **Learn how the Theta works**(for Shriyans)
    chassis.setPose(-45,10, 90); 

    /*//intaking the first 3 balls
    chassis.turnToPoint(-40,17.5,750,{},false );//Turns to the balls
    chassis.moveToPoint(-40,17.5,900,{},false);//Moves to the halfway point really fast
    chassis.turnToHeading(90,900,{},false);//Moves to the balls slowly
    intake.move(127);
    chassis.moveToPoint(-6,20,1750,{.maxSpeed = 75},false);//Moves to the lower goal

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
    chassis.moveToPoint(-40,17.5, 1000,{.forwards = false,.maxSpeed = 85},false);
    chassis.turnToPoint(-40,42,1000);
    chassis.moveToPoint(-40,42,1750,{.maxSpeed = 70},false);
    intake.move(127);




    //Intake from the loader
    chassis.turnToPoint(-56,42,900);
    clamp.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-57.5,42,1750,{.maxSpeed = 70},false);
    chassis.moveToPoint(-59.5,42,750,{.maxSpeed = 75},false);
    intake.move(127);
    pros::delay(750);
    

    //Scoring in the long goal
    // chassis.moveToPoint(-35,-43.2,1750,{.forwards = false,.maxSpeed = 70},false);
    chassis.moveToPoint(-20.5,41.9,1750,{.forwards = false,.maxSpeed = 70},false);
    //clamp.set_value(false);
    intake.move(-127);
    pros::delay(450);
    top.move(127)&& intake.move(127);//Scores8*/


//new version 
    //set pose
    chassis.setPose(-45,10, 90); 

    //intake three balls
    chassis.turnToPoint(-21,21,500,{},false);
    intake.move(127);
    chassis.moveToPoint(-21,21,750,{.maxSpeed = 86},false);
    chassis.moveToPoint(-20,20,750,{.forwards = false},false);
    intake.move(0);


    //Score middle goal
    chassis.turnToHeading(315,750,{},false);
    chassis.moveToPoint(-9,0.8,1000,{.forwards = false,.maxSpeed = 75},false);
    clamp3.set_value(false) && intake.move(127);
    pros::delay(1600);

    //Move to loader
    chassis.turnToPoint(-40,41,750,{},false);
    chassis.moveToPoint(-40,41,1000,{.maxSpeed = 140},false);
    clamp3.set_value(true);
    clamp.set_value(true);
    chassis.turnToPoint(-57,41,750,{},false);

    //Intake from loader
    intake.move(127);
    chassis.moveToPoint(-57,41,1000,{.maxSpeed = 90},false);
    chassis.moveToPoint(-59,41,750,{.maxSpeed = 95},false);
    pros::delay(20);
    
    //Score
    chassis.moveToPoint(-20,41,750,{.forwards = false,.maxSpeed = 90},false);
    intake.move(127) && top.move(127);
    pros::delay(900);

    //descore
    chassis.moveToPoint(-30,42,750,{.maxSpeed = 70},false);
    chassis.turnToPoint(-30,31.16,750,{},false);
    chassis.moveToPoint(-30,31.16,750,{.maxSpeed = 70},false);
    chassis.turnToHeading(270,750,{},false);
    clamp2.set_value(true);
    chassis.moveToPoint(-20,31.16,1000,{.forwards = false,.maxSpeed = 70},false);  
    clamp2.set_value(false);
    chassis.moveToPoint(-9,31.16,750,{.forwards = false,.maxSpeed = 100},false);
}
    
void AutoRight()
{
    //** All coordinates are in INCHES and are CLOSE APPROXIMARIONS 
    //Sets position on right edge of the parking zone **Learn how the Theta works**(for Shriyans)
    chassis.setPose(-45,-10, 90); 

    //intaking the first 3 balls
    chassis.turnToPoint(-20,-20,750,{},false );//Turns to the balls
    intake.move(127);
    chassis.moveToPoint(-20,-20,750,{},false);//Moves to the halfway point really fast
    chassis.turnToHeading(90,900,{},false);//Moves to the balls slowly
    
   // chassis.moveToPoint(-13,-20,1000,{.maxSpeed = 85},false);//Moves to the lower goal

    /*//two balls
    chassis.turnToPoint(-5,-45,900);
    chassis.moveToPoint(-5,-45,2000,{.maxSpeed = 65});
    
    //Scoring in the low goal
    chassis.turnToPoint(-20,-20,750);
    chassis.moveToPoint(-20,-20,1750,{.forwards = false,.maxSpeed = 70});
    chassis.turnToPoint(-10,-10,750);
    chassis.moveToPoint(-10,-10,1750,{ .maxSpeed = 60});
    intake.move(127);
    pros::delay(1750);
*/
  

    //Moving to the loader
    chassis.moveToPoint(-40,-17.5, 900,{.forwards = false,.maxSpeed = 85},false);
    chassis.turnToPoint(-40,-42,750,{},false);
    chassis.moveToPoint(-40,-42,750,{.maxSpeed = 70},false);




    /*//Intake from the loader
    chassis.turnToPoint(-53,-45,900);
    clamp.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-53.5,-45,1750,{.maxSpeed = 70},false);
    chassis.moveToPoint(-55.5,-45,750,{.maxSpeed = 75},false);
    intake.move(127);
    pros::delay(2000);*/
    

    //Scoring in the long goal
    // chassis.moveToPoint(-35,-43.2,1750,{.forwards = false,.maxSpeed = 70},false);
    chassis.turnToHeading(270,750,{},false);
    chassis.moveToPoint(-20,-44.5,1000,{.forwards = false,.maxSpeed = 70},false);
    //clamp.set_value(false);
    //intake.move(-127);
    pros::delay(450);
    top.move(127)&& intake.move(127);//Scores
    pros::delay(1500);

    //wing
    chassis.moveToPoint(-30,-45,750,{.maxSpeed = 90},false);
    chassis.turnToPoint(-30,-26.25,750);
    chassis.moveToPoint(-30,-26.25,750,{.maxSpeed = 90},false);
    chassis.turnToHeading(90,750,{},false);
    clamp2.set_value(true);
    chassis.moveToPoint(-10,-26.25,1000,{.maxSpeed = 70},false);
    clamp2.set_value(false);
    chassis.moveToPoint(-5,-26.25,750,{.maxSpeed = 90},false);
}
void AutoWinPoint()
{
    chassis.setPose(-45,-10,180);
    chassis.turnToPoint(-45,-47,750,{},false);
    chassis.moveToPoint(-45,-47,1000,{.maxSpeed = 140},false);
    
    //intake from loader
    clamp.set_value(true);
    intake.move(127);
    chassis.turnToPoint(-57,-47,500,{},false);
    chassis.moveToPoint(-57,-47,900,{.maxSpeed = 90},false);
    chassis.moveToPoint(-59,-47,750,{.maxSpeed = 95},false);
    intake.move(127);

    //Score
    chassis.moveToPoint(-35,-49,800,{.forwards = false,.maxSpeed = 140},false);
    chassis.moveToPoint(-20,-50,750,{.forwards = false,.maxSpeed = 100},false);
    clamp.set_value(false);
    top.move(127)&& intake.move(127);


    //middle goal
    chassis.moveToPoint(-35,-50,750,{.maxSpeed = 140},false);
    chassis.turnToPoint(-20,-20,500);
    chassis.moveToPoint(-20,-20,750,{.maxSpeed = 140},false);
    chassis.turnToPoint(-20,20,500);
    chassis.moveToPoint(-20,20,750,{.maxSpeed = 140},false);
    chassis.turnToHeading(315,500);
    chassis.moveToPoint(-10,10,750,{.forwards = false,.maxSpeed = 100},false);
    clamp3.set_value(true);
    intake.move(127);
    pros::delay(900);

    //Match load
    clamp3.set_value(false);
    chassis.turnToPoint(-45,42,500,{},false);
    chassis.moveToPoint(-45,42,750,{.maxSpeed = 140},false);
    chassis.turnToPoint(-57.5,42,500,{},false);
    clamp.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-57.5,42,750,{.maxSpeed = 90},false);
    chassis.moveToPoint(-59.5,42,750,{.maxSpeed = 95},false);
    intake.move(127);

    //Score
     chassis.moveToPoint(-35,41.9,900,{.forwards = false,.maxSpeed = 140},false);
     chassis.moveToPoint(-20,41.9,750,{.forwards = false,.maxSpeed = 100},false);
    //clamp.set_value(false);
    pros::delay(450);
    top.move(127)&& intake.move(127);
    

    



}
void AutoSkills()
{
    //Sets position on the edge of the parking zone **Learn how the Theta works**(for Shriyans);
    chassis.setPose(-45,10,360); 

    //intake
    chassis.turnToPoint(-45,46,750);
    chassis.moveToPoint(-45,46,1750,{.maxSpeed = 70},false);
    clamp.set_value(true);
    chassis.turnToPoint(-59,45,750,{},false);
    intake.move(127);
    chassis.moveToPoint(-57,45,1000,{.maxSpeed = 90},false);
    chassis.moveToPoint(-58.5,45.5,900,{.maxSpeed = 95},false);
    intake.move(127);
    pros::delay(1000);
    intake.move(127);
    chassis.moveToPoint(-59.75,45,1000,{.maxSpeed = 95});

    //Move to score
    chassis.moveToPoint(-45,45,1000,{.forwards = false,.maxSpeed = 70});
    chassis.turnToPoint(-20,20,750,{},false);
    intake.move(0);
    chassis.moveToPoint(-20,20,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(45,20,750);
    intake.move(127);
    chassis.moveToPoint(45,20,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(45,40,750);
    chassis.moveToPoint(45,40,750,{.maxSpeed = 140},false);
    chassis.turnToHeading(90,750,{},false);
    chassis.moveToPoint(21,47.5,750,{.forwards = false,.maxSpeed = 70},false);
   // intake.move(-127);
    pros::delay(600);
    intake.move(127) && top.move(127);
    pros::delay(450);
    intake.move(-127);
    pros::delay(650);
    top.move(127) && intake.move(127);
    pros::delay(3500);

    //intake 
    top.move(0)&&intake.move(127);
    chassis.turnToPoint(45,41,1000);
    chassis.moveToPoint(45,41,1000,{.maxSpeed = 70});
    chassis.turnToPoint(59,41,900);
    clamp.set_value(true);
    intake.move(127);
    chassis.moveToPoint(59,41,1750,{.maxSpeed = 90},false);
    chassis.moveToPoint(61.5,41,1000,{.maxSpeed = 95},false);
    intake.move(127);
    pros::delay(850);
   // intake.move(-127);
    //pros::delay(600);
   // intake.move(127)&& top.move(0);
    chassis.moveToPoint(62,40,1000,{.maxSpeed = 95});

    //score
    chassis.moveToPoint(20.5,47.5,900,{.forwards = false,.maxSpeed = 70},false);
    pros::delay(650);
   intake.move(127)&& top.move(127);
    pros::delay(500);
   intake.move(-127);
   pros::delay(650);
    top.move(127) && intake.move(127);
    pros::delay(3500);
    //clamp.set_value(false);
    

    /*//move To Park
    chassis.turnToPoint(20,20,1000);
    chassis.moveToPoint(20,20,2000,{.maxSpeed = 70}, false);
    chassis.turnToPoint(-30,20,1000);   
    chassis.moveToPoint(-30,20,2000,{.maxSpeed = 70}, false);
    chassis.turnToPoint(-30,-2,1000);
    intake.move(127) && top.move(127);
    chassis.moveToPoint(-30,-2,2000,{.maxSpeed = 70},  false);
    chassis.turnToPoint(-70,-2,900);
    chassis.moveToPoint(-70,-2,900);
   chassis.moveToPoint(-70,-2,1750,{.minSpeed = MAX_INPUT},false);*/

   /*//clear park
    top.move(0) && intake.move(0);
    clamp.set_value(false);
    chassis.turnToPoint(55,25,900);
    chassis.moveToPoint(56,25,2000,{.maxSpeed = 70});
    chassis.turnToPoint(56,-25,900);
    intake.move(127);
    chassis.moveToPoint(56,-25,4000,{.minSpeed = MAX_INPUT});
    top.move(127);*/

    //moving
   chassis.moveToPoint(45,-45,1000,{.maxSpeed = 70});
   chassis.turnToPoint(25,25,750,{},false);
    chassis.moveToPoint(25,25,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(25,-25,750,{},false);
    chassis.moveToPoint(25,-25,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(45,-53,750,{},false);
    chassis.moveToPoint(45,-53,2750,{.maxSpeed = 140},false);
    clamp.set_value(true);
    chassis.turnToPoint(57,-53,750,{},false);

    //intake
    intake.move(127)&&top.move(0);
    chassis.moveToPoint(57,-53,1750,{.maxSpeed = 90},false);
    chassis.moveToPoint(59,-53,1000,{.maxSpeed = 95});
    intake.move(127);
    pros::delay(1750);
    intake.move(0);


    //Mov
    chassis.moveToPoint(45,-53,750,{.forwards = false,.maxSpeed = 70});
    chassis.turnToPoint(20,-25,750);
    chassis.moveToPoint(20,-25,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(-45,-25,750);
    chassis.moveToPoint(-45,-25,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(-45,-45,1000);
    chassis.moveToPoint(-45,-45,2000,{.maxSpeed = 70});

    //score
    chassis.turnToHeading(270,750,{},false);
    chassis.moveToPoint(-20.5,-47,900,{.forwards = false,.maxSpeed = 70},false);
    pros::delay(750);
    intake.move(127)&& top.move(127);
    pros::delay(650);
    intake.move(-127);
    pros::delay(650);
    top.move(127) && intake.move(127);
    pros::delay(2900);

   /*/ //intake
    top.move(0)&&intake.move(-127);
    pros::delay(600);
    intake.move(127);
    chassis.turnToPoint(-45,-43,750);
    chassis.moveToPoint(-45,-43,750,{.maxSpeed = 70});
    chassis.turnToPoint(-59,-43,750);
    clamp.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-60,-43,1000,{.maxSpeed = 90},false);
    chassis.moveToPoint(-61.75,-43,900,{.maxSpeed = 95},false);
    intake.move(127);
    pros::delay(2000);

    //score
    chassis.moveToPoint(-21,-47,900,{.forwards = false,.maxSpeed = 70},false);
    pros::delay(750);
    top.move(127) && intake.move(127);
    pros::delay(2750);
    clamp.set_value(false);*/

    //park
    clamp.set_value(false);
    chassis.moveToPoint(-35,-45,750,{.maxSpeed = 70});
    chassis.turnToPoint(-20,20,750,{},false);
    chassis.moveToPoint(-20,20,750,{.maxSpeed = 140});
    chassis.turnToPoint(-20,8,900);
    chassis.moveToPoint(-20,8,750,{.maxSpeed = 140},false);
    chassis.turnToPoint(-70,8,900);
    intake.move(127) && top.move(127);
    chassis.moveToPoint(-70,8,4000,{.minSpeed = 140});


    /*/move 
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
   

   * //clear
    chassis.moveToPoint(45,45,1000,{.forwards = false,.maxSpeed = 70});
    clamp.set_value(false);
    chassis.turnToPoint(50,15,900);
    chassis.moveToPoint(50,15,2000,{.maxSpeed = 70});
    chassis.turnToPoint(50,-15,900);
    chassis.moveToPoint(50,-15,2000,{.maxSpeed = 80});
*/
}

void AllianceWinPoint()
{
    chassis.setPose(-45,10,360);
    chassis.moveToPoint(-45,25,1000, {.maxSpeed = 40},false);
    chassis.turnToHeading(90, 1000, {.maxSpeed = 40},false);
    chassis.moveToPoint(-55,25,1000, {.forwards = false,.maxSpeed = 40});
}


void TestAuto(){
    chassis.setPose(0, 0, 0);

    chassis.moveToPoint(0, 24, 3000, {.maxSpeed = 40});// comment 1
    chassis.turnToHeading(90, 1000, {.maxSpeed = 40});// comment 2
    chassis.moveToPoint(12, 24, 3000, {.maxSpeed = 40});// comment 3
    chassis.moveToPoint(24,24,3000, {.maxSpeed = 40});// comment 4
    chassis.turnToHeading(0, 1000, {.maxSpeed = 40});// comment 5
    chassis.moveToPoint(24, 36, 3000, {.maxSpeed = 40});// comment 6
    chassis.moveToPoint(24,48,3000, {.maxSpeed = 40});// comment 7
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
        case Auto::Test:
            TestAuto();
            break;
        case Auto::Alliance:
            AllianceWinPoint();
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
            lockM = false;
        }
        pros::delay(20); // Delay to reduce CPU usage
    }
}