//# 63310K-
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <sys/syslimits.h>
#include "pros/apix.h"

LV_IMAGE_DECLARE(WIN_20250904_16_05_21_Pro);

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

//controls
constexpr auto in = pros::E_CONTROLLER_DIGITAL_L2
constexpr auto out = pros::E_CONTROLLER_DIGITAL_R2
constexpr auto intop = pros::E_CONTROLLER_DIGITAL_L1
constexpr auto outtop = pros::E_CONTROLLER_DIGITAL_R1
constexpr auto up = pros::E_CONTROLLER_DIGITAL_RIGHT
constexpr auto down = pros::E_CONTROLLER_DIGITAL_Y

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

pros::adi::DigitalOut clamp('A');  // Pneumatic clamp on ADI port A
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
    14.5,                          // Track width (inches)
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

    // Task to continuously print pose data to the brain screen
    lv_obj_t * img = lv_image_create(lv_screen_active());
    lv_image_set_src(img, &WIN_20250904_16_05_21_Pro);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
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
    Full
};

constexpr Auto AutoSelect = Auto::Left;

void AutoLeft()
{
    chassis.setPose();//set position
    //collect first 3 blocks
    chassis.turnToPoint();//turn to the point
    chassis.moveToPoint();//move to point
    chassis.turnToPoint();//face the balls
    intake.move(MAX_INPUT);//Intake on
    chassis.moveToPoint();
    intake.move(0);
    pros::delay(1000);

    //Score the first set of blocks
    chassis.turnToPoint();
    chassis.moveToPoint();
    chassis.turnToHeading(-45,750);
    chassis.moveToPoint();
    top.move(127) && intake.move(127);
    pros::delay(2000);
    top.move(0) && intake.move(0);
    chassis.turnToPoint();
    chassis.moveToPoint();
    pros::delay(1000);

    //collect from match loader and score
    clamp.set_value(false);
    chassis.turnToPoint();
    chassis.moveToPoint();
    intake.move(127);
    pros::delay(2000);
    intake.move(0);
    chassis.moveToPoint();
    clamp.set_value(true);
    top.move(127)&&intake.move(127);
    pros::delay(2000);
    top.move(0)&&intake.move(0);
    pros::delay(1000);
}

void AutoRight()
{
    chassis.setPose();
    //collect the three blocks
    chassis.turnToPoint();
    chassis.moveToPoint();
    chassis.turnToPoint();
    intake.move(MAX_INPUT);
    chassis.moveToPoint();//slowly
    intake.move(0);
    pros::delay(1000);

    //score in lower
    chassis.turnToPoint();
    chassis.moveToPoint();//move slow
    intake.move(-MAX_INPUT);
    pros::delay(1000);
    intake.move(0);
    chassis.moveToPoint();//backwards
    pros::delay(1000);

    //match load and score
    chassis.turnToPoint();
    chassis.moveToPoint();
    chassis.turnToPoint();
    clamp.set_value(false);
    chassis.moveToPoint();
    intake.move(127);
    pros.delay(1000);
    intake.move(0);
    chassis.moveToPoint();//go backwards
    clamp.set_value(true);
    top.move(127) && intake.move(127);
    pros::delay(1000);
    top.move(0) && intake.move(0);
    pros::delay(1000);

}

void AutoFull()
{
    

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
        case Auto::Full:
            AutoFull();
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
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            clampValue = !clampValue;
            clamp.set_value(clampValue);
        }

        pros::delay(20); // Delay to reduce CPU usage
    }
}