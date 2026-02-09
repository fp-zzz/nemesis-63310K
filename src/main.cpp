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

constexpr auto in = pros::E_CONTROLLER_DIGITAL_L2; 
constexpr auto outtake = pros::E_CONTROLLER_DIGITAL_R2; 
constexpr auto score = pros::E_CONTROLLER_DIGITAL_L1; 
constexpr auto middle = pros::E_CONTROLLER_DIGITAL_R1; 
constexpr auto tounge = pros::E_CONTROLLER_DIGITAL_Y; 
constexpr auto wing = pros::E_CONTROLLER_DIGITAL_RIGHT; 


// Left motor group on ports 1, 2, 3 (1 & 3 reversed)
pros::MotorGroup left_motors({-2,-6,-13},pros::MotorGears::blue);

// Right motor group on ports 4, 5, 6 (5 reversed)
pros::MotorGroup right_motors({21,8,7}, pros::MotorGears::blue);

// Lever and intake motors
pros::Motor intake(5, pros::MotorGears::blue); // Motor for intake
pros::Motor lever(16, pros::MotorGears::red); // Motor for lever mech
int leverMax = -555;
bool leverLock = false;
int tolerance = 5;

/*
-----------------------------------------------------------
2️⃣ SENSOR CONFIGURATION
-----------------------------------------------------------
💡 Includes IMU, tracking wheels, encoders, etc.
- Tracking wheels measure distance and direction.
- IMU provides inertial heading tracking.
*/

pros::Imu imu(20); // IMU on port 20

/*
-----------------------------------------------------------
3️⃣ PNEUMATICS (ADI EXAMPLE)
-----------------------------------------------------------
💡 Pneumatics are controlled via ADI digital outputs.
- Setting HIGH opens the solenoid.
- Example button control included in opcontrol().
*/

// TONGUE PISTON
pros::adi::DigitalOut tongue_piston('A');  // Pneumatic clamp on ADI port A
bool tongueValue = false;           // Initial state of pneumatic clamp
bool lockT = false;

//  WING PISTON
pros::adi::DigitalOut wing_piston('B'); // Pneumatic clamp on ADI port B
bool wingValue = false;
bool lockW = false;

// MIDDLE PISTON
pros::adi::DigitalOut mid_piston('C');
bool midValue = false;
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

float avg(std::vector<double> vars) {
    float sum = 0;
    for (size_t i = 0; i < vars.size(); i++)
        sum += vars[i];
    return sum / vars.size();
}

void initialize() {
    pros::lcd::initialize();
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "Lever: %f", lever.get_position());
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            // lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::lcd::print(4, "C Temp: %.0f", intake.get_temperature());
            pros::lcd::print(5, "S Temp: %.0f", lever.get_temperature());
            pros::lcd::print(6, "L DT Temp: %.0f", avg(left_motors.get_temperature_all()));
            pros::lcd::print(7, "R DT Temp: %.0f", avg(right_motors.get_temperature_all()));
            // pros::lcd::print(6, "L: %f", controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
            // pros::lcd::print(7, "R: %f", controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
            pros::delay(50);
        }

    });

    chassis.calibrate();  // Calibrate IMU & encoders
    tongue_piston.set_value(false); // Ensure clamp is in initial state
    wing_piston.set_value(false); // Ensure clamp is in initial state
    mid_piston.set_value(true);
    lever.tare_position();
    lever.set_zero_position(0);
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
    Skills1,
    Test,
    Alliance,
    Skills2
};

constexpr Auto AutoSelect = Auto::Right;

void AutoLeft()
{
//start position
    chassis.setPose(-47,12,70);

//intake first three blocks
    chassis.turnToPoint(-22,22,500,{},false);
    //turn inake on 
    chassis.moveToPoint(-22,22,750,{.maxSpeed = 100},false);//change for accurrcy and consistanty of intake

//Score in middle(4 balls)
    //intake off
    chassis.turnToHeading(315,500,{},false);//change for consistancy
    chassis.moveToPoint(-13,14,750,{.forwards = false,.maxSpeed = 85},false);//decrease speed for accurecy, increase for arrive faster
    //score with lever SLOWL:Y

//match load
    //tounge down
    // dont knwow wtf happened here *****chassis.moveToPose(-57,47,270,900,{.maxSpeed = 100,.lead = 0.3,.horizontalDrift = 8},false);//check if right
    //adjust by adding points if needed
    //intake for 500ms

//Score in long(3 balls)
    chassis.moveToPoint(-30,47,750,{.forwards = false,.maxSpeed = 90},false);
    //score with lever

//wing in the control(Do this if there is time)   
}
    
void AutoRight()
{
//Set position
    chassis.setPose(-47,-12,110);

//collection 3 blocks
    chassis.turnToPoint(-22,-22,500,{},false);
    //intake on
    chassis.moveToPoint(-22,-22,750,{.maxSpeed = 100},false);//adjust speed

//score in middle(4 balls)
    //intake off
    
    
}
void AutoWinPoint()
{
    
}
void AutoSkills2()//MIDDLE AND LOW GOALS
{

}
void AutoSkills1()//3 LOADERS AND LONG GOALS
{
    //Sets position on the edge of the parking zone **Learn how the Theta works**(for Shriyans);
    chassis.setPose(-45,10,360); 

    //intake
    chassis.turnToPoint(-45,46,750);
    chassis.moveToPoint(-45,46,1750,{.maxSpeed = 70},false);
    tongue_piston.set_value(true);
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
    intake.move(127) && lever.move(127);
    pros::delay(450);
    intake.move(-127);
    pros::delay(650);
    lever.move(127) && intake.move(127);
    pros::delay(3500);

    //intake 
    lever.move(0)&&intake.move(127);
    chassis.turnToPoint(45,41,1000);
    chassis.moveToPoint(45,41,1000,{.maxSpeed = 70});
    chassis.turnToPoint(59,41,900);
    tongue_piston.set_value(true);
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
   intake.move(127)&& lever.move(127);
    pros::delay(500);
   intake.move(-127);
   pros::delay(650);
    lever.move(127) && intake.move(127);
    pros::delay(3500);
    
    


    //moving
   chassis.moveToPoint(45,-45,1000,{.maxSpeed = 70});
   chassis.turnToPoint(25,25,750,{},false);
    chassis.moveToPoint(25,25,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(25,-25,750,{},false);
    chassis.moveToPoint(25,-25,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(45,-53,750,{},false);
    chassis.moveToPoint(45,-53,2750,{.maxSpeed = 140},false);
    tongue_piston.set_value(true);
    chassis.turnToPoint(57,-53,750,{},false);

    //intake
    intake.move(127)&&lever.move(0);
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
    intake.move(127)&& lever.move(127);
    pros::delay(650);
    intake.move(-127);
    pros::delay(650);
    lever.move(127) && intake.move(127);
    pros::delay(2900);

    //park
    tongue_piston.set_value(false);
    chassis.moveToPoint(-35,-45,750,{.maxSpeed = 70});
    chassis.turnToPoint(-20,20,750,{},false);
    chassis.moveToPoint(-20,20,750,{.maxSpeed = 140});
    chassis.turnToPoint(-20,8,900);
    chassis.moveToPoint(-20,8,750,{.maxSpeed = 140},false);
    chassis.turnToPoint(-70,8,900);
    intake.move(127) && lever.move(127);
    chassis.moveToPoint(-70,8,4000,{.minSpeed = 140});


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
        case Auto::Skills1:
            AutoSkills1();
            break;
        case Auto::Test:
            TestAuto();
            break;
        case Auto::Alliance:
            AllianceWinPoint();
            break;
        case Auto::Skills2:
            AutoSkills2();
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
            intake.move(MAX_INPUT);  // Full forward
        } 
        else if (master.get_digital(outtake)) {
            intake.move(-MAX_INPUT); // Full reverse
        } 
        else if (master.get_digital(score) && !leverLock) {
            //0 is bottom, -555 is max
            pros::Task leverTask([&]() {
                while(!(lever.get_position() > leverMax - tolerance && lever.get_position() < leverMax + tolerance))
                {
                    lever.move_absolute(leverMax, -100);
                }
            });
            
            // lever.move_absolute(-240, 100);
            // pros::delay(500);
            // // lever.move(127);
            // // Move to 900 degrees (e.g., 90-degree lift turn) at 100 RPM
            // lever.move_absolute(248, 100);
            // while (!((lever.get_position() < (248 + 5)) && (lever.get_position() > (248 - 5)))) {
            //     pros::delay(2);
            // }
        }
        else if (!(master.get_digital(score)) && leverLock){
            lever.brake();
            pros::Task leverTask([&]() {
                while(!(lever.get_position() > 0 - tolerance && lever.get_position() < 0 + tolerance))
                {
                    lever.move_absolute(1, 100);
                }
            });
            leverLock = false;

        }
        else {
            intake.brake();    // Stop (optional — can replace with .move(0))
            lever.brake();
        }

        // --- Pneumatics Toggle ---
        //  TONGUE PISTON CONTROL
        if (master.get_digital(tounge) && !lockT) {
            tongueValue = !tongueValue;
            tongue_piston.set_value(tongueValue);
            lockT = true;
        }
        else if(!(master.get_digital(tounge)))
        {
            lockT = false;
        }

        //  WING PISTON CONTROL
        if (master.get_digital(wing) && !lockW) {
            wingValue = !wingValue;
            wing_piston.set_value(wingValue);
            lockW = true;
        }
        else if(!(master.get_digital(wing)))
        {
            lockW = false;
        }

        // MIDDLE PISTON CONTROL
        
        if (master.get_digital(middle) && !lockM) {
            midValue = !midValue;
            mid_piston.set_value(midValue);
            lockM = true;
        }
        else if(!(master.get_digital(middle))) {
            lockM = false;
        }
        pros::delay(20); // Delay to reduce CPU usage
    }
}