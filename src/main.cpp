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

Table of Contents:
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
*/

// Controller declaration for driver control
pros::Controller master(pros::E_CONTROLLER_MASTER);

constexpr auto in = pros::E_CONTROLLER_DIGITAL_L2; // Intake
constexpr auto outtake = pros::E_CONTROLLER_DIGITAL_R2; // Lower goal score/outtake. Reverses intake
constexpr auto score = pros::E_CONTROLLER_DIGITAL_L1; // Raise lever to score
constexpr auto middle = pros::E_CONTROLLER_DIGITAL_R1; // Lowers 4 arm to middle pos
constexpr auto tounge = pros::E_CONTROLLER_DIGITAL_Y; // Tongue piston control
constexpr auto wing = pros::E_CONTROLLER_DIGITAL_RIGHT; // Wing piston control


// Left motor group on ports 2, 6, 13 (All reversed)
pros::MotorGroup left_motors({-2,-6,-13},pros::MotorGears::blue);

// Right motor group on ports 21, 8, 7 (None reversed)
pros::MotorGroup right_motors({21,8,19}, pros::MotorGears::blue);

// Lever and intake motors
pros::Motor intake(5, pros::MotorGears::blue); // Motor for intake
pros::Motor lever(-16, pros::MotorGears::red); // Motor for lever mech
int leverMax = -580;
bool leverLock = false;
int tolerance = 20;
int bottomPos = 0;


/*
-----------------------------------------------------------
2️⃣ SENSOR CONFIGURATION
-----------------------------------------------------------
*/

pros::Imu imu(20); // IMU on port 20

/*
-----------------------------------------------------------
3️⃣ PNEUMATICS CONFIGURATION
-----------------------------------------------------------
*/

// TONGUE PISTON
pros::adi::DigitalOut tongue_piston('G');  // Pneumatic clamp on ADI port A
bool tongueValue = false;           // Initial state of pneumatic clamp
bool lockT = false;

//  WING PISTON
pros::adi::DigitalOut wing_piston('H'); // Pneumatic clamp on ADI port B
bool wingValue = false;
bool lockW = false;

// MIDDLE PISTON
pros::adi::DigitalOut mid_piston('F'); // Pneumatic clamp on ADI port F
bool topScore = true;
bool lockM = false;

/*
-----------------------------------------------------------
4️⃣ LEMLIB DRIVETRAIN & CONTROLLERS
-----------------------------------------------------------
*/

lemlib::Drivetrain drivetrain(
    &left_motors,                // Left motor group
    &right_motors,               // Right motor group
    12,                          // Track width (inches)
    lemlib::Omniwheel::NEW_275,    // Wheel type (4" omni)
    450,                         // Max RPM
    4                           // Drift (measured experimentally)
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
    2.5, 0.7, 51, 4,  // kP, kI, kD, anti-windup
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
*/

// Helper function to calculate average of drive motor temperatures
float avg(std::vector<double> vars) {
    float sum = 0;
    for (size_t i = 0; i < vars.size(); i++)
        sum += vars[i];
    return sum / vars.size();
}

//MUTEX function for leverScore
enum class ScoreType{
    Top,
    Mid,
};

ScoreType leverType = ScoreType::Top;

void leverScoreLogic() {
    // Move to max position (score)
    //lever.move_absolute(leverMax, 100);  // Positive velocity to move toward negative position
    int loopCount = 0;
    int timeout = 1000;
    //double leverEff = 0;

    if(leverType == ScoreType::Top)
    {
        timeout = 1000;
        lever.move(MAX_INPUT);

        // Wait until reaching target
        while(fabs(lever.get_position() - leverMax) > tolerance) {
            pros::delay(20);
            //leverEff = lever.get_efficiency();
            loopCount++;
            if(loopCount * 20 > timeout) break;
        }
    }
    else if(leverType == ScoreType::Mid)
    {
        timeout = 2000;
        lever.move(127/2);

        // Wait until reaching target
        while(fabs(lever.get_position() - leverMax) > tolerance) {
            pros::delay(20);
            //leverEff = lever.get_efficiency();
            loopCount++;
            if(loopCount * 20 > timeout) break;
        }
    }
    loopCount = 0; //reset
    lever.brake(); // Stop at scoring position
    
    // Return to starting position
    lever.move_absolute(0, 100);
    
    // Wait until back at zero
    while(fabs(lever.get_position() - bottomPos) > bottomPos) {
        pros::delay(20);
        //leverEff = lever.get_efficiency();
        loopCount++;
        if(loopCount * 20 > timeout) break;
    }
    
    lever.brake();
    leverLock = false; //allow to score again
}

void leverScore(ScoreType leverScoreMid = ScoreType::Top){
    leverType = leverScoreMid;
    pros::Task leverTask(leverScoreLogic);
}

void initialize() {
    lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::lcd::initialize();
    pros::Task screenTask([&]() {
        // Variables for screen colors
        const uint32_t BLACK = 0x000000;
        const uint32_t WHITE = 0xFFFFFF;
        // Print info to brain screen
        while (true) {
            // Print robot location to the brain screen
            // Set background to black and clear screen
            pros::screen::set_eraser(BLACK);
            pros::screen::erase();
            // Set text color to white
            pros::screen::set_pen(WHITE);
            pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Lever: %f", lever.get_position()); // Lever position
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "X: %f", chassis.getPose().x);
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Y: %f", chassis.getPose().y);
            pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Theta: %f", chassis.getPose().theta);
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "IN Temp: %.0f", intake.get_temperature()); // Intake motor temp
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "LVR Temp: %.0f", lever.get_temperature()); // Lever motor temp
            pros::screen::print(pros::E_TEXT_MEDIUM, 6, "L DT Temp: %.0f", avg(left_motors.get_temperature_all())); // Average temp of left motor drive temps
            pros::screen::print(pros::E_TEXT_MEDIUM, 7, "R DT Temp: %.0f", avg(right_motors.get_temperature_all())); // Average temp of right motor drive temps
            // Delay
            pros::delay(50);
        }
    });
    // Calibration of sensors and initial states of motors/pneumatics
    chassis.calibrate();  // Calibrate IMU & encoders
    tongue_piston.set_value(false); // Ensure clamp is in initial state
    wing_piston.set_value(false); // Ensure clamp is in initial state
    mid_piston.set_value(false); // Ensure clamp is in initial state
    lever.tare_position(); 
    lever.set_zero_position(0);
}

/*
-----------------------------------------------------------
6️⃣ COMPETITION TEMPLATE FUNCTIONS
-----------------------------------------------------------
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
    chassis.setPose(-47,12,90);

//intake first three blocks
    chassis.turnToPoint(-18,25,750,{},false);
    intake.move(127);
    chassis.moveToPoint(-18,25,1750,{.maxSpeed = 70},false);
    chassis.moveToPoint(-25,20,750,{.forwards = false,.maxSpeed = 70},false);
    //chassis.moveToPoint(-25,25,1000,{.maxSpeed = 85});//change for accurrcy and consistanty of int

/*//Score in middle(4 balls)
    chassis.moveToPoint(-18,25,1750,{.maxSpeed = 70},false);//change for accurrcy and consistanty of intake
    chassis.moveToPoint(-25,20,750,{.forwards = false,.maxSpeed = 70},false);//decrease speed for accurecy, increase for arrive faster 
    mid_piston.set_value(false);
    chassis.turnToHeading(315,500,{},false);//change for consistancy
    chassis.moveToPoint(-10,10,800,{.forwards = false,.maxSpeed = 90},false);//decrease speed for accurecy, increase for arrive faster
    leverScore(leverType = ScoreType::Mid);
    pros::delay(500);
*/
   
//match load
    chassis.turnToPoint(-50,41,750,{},false);
    chassis.moveToPoint(-50,41,950,{.maxSpeed = 100},false);
    mid_piston.set_value(true);
    chassis.moveToPoint(-27,40.5,900,{.forwards = false,.maxSpeed = 90},false);
    intake.move(-127);
    leverScore(leverType = ScoreType::Top);
    pros::delay(1000);
    tongue_piston.set_value(true);
    chassis.turnToPoint(-50,41,750,{},false);
    chassis.moveToPoint(-50,41,950,{.maxSpeed = 100},false);
    chassis.turnToHeading(270,750,{},false);
    intake.move(127);
    chassis.moveToPoint(-59,40,900,{.maxSpeed = 75},false);
    chassis.moveToPoint(-59,40,450,{.maxSpeed = 85},false);
    pros::delay(400);

//Score in long(3 balls)
    mid_piston.set_value(true);
    chassis.moveToPoint(-27,40.5,900,{.forwards = false,.maxSpeed = 90},false);
    intake.move(-127);
    leverScore(leverType = ScoreType::Top);
    pros::delay(1000);
    tongue_piston.set_value(false);

/*//wing in the control(Do this if there is time)   
    chassis.moveToPoint(-45,39,750,{.maxSpeed = 90},false);
    chassis.turnToPoint(-45,50,750,{},false);
    chassis.moveToPoint(-45,50,750,{.maxSpeed = 90},false);
    chassis.turnToHeading(270,750,{},false);
    chassis.moveToPoint(-15,50,750,{.forwards=false,.maxSpeed = 90},false);*/
}
    
void AutoRight()
{
//start position
    chassis.setPose(-47,-12,180);

//intake first three blocks
   // chassis.turnToPoint(-15,-20,750,{},false);
    //intake.move(127);
    //chassis.moveToPoint(-25,25,1000,{.maxSpeed = 85});//change for accurrcy and consistanty of int

/*/Score in low(4 balls)
    chassis.moveToPoint(-15,-20,1750,{.maxSpeed = 70},false);//change for accurrcy and consistanty of intake
    chassis.moveToPoint(-25,-20,750,{.forwards = false,.maxSpeed = 70},false);//decrease speed for accurecy, increase for arrive faster 
    mid_piston.set_value(false);
    chassis.turnToHeading(45,500,{},false);//change for consistancy
    chassis.moveToPoint(-12,-12,800,{.maxSpeed = 85},false);//decrease speed for accurecy, increase for arrive faster
    pros::delay(500);
    intake.move(-127);
    pros::delay(500);*/

//match load
    chassis.moveToPoint(-50,-45,950,{.maxSpeed = 100},false);
    tongue_piston.set_value(true);
    chassis.turnToHeading(270,750,{},false);
    intake.move(127);
    chassis.moveToPoint(-60,-40,900,{.maxSpeed = 75},false);
    chassis.moveToPoint(-60,-40,450,{.maxSpeed = 80},false);
    pros::delay(400);

//Score in long(3 balls)
    mid_piston.set_value(true);
    chassis.moveToPoint(-27,-42,900,{.forwards = false,.maxSpeed = 75},false);
    intake.move(-127);
    leverScore(leverType = ScoreType::Top);
    pros::delay(1000);
    tongue_piston.set_value(false);
}
void AutoWinPoint()
{
    
}
void AutoSkills2()//For WORLDS
{
 //set Position in the middle of the parking zone
    chassis.setPose(-61,-23,90);

//Clear Park**NEEDS TESTING!!!
    /*intake.move(127);
    chassis.moveToPoint(-62,-5,500,{.maxSpeed = 85});//over the park
    chassis.moveToPoint(-62,-10,1000,{.maxSpeed = 110},false);//slowing start intaking*/

//get out of the park and set for the next step**PART OF TESTING
    //chassis.moveToPoint(-62,-45,4000,{.maxSpeed = 110},false);
   // intake.move(0);

//intake one single blue ball** also needs testing
    chassis.turnToPoint(-20,-23,600,{},false);//turn to ball
    intake.move(127);
    chassis.moveToPoint(-20,-23,900,{.maxSpeed = 80},false);//TEst this
    tongue_piston.set_value(true);
    chassis.moveToPoint(-18,23,500,{.maxSpeed = 80});//test this
    tongue_piston.set_value(false);
    intake.move(0);
   
//score 7 balls in the low goal
    chassis.moveToPoint(-22,-24,750,{.forwards = false,.maxSpeed = 80},false);//tune this to make it smooth and save time
    chassis.turnToPoint(-5,-5,600,{},false);//turn to score
    chassis.moveToPoint(-5,-5,800,{.maxSpeed = 75},false);//tune speed and timing
    intake.move(-127);//test this
    pros::delay(900);//test this delay to make sure it scores all the balls and doesn't leave any in the bot
    intake.move(0);

//match load
    chassis.moveToPoint(-50,-47,1750,{.forwards = false,.maxSpeed = 100},false);//tune speed to save time and timeout
    tongue_piston.set_value(true);
    chassis.turnToPoint(-57,-55,500,{},false);
    intake.move(127);
    chassis.moveToPoint(-57,-55,1000,{.maxSpeed = 90},false);//tune pose so the balls fall freely and the bot doesn't get cooked
    //add delay if needed try no to 
    intake.move(0);

//get to the other side
    chassis.moveToPoint(-25,-62,750,{.forwards =false,.maxSpeed = 90},false);//tune this too make it smooth
    chassis.turnToHeading(90,500,{},false);
    chassis.moveToPoint(25,-62,750,{.forwards =false,.maxSpeed = 90},false);//tune this too make it smooth

//set up for scoring
    chassis.turnToHeading(225,500,{},false);//tune the turn
    chassis.moveToPoint(38,-47,500,{.forwards = false,.maxSpeed = 80});//tune speed
    chassis.turnToHeading(90,500,{},false);
    chassis.moveToPoint(32,-47,500,{.forwards = false,.maxSpeed = 75},false);

//score
    leverScore(leverType = ScoreType::Top);

//Match load
    intake.move(127);
    chassis.turnToPoint(59,-41,900);
    tongue_piston.set_value(true);
    chassis.moveToPoint(59,41,2000,{.maxSpeed = 90},false);//tune timeout and distance
    //add delay if needed
    intake.move(0);

//score
    chassis.moveToPoint(32,-47,700,{.forwards = false,.maxSpeed = 75},false);
    leverScore(leverType = ScoreType::Top);

//clear blue park**NeEDS TESTING
    chassis.turnToPoint(62,-25,500);
    chassis.moveToPoint(62,-25,750,{.maxSpeed = 100},false);
    chassis.turnToPoint(62,-3,500);
    intake.move(127);
    chassis.moveToPoint(62,-3,900,{.minSpeed = 100},false);//agjust timeout
    chassis.moveToPoint(62,3,2000,{.maxSpeed = 50},false);
    chassis.moveToPoint(62,20,800,{.minSpeed = 100},false);//test timeout
    intake.move(0);

//get the single red ball
    chassis.turnToPoint(48,20,500);
    chassis.moveToPoint(48,20,750,{.maxSpeed = 80});
    chassis.turnToPoint(30,-14,500);
    intake.move(127);
    chassis.moveToPoint(30,-14,750,{.maxSpeed = 80});
    intake.brake();//test this

//score in the middle** TEST and TUNE for smoothness and timing
    chassis.moveToPoint(24,23,500,{.maxSpeed = 70},false);
    chassis.turnToHeading(135,500,{},false);
    chassis.moveToPoint(14,-13,800,{.maxSpeed = 75},false);
    leverScore(leverType = ScoreType::Mid);
    intake.move(127);
    leverScore(leverType = ScoreType::Mid);
    intake.move(0);

//intake four balls**Test this
    chassis.moveToPoint(18,-17,500,{.maxSpeed = 100});
    chassis.turnToPoint(23,22.5,500,{},false);
    intake.move(127);
    chassis.moveToPoint(23,22.5,1000,{.maxSpeed = 90},false);

//score in the long 
    chassis.turnToPoint(38,47,500);
    chassis.moveToPoint(38,47,750,{.maxSpeed = 90},false);
    chassis.turnToHeading(90,500,{},false);
    chassis.moveToPoint(21,47.5,750,{.forwards = false,.maxSpeed = 70},false);
    leverScore(leverType = ScoreType::Top);


//match load
    chassis.moveToPoint(45,41,1000,{.maxSpeed = 100},false);//ajust timeout
    intake.move(127);
    chassis.turnToPoint(59,41,500,{},false);
    tongue_piston.set_value(true);
    chassis.moveToPoint(59,41,1000,{.maxSpeed = 90},false);//tune timeout and distance
    intake.move(0);

//get to the other side
    //chassis.moveToPose(25,60,90,750,{.maxSpeed = 100,.forwards = false},false);//tune this
    chassis.moveToPoint(-25,60,750,{.forwards =false,.maxSpeed = 100},false);//tune this too make it smooth

//set up for scoring
    chassis.turnToHeading(45,500,{},false);//tune the turn
    chassis.moveToPoint(-38,47,500,{.forwards = false,.maxSpeed = 80});//tune speed
    chassis.turnToHeading(90,500,{},false);
    chassis.moveToPoint(-32,47,500,{.forwards = false,.maxSpeed = 75},false);

//score
    leverScore(leverType = ScoreType::Top);

//match load
    intake.move(127);
    tongue_piston.set_value(true);
    chassis.moveToPoint(-59,41,1750,{.maxSpeed = 90},false);//tune timeout and distance
    //add delay if needed
    intake.move(0);

//score
    chassis.moveToPoint(-32,47,500,{.forwards = false,.maxSpeed = 75},false);
    leverScore(leverType = ScoreType::Top);

//park**test this
    chassis.turnToPoint(-62,25,500);
    chassis.moveToPoint(-62,25,750,{.maxSpeed = 100},false);
    chassis.turnToPoint(-62,0,500,{},false);
    chassis.moveToPoint(-62,0,900,{.minSpeed = 100},false);



}
void AutoSkills1()//3 LOADERS AND LONG GOALS
{
    //Sets position on the edge of the parking zone **Learn how the Theta works**(for Shriyans);
    chassis.setPose(-45,10,360); 

    //intake
    mid_piston.set_value(true);
    chassis.turnToPoint(-45,48,750,{},false);
    chassis.moveToPoint(-45,48,1750,{.maxSpeed = 100},false);
    tongue_piston.set_value(true);
    chassis.turnToPoint(-60,48,750,{},false);
    intake.move(127);
    chassis.moveToPoint(-60,48,1000,{.maxSpeed = 90},false);
    pros::delay(1750);

    //Move to score
    chassis.moveToPoint(-45,45,1000,{.forwards = false,.maxSpeed = 70});
    chassis.turnToPoint(-20,25,750,{},false);
    chassis.moveToPoint(-20,25,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(45,25,750,{},false);
    chassis.moveToPoint(45,25,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(45,47,750);
    chassis.moveToPoint(45,47,750,{.maxSpeed = 140},false);
    chassis.turnToHeading(90,750,{},false);
    chassis.moveToPoint(23,49,750,{.forwards = false,.maxSpeed = 70},false);
    pros::delay(600);
    leverScore(leverType = ScoreType::Top);

    //intake 
    intake.move(127);
    chassis.turnToPoint(45,46,1000);
    chassis.moveToPoint(45,46,1000,{.maxSpeed = 70});
    chassis.turnToPoint(62,46,900);
    tongue_piston.set_value(true);
    intake.move(127);
    chassis.moveToPoint(62,46,1000,{.maxSpeed = 90},false);
    intake.move(127);
    pros::delay(2000);

    //score
    chassis.moveToPoint(20.5,50,900,{.forwards = false,.maxSpeed = 70},false);
    pros::delay(650);
    leverScore(leverType = ScoreType::Top);
    
    


    //moving
   chassis.moveToPoint(45,-45,1000,{.maxSpeed = 70});
   chassis.turnToPoint(25,25,750,{},false);
    chassis.moveToPoint(25,25,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(25,-25,750,{},false);
    chassis.moveToPoint(25,-25,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(45,-47,750,{},false);
    chassis.moveToPoint(45,-47,2750,{.maxSpeed = 140},false);
    tongue_piston.set_value(true);
    chassis.turnToPoint(57,-47,750,{},false);

    //intake
    intake.move(127);
    chassis.moveToPoint(59.5,-47,1000,{.maxSpeed = 90});
    pros::delay(2000);


    //Mov
    chassis.moveToPoint(45,-53,750,{.forwards = false,.maxSpeed = 70});
    chassis.turnToPoint(25,-25,750);
    chassis.moveToPoint(25,-25,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(-45,-25,750,{},false);
    chassis.moveToPoint(-45,-25,2000,{.maxSpeed = 140},false);
    chassis.turnToPoint(-45,-47,750,{},false);
    chassis.moveToPoint(-45,-47,2000,{.maxSpeed = 70});

    //score
    chassis.turnToHeading(270,750,{},false);
    chassis.moveToPoint(-25,-47,900,{.forwards = false,.maxSpeed = 70},false);
    pros::delay(750);
    leverScore(leverType = ScoreType::Top);

    //intake
    intake.move(127);
    chassis.moveToPoint(-57,-47,1750,{.maxSpeed = 90},false);
    chassis.moveToPoint(-59,-47,1000,{.maxSpeed = 95});
    intake.move(127);
    pros::delay(1000);
    intake.move(0);

    //score
    chassis.moveToPoint(-20.5,-47,900,{.forwards = false,.maxSpeed = 70},false);
    pros::delay(750);
    leverScore(leverType = ScoreType::Top);

    //park
    tongue_piston.set_value(false);
    chassis.moveToPoint(-35,-47,750,{.maxSpeed = 70});
    chassis.turnToPoint(-35,-10,900);
    chassis.moveToPoint(-35,-10,900,{.maxSpeed = 100},false);
    chassis.turnToPoint(-70,-10,750);
    intake.move(127);
    chassis.moveToPoint(-70,-10,4000,{.minSpeed = 140});


}

void AllianceWinPoint()
{
    /*chassis.setPose(-45,10,360);
    chassis.moveToPoint(-45,25,1000, {.maxSpeed = 40},false);
    chassis.turnToHeading(90, 1000, {.maxSpeed = 40},false);
    chassis.moveToPoint(-55,25,1000, {.forwards = false,.maxSpeed = 40});*/
}


void TestAuto(){
    chassis.setPose(0, 0, 0);

    chassis.moveToPoint(0, 24, 3000, {.maxSpeed = 40});// comment 1
    chassis.turnToHeading(90, 1000, {.maxSpeed = 40});// comment 2
    chassis.moveToPoint(12, 24, 3000, {.maxSpeed = 40});// comment 3
   // chassis.moveToPoint(24,24,3000, {.maxSpeed = 40});// comment 4
    chassis.turnToHeading(0, 1000, {.maxSpeed = 40});// comment 5
    chassis.moveToPoint(12, 36, 3000, {.maxSpeed = 40});// comment 6
    //chassis.moveToPoint(24,48,3000, {.maxSpeed = 40});// comment 7
    

   /* //set Position in the middle of the parking zone
    chassis.setPose(-49,0,270);

//Clear Park**NEEDS TESTING!!!
    intake.move(127);
    chassis.moveToPoint(-73,0,1000,{.maxSpeed = 60},false);//over the park
    chassis.moveToPoint(-80,0,4000,{.maxSpeed = 20});//slowing start intaking
    //tongue_piston.set_value(true);

//get out of the park and set for the next step**PART OF TESTING
    //chassis.moveToPoint(-47,0,800,{.forwards = false,.minSpeed = 100});
    //intake.move(0);*/
    
    //set Position in the middle of the parking zone
   // chassis.setPose(-62,13,180);

//Clear Park**NEEDS TESTING!!!
   // intake.move(127);
    //chassis.moveToPoint(-62,-5,500,{.minSpeed = 110});//over the park
    //tongue_piston.set_value(true);
    //chassis.moveToPoint(-62,-47,800,{.maxSpeed = 85});
    //chassis.moveToPoint(-62,-10,1000,{.maxSpeed = 85},false);//slowing start intaking
    //tongue_piston.set_value(false);

//get out of the park and set for the next step**PART OF TESTING
   // chassis.moveToPoint(-62,-55,4000,{.minSpeed = 110},false);
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
*/

void opcontrol() {
    
pros::Task* leverTask = nullptr;
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
        else if (master.get_digital(score) && !leverLock && topScore) {
            leverLock = true;
            leverScore(ScoreType::Top);    
        }
        else if (master.get_digital(score) && !leverLock && !topScore) {
            leverLock = true;
            leverScore(ScoreType::Mid);    
        }
        else {
            intake.brake();    // Stop (optional — can replace with .move(0))
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
            topScore = !topScore;
            mid_piston.set_value(topScore);
            lockM = true;
        }
        else if(!(master.get_digital(middle))) {
            lockM = false;
        }
        pros::delay(20); // Delay to reduce CPU usage
    }
}





/*

if u want to score mid
leverScore(ScoreType::Mid);

if u want to score top
leverScore(ScoreType::Top);

if you do this
leverScore();
it will default to top score

*/