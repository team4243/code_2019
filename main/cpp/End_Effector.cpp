/*************************************************************************************************/
/**** Includes ****/

#include "Excelsior_Classes.h"

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include <frc/Talon.h>
#include "frc/PWM.h"

#include <iostream>

/**** !!!!!!! TUNING VARIABLES !!!!!!!  ****/
/*************************************************************************************************/
/**** !!!!!!! TUNING VARIABLES !!!!!!!  ****/

// Enable Cargo Limit Switch
#define ENABLE_LIMIT_SWITCH_CARGO (false)

// Cargo Roller speed as Rotations Per Second (RPS) of output shaft, the setpoint of the PID controller
#define CARGO_ROLLER_SPEED_RPS (1.0)

// TalonSRX Configuration -- ENABLE
#define ROLLERS_WRITE_CONFIGURATION (false) // enabling bit to set configuration so we don't do it every time

// TalonSRX Configuration -- SET Values
#define ROLLERS_PEAK_OUTPUT_FWD (0.75) // In or out?
#define ROLLERS_PEAK_OUTPUT_REV (-0.35)
#define ROLLERS_PROPORTIONAL_CTRL (0.35)
#define ROLLERS_DERIVATIVE_CTRL (0.035)
#define ROLLERS_FEED_FWD_CTRL (0)
#define ROLLERS_RAMP_TIME (2) // Seconds to get from neutral (0) and full speed (peak output)
#define ROLLERS_SLOT_IDX (0)  // Which motor control profile to save the configuration to, 0 and 1 available

// Hatch Flower MAX/MIN values, 0->1
#define HATCH_FLOWER_MAX (0.8)
#define HATCH_FLOWER_MIN (0.25)

// Camera Tilt MAX/MIN values, 0->1
#define CAMERA_TILT_MAX (0.7)
#define CAMERA_TILT_MIN (0.2)

/*************************************************************************************************/
/**** Definitions ****/

// Rollers CAN device numbers
#define CARGO_ROLLER_DEVICENUMBER_LEADER (24)
#define CARGO_ROLLER_DEVICENUMBER_FOLLOWER (25)

// Hatch Flower Servo -- PWM Channel
#define HATCH_FLOWER_PWM_CHANNEL (1)

// Camera Tilt Servo -- PWM Channel
#define CAMERA_TILT_PWM_CHANNEL (2)

// End Effector Limit Switch -- DigitalIO Channel
#define LIMIT_SWITCH_CHANNEL (2)

// Converting to RPS for ToughBox output..
// .. and the bag motors will be different, but we can just leave this alone and tune the SPEED_RPS variable
#define CONVERT_TO_RPS_EE (1024)

/*************************************************************************************************/
/**** Object Declarations and Global Variables ****/

// Cargo Motor Drivers
WPI_TalonSRX Cargo_Roller_Leader{CARGO_ROLLER_DEVICENUMBER_LEADER};
WPI_TalonSRX Cargo_Roller_Follower{CARGO_ROLLER_DEVICENUMBER_FOLLOWER};

// End Effector -- Hatch Catch Servo
frc::PWM Hatch_Flower_Servo(HATCH_FLOWER_PWM_CHANNEL);

// End Effector -- Camera Tilt Servo
frc::PWM Camera_Tilt_Servo(CAMERA_TILT_PWM_CHANNEL);

//We are adding a limit switch to check to see if the cargo is firmly secured within the end effector
frc::DigitalInput End_Effector_Limit_Switch(LIMIT_SWITCH_CHANNEL);

/*************************************************************************************************/
/**** Configuration ****/

void Excelsior_End_Effector::Configure_End_Effector()
{
    // Set rotation direction, clockwise == false
    Cargo_Roller_Leader.SetInverted(true);
    Cargo_Roller_Leader.ConfigPeakOutputForward(ROLLERS_PEAK_OUTPUT_FWD);
    Cargo_Roller_Leader.ConfigPeakOutputReverse(ROLLERS_PEAK_OUTPUT_REV);
    Cargo_Roller_Leader.ConfigClosedloopRamp(ROLLERS_RAMP_TIME);
    Cargo_Roller_Leader.Config_kP(ROLLERS_SLOT_IDX, ROLLERS_PROPORTIONAL_CTRL);
    Cargo_Roller_Leader.Config_kD(ROLLERS_SLOT_IDX, ROLLERS_DERIVATIVE_CTRL);
    Cargo_Roller_Leader.Config_kF(ROLLERS_SLOT_IDX, ROLLERS_FEED_FWD_CTRL);

    Cargo_Roller_Follower.SetInverted(false);
    Cargo_Roller_Follower.Follow(Cargo_Roller_Leader);

    Camera_Tilt_Servo.SetPeriodMultiplier(frc::PWM::kPeriodMultiplier_4X); // _4X == 50Hz, _2X == 100Hz, _1X == 200Hz
}

/*************************************************************************************************/
/**** Actions ****/

void Excelsior_End_Effector::Cargo_Roller_Action(bool dispense, double speed)
{
    if (dispense)
        Cargo_Roller_Leader.Set(ControlMode::Velocity, speed * CARGO_ROLLER_SPEED_RPS * CONVERT_TO_RPS_EE);

    else if (!ENABLE_LIMIT_SWITCH_CARGO || !End_Effector_Limit_Switch.Get())
        Cargo_Roller_Leader.Set(ControlMode::Velocity, -speed * CARGO_ROLLER_SPEED_RPS * CONVERT_TO_RPS_EE);
}

void Excelsior_End_Effector::Cargo_Roller_Manual(double speed)
{
    Cargo_Roller_Leader.Set(ControlMode::PercentOutput, speed);
}

void Excelsior_End_Effector::Hatch_Flower_Action(bool extend)
{
    if (extend)
        Hatch_Flower_Servo.SetPosition(HATCH_FLOWER_MAX);
    else
        Hatch_Flower_Servo.SetPosition(HATCH_FLOWER_MIN);
}

void Excelsior_End_Effector::Camera_Tilt_Action(bool tiltUp)
{
    if (tiltUp)
        Camera_Tilt_Servo.SetPosition(CAMERA_TILT_MAX);
    else
        Camera_Tilt_Servo.SetPosition(CAMERA_TILT_MIN);
}

/*************************************************************************************************/
/**** Helper Functions ****/

void Excelsior_End_Effector::Print_Roller_Encoders()
{
    std::cout << "Roller: " << Cargo_Roller_Leader.GetSensorCollection().GetQuadraturePosition()
              << std::endl;
}
