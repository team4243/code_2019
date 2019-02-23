/*************************************************************************************************/
/**** Includes ****/

#include "Excelsior_Classes.h"

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include <frc/Talon.h>
#include "frc/PWM.h"

#include <iostream>

/*************************************************************************************************/
/**** Definitions ****/

// Rollers CAN device numbers
#define CARGO_ROLLER_DEVICENUMBER_LEADER (24)
#define CARGO_ROLLER_DEVICENUMBER_FOLLOWER (25)

// Hatch Flower Servo PWM Channel
#define HATCH_FLOWER_PWM_CHANNEL (1)

// End Effector Limit Switch Channel
#define LIMIT_SWITCH_CHANNEL (2)

// Speed as Rotations Per Second (RPS)
#define CARGO_ROLLER_SPEED_RPS (1.0)

// Converting to RPS for ToughBox output
#define CONVERT_TO_RPS_EE (1024)

// Cap the maximum output
#define ROLLERS_PEAK_OUTPUT_FWD (0.35)
#define ROLLERS_PEAK_OUTPUT_REV (-0.35)

/*************************************************************************************************/
/**** Declarations ****/

// Cargo Motor Drivers
WPI_TalonSRX Cargo_Roller_Leader{CARGO_ROLLER_DEVICENUMBER_LEADER};
WPI_TalonSRX Cargo_Roller_Follower{CARGO_ROLLER_DEVICENUMBER_FOLLOWER};

// End Effector -- Hatch Catch Servo
frc::PWM Hatch_Flower_Servo(HATCH_FLOWER_PWM_CHANNEL);

//We are adding a limit switch to check to see if the cargo is firmly secured within the end effector.
//pressed limit switch = true
// DigitalInput End_Effector_Limit_Switch(LIMIT_SWITCH_CHANNEL);

/*************************************************************************************************/
/**** Configuration ****/

void Excelsior_End_Effector::Configure_End_Effector()
{
    // Set rotation direction, clockwise == false
    Cargo_Roller_Leader.SetInverted(true);
    Cargo_Roller_Follower.SetInverted(false);
    Cargo_Roller_Follower.Follow(Cargo_Roller_Leader);
}

/*************************************************************************************************/
/**** Actions ****/

void Excelsior_End_Effector::Cargo_Roller_Action(bool dispense, double speed, bool manual)
{
    // Auto
    if (!manual)
    {
        if (dispense)
        {
            Cargo_Roller_Leader.Set(ControlMode::Velocity, speed * CARGO_ROLLER_SPEED_RPS * CONVERT_TO_RPS_EE);
        }

        else
        {
            // if(!End_Effector_Limit_Switch.Get())
            Cargo_Roller_Leader.Set(ControlMode::Velocity, -speed * CARGO_ROLLER_SPEED_RPS * CONVERT_TO_RPS_EE);
        }
    }

    // Manual
    else
    {
        if (dispense)
        {
            Cargo_Roller_Leader.Set(ControlMode::PercentOutput, speed);
        }

        else
        {
            // if(!End_Effector_Limit_Switch.Get())
            Cargo_Roller_Leader.Set(ControlMode::PercentOutput, -speed);
        }
    }
}

void Excelsior_End_Effector::Hatch_Flower_Action(bool extend)
{
    if (extend)
        Hatch_Flower_Servo.SetPosition(1);
    else
        Hatch_Flower_Servo.SetPosition(0);
}

/*************************************************************************************************/
/**** Helper Functions ****/

void Excelsior_End_Effector::Print_Roller_Encoders()
{
    std::cout << "Roller: " << Cargo_Roller_Leader.GetSensorCollection().GetQuadraturePosition()
              << std::endl;
}
