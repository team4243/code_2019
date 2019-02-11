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
#define CARGO_ROLLER_DEVICENUMBER_LEADER 24
#define CARGO_ROLLER_DEVICENUMBER_FOLLOWER 25

// Hatch Flower Servo PWM Channel
#define HATCH_FLOWER_PWM_CHANNEL 1

// Speed as Rotations Per Second (RPS)
#define CARGO_ROLLER_SPEED_RPS 1.0

// Converting to RPS for ToughBox output
#define CONVERT_TO_RPS 1024

/*************************************************************************************************/
/**** Declarations ****/

// Cargo Motor Drivers
WPI_TalonSRX Cargo_Roller_Leader{CARGO_ROLLER_DEVICENUMBER_LEADER};
VictorSPX Cargo_Roller_Follower{CARGO_ROLLER_DEVICENUMBER_FOLLOWER};

// End Effector -- Hatch Catch Servo
frc::PWM Hatch_Flower_Servo(HATCH_FLOWER_PWM_CHANNEL);

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

void Excelsior_End_Effector::Cargo_Roller_Action(bool dispense, double speed)
{
    if (dispense)
        Cargo_Roller_Leader.Set(ControlMode::Velocity, speed * CARGO_ROLLER_SPEED_RPS * CONVERT_TO_RPS);
    else
        Cargo_Roller_Leader.Set(ControlMode::Velocity, speed * -CARGO_ROLLER_SPEED_RPS * CONVERT_TO_RPS);
}

void Excelsior_End_Effector::Hatch_Flower_Action(bool extend)
{
    if (extend)
        Hatch_Flower_Servo.SetPosition(1);
    else
        Hatch_Flower_Servo.SetPosition(0);
}
