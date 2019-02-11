/*************************************************************************************************/
/**** Includes ****/

#include "Excelsior_Classes.h"

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include <frc/Talon.h>
#include "frc/PWM.h"

#include <iostream>
#include <Math.h>


/*************************************************************************************************/
/*** Definitions ****/

// Rollers CAN device numbers
#define CARGO_ROLLER_DEVICENUMBER_LEADER 24 
#define CARGO_ROLLER_DEVICENUMBER_FOLLOWER 25 

// Hatch Flower Servo PWM Channel
#define HATCH_FLOWER_PWM_CHANNEL 1


/*************************************************************************************************/
/*** Declarations ****/

// Cargo Motor Drivers
WPI_TalonSRX Cargo_Roller_Leader { CARGO_ROLLER_DEVICENUMBER_LEADER };
VictorSPX Cargo_Roller_Follower { CARGO_ROLLER_DEVICENUMBER_FOLLOWER };

// End Effector -- Hatch Catch Servo
frc::PWM Hatch_Flower_Servo ( HATCH_FLOWER_PWM_CHANNEL );


/*************************************************************************************************/
/*** Custom Functions ****/
 
void End_Effector::Configure_End_Effector()
{
    // Set rotation direction, clockwise == false
    Cargo_Roller_Leader.SetInverted(true);
    Cargo_Roller_Follower.SetInverted(false);
    Cargo_Roller_Follower.Follow(Cargo_Roller_Leader);
}

void End_Effector::Cargo_Roller_Action(bool dispense)
{
    // TODO: Add Action
}

void End_Effector::Hatch_Flower_Action(bool extend)
{
    // TODO: Add Action
}
