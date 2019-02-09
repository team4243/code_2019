/*************************************************************************************************/
/**** Includes ****/

#include "Excelsior_Classes.h"
#include "frc/WPILib.h"
#include "frc/PWM.h"
#include "frc/drive/MecanumDrive.h"
#include "ctre/Phoenix.h"
#include <iostream>
#include <frc/Talon.h>
#include <Math.h>


/*************************************************************************************************/
/*** Definitions ****/

// End Effector Roller -- CAN device numbers
#define ROLLER_DEVICENUMBER_LEADER 24 
#define ROLLER_DEVICENUMBER_FOLLOWER 25 

// End Effector Hatch Catch Servo PWM Channel
#define HATCH_CATCH_PWM_CHANNEL 1

/*************************************************************************************************/
/*** Declarations ****/

// End Effector -- Cargo Motor Drivers
WPI_TalonSRX Cargo_Roller_Leader { ROLLER_DEVICENUMBER_LEADER };
VictorSPX Cargo_Roller_Follower { ROLLER_DEVICENUMBER_FOLLOWER };

// End Effector -- Hatch Catch Servo
frc::PWM Hatch_Catch_Servo ( HATCH_CATCH_PWM_CHANNEL );

/*************************************************************************************************/
/*** Custom Functions ****/
 
 void Hatch_Flower::Configure_End_Effector()
{
    // TODO: Add Configuration
}

