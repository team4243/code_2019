/*************************************************************************************************/
/**** Includes ****/

#include "Excelsior_Classes.h"

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include <frc/Talon.h>

#include <iostream>
#include <Math.h>


/*************************************************************************************************/
/*** Definitions ****/

// Omni Drive -- CAN device numbers
#define DEVICENUMBER_FRONTLEFT_LEADER 1
#define DEVICENUMBER_FRONTLEFT_FOLLOWER 2

#define DEVICENUMBER_REARLEFT_LEADER 3
#define DEVICENUMBER_REARLEFT_FOLLOWER 4

#define DEVICENUMBER_FRONTRIGHT_LEADER 5
#define DEVICENUMBER_FRONTRIGHT_FOLLOWER 6

#define DEVICENUMBER_REARRIGHT_LEADER 7
#define DEVICENUMBER_REARRIGHT_FOLLOWER 8

// Speed as Rotations Per Second (RPS)
#define OMNI_DRIVE_SPEED_RPS 2.0

// Deadband for rejecting small movements of Joystick
#define OMNI_DRIVE_DEADBAND_VALUE 0.12

// Converting to RPS for ToughBox output
#define CONVERT_TO_RPS 1024 


/*************************************************************************************************/
/*** Declarations ****/

// Omni Drive -- Motor Drivers
WPI_TalonSRX FrontLeft_Leader { DEVICENUMBER_FRONTLEFT_LEADER }; 
VictorSPX FrontLeft_Follower { DEVICENUMBER_FRONTLEFT_FOLLOWER }; 

WPI_TalonSRX RearLeft_Leader { DEVICENUMBER_REARLEFT_LEADER };  
VictorSPX RearLeft_Follower { DEVICENUMBER_REARLEFT_FOLLOWER };  

WPI_TalonSRX FrontRight_Leader { DEVICENUMBER_FRONTRIGHT_LEADER }; 
VictorSPX FrontRight_Follower { DEVICENUMBER_FRONTRIGHT_FOLLOWER }; 

WPI_TalonSRX RearRight_Leader { DEVICENUMBER_REARRIGHT_LEADER }; 
VictorSPX RearRight_Follower { DEVICENUMBER_REARRIGHT_FOLLOWER }; 


/*************************************************************************************************/
/*** Configuration ****/

void Excelsior_Omni_Drive::Configure_Omni_Drive()
{
    // Set rotation directions (clockwise == false) and set follower to leaders
    FrontLeft_Leader.SetInverted(true);
    FrontLeft_Follower.SetInverted(true);
    FrontLeft_Follower.Follow(FrontLeft_Leader);

    RearLeft_Leader.SetInverted(true);
    RearLeft_Follower.SetInverted(true);
    RearLeft_Follower.Follow(RearLeft_Leader);

    FrontRight_Leader.SetInverted(true);
    FrontRight_Follower.SetInverted(true);
    FrontRight_Follower.Follow(FrontRight_Leader);

    RearRight_Leader.SetInverted(true);
    RearRight_Follower.SetInverted(true);
    RearRight_Follower.Follow(RearRight_Leader);
}


/*************************************************************************************************/
/*** Actions ****/

void Excelsior_Omni_Drive::Omni_Drive_Action(double x_value, double y_value, double rotation)
{
    // Deadband for joystick
    if(x_value < OMNI_DRIVE_DEADBAND_VALUE && x_value > -OMNI_DRIVE_DEADBAND_VALUE) x_value = 0;
    if(y_value < OMNI_DRIVE_DEADBAND_VALUE && y_value > -OMNI_DRIVE_DEADBAND_VALUE) y_value = 0;

    // Scale and vector direction
    double scalar = hypot(x_value, y_value);
    double direction = atan2(y_value, x_value) - (M_PI / 4);
    
    // Set calculated speed -1 to 1
    double speed_frontLeft = scalar * cos(direction) + rotation;
    double speed_frontRight = scalar * sin(direction) - rotation;
    double speed_rearLeft = scalar * sin(direction) + rotation;
    double speed_rearRight = scalar * cos(direction) - rotation;

    // Drive motors at actual speed (calculated speed scaled by MAX rotations per second)
    FrontLeft_Leader.Set(ControlMode::Velocity, speed_frontLeft * OMNI_DRIVE_SPEED_RPS * CONVERT_TO_RPS);
    FrontRight_Leader.Set(ControlMode::Velocity, -speed_frontRight * OMNI_DRIVE_SPEED_RPS * CONVERT_TO_RPS);
    RearLeft_Leader.Set(ControlMode::Velocity, speed_rearLeft * OMNI_DRIVE_SPEED_RPS * CONVERT_TO_RPS);
    RearRight_Leader.Set(ControlMode::Velocity, -speed_rearRight * OMNI_DRIVE_SPEED_RPS * CONVERT_TO_RPS);
}