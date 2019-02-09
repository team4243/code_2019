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

// Omni Drive -- CAN device numbers
#define OMNI_DEVICENUMBER_FRONTLEFT_LEADER 1
#define OMNI_DEVICENUMBER_FRONTLEFT_FOLLOWER 2
#define OMNI_DEVICENUMBER_REARLEFT_LEADER 3
#define OMNI_DEVICENUMBER_REARLEFT_FOLLOWER 4
#define OMNI_DEVICENUMBER_FRONTRIGHT_LEADER 5
#define OMNI_DEVICENUMBER_FRONTRIGHT_FOLLOWER 6
#define OMNI_DEVICENUMBER_REARRIGHT_LEADER 7
#define OMNI_DEVICENUMBER_REARRIGHT_FOLLOWER 8

// Omni Drive -- Speed Gain
#define OMNI_SPEED_GAIN 0.2
#define OMNI_DRIVE_MAX_RPS 2.0
#define OMNI_DRIVE_DEADBAND_VALUE 0.12

// Converting to rotations per second for ToughBox output
#define CONVERT_TO_RPS 1024


/*************************************************************************************************/
/*** Declarations ****/

// Omni Drive -- Motor Drivers
WPI_TalonSRX Omni_FrontLeft_Leader { OMNI_DEVICENUMBER_FRONTLEFT_LEADER }; 
VictorSPX Omni_FrontLeft_Follower { OMNI_DEVICENUMBER_FRONTLEFT_FOLLOWER }; 
WPI_TalonSRX Omni_RearLeft_Leader { OMNI_DEVICENUMBER_REARLEFT_LEADER };  
VictorSPX Omni_RearLeft_Follower { OMNI_DEVICENUMBER_REARLEFT_FOLLOWER };  
WPI_TalonSRX Omni_FrontRight_Leader { OMNI_DEVICENUMBER_FRONTRIGHT_LEADER }; 
VictorSPX Omni_FrontRight_Follower { OMNI_DEVICENUMBER_FRONTRIGHT_FOLLOWER }; 
WPI_TalonSRX Omni_RearRight_Leader { OMNI_DEVICENUMBER_REARRIGHT_LEADER }; 
VictorSPX Omni_RearRight_Follower { OMNI_DEVICENUMBER_REARRIGHT_FOLLOWER }; 


/*************************************************************************************************/
/*** Custom Functions ****/

void Excelsior_OmniDrive::Configure_Omni_Drive()
{
    // Set rotation directions (clockwise == false) and set follower to leaders
    Omni_FrontLeft_Leader.SetInverted(true);
    Omni_FrontLeft_Follower.SetInverted(true);
    Omni_FrontLeft_Follower.Follow(Omni_FrontLeft_Leader);

    Omni_RearLeft_Leader.SetInverted(true);
    Omni_RearLeft_Follower.SetInverted(true);
    Omni_RearLeft_Follower.Follow(Omni_RearLeft_Leader);

    Omni_FrontRight_Leader.SetInverted(true);
    Omni_FrontRight_Follower.SetInverted(true);
    Omni_FrontRight_Follower.Follow(Omni_FrontRight_Leader);

    Omni_RearRight_Leader.SetInverted(true);
    Omni_RearRight_Follower.SetInverted(true);
    Omni_RearRight_Follower.Follow(Omni_RearRight_Leader);
}

void Excelsior_OmniDrive::OmniDrive_SpeedControl(double x_value, double y_value, double rotation)
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
    Omni_FrontLeft_Leader.Set(ControlMode::Velocity, speed_frontLeft * OMNI_DRIVE_MAX_RPS * 1024);
    Omni_FrontRight_Leader.Set(ControlMode::Velocity, -speed_frontRight * OMNI_DRIVE_MAX_RPS * 1024);
    Omni_RearLeft_Leader.Set(ControlMode::Velocity, speed_rearLeft * OMNI_DRIVE_MAX_RPS * 1024);
    Omni_RearRight_Leader.Set(ControlMode::Velocity, -speed_rearRight * OMNI_DRIVE_MAX_RPS * 1024);
}