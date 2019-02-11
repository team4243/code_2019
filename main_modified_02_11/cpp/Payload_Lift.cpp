/*************************************************************************************************/
/**** Includes ****/

#include "Excelsior_Classes.h"

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include <frc/Talon.h>

#include <iostream>
#include <map>


/*************************************************************************************************/
/*** Definitions ****/

// CAN device numbers
#define PAYLOAD_LIFT_DEVICENUMBER_LEADER 22
#define PAYLOAD_LIFT_DEVICENUMBER_FOLLOWER 23

// Speed as Rotations Per Second (RPS)
#define PAYLOAD_LIFT_SPEED_RPS 1.0

// Lift Positions by Encoded Rotations
std::map <Payload_Lift_Position, int> Lift_Position
{
    {Ground_Position, 0},
    {Travel_Position, 5},
    {Lowest_Hatch_Position, 10},
    {Lowest_Cargo_Position, 15},
    {Middle_Hatch_Position, 20},
    {Middle_Cargo_Position, 25},
    {Highest_Hatch_Position, 30},   
    {Highest_Cargo_Position, 35},
    {Maximum_Height_Position, 40}
};

// Converting to RPS for ToughBox output
#define CONVERT_TO_RPS 1024 


/*************************************************************************************************/
/*** Declarations ****/
 
// Motor Drivers
WPI_TalonSRX Payload_Lift_Leader { PAYLOAD_LIFT_DEVICENUMBER_LEADER };
VictorSPX Payload_Lift_Follower { PAYLOAD_LIFT_DEVICENUMBER_FOLLOWER };
 

/*************************************************************************************************/
/*** Custom Functions ****/
 
void Excelsior_Payload_Lift::Configure_Payload_Lift() 
{
    // Set up encoder
    Payload_Lift_Leader.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
    Payload_Lift_Leader.SetSensorPhase(true); 
	Payload_Lift_Leader.SetSelectedSensorPosition(0, 0, 10);

    // Set rotation direction, clockwise == false
    Payload_Lift_Leader.SetInverted(false);
    Payload_Lift_Follower.SetInverted(false);
    Payload_Lift_Follower.Follow(Payload_Lift_Leader);
}

void Excelsior_Payload_Lift::Payload_Lift_Action(Payload_Lift_Position position) 
{ 
    // Tell motor drive to use encoder-feedback for a lift position
    Payload_Lift_Leader.Set(ControlMode::Position, position);

    // Velocity control example
    // Payload_Lift_Leader.Set(ControlMode::Velocity, CONVERT_TO_RPS * PAYLOAD_LIFT_SPEED_RPS);
}