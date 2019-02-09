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

// Payload Lift -- CAN device numbers
#define PAYLOAD_LIFT_DEVICENUMBER_LEADER 22
#define PAYLOAD_LIFT_DEVICENUMBER_FOLLOWER 23

// Payload Lift -- Speed Gain
#define PAYLOAD_LIFT_SPEED_GAIN 0.2
#define ROTATIONS_PER_SECOND 1

// Payload Lift Positions by Encoder Values
#define PAYLOAD_LOWEST_HATCH_LEVEL_MOTOR_POSITION 0
#define PAYLOAD_MIDDLE_HATCH_LEVEL_MOTOR_POSITION 512
#define PAYLOAD_HIGHEST_HATCH_LEVEL_MOTOR_POSITION 1024

// Converting to rotations per second for ToughBox output
#define CONVERT_TO_RPS 1024 

/*************************************************************************************************/
/*** Declarations ****/
 
// Payload Lift -- Motor Drivers
WPI_TalonSRX Payload_Lift_Leader { PAYLOAD_LIFT_DEVICENUMBER_LEADER };
VictorSPX Payload_Lift_Follower { PAYLOAD_LIFT_DEVICENUMBER_FOLLOWER };
 
/*************************************************************************************************/
/*** Custom Functions ****/
 
void Excelsior_Payload::Configure_Payload_Lift() {
    Payload_Lift_Leader.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
    Payload_Lift_Leader.SetSensorPhase(true); 
	Payload_Lift_Leader.SetSelectedSensorPosition(0, 0, 10);

    // Set rotation direction, clockwise == false
    Payload_Lift_Leader.SetInverted(false);
    Payload_Lift_Follower.SetInverted(false);
    Payload_Lift_Follower.Follow(Payload_Lift_Leader);
}

void Excelsior_Payload::Payload_Action(PYActions action) {
    switch (action) {

        case Lowest_Hatch_Position: 
            Payload_Lift_Leader.Set(ControlMode::Position, PAYLOAD_LOWEST_HATCH_LEVEL_MOTOR_POSITION);
        break;

        case Middle_Hatch_Position: 
            Payload_Lift_Leader.Set(ControlMode::Position, PAYLOAD_MIDDLE_HATCH_LEVEL_MOTOR_POSITION);
        break;

        case Highest_Hatch_Position: 
            Payload_Lift_Leader.Set(ControlMode::Position, PAYLOAD_HIGHEST_HATCH_LEVEL_MOTOR_POSITION);
        break;

        case Lower_Hatch: 
            Payload_Lift_Leader.Set(ControlMode::Velocity, -CONVERT_TO_RPS * ROTATIONS_PER_SECOND); 
        break;

        case Raise_Hatch: 
            Payload_Lift_Leader.Set(ControlMode::Velocity, CONVERT_TO_RPS * ROTATIONS_PER_SECOND); 
        break;

        case Halt_Motor: 
            Payload_Lift_Leader.Set(ControlMode::PercentOutput, 0);  
        break;

        default:
            Payload_Lift_Leader.Set(ControlMode::PercentOutput, 0);  
        break;
    }


}