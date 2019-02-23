/*************************************************************************************************/
/**** Includes ****/

#include "Excelsior_Classes.h"

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include <frc/Talon.h>

#include <iostream>
#include <map>

/*************************************************************************************************/
/**** Definitions ****/

// CAN device numbers
#define PAYLOAD_LIFT_DEVICENUMBER_LEADER (34)
#define PAYLOAD_LIFT_DEVICENUMBER_FOLLOWER (35)

// Scaling the Lift Position for AUTO driving
#define PAYLOAD_LIFT_POSITION_SCALAR (512)

// Speed in RPS (rotations per second) for MANUAL driving
#define PAYLOAD_LIFT_SPEED (0.5) // 0 to 1

// Cap the maximum output
#define PAYLOAD_LIFT_PEAK_OUTPUT_FWD (0.15)  // DOWN
#define PAYLOAD_LIFT_PEAK_OUTPUT_REV (-0.35) // UP

// Converting to RPS for ToughBox output
#define CONVERT_TO_RPS_LIFT 1024

/*************************************************************************************************/
/**** Declarations ****/

// Lift Positions by Encoded Rotations
std::map<Payload_Lift_Position, int> Lift_Position{
    {Ground_Position, 0},
    {Travel_Position, 2},
    {Lowest_Hatch_Position, 5},
    {Lowest_Cargo_Position, 13},
    {Middle_Hatch_Position, 17},
    {Middle_Cargo_Position, 24},
    {Highest_Hatch_Position, 28},
    {Highest_Cargo_Position, 38},
    {Maximum_Height_Position, 40}};

// Motor Drivers
WPI_TalonSRX Payload_Lift_Leader{PAYLOAD_LIFT_DEVICENUMBER_LEADER};
WPI_TalonSRX Payload_Lift_Follower{PAYLOAD_LIFT_DEVICENUMBER_FOLLOWER};

/*************************************************************************************************/
/**** Configuration ****/

void Excelsior_Payload_Lift::Configure_Payload_Lift()
{
    // Set up encoder
    Payload_Lift_Leader.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
    Payload_Lift_Leader.SetSensorPhase(true);
    Payload_Lift_Leader.SetSelectedSensorPosition(0, 0, 10);

    // Set rotation direction, clockwise == false
    Payload_Lift_Leader.SetInverted(false);
    // Payload_Lift_Leader.ConfigPeakOutputForward(PAYLOAD_LIFT_PEAK_OUTPUT_FWD);
    // Payload_Lift_Leader.ConfigPeakOutputReverse(PAYLOAD_LIFT_PEAK_OUTPUT_REV);

    Payload_Lift_Follower.SetInverted(false);
    Payload_Lift_Follower.Follow(Payload_Lift_Leader);
    // Payload_Lift_Follower.ConfigPeakOutputForward(PAYLOAD_LIFT_PEAK_OUTPUT_FWD);
    // Payload_Lift_Follower.ConfigPeakOutputReverse(PAYLOAD_LIFT_PEAK_OUTPUT_REV);
}

/*************************************************************************************************/
/**** Actions ****/

void Excelsior_Payload_Lift::Payload_Lift_Action(Payload_Lift_Position position)
{
    // Tell motor drive to use encoder-feedback for a lift position
    Payload_Lift_Leader.Set(ControlMode::Position, -Lift_Position[position] * PAYLOAD_LIFT_POSITION_SCALAR);
}

void Excelsior_Payload_Lift::Payload_Lift_Manual(bool direction)
{
    if (direction)
        Payload_Lift_Leader.Set(ControlMode::PercentOutput, -PAYLOAD_LIFT_SPEED);
    else
        Payload_Lift_Leader.Set(ControlMode::PercentOutput, PAYLOAD_LIFT_SPEED);
}

void Excelsior_Payload_Lift::Stop()
{
    Payload_Lift_Leader.Set(ControlMode::Position, 0);
}

/*************************************************************************************************/
/**** Helper Functions ****/

void Excelsior_Payload_Lift::Print_Lift_Encoder(Payload_Lift_Position position)
{
    std::cout << "Lift Encoder: " << Payload_Lift_Leader.GetSensorCollection().GetQuadraturePosition()
              << ", Target: " << -Lift_Position[position] * PAYLOAD_LIFT_POSITION_SCALAR
              << ", Pos: " << position << std::endl;
}