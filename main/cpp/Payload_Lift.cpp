/*************************************************************************************************/
/**** Includes ****/

#include "Excelsior_Classes.h"

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include <frc/Talon.h>

#include <iostream>
#include <map>

/**** !!!!!!! TUNING VARIABLES !!!!!!!  ****/
/*************************************************************************************************/
/**** !!!!!!! TUNING VARIABLES !!!!!!!  ****/

// TalonSRX Configuration -- ENABLE
#define PAYLOAD_LIFT_WRITE_CONFIGURATION (false) // enabling bit to set configuration so we don't do it every time

// TalonSRX Configuration -- SET Values
#define PAYLOAD_LIFT_PEAK_OUTPUT_FWD (0.15)  // DOWN
#define PAYLOAD_LIFT_PEAK_OUTPUT_REV (-0.35) // UP
#define PAYLOAD_LIFT_PROPORTIONAL_CTRL (0.35)
#define PAYLOAD_LIFT_DERIVATIVE_CTRL (0.035)
#define PAYLOAD_LIFT_FEED_FWD_CTRL (0)
#define PAYLOAD_LIFT_RAMP_TIME (2) // Seconds to get from neutral (0) and full speed (peak output)
#define PAYLOAD_LIFT_SLOT_IDX (0)  // Which motor control profile to save the configuration to, 0 and 1 available

/*************************************************************************************************/
/**** Definitions ****/

// CAN device numbers
#define PAYLOAD_LIFT_DEVICENUMBER_LEADER (34)
#define PAYLOAD_LIFT_DEVICENUMBER_FOLLOWER (35)

// Scaling the Lift Position for AUTO driving
#define PAYLOAD_LIFT_POSITION_SCALAR (512)

/*************************************************************************************************/
/**** Object Declarations and Global Variables ****/

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

// Keeping track of the current Lift position
int targetPayloadHeight = -1;

/*************************************************************************************************/
/**** Configuration ****/

void Excelsior_Payload_Lift::Configure_Payload_Lift()
{
    // Set up encoder
    Payload_Lift_Leader.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
    Payload_Lift_Leader.SetSensorPhase(true);

    // Set rotation direction, clockwise == false
    Payload_Lift_Leader.SetInverted(false);
    if (PAYLOAD_LIFT_WRITE_CONFIGURATION)
    {
        Payload_Lift_Leader.ConfigPeakOutputForward(PAYLOAD_LIFT_PEAK_OUTPUT_FWD);
        Payload_Lift_Leader.ConfigPeakOutputReverse(PAYLOAD_LIFT_PEAK_OUTPUT_REV);
        Payload_Lift_Leader.ConfigClosedloopRamp(PAYLOAD_LIFT_RAMP_TIME);
        Payload_Lift_Leader.Config_kP(PAYLOAD_LIFT_SLOT_IDX, PAYLOAD_LIFT_PROPORTIONAL_CTRL);
        Payload_Lift_Leader.Config_kD(PAYLOAD_LIFT_SLOT_IDX, PAYLOAD_LIFT_DERIVATIVE_CTRL);
        Payload_Lift_Leader.Config_kF(PAYLOAD_LIFT_SLOT_IDX, PAYLOAD_LIFT_FEED_FWD_CTRL);
    }

    Payload_Lift_Follower.SetInverted(false);
    Payload_Lift_Follower.Follow(Payload_Lift_Leader);
}

/*************************************************************************************************/
/**** Actions ****/

void Excelsior_Payload_Lift::Payload_Lift_Action(Payload_Lift_Position position)
{
    // Tell motor drive to use encoder-feedback for a lift position
    Payload_Lift_Leader.Set(ControlMode::Position, -Lift_Position[position] * PAYLOAD_LIFT_POSITION_SCALAR);

    targetPayloadHeight = (int)position;
}

void Excelsior_Payload_Lift::Payload_Lift_Manual(double speed)
{
    // Manual lifting
    Payload_Lift_Leader.Set(ControlMode::PercentOutput, speed);
}

void Excelsior_Payload_Lift::Payload_Lift_Step(bool stepUp)
{
    // Make sure we don't exceed maximum height
    if (stepUp && (targetPayloadHeight < Maximum_Height_Position))
        targetPayloadHeight++;

    // Make sure we don't exceed minimum height
    else if (!stepUp && (targetPayloadHeight > Ground_Position))
        targetPayloadHeight--;

    Payload_Lift_Action((Payload_Lift_Position)targetPayloadHeight);
}

void Excelsior_Payload_Lift::Zero_Encoder_Position()
{
    Payload_Lift_Leader.SetSelectedSensorPosition(0, 0, 10);
}

/*************************************************************************************************/
/**** Helper Functions ****/

void Excelsior_Payload_Lift::Print_Lift_Encoder()
{
    std::cout << "Lift Encoder: " << Payload_Lift_Leader.GetSensorCollection().GetQuadraturePosition()
              << ", Target: " << -Lift_Position[(Payload_Lift_Position)targetPayloadHeight] * PAYLOAD_LIFT_POSITION_SCALAR
              << ", Pos: " << (Payload_Lift_Position)targetPayloadHeight << std::endl;
}