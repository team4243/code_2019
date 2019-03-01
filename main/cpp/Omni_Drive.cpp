/*************************************************************************************************/
/**** Includes ****/

#include "Excelsior_Classes.h"

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include <frc/Talon.h>

#include <iostream>
#include <Math.h>

/**** !!!!!!! TUNING VARIABLES !!!!!!!  ****/
/*************************************************************************************************/
/**** !!!!!!! TUNING VARIABLES !!!!!!!  ****/

#define OMNI_DRIVE_SPEED_ALIGNING (0.3)
#define OMNI_DRIVE_SPEED_DRIVING (0.7)

// TalonSRX Configuration -- ENABLE
#define OMNI_DRIVE_WRITE_CONFIGURATION (false) // enabling bit to set configuration so we don't do it every time

// TalonSRX Configuration -- SET Values
#define OMNI_DRIVE_PEAK_OUTPUT_FWD (0.35) // Maximum output speed 0->1
#define OMNI_DRIVE_PEAK_OUTPUT_REV (-0.35)
#define OMNI_DRIVE_PROPORTIONAL_CTRL (0.01)
#define OMNI_DRIVE_DERIVATIVE_CTRL (0.001)
#define OMNI_DRIVE_FEED_FWD_CTRL (0)
#define OMNI_DRIVE_RAMP_TIME (0) // Seconds to get from neutral to full speed (peak output)
#define OMNI_DRIVE_SLOT_IDX (0)  // Which motor control profile to save the configuration to, 0 and 1 available

// Deadband value for rejecting small movements of Joystick accidently by operator
#define OMNI_DRIVE_DEADBAND_VALUE (0.12)

/*************************************************************************************************/
/**** Definitions ****/

// Omni Drive -- CAN device numbers
#define DEVICENUMBER_FRONTLEFT_LEADER (11) // yes, 11 not 1 since PDP only likes channel 1
#define DEVICENUMBER_FRONTLEFT_FOLLOWER (2)

#define DEVICENUMBER_REARLEFT_LEADER (3)
#define DEVICENUMBER_REARLEFT_FOLLOWER (4)

#define DEVICENUMBER_FRONTRIGHT_LEADER (5)
#define DEVICENUMBER_FRONTRIGHT_FOLLOWER (6)

#define DEVICENUMBER_REARRIGHT_LEADER (7)
#define DEVICENUMBER_REARRIGHT_FOLLOWER (8)

// Converting to RPS for ToughBox output..
// .. and the omni boxes will be different, but we can just leave this alone and tune the SPEED_RPS variable
#define CONVERT_TO_RPS_DRIVE (1024)

/*************************************************************************************************/
/**** Object Declarations and Global Variables ****/

// Omni Drive -- Motor Drivers
WPI_TalonSRX FrontLeft_Leader{DEVICENUMBER_FRONTLEFT_LEADER};
VictorSPX FrontLeft_Follower{DEVICENUMBER_FRONTLEFT_FOLLOWER};

WPI_TalonSRX RearLeft_Leader{DEVICENUMBER_REARLEFT_LEADER};
VictorSPX RearLeft_Follower{DEVICENUMBER_REARLEFT_FOLLOWER};

WPI_TalonSRX FrontRight_Leader{DEVICENUMBER_FRONTRIGHT_LEADER};
VictorSPX FrontRight_Follower{DEVICENUMBER_FRONTRIGHT_FOLLOWER};

WPI_TalonSRX RearRight_Leader{DEVICENUMBER_REARRIGHT_LEADER};
VictorSPX RearRight_Follower{DEVICENUMBER_REARRIGHT_FOLLOWER};

/*************************************************************************************************/
/**** Configuration ****/

void Excelsior_Omni_Drive::Configure_Omni_Drive()
{
    // Set rotation directions (clockwise == false) and set follower to leaders
    FrontLeft_Leader.SetInverted(true);
    if (OMNI_DRIVE_WRITE_CONFIGURATION)
    {
        FrontLeft_Leader.ConfigPeakOutputForward(OMNI_DRIVE_PEAK_OUTPUT_FWD);
        FrontLeft_Leader.ConfigPeakOutputReverse(OMNI_DRIVE_PEAK_OUTPUT_REV);
        FrontLeft_Leader.ConfigClosedloopRamp(OMNI_DRIVE_RAMP_TIME);
        FrontLeft_Leader.Config_kP(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_PROPORTIONAL_CTRL);
        FrontLeft_Leader.Config_kD(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_DERIVATIVE_CTRL);
        FrontLeft_Leader.Config_kF(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_FEED_FWD_CTRL);
    }

    FrontLeft_Follower.SetInverted(true);
    FrontLeft_Follower.Follow(FrontLeft_Leader);

    RearLeft_Leader.SetInverted(true);
    if (OMNI_DRIVE_WRITE_CONFIGURATION)
    {
        RearLeft_Leader.ConfigPeakOutputForward(OMNI_DRIVE_PEAK_OUTPUT_FWD);
        RearLeft_Leader.ConfigPeakOutputReverse(OMNI_DRIVE_PEAK_OUTPUT_REV);
        RearLeft_Leader.ConfigClosedloopRamp(OMNI_DRIVE_RAMP_TIME);
        RearLeft_Leader.Config_kP(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_PROPORTIONAL_CTRL);
        RearLeft_Leader.Config_kD(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_DERIVATIVE_CTRL);
        RearLeft_Leader.Config_kF(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_FEED_FWD_CTRL);
    }

    RearLeft_Follower.SetInverted(true);
    RearLeft_Follower.Follow(RearLeft_Leader);

    FrontRight_Leader.SetInverted(true);
    if (OMNI_DRIVE_WRITE_CONFIGURATION)
    {
        FrontRight_Leader.ConfigPeakOutputForward(OMNI_DRIVE_PEAK_OUTPUT_FWD);
        FrontRight_Leader.ConfigPeakOutputReverse(OMNI_DRIVE_PEAK_OUTPUT_REV);
        FrontRight_Leader.ConfigClosedloopRamp(OMNI_DRIVE_RAMP_TIME);
        FrontRight_Leader.Config_kP(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_PROPORTIONAL_CTRL);
        FrontRight_Leader.Config_kD(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_DERIVATIVE_CTRL);
        FrontRight_Leader.Config_kF(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_FEED_FWD_CTRL);
    }

    FrontRight_Follower.SetInverted(true);
    FrontRight_Follower.Follow(FrontRight_Leader);

    RearRight_Leader.SetInverted(true);
    if (OMNI_DRIVE_WRITE_CONFIGURATION)
    {
        RearRight_Leader.ConfigPeakOutputForward(OMNI_DRIVE_PEAK_OUTPUT_FWD);
        RearRight_Leader.ConfigPeakOutputReverse(OMNI_DRIVE_PEAK_OUTPUT_REV);
        RearRight_Leader.ConfigClosedloopRamp(OMNI_DRIVE_RAMP_TIME);
        RearRight_Leader.Config_kP(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_PROPORTIONAL_CTRL);
        RearRight_Leader.Config_kD(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_DERIVATIVE_CTRL);
        RearRight_Leader.Config_kF(OMNI_DRIVE_SLOT_IDX, OMNI_DRIVE_FEED_FWD_CTRL);
    }

    RearRight_Follower.SetInverted(true);
    RearRight_Follower.Follow(RearRight_Leader);
}

/*************************************************************************************************/
/**** Actions ****/

void Excelsior_Omni_Drive::Omni_Drive_Action(double x_value, double y_value, double rotation, bool aligning)
{
    // Deadband for joystick
    if (x_value < OMNI_DRIVE_DEADBAND_VALUE && x_value > -OMNI_DRIVE_DEADBAND_VALUE)
        x_value = 0;

    if (y_value < OMNI_DRIVE_DEADBAND_VALUE && y_value > -OMNI_DRIVE_DEADBAND_VALUE)
        y_value = 0;

    // Scale and vector direction
    double scalar = hypot(x_value, y_value);
    double direction = atan2(y_value, x_value) - (M_PI / 4);

    // Set calculated speed -1 to 1
    double speed_frontLeft = scalar * cos(direction) + rotation;
    double speed_frontRight = scalar * sin(direction) - rotation;
    double speed_rearLeft = scalar * sin(direction) + rotation;
    double speed_rearRight = scalar * cos(direction) - rotation;

    // if (!manual)
    // {
    //     // Drive motors at actual speed (calculated speed scaled by MAX rotations per second)
    //     FrontLeft_Leader.Set(ControlMode::Velocity, speed_frontLeft * OMNI_DRIVE_SPEED_DRIVING * CONVERT_TO_RPS_DRIVE);
    //     FrontRight_Leader.Set(ControlMode::Velocity, -speed_frontRight * OMNI_DRIVE_SPEED_DRIVING * CONVERT_TO_RPS_DRIVE);
    //     RearLeft_Leader.Set(ControlMode::Velocity, speed_rearLeft * OMNI_DRIVE_SPEED_DRIVING * CONVERT_TO_RPS_DRIVE);
    //     RearRight_Leader.Set(ControlMode::Velocity, -speed_rearRight * OMNI_DRIVE_SPEED_DRIVING * CONVERT_TO_RPS_DRIVE);
    // }

    // else
    // {
    // Manual driving
    FrontLeft_Leader.Set(ControlMode::PercentOutput, (aligning ? OMNI_DRIVE_SPEED_ALIGNING : OMNI_DRIVE_SPEED_DRIVING) * speed_frontLeft);
    FrontRight_Leader.Set(ControlMode::PercentOutput, (aligning ? OMNI_DRIVE_SPEED_ALIGNING : OMNI_DRIVE_SPEED_DRIVING) * -speed_frontRight);
    RearLeft_Leader.Set(ControlMode::PercentOutput, (aligning ? OMNI_DRIVE_SPEED_ALIGNING : OMNI_DRIVE_SPEED_DRIVING) * speed_rearLeft);
    RearRight_Leader.Set(ControlMode::PercentOutput, (aligning ? OMNI_DRIVE_SPEED_ALIGNING : OMNI_DRIVE_SPEED_DRIVING) * -speed_rearRight);
    // }
}

/*************************************************************************************************/
/**** Helper Functions ****/

void Excelsior_Omni_Drive::Print_Omni_Encoders()
{
    std::cout << "Omni FL: " << FrontLeft_Leader.GetSensorCollection().GetQuadraturePosition() << std::endl;
    std::cout << "Omni FR: " << FrontRight_Leader.GetSensorCollection().GetQuadraturePosition() << std::endl;
    std::cout << "Omni RL: " << RearLeft_Leader.GetSensorCollection().GetQuadraturePosition() << std::endl;
    std::cout << "Omni RR: " << RearRight_Leader.GetSensorCollection().GetQuadraturePosition() << std::endl;
    std::cout << std::endl;
}