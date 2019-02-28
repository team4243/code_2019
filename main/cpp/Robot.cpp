/*************************************************************************************************/
/**** Includes ****/

#include "Robot.h"
#include "frc/WPILib.h"

#include "Excelsior_Classes.h"
#include <iostream>

/**** !!!!!!! TUNING VARIABLES !!!!!!!  ****/
/*************************************************************************************************/
/**** !!!!!!! TUNING VARIABLES !!!!!!!  ****/

// Joystick COM channels
#define DRIVER_ONE_CHANNEL (1) // SWAP THESE CHANNELS IF TESTING JUST DRIVER TWO CONTROLS!
#define DRIVER_TWO_CHANNEL (0) // SWAP THESE CHANNELS IF TESTING JUST DRIVER TWO CONTROLS!

// Enabling Bits
#define ENABLE_OMNI_DRIVE (false)
#define ENABLE_PAYLOAD_LIFT (false)
#define ENABLE_END_EFFECTOR (true)

// Print encoder values for any ENABLED mechanisms
#define PRINT_ENCODER_VALUES (false)

// Deadband for Gamepad Triggers
#define DEADBAND_TRIGGER (0.12)

/*************************************************************************************************/
/**** Control Logic Switchboard -- Omni Drive ****/

// Drive logic -- Velocity Mode and Manual
#define CTRL_DRIVE_LEFT_RIGHT (Driver_One.GetX())
#define CTRL_DRIVE_FWD_BWD (-Driver_One.GetY())
#define CTRL_DRIVE_ROTATE (Driver_One.GetRawAxis(AXIS_RIGHT_X))
#define CTRL_DRIVE_SWITCH_MANUAL (Driver_One.GetRawButton(BUTTON_BUMPER_LEFT) && Driver_One.GetRawButton(BUTTON_BUMPER_RIGHT))

/*************************************************************************************************/
/**** Control Logic Switchboard -- Payload Lift ****/

// Lift logic -- Position Mode -- Cargo
#define CTRL_LIFT_POSITION_LOW_CARGO (Driver_Two.GetRawButton(BUTTON_GREEN) && Driver_Two.GetRawButton(BUTTON_BUMPER_LEFT))
#define CTRL_LIFT_POSITION_MID_CARGO (Driver_Two.GetRawButton(BUTTON_RED) && Driver_Two.GetRawButton(BUTTON_BUMPER_LEFT))
#define CTRL_LIFT_POSITION_HIGH_CARGO (Driver_Two.GetRawButton(BUTTON_YELLOW) && Driver_Two.GetRawButton(BUTTON_BUMPER_LEFT))

// Lift logic -- Position Mode -- Hatch
#define CTRL_LIFT_POSITION_LOW_HATCH (Driver_Two.GetRawButton(BUTTON_GREEN) && Driver_Two.GetRawButton(BUTTON_BUMPER_RIGHT))
#define CTRL_LIFT_POSITION_MID_HATCH (Driver_Two.GetRawButton(BUTTON_RED) && Driver_Two.GetRawButton(BUTTON_BUMPER_RIGHT))
#define CTRL_LIFT_POSITION_HIGH_HATCH (Driver_Two.GetRawButton(BUTTON_YELLOW) && Driver_Two.GetRawButton(BUTTON_BUMPER_RIGHT))

// Lift logic -- Position Adjustment
#define CTRL_LIFT_POSITION_STEP_UP (Driver_Two.GetPOV() == 0)
#define CTRL_LIFT_POSITION_STEP_DOWN (Driver_Two.GetPOV() == 180)

// Lift logic -- Manual
#define CTRL_LIFT_UP_DOWN (-Driver_Two.GetY())
#define CTRL_LIFT_SWITCH_MANUAL (Driver_Two.GetRawButton(BUTTON_RIGHT_STICK_PRESS))

// Lift logic -- Encoder Zeroing for initial calibration
#define CTRL_ALL_BUMPERS (Driver_Two.GetRawButton(BUTTON_BUMPER_LEFT) && Driver_Two.GetRawButton(BUTTON_BUMPER_RIGHT))
#define CTRL_ALL_TRIGGERS ((Driver_Two.GetRawAxis(AXIS_LEFT_TRIGGER) == 1) && (Driver_Two.GetRawAxis(AXIS_RIGHT_TRIGGER) == 1))
#define CTRL_ALL_MENUS (Driver_Two.GetRawButton(BUTTON_BACK) && Driver_Two.GetRawButton(BUTTON_START))
#define CTRL_LIFT_ENCODER_ZERO (CTRL_ALL_BUMPERS && CTRL_ALL_TRIGGERS && CTRL_ALL_MENUS)

/*************************************************************************************************/
/**** Control Logic Switchboard -- End Effector ****/

// Trigger values for Cargo Roller and Hatch Flower
#define LEFT_TRIGGER_VALUE (Driver_Two.GetRawAxis(AXIS_LEFT_TRIGGER))
#define RIGHT_TRIGGER_VALUE (Driver_Two.GetRawAxis(AXIS_RIGHT_TRIGGER))

// Cargo Roller logic -- Velocity Mode
#define CTRL_ROLL_IN (LEFT_TRIGGER_VALUE > DEADBAND_TRIGGER && Driver_Two.GetRawButton(BUTTON_BUMPER_LEFT))
#define CTRL_ROLL_OUT (RIGHT_TRIGGER_VALUE > DEADBAND_TRIGGER && Driver_Two.GetRawButton(BUTTON_BUMPER_LEFT))

// Cargo Roller logic -- MANUAL
#define CTRL_ROLL_IN_OUT (Driver_Two.GetX())
#define CTRL_ROLL_SWITCH_MANUAL (Driver_Two.GetRawButton(BUTTON_RIGHT_STICK_PRESS))

// Hatch Flower logic
#define CTRL_HATCH_IN (LEFT_TRIGGER_VALUE > DEADBAND_TRIGGER)
#define CTRL_HATCH_OUT (RIGHT_TRIGGER_VALUE > DEADBAND_TRIGGER)

// Camera Tilt logic
#define CTRL_CAMERA_UP ((Driver_One.GetRawAxis(AXIS_LEFT_TRIGGER) == 1) || Driver_Two.GetRawButton(BUTTON_YELLOW))
#define CTRL_CAMERA_DOWN ((Driver_One.GetRawAxis(AXIS_RIGHT_TRIGGER) == 1) || Driver_Two.GetRawButton(BUTTON_GREEN))

/*************************************************************************************************/
/**** Object Declarations and Global Variables ****/

// Custom objects
Excelsior_Omni_Drive Omni_Drive;
Excelsior_Payload_Lift Payload_Lift;
Excelsior_End_Effector End_Effector;

// Joystick Controllers, in this case we use Gamepad's instead of traditional joysticks
frc::Joystick Driver_One{DRIVER_ONE_CHANNEL};
frc::Joystick Driver_Two{DRIVER_TWO_CHANNEL};

// Operator must press and release button for Payload Lift position changes
bool pressedLastFrame_autoLift = false;

// Operator must release manual lift action button to stop motion after press-n-hold action
bool pressedLastFrame_manualLift = false;

// Operator must release manual roller action button to stop motion after press-n-hold action
bool pressedLastFrame_cargoRollers = false;

/*************************************************************************************************/
/**** Teleop Periodic ****/

void Robot::TeleopPeriodic()
{
    /************* OMNI DRIVE LOGIC ***************/
    if (ENABLE_OMNI_DRIVE)
    {
        // Drive robot
        Omni_Drive.Omni_Drive_Action(CTRL_DRIVE_LEFT_RIGHT, CTRL_DRIVE_FWD_BWD, CTRL_DRIVE_ROTATE, CTRL_DRIVE_SWITCH_MANUAL);

        // Print raw encoder values
        if (PRINT_ENCODER_VALUES)
            Omni_Drive.Print_Omni_Encoders();
    }

    /************* PAYLOAD LIFT LOGIC ***************/
    if (ENABLE_PAYLOAD_LIFT)
    {
        // Zero out the Lift Arm
        if (CTRL_LIFT_ENCODER_ZERO)
            Payload_Lift.Zero_Encoder_Position();

        // Set Lift Position -- Cargo LOW
        else if (CTRL_LIFT_POSITION_LOW_CARGO)
            Payload_Lift.Payload_Lift_Action(Lowest_Cargo_Position);

        // Set Lift Position -- Hatch LOW
        else if (CTRL_LIFT_POSITION_LOW_HATCH)
            Payload_Lift.Payload_Lift_Action(Lowest_Hatch_Position);

        // Set Lift Position -- Cargo MIDDLE
        else if (CTRL_LIFT_POSITION_MID_CARGO)
            Payload_Lift.Payload_Lift_Action(Middle_Cargo_Position);

        // Set Lift Position -- Hatch MIDDLE
        else if (CTRL_LIFT_POSITION_MID_HATCH)
            Payload_Lift.Payload_Lift_Action(Middle_Hatch_Position);

        // Set Lift Position -- Cargo HIGH
        else if (CTRL_LIFT_POSITION_HIGH_CARGO)
            Payload_Lift.Payload_Lift_Action(Highest_Cargo_Position);

        // Set Lift Position -- Hatch HIGH
        else if (CTRL_LIFT_POSITION_HIGH_HATCH)
            Payload_Lift.Payload_Lift_Action(Highest_Hatch_Position);

        // Set Lift Position -- Move to next highest lift position
        else if (CTRL_LIFT_POSITION_STEP_UP)
        {
            // Intentionally separated, don't combine with CTRL_LIFT_POSITION_STEP_UP conditional
            if (!pressedLastFrame_autoLift)
            {
                pressedLastFrame_autoLift = true;
                Payload_Lift.Payload_Lift_Step(true);
            }
        }

        // Set Lift Position -- Move to next lowest lift position
        else if (CTRL_LIFT_POSITION_STEP_DOWN)
        {
            if (!pressedLastFrame_autoLift)
            {
                pressedLastFrame_autoLift = true;
                Payload_Lift.Payload_Lift_Step(false);
            }
        }

        // Set Lift Position -- MANUAL
        else if (CTRL_LIFT_SWITCH_MANUAL)
        {
            Payload_Lift.Payload_Lift_Manual(CTRL_LIFT_UP_DOWN);
            pressedLastFrame_manualLift = true;
        }

        // To prevent multiple presses of lift position adjustment
        else
        {
            pressedLastFrame_autoLift = false;
        }

        // Stop the Lift if operator releases manual switch
        if (!CTRL_LIFT_SWITCH_MANUAL && pressedLastFrame_manualLift)
        {
            Payload_Lift.Payload_Lift_Manual(0);
            pressedLastFrame_manualLift = false;
        }

        // Print raw encoder values
        if (PRINT_ENCODER_VALUES)
            Payload_Lift.Print_Lift_Encoder();
    }

    /************* END EFFECTOR LOGIC ***************/
    if (ENABLE_END_EFFECTOR)
    {
        // Cargo Rollers -- Manual Control
        if (CTRL_ROLL_SWITCH_MANUAL)
        {
            pressedLastFrame_cargoRollers = true;
            End_Effector.Cargo_Roller_Manual(CTRL_ROLL_IN_OUT);
        }

        // Check if the Manual Cargo Roller switch has been released
        else if (!CTRL_ROLL_SWITCH_MANUAL && pressedLastFrame_cargoRollers)
        {
            pressedLastFrame_cargoRollers = false;
            End_Effector.Cargo_Roller_Manual(0);
        }

        // Cargo Rollers -- Velocity Control
        else if (CTRL_ROLL_IN)
            End_Effector.Cargo_Roller_Action(true, LEFT_TRIGGER_VALUE);

        else if (CTRL_ROLL_OUT)
            End_Effector.Cargo_Roller_Action(false, RIGHT_TRIGGER_VALUE);

        // Hatch Flower
        else if (CTRL_HATCH_IN)
            End_Effector.Hatch_Flower_Action(true);

        else if (CTRL_HATCH_OUT)
            End_Effector.Hatch_Flower_Action(false);

        // Camera Tilt
        if (CTRL_CAMERA_UP)
            End_Effector.Camera_Tilt_Action(true);

        else if (CTRL_CAMERA_DOWN)
            End_Effector.Camera_Tilt_Action(false);

        // Print raw encoder values
        if (PRINT_ENCODER_VALUES)
            End_Effector.Print_Roller_Encoders();
    }
}

/*************************************************************************************************/
/**** Robot Initialization ****/

void Robot::RobotInit()
{
    Omni_Drive.Configure_Omni_Drive();
    Payload_Lift.Configure_Payload_Lift();
    End_Effector.Configure_End_Effector();
}

/*************************************************************************************************/
/**** Misc Required Functions ****/

void Robot::RobotPeriodic() {}
void Robot::TestPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}

/*************************************************************************************************/
/**** Start Robot -- Main() ****/

#ifndef RUNNING_FRC_TESTS

int main()
{
    return frc::StartRobot<Robot>();
}

#endif