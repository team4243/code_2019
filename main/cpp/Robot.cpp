/*************************************************************************************************/
/**** Includes ****/

#include "Robot.h"
#include "frc/WPILib.h"

#include "Excelsior_Classes.h"
#include <iostream>

/*************************************************************************************************/
/**** Enabling and Misc Definitions ****/

// Enabling Bits
#define ENABLE_OMNI_DRIVE (false)
#define ENABLE_PAYLOAD_LIFT (false)
#define ENABLE_END_EFFECTOR (true)

// Enabled Encoder Printing
#define PRINT_ENCODER_VALUES (true)

// Joystick COM channels
#define DRIVER_ONE_CHANNEL (0)
#define DRIVER_TWO_CHANNEL (1)

// Deadband for Triggers
#define DEADBAND_TRIGGER (0.12)

/*************************************************************************************************/
/**** Control Logic Switchboard Definitions ****/

// Drive logic
#define CTRL_DRIVE_LEFT_RIGHT (Driver_One.GetX())
#define CTRL_DRIVE_FWD_BWD (-Driver_One.GetY())
#define CTRL_DRIVE_ROTATE (Driver_One.GetRawAxis(AXIS_RIGHT_X))

#define CTRL_DRIVE_SWITCH_MANUAL (Driver_One.GetRawButton(BUTTON_BUMPER_RIGHT))

// Lift logic -- Position Mode
#define CTRL_LIFT_POSITION_LOW_CARGO (Driver_One.GetRawButton(BUTTON_GREEN) && Driver_One.GetRawButton(BUTTON_BUMPER_LEFT))
#define CTRL_LIFT_POSITION_LOW_HATCH (Driver_One.GetRawButton(BUTTON_GREEN))
#define CTRL_LIFT_POSITION_MID_CARGO (Driver_One.GetRawButton(BUTTON_RED) && Driver_One.GetRawButton(BUTTON_BUMPER_LEFT))
#define CTRL_LIFT_POSITION_MID_HATCH (Driver_One.GetRawButton(BUTTON_RED))
#define CTRL_LIFT_POSITION_HIGH_CARGO (Driver_One.GetRawButton(BUTTON_YELLOW) && Driver_One.GetRawButton(BUTTON_BUMPER_LEFT))
#define CTRL_LIFT_POSITION_HIGH_HATCH (Driver_One.GetRawButton(BUTTON_YELLOW))

// Lift logic -- Position Adjustment
#define CTRL_LIFT_POSITION_STEP_UP (Driver_One.GetPOV() == 0)
#define CTRL_LIFT_POSITION_STEP_DOWN (Driver_One.GetPOV() == 180)

// Lift logic -- Manual
#define CTRL_LIFT_UP ()
#define CTRL_LIFT_DOWN ()
#define CTRL_LIFT_STOP ()

// Cargo Roller logic -- Velocity Mode
#define CTRL_ROLL_IN (Driver_One.GetRawAxis(AXIS_LEFT_TRIGGER))
#define CTRL_ROLL_OUT (Driver_One.GetRawAxis(AXIS_RIGHT_TRIGGER))

#define CTRL_SWITCH_TO_ROLLERS (Driver_One.GetRawButton(BUTTON_BUMPER_LEFT))
#define CTRL_ROLL_MANUAL_SWITCH (Driver_One.GetRawButton(BUTTON_BUMPER_RIGHT))

// Hatch logic
#define CTRL_HATCH_IN (Driver_One.GetRawAxis(AXIS_LEFT_TRIGGER))
#define CTRL_HATCH_OUT (Driver_One.GetRawAxis(AXIS_RIGHT_TRIGGER))

// Camera Tilt logic
#define CTRL_CAMERA_UP ()
#define CTRL_CAMERA_DOWN ()

// Lift Encoder Zero logic
#define CTRL_LIFT_ENCODER_ZERO (Driver_One.GetRawButton(BUTTON_BACK) && Driver_One.GetRawButton(BUTTON_START))

/*************************************************************************************************/
/**** Object Declarations and Global Variables ****/

// Custom objects
Excelsior_Omni_Drive Omni_Drive;
Excelsior_Payload_Lift Payload_Lift;
Excelsior_End_Effector End_Effector;

// Joystick -- Controllers
frc::Joystick Driver_One{DRIVER_ONE_CHANNEL};
frc::Joystick Driver_Two{DRIVER_TWO_CHANNEL};

// Keeping track of the current Lift position
int targetPayloadHeight = -1;

// Operator must press and release POV for Payload Lift position changes
bool pressedLastFrame_autoLift = false;

/*************************************************************************************************/
/**** Teleop Periodic ****/

void Robot::TeleopPeriodic()
{
    /************* OMNI DRIVETRAIN LOGIC ***************/
    if (ENABLE_OMNI_DRIVE)
    {
        // Drive robot using GamePad -- Left Joystick for translation, Right Joystick X for rotation
        Omni_Drive.Omni_Drive_Action(CTRL_DRIVE_LEFT_RIGHT, CTRL_DRIVE_FWD_BWD, CTRL_DRIVE_ROTATE, CTRL_DRIVE_SWITCH_MANUAL);

        // Print raw encoder values
        if (PRINT_ENCODER_VALUES)
            Omni_Drive.Print_Omni_Encoders();
    }

    /************* PAYLOAD LIFT ARM LOGIC ***************/
    if (ENABLE_PAYLOAD_LIFT)
    {
        // Zero out the Lift Arm
        if (CTRL_LIFT_ENCODER_ZERO)
            Payload_Lift.Configure_Payload_Lift();

        // Cargo LOW
        if (CTRL_LIFT_POSITION_LOW_CARGO)
        {
            targetPayloadHeight = (int)Lowest_Cargo_Position;
            Payload_Lift.Payload_Lift_Action(Lowest_Cargo_Position);
        }

        // Hatch LOW
        else if (CTRL_LIFT_POSITION_LOW_HATCH)
        {
            targetPayloadHeight = (int)Lowest_Hatch_Position;
            Payload_Lift.Payload_Lift_Action(Lowest_Hatch_Position);
        }

        // Cargo MIDDLE
        else if (CTRL_LIFT_POSITION_MID_CARGO)
        {
            targetPayloadHeight = (int)Middle_Cargo_Position;
            Payload_Lift.Payload_Lift_Action(Middle_Cargo_Position);
        }

        // Hatch MIDDLE
        else if (CTRL_LIFT_POSITION_MID_HATCH)
        {
            targetPayloadHeight = (int)Middle_Hatch_Position;
            Payload_Lift.Payload_Lift_Action(Middle_Hatch_Position);
        }

        // Cargo HIGH
        else if (CTRL_LIFT_POSITION_HIGH_CARGO)
        {
            targetPayloadHeight = (int)Highest_Cargo_Position;
            Payload_Lift.Payload_Lift_Action(Highest_Cargo_Position);
        }

        // Hatch HIGH
        else if (CTRL_LIFT_POSITION_HIGH_HATCH)
        {
            targetPayloadHeight = (int)Highest_Hatch_Position;
            Payload_Lift.Payload_Lift_Action(Highest_Hatch_Position);
        }

        // Move to next highest lift position
        else if (CTRL_LIFT_POSITION_STEP_UP)
        {
            if (!pressedLastFrame_autoLift)
            {
                pressedLastFrame_autoLift = true;
                if (targetPayloadHeight < Maximum_Height_Position)
                {
                    targetPayloadHeight++;
                }
                Payload_Lift.Payload_Lift_Action((Payload_Lift_Position)targetPayloadHeight);
            }
        }

        // Move to next lowest lift position
        else if (CTRL_LIFT_POSITION_STEP_DOWN)
        {
            if (!pressedLastFrame_autoLift)
            {
                pressedLastFrame_autoLift = true;
                if (targetPayloadHeight > Ground_Position)
                {
                    targetPayloadHeight--;
                }
                Payload_Lift.Payload_Lift_Action((Payload_Lift_Position)targetPayloadHeight);
            }
        }

        // To prevent multiple presses of lift position adjustment
        else
        {
            pressedLastFrame_autoLift = false;
        }

        // Print raw encoder values
        if (PRINT_ENCODER_VALUES)
            Payload_Lift.Print_Lift_Encoder((Payload_Lift_Position)targetPayloadHeight);
    }

    /************* END EFFECTOR LOGIC ***************/
    if (ENABLE_END_EFFECTOR)
    {
        double Left_Trigger_Value = CTRL_ROLL_OUT;
        double Right_Trigger_Value = CTRL_ROLL_IN;

        bool Left_Trigger_Pressed = (Left_Trigger_Value > DEADBAND_TRIGGER);
        bool Right_Trigger_Pressed = (Right_Trigger_Value > DEADBAND_TRIGGER);

        // Cargo Rollers
        if (Left_Trigger_Pressed && CTRL_SWITCH_TO_ROLLERS)
        {
            End_Effector.Cargo_Roller_Action(true, Left_Trigger_Value, CTRL_ROLL_MANUAL_SWITCH);
        }

        else if (Right_Trigger_Pressed && CTRL_SWITCH_TO_ROLLERS)
        {
            End_Effector.Cargo_Roller_Action(false, Right_Trigger_Value, CTRL_ROLL_MANUAL_SWITCH);
        }

        // Hatch Flower
        else if (Left_Trigger_Pressed)
        {
            End_Effector.Hatch_Flower_Action(true);
        }

        else if (Right_Trigger_Pressed)
        {
            End_Effector.Hatch_Flower_Action(false);
        }

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