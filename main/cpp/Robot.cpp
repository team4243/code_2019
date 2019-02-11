/*************************************************************************************************/
/**** Includes ****/

#include "Robot.h"
#include "frc/WPILib.h"

#include "Excelsior_Classes.h"
#include <iostream>

/*************************************************************************************************/
/**** Definitions ****/

// Enabling Bits
#define ENABLE_OMNI_DRIVE true
#define ENABLE_PAYLOAD_LIFT false
#define ENABLE_END_EFFECTOR false

// Joystick COM channels
#define JOYSTICK_CHANNEL_GAMEPAD 0
#define JOYSTICK_CHANNEL_DRIVESTICK 1

// Joystick Buttons
#define BUTTON_GREEN 1
#define BUTTON_RED 2
#define BUTTON_BLUE 3
#define BUTTON_YELLOW 4

#define BUTTON_BUMPER_LEFT 5
#define BUTTON_BUMPER_RIGHT 6

#define BUTTON_BACK 7
#define BUTTON_START 8

#define BUTTON_LEFT_STICK_PRESS 9
#define BUTTON_RIGHT_STICK_PRESS 10

// Joystick Directional Pad use POV()

// Joystick Triggers
#define AXIS_LEFT_TRIGGER 2 // 0 to 1
#define AXIS_RIGHT_TRIGGER 3 // 0 to 1

// Joystick Wheels
#define AXIS_LEFT_X 0
#define AXIS_LEFT_Y 1

#define AXIS_RIGHT_X 4
#define AXIS_RIGHT_Y 5

// Deadband for Triggers
#define DEADBAND_TRIGGER 0.12

/*************************************************************************************************/
/**** Declarations ****/

// Custom objects
Excelsior_Omni_Drive Omni_Drive;
Excelsior_Payload_Lift Payload_Lift;
Excelsior_End_Effector End_Effector;

// Joystick -- Controllers
frc::Joystick Joystick_GamePad{JOYSTICK_CHANNEL_GAMEPAD};
frc::Joystick Joystick_DriveStick{JOYSTICK_CHANNEL_DRIVESTICK};

// Keeping track of the current Lift position
int targetPayloadHeight = -1;

/*************************************************************************************************/
/**** Teleop Periodic ****/

void Robot::TeleopPeriodic()
{
    if (ENABLE_OMNI_DRIVE)
    {
        // Drive robot using GamePad -- Left Joystick for translation, Right Joystick X for rotation
        Omni_Drive.Omni_Drive_Action(Joystick_GamePad.GetX(), -Joystick_GamePad.GetY(), Joystick_GamePad.GetRawAxis(AXIS_RIGHT_X));
    }

    if (ENABLE_PAYLOAD_LIFT)
    {

        // Cargo LOW
        if (Joystick_GamePad.GetRawButton(BUTTON_GREEN) && Joystick_GamePad.GetRawButton(BUTTON_BUMPER_LEFT))
        {
            targetPayloadHeight = (int)Lowest_Cargo_Position;
            Payload_Lift.Payload_Lift_Action(Lowest_Cargo_Position);
        }

        // Hatch LOW
        else if (Joystick_GamePad.GetRawButton(BUTTON_GREEN))
        {
            targetPayloadHeight = (int)Lowest_Hatch_Position;
            Payload_Lift.Payload_Lift_Action(Lowest_Hatch_Position);
        }

        // Cargo MIDDLE
        else if (Joystick_GamePad.GetRawButton(BUTTON_RED) && Joystick_GamePad.GetRawButton(BUTTON_BUMPER_LEFT))
        {
            targetPayloadHeight = (int)Middle_Cargo_Position;
            Payload_Lift.Payload_Lift_Action(Middle_Cargo_Position);
        }

        // Hatch MIDDLE
        else if (Joystick_GamePad.GetRawButton(BUTTON_RED))
        {
            targetPayloadHeight = (int)Middle_Hatch_Position;
            Payload_Lift.Payload_Lift_Action(Middle_Hatch_Position);
        }

        // Cargo HIGH
        else if (Joystick_GamePad.GetRawButton(BUTTON_YELLOW) && Joystick_GamePad.GetRawButton(BUTTON_BUMPER_LEFT))
        {
            targetPayloadHeight = (int)Highest_Cargo_Position;
            Payload_Lift.Payload_Lift_Action(Highest_Cargo_Position);
        }

        // Hatch HIGH
        else if (Joystick_GamePad.GetRawButton(BUTTON_YELLOW))
        {
            targetPayloadHeight = (int)Highest_Hatch_Position;
            Payload_Lift.Payload_Lift_Action(Highest_Hatch_Position);
        }

        // Move to next highest lift position
        else if (Joystick_GamePad.GetPOV() == 0)
        {
            if (targetPayloadHeight < Highest_Cargo_Position)
            {
                targetPayloadHeight++;
            }
            Payload_Lift.Payload_Lift_Action((Payload_Lift_Position)targetPayloadHeight);
        }

        // Move to next lowest lift position
        else if (Joystick_GamePad.GetPOV() == 180)
        {
            if (targetPayloadHeight > Ground_Position)
            {
                targetPayloadHeight--;
            }
            Payload_Lift.Payload_Lift_Action((Payload_Lift_Position)targetPayloadHeight);
        }
    }

    if (ENABLE_END_EFFECTOR)
    {
        double Left_Trigger_Value = Joystick_GamePad.GetRawAxis(AXIS_LEFT_TRIGGER);
        double Right_Trigger_Value = Joystick_GamePad.GetRawAxis(AXIS_RIGHT_TRIGGER);

        bool Left_Trigger_Pressed = (Left_Trigger_Value > DEADBAND_TRIGGER);
        bool Right_Trigger_Pressed = (Right_Trigger_Value > DEADBAND_TRIGGER);


        // Cargo Rollers
        if (Left_Trigger_Pressed && Joystick_GamePad.GetRawButton(BUTTON_BUMPER_LEFT))
        {
            End_Effector.Cargo_Roller_Action(true, Left_Trigger_Value);
        }

        else if (Right_Trigger_Pressed && Joystick_GamePad.GetRawButton(BUTTON_BUMPER_LEFT))
        {
            End_Effector.Cargo_Roller_Action(false, Right_Trigger_Value);
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