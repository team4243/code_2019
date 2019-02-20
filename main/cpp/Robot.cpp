/*************************************************************************************************/
/**** Includes ****/

#include "Robot.h"
#include "frc/WPILib.h"

#include "Excelsior_Classes.h"
#include <iostream>

/*************************************************************************************************/
/**** Definitions ****/

// Enabling Bits
#define ENABLE_OMNI_DRIVE (false)
#define ENABLE_PAYLOAD_LIFT (true)
#define ENABLE_END_EFFECTOR (false)

// Enabled Encoder Printing
#define PRINT_ENCODER_VALUES (true)

// Joystick COM channels
#define JOYSTICK_CHANNEL_GAMEPAD (0)
#define JOYSTICK_CHANNEL_DRIVESTICK (1)

// Joystick Buttons
#define BUTTON_GREEN (1)
#define BUTTON_RED (2)
#define BUTTON_BLUE (3)
#define BUTTON_YELLOW (4)

#define BUTTON_BUMPER_LEFT (5)
#define BUTTON_BUMPER_RIGHT (6)

#define BUTTON_BACK (7)
#define BUTTON_START (8)

#define BUTTON_LEFT_STICK_PRESS (9)
#define BUTTON_RIGHT_STICK_PRESS (10)

// Joystick Directional Pad use POV()

// Joystick Triggers
#define AXIS_LEFT_TRIGGER (2) // 0 to 1
#define AXIS_RIGHT_TRIGGER (3) // 0 to 1

// Joystick Wheels
#define AXIS_LEFT_X (0)
#define AXIS_LEFT_Y (1)

#define AXIS_RIGHT_X (4)
#define AXIS_RIGHT_Y (5)

// Deadband for Triggers
#define DEADBAND_TRIGGER (0.12)

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

// Operator must press and release POV for Payload Lift position changes
bool pressedLastFrame_autoLift = false;
bool pressedLastFrame_manualLift = false;

/*************************************************************************************************/
/**** Teleop Periodic ****/

void Robot::TeleopPeriodic()
{
    /************* OMNI DRIVETRAIN LOGIC ***************/
    if (ENABLE_OMNI_DRIVE)
    {
        // Drive robot using GamePad -- Left Joystick for translation, Right Joystick X for rotation
        Omni_Drive.Omni_Drive_Action(Joystick_GamePad.GetX(), -Joystick_GamePad.GetY(), 
            Joystick_GamePad.GetRawAxis(AXIS_RIGHT_X), Joystick_GamePad.GetRawButton(BUTTON_BUMPER_RIGHT));

        // Print raw encoder values
        if(PRINT_ENCODER_VALUES) Omni_Drive.Print_Omni_Encoders();
    }

    /************* PAYLOAD LIFT ARM LOGIC ***************/
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
        else if (Joystick_GamePad.GetPOV() == 180)
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

        // Manual drive -- LIFT
        else if (Joystick_GamePad.GetPOV() == 90)
        {
            if(!pressedLastFrame_manualLift) 
            {
                pressedLastFrame_manualLift = true;
                Payload_Lift.Payload_Lift_Manual(true);
            }
        } 
        
        // Manual drive -- LOWER
        else if (Joystick_GamePad.GetPOV() == 270)
        {
           if(!pressedLastFrame_manualLift) 
            {
                pressedLastFrame_manualLift = true;
                Payload_Lift.Payload_Lift_Manual(false);
            }
        } 

        else 
        { 
            pressedLastFrame_autoLift = false;
            pressedLastFrame_manualLift = false; 
            // Payload_Lift.Stop();
        }

        // Print raw encoder values
        if(PRINT_ENCODER_VALUES) Payload_Lift.Print_Lift_Encoder((Payload_Lift_Position)targetPayloadHeight);

        // Zero out the Lift Arm
        if(Joystick_GamePad.GetRawButton(BUTTON_BACK) && Joystick_GamePad.GetRawButton(BUTTON_START)) Payload_Lift.Configure_Payload_Lift();
    }

    /************* END EFFECTOR LOGIC ***************/
    if (ENABLE_END_EFFECTOR)
    {
        double Left_Trigger_Value = Joystick_GamePad.GetRawAxis(AXIS_LEFT_TRIGGER);
        double Right_Trigger_Value = Joystick_GamePad.GetRawAxis(AXIS_RIGHT_TRIGGER);

        bool Left_Trigger_Pressed = (Left_Trigger_Value > DEADBAND_TRIGGER);
        bool Right_Trigger_Pressed = (Right_Trigger_Value > DEADBAND_TRIGGER);

        // Cargo Rollers
        if (Left_Trigger_Pressed && Joystick_GamePad.GetRawButton(BUTTON_BUMPER_LEFT))
        {
            End_Effector.Cargo_Roller_Action(true, Left_Trigger_Value, Joystick_GamePad.GetRawButton(BUTTON_BUMPER_RIGHT));
        }

        else if (Right_Trigger_Pressed && Joystick_GamePad.GetRawButton(BUTTON_BUMPER_LEFT))
        {
            End_Effector.Cargo_Roller_Action(false, Right_Trigger_Value, Joystick_GamePad.GetRawButton(BUTTON_BUMPER_RIGHT));
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
        if(PRINT_ENCODER_VALUES) End_Effector.Print_Roller_Encoders();
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