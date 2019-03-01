/*************************************************************************************************/
/**** Includes ****/

#include "Robot.h"
#include "frc/WPILib.h"

#include "Excelsior_Classes.h"
#include <iostream>

#include <Math.h>

/**** !!!!!!! TUNING VARIABLES !!!!!!!  ****/
/*************************************************************************************************/
/**** !!!!!!! TUNING VARIABLES !!!!!!!  ****/

// Joystick COM channels
#define DRIVER_ONE_CHANNEL (0) // SWAP THESE CHANNELS IF TESTING JUST DRIVER TWO CONTROLS!
#define DRIVER_TWO_CHANNEL (1) // SWAP THESE CHANNELS IF TESTING JUST DRIVER TWO CONTROLS!

// Enabling Bits
#define ENABLE_OMNI_DRIVE (true)
#define ENABLE_PAYLOAD_LIFT (true)
#define ENABLE_END_EFFECTOR (false)

// Print operator control
#define PRINT_CONTROL_VALUES (true)

// Print encoder values for any ENABLED mechanisms
#define PRINT_ENCODER_VALUES (true)

// Deadband for Gamepad Triggers
#define DEADBAND_TRIGGER (0.12)

/*************************************************************************************************/
/**** Control Logic Switchboard -- Omni Drive ****/

// Drive logic -- Velocity Mode and Manual
#define CTRL_DRIVE_LEFT_RIGHT (Driver_One.GetX())
#define CTRL_DRIVE_FWD_BWD (-Driver_One.GetY())
#define CTRL_DRIVE_ROTATE (Driver_One.GetRawAxis(AXIS_R3_X))
#define CTRL_DRIVE_SWITCH_MANUAL (true) //(Driver_One.GetRawButton(BUTTON_L1) && Driver_One.GetRawButton(BUTTON_R1))

/*************************************************************************************************/
/**** Control Logic Switchboard -- Payload Lift ****/

// Lift logic -- Position Mode -- Cargo
#define CTRL_LIFT_POSITION_LOW_CARGO (Driver_Two.GetRawButton(BUTTON_CROSS) && Driver_Two.GetRawButton(BUTTON_L1))
#define CTRL_LIFT_POSITION_MID_CARGO (Driver_Two.GetRawButton(BUTTON_CIRCLE) && Driver_Two.GetRawButton(BUTTON_L1))
#define CTRL_LIFT_POSITION_HIGH_CARGO (Driver_Two.GetRawButton(BUTTON_TRIANGLE) && Driver_Two.GetRawButton(BUTTON_L1))

// Lift logic -- Position Mode -- Hatch
#define CTRL_LIFT_POSITION_LOW_HATCH (Driver_Two.GetRawButton(BUTTON_CROSS) && Driver_Two.GetRawButton(BUTTON_R1))
#define CTRL_LIFT_POSITION_MID_HATCH (Driver_Two.GetRawButton(BUTTON_CIRCLE) && Driver_Two.GetRawButton(BUTTON_R1))
#define CTRL_LIFT_POSITION_HIGH_HATCH (Driver_Two.GetRawButton(BUTTON_TRIANGLE) && Driver_Two.GetRawButton(BUTTON_R1))

// Lift logic -- Position Adjustment
#define CTRL_LIFT_POSITION_STEP_UP (Driver_Two.GetPOV() == 0)
#define CTRL_LIFT_POSITION_STEP_DOWN (Driver_Two.GetPOV() == 180)

// Lift logic -- Manual
#define CTRL_LIFT_UP_DOWN (-Driver_Two.GetY())
#define CTRL_LIFT_SWITCH_MANUAL (Driver_Two.GetRawButton(BUTTON_R3))

// Lift logic -- Encoder Zeroing for initial calibration
#define CTRL_ALL_TRIGGERS (Driver_Two.GetRawButton(BUTTON_L2) && Driver_Two.GetRawButton(BUTTON_R2))
#define CTRL_ALL_MENUS (Driver_Two.GetRawButton(BUTTON_SHARE) && Driver_Two.GetRawButton(BUTTON_OPTIONS))
#define CTRL_LIFT_ENCODER_ZERO (CTRL_ALL_TRIGGERS && CTRL_ALL_MENUS)

/*************************************************************************************************/
/**** Control Logic Switchboard -- End Effector ****/

// Trigger values for Cargo Roller and Hatch Flower
#define LEFT_TRIGGER_VALUE (Driver_Two.GetRawAxis(AXIS_L2))
#define RIGHT_TRIGGER_VALUE (Driver_Two.GetRawAxis(AXIS_R2))

// Cargo Roller logic -- Velocity Mode
#define CTRL_ROLL_IN ((LEFT_TRIGGER_VALUE > DEADBAND_TRIGGER) && Driver_Two.GetRawButton(BUTTON_L1))
#define CTRL_ROLL_OUT ((RIGHT_TRIGGER_VALUE > DEADBAND_TRIGGER) && Driver_Two.GetRawButton(BUTTON_L1))

// Cargo Roller logic -- MANUAL
#define CTRL_ROLL_IN_OUT (Driver_Two.GetX())
#define CTRL_ROLL_SWITCH_MANUAL (true) //(Driver_Two.GetRawButton(BUTTON_R3))

// Mechanical Lili-Pad Balloon logic
#define CTRL_HATCH_IN ((LEFT_TRIGGER_VALUE > DEADBAND_TRIGGER) && Driver_Two.GetRawButton(BUTTON_R1))
#define CTRL_HATCH_OUT ((RIGHT_TRIGGER_VALUE > DEADBAND_TRIGGER) && Driver_Two.GetRawButton(BUTTON_R1))

// Camera Tilt logic
#define CTRL_CAMERA_UP (Driver_One.GetRawButton(BUTTON_L2))
#define CTRL_CAMERA_DOWN (Driver_One.GetRawButton(BUTTON_R2))

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
        if (PRINT_CONTROL_VALUES)
        {
            if (abs(CTRL_DRIVE_FWD_BWD) > DEADBAND_TRIGGER)
            {
                std::cout << "WARNING: "
                          << "Forwards by" << CTRL_DRIVE_FWD_BWD << std::endl;
            }
            if (abs(CTRL_DRIVE_LEFT_RIGHT) > DEADBAND_TRIGGER)
            {
                std::cout << "WARNING: "
                          << "Strafing by" << CTRL_DRIVE_LEFT_RIGHT << std::endl;
            }
            if (abs(CTRL_DRIVE_ROTATE) > DEADBAND_TRIGGER)
            {
                std::cout << "WARNING: "
                          << "Rotating by" << CTRL_DRIVE_ROTATE << std::endl;
            }
            if (CTRL_DRIVE_SWITCH_MANUAL)
            {
                std::cout << "WARNING: "
                          << "{M} Manual Drive Mode Activated"
                          << "" << std::endl;
            }
        }

        else
            Omni_Drive.Omni_Drive_Action(CTRL_DRIVE_LEFT_RIGHT, CTRL_DRIVE_FWD_BWD, CTRL_DRIVE_ROTATE, CTRL_DRIVE_SWITCH_MANUAL);
    }

    /************* PAYLOAD LIFT LOGIC ***************/
    if (ENABLE_PAYLOAD_LIFT)
    {
        // Zero out the Lift Arm
        if (CTRL_LIFT_ENCODER_ZERO)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "Enconders Zeroed"
                          << "" << std::endl;
            else
                Payload_Lift.Zero_Encoder_Position();
        }

        // Set Lift Position -- Cargo LOW
        else if (CTRL_LIFT_POSITION_LOW_CARGO)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "(C) Lowest Position Targetted by Arm -- Cargo"
                          << "" << std::endl;
            else
                Payload_Lift.Payload_Lift_Action(Lowest_Cargo_Position);
        }

        // Set Lift Position -- Hatch LOW
        else if (CTRL_LIFT_POSITION_LOW_HATCH)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "(H) Lowest Position Targetted by Arm -- Hatch"
                          << "" << std::endl;
            else
                Payload_Lift.Payload_Lift_Action(Lowest_Hatch_Position);
        }

        // Set Lift Position -- Cargo MIDDLE
        else if (CTRL_LIFT_POSITION_MID_CARGO)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "(C) Middle Position Targetted by Arm -- Cargo"
                          << "" << std::endl;
            else
                Payload_Lift.Payload_Lift_Action(Middle_Cargo_Position);
        }

        // Set Lift Position -- Hatch MIDDLE
        else if (CTRL_LIFT_POSITION_MID_HATCH)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "(H) Middle Position Targetted by Arm -- Hatch"
                          << "" << std::endl;
            else
                Payload_Lift.Payload_Lift_Action(Middle_Hatch_Position);
        }

        // Set Lift Position -- Cargo HIGH
        else if (CTRL_LIFT_POSITION_HIGH_CARGO)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "(C) Highest Position Targetted by Arm -- Cargo"
                          << "" << std::endl;
            else
                Payload_Lift.Payload_Lift_Action(Highest_Cargo_Position);
        }

        // Set Lift Position -- Hatch HIGH
        else if (CTRL_LIFT_POSITION_HIGH_HATCH)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "(H) Highest Position Targetted by Arm -- Hatch"
                          << "" << std::endl;
            else
                Payload_Lift.Payload_Lift_Action(Highest_Hatch_Position);
        }

        // Set Lift Position -- Move to next highest lift position
        else if (CTRL_LIFT_POSITION_STEP_UP)
        {
            // Intentionally separated, don't combine with CTRL_LIFT_POSITION_STEP_UP conditional
            if (!pressedLastFrame_autoLift)
            {
                pressedLastFrame_autoLift = true;

                if (PRINT_CONTROL_VALUES)
                    std::cout << "WARNING: "
                              << "Next Position Targetted by Arm"
                              << "" << std::endl;
                else
                    Payload_Lift.Payload_Lift_Step(true);
            }
        }

        // Set Lift Position -- Move to next lowest lift position
        else if (CTRL_LIFT_POSITION_STEP_DOWN)
        {
            if (!pressedLastFrame_autoLift)
            {
                pressedLastFrame_autoLift = true;

                if (PRINT_CONTROL_VALUES)
                    std::cout << "WARNING: "
                              << "Previous Position Targetted by Arm"
                              << "" << std::endl;
                else
                    Payload_Lift.Payload_Lift_Step(false);
            }
        }

        // Set Lift Position -- MANUAL
        else if (CTRL_LIFT_SWITCH_MANUAL)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "(M) Moving Arm by " << CTRL_LIFT_UP_DOWN << std::endl;
            else
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
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "(M) Stopping Arm"
                          << "" << std::endl;

            pressedLastFrame_manualLift = false;
        }
    }

    /************* END EFFECTOR LOGIC ***************/
    if (ENABLE_END_EFFECTOR)
    {
        // Cargo Rollers -- Manual Control
        if (CTRL_ROLL_SWITCH_MANUAL)
        {
            pressedLastFrame_cargoRollers = true;

            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "[M] Rolling Cargo Ball by " << CTRL_ROLL_IN_OUT << std::endl;
            else
                End_Effector.Cargo_Roller_Manual(CTRL_ROLL_IN_OUT);
        }

        // Check if the Manual Cargo Roller switch has been released
        else if (!CTRL_ROLL_SWITCH_MANUAL && pressedLastFrame_cargoRollers)
        {
            pressedLastFrame_cargoRollers = false;

            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "[M] Stopping Rollers" << std::endl;
            else
                End_Effector.Cargo_Roller_Manual(0);
        }

        // Cargo Rollers -- Velocity Control
        else if (CTRL_ROLL_IN)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "Swallowing Cargo Ball" << std::endl;
            else
                End_Effector.Cargo_Roller_Action(true, LEFT_TRIGGER_VALUE);
        }

        else if (CTRL_ROLL_OUT)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "Spitting Cargo Ball" << std::endl;
            else
                End_Effector.Cargo_Roller_Action(false, RIGHT_TRIGGER_VALUE);
        }

        // Hatch Flower
        else if (CTRL_HATCH_IN)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "Contracting Mechanical Lili-Pad Ballon" << std::endl;
            else
                End_Effector.Hatch_Flower_Action(true);
        }

        else if (CTRL_HATCH_OUT)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "Extending Mechanical Lili-Pad Balloon" << std::endl;
            else
                End_Effector.Hatch_Flower_Action(false);
        }

        // Camera Tilt
        if (CTRL_CAMERA_UP)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "Upwards Camera Movement" << std::endl;
            else
                End_Effector.Camera_Tilt_Action(true);
        }

        else if (CTRL_CAMERA_DOWN)
        {
            if (PRINT_CONTROL_VALUES)
                std::cout << "WARNING: "
                          << "Downwards Camera Movement" << std::endl;
            else
                End_Effector.Camera_Tilt_Action(false);
        }
    }
}

/*************************************************************************************************/
/**** Robot Initialization ****/

void Robot::RobotInit()
{
    if (ENABLE_OMNI_DRIVE)
        Omni_Drive.Configure_Omni_Drive();

    if (ENABLE_PAYLOAD_LIFT)
        Payload_Lift.Configure_Payload_Lift();

    if (ENABLE_END_EFFECTOR)
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