/*************************************************************************************************/
/**** Includes ****/

#include "Robot.h"
#include "frc/WPILib.h"
#include "frc/PWM.h"
#include "frc/drive/MecanumDrive.h"
#include "ctre/Phoenix.h"
#include <iostream>
#include <frc/Talon.h>
#include <Math.h>
#include "Excelsior_Classes.h"


/*************************************************************************************************/
/*** Definitions ****/

// Enabling Bits
#define ENABLE_DRIVETRAIN true
#define ENABLE_PAYLOAD false
#define ENABLE_END_EFFECTOR false

// Joystick COM channels
#define JOYSTICK_CHANNEL_GAMEPAD 0
#define JOYSTICK_CHANNEL_DRIVESTICK 1

// Joystick Button and Axis Values
#define BUTTON_GREEN 1
#define BUTTON_RED 2
#define BUTTON_BLUE 3
#define BUTTON_YELLOW 4

#define BUTTON_BUMPER_LEFT 5
#define BUTTON_BUMPER_RIGHT 6

#define BUTTON_BACK 7
#define BUTTON_START 8

#define AXIS_LEFT_X 0
#define AXIS_LEFT_Y 1
#define AXIS_LEFT_TRIGGER 2
#define AXIS_RIGHT_TRIGGER 3
#define AXIS_RIGHT_X 4
#define AXIS_RIGHT_Y 5




/*************************************************************************************************/
/*** Declarations ****/

Excelsior_OmniDrive Omni_Drive;
Excelsior_Payload Payload;
Hatch_Flower Hatch;

// Joystick -- Controllers
frc::Joystick Joystick_GamePad { JOYSTICK_CHANNEL_GAMEPAD };
frc::Joystick Joystick_DriveStick { JOYSTICK_CHANNEL_DRIVESTICK };

enum PHeights {
    Ground,
    HatchLow,
    HatchMid,
    HatchHigh,
    CargoLow,
    CargoMid,
    CargoHigh
};
int targetPayloadHeight;

/*************************************************************************************************/
/*** Custom Functions ****/ 



/*************************************************************************************************/
/*** Robot Initialization ****/

void Robot::RobotInit()
{    
    Omni_Drive.Configure_Omni_Drive();

    Payload.Configure_Payload_Lift();

    Hatch.Configure_End_Effector();
}


/*************************************************************************************************/
/*** Misc Required Functions ****/

void Robot::RobotPeriodic() {}
void Robot::TestPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {} 

/*************************************************************************************************/
/*** Teleop Periodic ****/

void Robot::TeleopPeriodic()
{
    if(ENABLE_DRIVETRAIN)
    {
        // Drive robot using GamePad joystick
        Omni_Drive.OmniDrive_SpeedControl(Joystick_GamePad.GetX(), -Joystick_GamePad.GetY(), Joystick_GamePad.GetRawAxis(AXIS_RIGHT_X));
    }

    if(ENABLE_PAYLOAD) {   
        
        if (Joystick_GamePad.GetRawButton(BUTTON_GREEN)) {    
            if (Joystick_GamePad.GetRawButton(BUTTON_BUMPER_LEFT)) { 
                targetPayloadHeight = (int)CargoLow; 
                Payload.Payload_Action(Lowest_Cargo_Position);

            } else {    
                targetPayloadHeight = (int)HatchLow; 
                Payload.Payload_Action(Lowest_Hatch_Position);
            }
        } else if (Joystick_GamePad.GetRawButton(BUTTON_BLUE)) {  
            if (Joystick_GamePad.GetRawButton(BUTTON_BUMPER_LEFT)) { 
                targetPayloadHeight = (int)CargoMid; 
                Payload.Payload_Action(Middle_Cargo_Position);

            } else { 
                targetPayloadHeight = (int)HatchMid; 
                Payload.Payload_Action(Middle_Hatch_Position);

            }
        } else if (Joystick_GamePad.GetRawButton(BUTTON_YELLOW)) {   
            if (Joystick_GamePad.GetRawButton(BUTTON_BUMPER_LEFT)) { 
                targetPayloadHeight = (int)CargoHigh; 
                Payload.Payload_Action(Highest_Cargo_Position);

            } else { 
                targetPayloadHeight = (int)HatchHigh; 
                Payload.Payload_Action(Highest_Hatch_Position);
                
            }
        } else if (Joystick_GamePad.GetPOV() == 0) {      
            //CargoHigh is the highest value for PHeights
            if (targetPayloadHeight < CargoHigh) {targetPayloadHeight++; }
            Payload.Payload_Action((PHeights)targetPayloadHeight);
            
        } else if (Joystick_GamePad.GetPOV() == 180) {     
            if (targetPayloadHeight > 0) {targetPayloadHeight--; }
            Payload.Payload_Action((PHeights)targetPayloadHeight);

        } else if (Joystick_GamePad.GetRawButton(BUTTON_RED))  {    
            Payload.Payload_Action(Halt_Motor);
        }

        //std::cout << "Current Quadrature Position: " << -Payload_Lift_Leader.GetSensorCollection().GetQuadraturePosition() << std::endl;
    }  

    if(ENABLE_END_EFFECTOR)
    {
        // Pick Up Cargo
        if (Joystick_GamePad.GetRawButton(BUTTON_YELLOW)) 
        {
			// TODO: Add drive logic for cargo pick up
        }

        // Deliver Cargo
        else if (Joystick_GamePad.GetRawButton(BUTTON_BLUE)) 
        {
			// TODO: Add drive logic for cargo delivery
        }

        // Catch Hatch
        else if (Joystick_GamePad.GetRawButton(BUTTON_BUMPER_LEFT)) 
        {
			// TODO: Add drive logic for hatch catch
        }

        // Release Hatch
        else if (Joystick_GamePad.GetRawButton(BUTTON_BUMPER_RIGHT)) 
        {
			// Hatch_Catch_Servo.SetPosition(0.5);
        }
    }  
}


/*************************************************************************************************/
/*** Start Robot -- Main() ****/

#ifndef RUNNING_FRC_TESTS

int main()
{
  return frc::StartRobot<Robot>();
}

#endif