/*************************************************************************************************/
/**** Includes ****/

#include "Robot.h"
#include "frc/WPILib.h"
#include "frc/drive/MecanumDrive.h"
#include "ctre/Phoenix.h"
#include <iostream>


/*************************************************************************************************/
/*** Definitions ****/

// Enabling Bits
#define ENABLE_DRIVETRAIN false
#define ENABLE_PAYLOAD true
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

// Omni Drive -- CAN device number
#define OMNI_DEVICENUMBER_FRONTLEFT_LEADER 1
#define OMNI_DEVICENUMBER_FRONTLEFT_FOLLOWER 2
#define OMNI_DEVICENUMBER_REARLEFT_LEADER 3
#define OMNI_DEVICENUMBER_REARLEFT_FOLLOWER 4
#define OMNI_DEVICENUMBER_FRONTRIGHT_LEADER 5
#define OMNI_DEVICENUMBER_FRONTRIGHT_FOLLOWER 6
#define OMNI_DEVICENUMBER_REARRIGHT_LEADER 7
#define OMNI_DEVICENUMBER_REARRIGHT_FOLLOWER 8

// Payload Lift -- CAN device number
#define PAYLOAD_LIFT_DEVICENUMBER_LEADER 9
#define PAYLOAD_LIFT_DEVICENUMBER_FOLLOWER 10

// Omni Drive -- Speed Gain
#define OMNI_SPEED_GAIN 1.0

// Payload Lift -- Speed Gain
#define PAYLOAD_LIFT_SPEED_GAIN 0.2

#define PAYLOAD_LOWEST_HATCH_LEVEL_MOTOR_POSITION 0
#define PAYLOAD_MIDDLE_HATCH_LEVEL_MOTOR_POSITION 512
#define PAYLOAD_HIGHEST_HATCH_LEVEL_MOTOR_POSITION 1024

// Motor Safety -- Timeout value
#define MOTOR_SAFETY_TIMEOUT 0.5


/*************************************************************************************************/
/*** Declarations ****/

// Joystick -- Controllers
frc::Joystick Joystick_GamePad { JOYSTICK_CHANNEL_GAMEPAD };
frc::Joystick Joystick_DriveStick { JOYSTICK_CHANNEL_DRIVESTICK };

// Omni Drive -- Motor Drivers
WPI_TalonSRX Omni_FrontLeft_Leader { OMNI_DEVICENUMBER_FRONTLEFT_LEADER }; 
VictorSPX Omni_FrontLeft_Follower { OMNI_DEVICENUMBER_FRONTLEFT_FOLLOWER }; 
WPI_TalonSRX Omni_RearLeft_Leader { OMNI_DEVICENUMBER_REARLEFT_LEADER };  
VictorSPX Omni_RearLeft_Follower { OMNI_DEVICENUMBER_REARLEFT_FOLLOWER };  
WPI_TalonSRX Omni_FrontRight_Leader { OMNI_DEVICENUMBER_FRONTRIGHT_LEADER }; 
VictorSPX Omni_FrontRight_Follower { OMNI_DEVICENUMBER_FRONTRIGHT_FOLLOWER }; 
WPI_TalonSRX Omni_RearRight_Leader { OMNI_DEVICENUMBER_REARRIGHT_LEADER }; 
VictorSPX Omni_RearRight_Follower { OMNI_DEVICENUMBER_REARRIGHT_FOLLOWER }; 

// Omni Drive -- Driving Object
frc::MecanumDrive Robot_Drive { Omni_FrontLeft_Leader, Omni_RearLeft_Leader, Omni_FrontRight_Leader, Omni_RearRight_Leader };

// Payload Lift -- Motor Drivers
WPI_TalonSRX Payload_Lift_Leader { PAYLOAD_LIFT_DEVICENUMBER_LEADER };
VictorSPX Payload_Lift_Follower { PAYLOAD_LIFT_DEVICENUMBER_FOLLOWER };

// End Effector -- Cargo Motor Drivers
// WPI_TalonSRX Cargo_Roller_Left;
// WPI_TalonSRX Cargo_Roller_Right;

// End Effector -- Hatch Motor Driver
// WPI_TalonSRX Hatch_Catch;


/*************************************************************************************************/
/*** Custom Functions ****/

void Configure_Omni_Drive()
{
    // TODO: Add configuration
 
    // Set rotation direction, clockwise == false
    Omni_FrontLeft_Leader.SetInverted(true);
    Omni_FrontLeft_Follower.SetInverted(true);

    Omni_RearLeft_Leader.SetInverted(true);
    Omni_RearLeft_Follower.SetInverted(true);

    Omni_FrontRight_Leader.SetInverted(true);
    Omni_FrontRight_Follower.SetInverted(true);

    Omni_RearRight_Leader.SetInverted(true);
    Omni_RearRight_Follower.SetInverted(true);
}

void Configure_Payload_Lift()
{
    Payload_Lift_Leader.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
    Payload_Lift_Leader.SetSensorPhase(false); 
	Payload_Lift_Leader.SetSelectedSensorPosition(0, 0, 10);

    // Set rotation direction, clockwise == false
    Payload_Lift_Leader.SetInverted(true);
    Payload_Lift_Follower.SetInverted(true);
}

void Configure_End_Effector()
{
    // TODO: Add configuration
}

void Stop_All_Motors()
{
    // Set output speed, 0 == stop
    Omni_FrontLeft_Leader.Set(ControlMode::PercentOutput, 0);
    Omni_FrontLeft_Follower.Set(ControlMode::PercentOutput, 0);

    Omni_RearLeft_Leader.Set(ControlMode::PercentOutput, 0);
    Omni_RearLeft_Follower.Set(ControlMode::PercentOutput, 0);

    Omni_FrontRight_Leader.Set(ControlMode::PercentOutput, 0);
    Omni_FrontRight_Follower.Set(ControlMode::PercentOutput, 0);
    
    Omni_RearRight_Leader.Set(ControlMode::PercentOutput, 0);
    Omni_RearRight_Follower.Set(ControlMode::PercentOutput, 0);

    Payload_Lift_Leader.Set(ControlMode::PercentOutput, 0);
    Payload_Lift_Follower.Set(ControlMode::PercentOutput, 0);
}

void Activate_Followers() {
    if (ENABLE_DRIVETRAIN) { 
        Omni_FrontLeft_Follower.Follow(Omni_FrontLeft_Leader); 
        Omni_RearLeft_Follower.Follow(Omni_RearLeft_Leader); 
        Omni_FrontRight_Follower.Follow(Omni_FrontRight_Leader); 
        Omni_RearRight_Follower.Follow(Omni_RearRight_Leader);
    }

    if (ENABLE_PAYLOAD) { 
        Payload_Lift_Follower.Follow(Payload_Lift_Leader);
    }
}

void Enable_Motor_Safety(bool enable)
{
    // Expiration is maximum time to update motor drivers, otherwise they kill automatically
    Robot_Drive.SetExpiration(MOTOR_SAFETY_TIMEOUT);

    // Enable/disable the safety feature using the Expiration above
    Robot_Drive.SetSafetyEnabled(enable);
}


/*************************************************************************************************/
/*** Robot Initialization ****/

void Robot::RobotInit()
{    
    Configure_Omni_Drive();
    Configure_Payload_Lift();
    Configure_End_Effector();

    Stop_All_Motors();
    Activate_Followers();
    Enable_Motor_Safety(false);

}


/*************************************************************************************************/
/*** Misc Required Functions ****/

void Robot::RobotPeriodic() {}
void Robot::TestPeriodic() {}
void Robot::TeleopInit() {}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}


/*************************************************************************************************/
/*** Teleop Periodic ****/

void Robot::TeleopPeriodic()
{
    if(ENABLE_DRIVETRAIN)
    {
        // Drive robot using GamePad joystick
        Robot_Drive.DriveCartesian(
            Joystick_GamePad.GetX() * OMNI_SPEED_GAIN,
            -Joystick_GamePad.GetY() * OMNI_SPEED_GAIN,
            Joystick_GamePad.GetRawAxis(AXIS_RIGHT_X) * OMNI_SPEED_GAIN
        );
    }

    if(ENABLE_PAYLOAD)
    {  

        // Send Payload Arm to hatch levels
        if (Joystick_GamePad.GetRawButton(BUTTON_GREEN)) {   
            Payload_Lift_Leader.Set(ControlMode::PercentOutput, PAYLOAD_LIFT_SPEED_GAIN);
            //Payload_Lift_Leader.Set(ControlMode::Position, PAYLOAD_LOWEST_HATCH_LEVEL_MOTOR_POSITION);
        } 
        else if (Joystick_GamePad.GetRawButton(BUTTON_BLUE)) 
        { 
            Payload_Lift_Leader.Set(ControlMode::Position, PAYLOAD_MIDDLE_HATCH_LEVEL_MOTOR_POSITION); 
        } 
        else if (Joystick_GamePad.GetRawButton(BUTTON_YELLOW)) 
        { 
            Payload_Lift_Leader.Set(ControlMode::Position, PAYLOAD_HIGHEST_HATCH_LEVEL_MOTOR_POSITION); 
        } 
        else if (Joystick_GamePad.GetRawButton(BUTTON_RED)) 
        { 
            Payload_Lift_Leader.Set(ControlMode::PercentOutput, -PAYLOAD_LIFT_SPEED_GAIN);
        } 
        else 
        { 
            Payload_Lift_Leader.Set(ControlMode::PercentOutput, 0);
        }

        std::cout << "Current Quadrature Position: " << Payload_Lift_Leader.GetSensorCollection().GetQuadraturePosition() << std::endl;
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
			// TODO: Add drive logic for hatch release
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