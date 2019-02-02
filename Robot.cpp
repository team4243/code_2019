#include "frc/WPILib.h"
#include "Robot.h"
#include <iostream>
#include "frc/drive/MecanumDrive.h" 
#include "ctre/Phoenix.h"

#define GAMEPAD_CHANNEL 0
#define JOYSTICK_CHANNEL 1

#define FRONTLEFT_DEVICENUMBER 1
#define REARLEFT_DEVICENUMBER 2
#define FRONTRIGHT_DEVICENUMBER 3
#define REARRIGHT_DEVICENUMBER 4
 
#pragma region Motors
  static const double JoyInStepDown = 0.35;   

  WPI_TalonSRX frontLeft{FRONTLEFT_DEVICENUMBER}; //left front
	WPI_TalonSRX rearLeft{REARLEFT_DEVICENUMBER}; //left rear
	WPI_TalonSRX frontRight{FRONTRIGHT_DEVICENUMBER}; //right front
	WPI_TalonSRX rearRight{REARRIGHT_DEVICENUMBER}; //right rea 
  
	frc::MecanumDrive m_robotDrive{frontLeft, rearLeft, frontRight, rearRight};
#pragma endregion 
 
#pragma region Controllers 
	frc::Joystick gamePad{GAMEPAD_CHANNEL};
	frc::Joystick m_driveStick{JOYSTICK_CHANNEL}; 

	const static int GreenBtn = 1;
	const static int RedBtn = 2;
	const static int BlueBtn = 3;			
	const static int YellowBtn = 4;	
	const static int LeftBumperBtn = 5;
	const static int RightBumperBtn = 6;
	const static int BackBtn = 7;
	const static int StartBtn = 8; 
	const static int LeftXAxis = 0;
	const static int LeftYAxis = 1;
	const static int LeftTriggerAxis = 2;
	const static int RightTriggerAxis = 3;
	const static int RightXAxis = 4;
	const static int RightYAxis = 5; 
	const static int POVAngle = 0;
#pragma endregion

  void Robot::RobotInit() {
    //Starts will motors at 0% power
    frontLeft.Set(ControlMode::PercentOutput, 0);
    rearLeft.Set(ControlMode::PercentOutput, 0);
    frontRight.Set(ControlMode::PercentOutput, 0);
    rearRight.Set(ControlMode::PercentOutput, 0);
    
    //Useless piece of code
    m_robotDrive.SetExpiration(0.5);
    m_robotDrive.SetSafetyEnabled(false); 
  } 

  void Robot::TeleopPeriodic() {
    //Drives the robot
    m_robotDrive.DriveCartesian(
      gamePad.GetX() * JoyInStepDown,
      -gamePad.GetY() * JoyInStepDown,  
      gamePad.GetRawAxis(RightXAxis) * JoyInStepDown
    );
  }
 
#pragma region GarbageRequiredToCompile
void Robot::TeleopInit() {} 
void Robot::AutonomousInit() {} 
void Robot::AutonomousPeriodic() {} 
void Robot::RobotPeriodic() {} 
void Robot::TestPeriodic() {} 
#pragma endregion

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
