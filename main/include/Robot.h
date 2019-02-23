/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

/*************************************************************************************************/
/**** Includes ****/

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

/*************************************************************************************************/
/**** Gamepad Joystick Definitions for Buttons and Axis' ****/

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
#define AXIS_LEFT_TRIGGER (2)  // 0 to 1
#define AXIS_RIGHT_TRIGGER (3) // 0 to 1

// Joystick Wheels
#define AXIS_LEFT_X (0)
#define AXIS_LEFT_Y (1)

#define AXIS_RIGHT_X (4)
#define AXIS_RIGHT_Y (5)

/*************************************************************************************************/
/**** Class Declaration ****/

class Robot : public frc::TimedRobot
{
  public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;

  private:
};
