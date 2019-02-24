#pragma once

#ifndef _H_
#define _H_

/*************************************************************************************************/
/**** Omni Drive Class Declaration ****/

class Excelsior_Omni_Drive
{
public:
  void Configure_Omni_Drive();
  void Omni_Drive_Action(double x_value, double y_value, double rotation, bool manual);
  void Print_Omni_Encoders();
};

/*************************************************************************************************/
/**** Payload Lift Enum Declaration ****/

// Enumerables inherit integers by default..
// .. starting at 0 and incrementing by 1 in the order shown here
enum Payload_Lift_Position
{
  Ground_Position,
  Travel_Position,

  Lowest_Hatch_Position,
  Lowest_Cargo_Position,

  Middle_Hatch_Position,
  Middle_Cargo_Position,

  Highest_Hatch_Position,
  Highest_Cargo_Position,

  Maximum_Height_Position
};

/*************************************************************************************************/
/**** Payload Lift Class Declaration ****/

class Excelsior_Payload_Lift
{
public:
  void Configure_Payload_Lift();
  void Payload_Lift_Action(Payload_Lift_Position);
  void Payload_Lift_Manual(double speed);
  void Payload_Lift_Step(bool stepUp);
  void Zero_Encoder_Position();
  void Print_Lift_Encoder();
};

/*************************************************************************************************/
/**** End Effector Class Declaration ****/

class Excelsior_End_Effector
{
public:
  void Configure_End_Effector();
  void Cargo_Roller_Action(bool, double);
  void Cargo_Roller_Manual(double);
  void Hatch_Flower_Action(bool);
  void Camera_Tilt_Action(bool);
  void Print_Roller_Encoders();
};

#endif