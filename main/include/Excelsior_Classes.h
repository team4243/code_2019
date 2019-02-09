#ifndef _H_
#define _H_

class Excelsior_OmniDrive
{

public:
    void Configure_Omni_Drive();
    void OmniDrive_SpeedControl(double,double,double);
};

enum PYActions 
{
    Lowest_Hatch_Position,
    Middle_Hatch_Position,
    Highest_Hatch_Position,
    Lowest_Cargo_Position,
    Middle_Cargo_Position,
    Highest_Cargo_Position,
    Halt_Motor
};

class Excelsior_Payload
{

public: 
    void Configure_Payload_Lift();
    void Payload_Action(PYActions);
};

class Hatch_Flower
{

public:
    void Configure_End_Effector();
};

#endif