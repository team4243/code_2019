#ifndef _H_
#define _H_

class Excelsior_Omni_Drive
{
    public:
        void Configure_Omni_Drive();
        void OmniDrive_SpeedControl(double,double,double);
};

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

class Excelsior_Payload_Lift
{
    public: 
        void Configure_Payload_Lift();
        void Payload_Lift_Action(Payload_Lift_Position);
};

class Excelsior_End_Effector
{
    public:
        void Configure_End_Effector();
        void Cargo_Roller_Action(bool);
        void Hatch_Flower_Action(bool);
};

#endif