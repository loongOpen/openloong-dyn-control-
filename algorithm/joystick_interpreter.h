/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#pragma once

#include "ramp_trajectory.h"
#include "data_bus.h"

class JoyStickInterpreter {
public:
    double dt;
    double thetaZ{0};
    JoyStickInterpreter(double dtIn): dt{dtIn}, vxLGen(dtIn), vyLGen(dtIn), wzLGen(dtIn), thetazGen(dtIn){};
    void setVxDesLPara(double vxDesLIn, double timeToReach);
    void setVyDesLPara(double vyDesLIn, double timeToReach);
    void setWzDesLPara(double wzDesLIn, double timeToReach);
    void setIniPos(double posX, double posY, double thetaZ);
    void step();
    double vx_W{0}, vy_W{0}; // generated velocity in x and y direction w.r.t world frame
    double px_W{0}, py_W{0}; // generated position in x and y direction w.r.t world frame
    double vx_L{0}, vy_L{0}, wz_L{0}; // generated linear velocity in x and y direction, angular velocity in z direction, w.r.t body frame
    void dataBusWrite(DataBus &dataBus);
    void reset();
    RampTrajectory vxLGen, vyLGen, wzLGen, thetazGen;
//private:


};


