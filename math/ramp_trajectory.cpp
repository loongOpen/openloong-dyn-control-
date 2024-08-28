/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "ramp_trajectory.h"


void RampTrajectory::setPara(double yDesIn, double timeToReach) {
    yDes=yDesIn;
    if (timeToReach<0.001)
        timeToReach=0.001;
    k=(yDesIn-yOld)/timeToReach;
}

void RampTrajectory::setParaDirt(double yDesIn, double delta_y) {
    yDes=yDesIn;
    k=delta_y/dt;
}

double RampTrajectory::step() {
    y=yOld+k*dt;
    if(fabs(yDes-y)<fabs(1.5*k*dt))
        y=yDes;
    yOld=y;
    return y;
}

bool RampTrajectory::isReachDes() {
    if (fabs(yDes-y)<fabs(k*dt))
        return true;
    else
        return false;
}

void RampTrajectory::resetOut(double yOut) {
    y=yOut;
    yOld=yOut;
}