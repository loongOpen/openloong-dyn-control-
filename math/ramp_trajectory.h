/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include <cmath>

class RampTrajectory {
public:
    double dt;
    double y;
    double yDes;
    RampTrajectory(double dtIn):dt{dtIn},yOld{0},k{0},yDes{0}{};
    void setPara(double yDesIn, double timeToReach); // timeToReach is the time to reach the yDes from the current yOld
    void setParaDirt(double yDesIn, double delta_y);
    double step();
    bool isReachDes();
    void resetOut(double yOut);
private:
    double yOld;
    double k;
};


