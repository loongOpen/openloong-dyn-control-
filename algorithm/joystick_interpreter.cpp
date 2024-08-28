/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include "joystick_interpreter.h"

void JoyStickInterpreter::setVxDesLPara(double vxDesLIn, double timeToReach) {
    vxLGen.setPara(vxDesLIn, timeToReach);
}

void JoyStickInterpreter::setVyDesLPara(double vyDesLIn, double timeToReach) {
    vyLGen.setPara(vyDesLIn, timeToReach);
}

void JoyStickInterpreter::setWzDesLPara(double wzDesLIn, double timeToReach) {
    wzLGen.setPara(wzDesLIn, timeToReach);
}

void JoyStickInterpreter::step() {
    vx_L=vxLGen.step();
    vy_L=vyLGen.step();
    wz_L=wzLGen.step();
    thetaZ=thetaZ+wz_L*dt;
    vx_W=cos(thetaZ)*vx_L-sin(thetaZ)*vy_L;
    vy_W=sin(thetaZ)*vx_L+cos(thetaZ)*vy_L;
    px_W+=vx_W*dt;
    py_W+=vy_W*dt;
}

// NOTE: currently only the  x, y directions are controlled. Walking on a slope is not considered here.
void JoyStickInterpreter::dataBusWrite(DataBus &dataBus) {
    dataBus.js_pos_des[0]=px_W;
    dataBus.js_pos_des[1]=py_W;
    dataBus.js_vel_des[0]=vx_W;
    dataBus.js_vel_des[1]=vy_W;
    dataBus.js_eul_des[2]=thetaZ;
    dataBus.js_omega_des[2]=wz_L;
}

void JoyStickInterpreter::reset() {
    vxLGen.resetOut(0);
    vyLGen.resetOut(0);
    wzLGen.resetOut(0);
    vx_L=0;
    vy_L=0;
    wz_L=0;
    thetaZ=0;
}

void JoyStickInterpreter::setIniPos(double posX, double posY, double thetaZ) {
    px_W=posX;
    py_W=posY;
    this->thetaZ=thetaZ;
}





