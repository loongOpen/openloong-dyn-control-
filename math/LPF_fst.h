/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#pragma once


class LPF_Fst {
private:
    double alpha{0};
    double dataOld{0};
    bool isIni{false};
public:
    LPF_Fst();
    LPF_Fst(double fc,double Ts);
    void setPara(double fc, double Ts);
    double ftOut(double dataIn);

};



