/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

// first order low-pass filter
//

#include "LPF_fst.h"
LPF_Fst::LPF_Fst() {
    alpha=0;
}

LPF_Fst::LPF_Fst(double fc, double Ts) {
    alpha=Ts/(Ts+1.0/(2*3.1415*fc));
}

double LPF_Fst::ftOut(double dataIn) {
    double res{0};
    if (isIni)
        res=(1-alpha)*dataOld+alpha*dataIn;
    else{
        res=dataIn;
        isIni=true;
    }
    dataOld=res;
    return res;
}

void LPF_Fst::setPara(double fc, double Ts) {
    alpha=Ts/(Ts+1.0/(2*3.1415*fc));
}