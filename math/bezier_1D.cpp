/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include <cmath>
#include "bezier_1D.h"

// positive integer required
double Bezier_1D::factorial(int num) {
    double res=1;
    if (num<0)
        num=0;
    while (num!=0)
    {res=res*num;num--;}
    return res;
}

// positive integer required
double Bezier_1D::nchoosek(int n, int k) {
    double res=0;
    res= factorial(n)/(factorial(n-k)* factorial(k));
    return res;
}

double Bezier_1D::getOut(double s) {
    double res=0;
    int Np=P.size();
    for (int i=0;i<Np;i++)
    {
        res+= nchoosek(Np-1,i)*pow(1-s,Np-1-i)*pow(s,i)*P[i];
    }
    return res;
}
