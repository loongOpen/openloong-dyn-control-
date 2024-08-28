/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://gitee.com/panda_23/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "data_bus.h"
#include "qpOASES.hpp"

const uint16_t  mpc_N = 10;
const uint16_t  ch = 3;
const uint16_t  nx = 12;
const uint16_t  nu = 13;

const uint16_t  ncfr_single = 4;
const uint16_t  ncfr = ncfr_single*2;

const uint16_t  ncstxya = 1;
const uint16_t  ncstxy_single = ncstxya*4;
const uint16_t  ncstxy = ncstxy_single*2;

const uint16_t  ncstza = 2;
const uint16_t  ncstz_single = ncstza*4;
const uint16_t  ncstz = ncstz_single*2;
const uint16_t  nc = ncfr + ncstxy + ncstz;

class MPC{
public:
    MPC(double dtIn);

    void    set_weight(double u_weight, Eigen::MatrixXd L_diag, Eigen::MatrixXd K_diag);
    void    cal();
    void    dataBusRead(DataBus &Data);
    void    dataBusWrite(DataBus &Data);

    void    enable();
    void    disable();
    bool    get_ENA();

private:
    void    copy_Eigen_to_real_t(qpOASES::real_t* target, Eigen::MatrixXd source, int nRows, int nCols);

    bool    EN = false;

    //single rigid body model
    Eigen::Matrix<double,nx,nx>   Ac[mpc_N], A[mpc_N];
    Eigen::Matrix<double,nx,nu>   Bc[mpc_N], B[mpc_N];
    Eigen::Matrix<double,nx,1>    Cc, C;

    Eigen::Matrix<double,nx*mpc_N,nx>         Aqp;
    Eigen::Matrix<double,nx*mpc_N,nx*mpc_N>   Aqp1;
    Eigen::Matrix<double,nx*mpc_N,nu*mpc_N>   Bqp1;
    Eigen::Matrix<double,nx*mpc_N,nu*ch>      Bqp;
    Eigen::Matrix<double,nx*mpc_N,1>          Cqp1;
    Eigen::Matrix<double,nx*mpc_N,1>          Cqp;

    Eigen::Matrix<double,nu*ch,1>           Ufe;
    Eigen::Matrix<double,nu,1>              Ufe_pre;
    Eigen::Matrix<double,nx*mpc_N,1>        Xd;
    Eigen::Matrix<double,nx,1>              X_cur;
    Eigen::Matrix<double,nx,1>              X_cal;
    Eigen::Matrix<double,nx,1>              X_cal_pre;
    Eigen::Matrix<double,nx,1>              dX_cal;

    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic>    L;
    Eigen::Matrix<double,nu*ch, nu*ch>            K, M;
    double alpha;
    Eigen::Matrix<double,nu*ch, nu*ch>          H;
    Eigen::Matrix<double,nu * ch, 1>              c;

    Eigen::Matrix<double,nu*ch,1>               u_low, u_up;
    Eigen::Matrix<double,nc*ch, nu*ch>          As;
    Eigen::Matrix<double,nc*ch,1>               bs;
    double      max[6], min[6];

    double m, g, miu, delta_foot[4];
    Eigen::Matrix<double,3,1>   pCoM;
    Eigen::Matrix<double,6,1>   pf2com, pf2comd, pe;
    Eigen::Matrix<double,6,1>   pf2comi[mpc_N];
    Eigen::Matrix<double,3,3>   Ic;
    Eigen::Matrix<double,3,3>   R_curz[mpc_N];
    Eigen::Matrix<double,3,3>   R_cur;
    Eigen::Matrix<double,3,3>   R_w2f, R_f2w;

    int legStateCur;
    int legStateNext;
    int legState[10];
    double  dt;

    //qpOASES
    qpOASES::QProblem QP;
    qpOASES::real_t qp_H[nu*ch * nu*ch];
    qpOASES::real_t qp_As[nc*ch * nu*ch];
    qpOASES::real_t qp_c[nu*ch];
    qpOASES::real_t qp_lbA[nc*ch];
    qpOASES::real_t qp_ubA[nc*ch];
    qpOASES::real_t qp_lu[nu*ch];
    qpOASES::real_t qp_uu[nu*ch];
    qpOASES::int_t nWSR=100;
    qpOASES::real_t cpu_time=0.1;
    qpOASES::real_t xOpt_iniGuess[nu*ch];

	double			qp_cpuTime;
    int 			qp_Status, qp_nWSR;
};

