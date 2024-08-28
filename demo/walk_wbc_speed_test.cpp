/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include <iostream>
#include "useful_math.h"
#include "PVT_ctrl.h"
#include "pino_kin_dyn.h"
#include "data_logger.h"
#include "wbc_priority.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
#include <chrono>

// main function
int main(int argc, const char **argv) {
    // ini classes
    double timestep = 0.001;
    Pin_KinDyn kinDynSolver("../models/AzureLoong.urdf"); // kinematics and dynamics solver
    DataBus RobotState(kinDynSolver.model_nv); // data bus
    WBC_priority WBC_solv(kinDynSolver.model_nv, 18, 22, 0.7, timestep); // WBC solver
    GaitScheduler gaitScheduler(0.4, timestep); // gait scheduler
    PVT_Ctr pvtCtr(timestep, "../common/joint_ctrl_config.json");// PVT joint control
    FootPlacement footPlacement; // foot-placement planner
    JoyStickInterpreter jsInterp(timestep); // desired baselink velocity generator
    DataLogger logger("../record/datalog.log"); // data logger

    // variables ini
    double stand_legLength = 1.01; //-0.95; // desired baselink height
    double foot_height = 0.07; // distance between the foot ankel joint and the bottom
    double xv_des = 0.7;  // desired velocity in x direction

    RobotState.width_hips = 0.229;
    footPlacement.kp_vx = 0.03;
    footPlacement.kp_vy = 0.03;
    footPlacement.kp_wz = 0.03;
    footPlacement.stepHeight = 0.2;
    footPlacement.legLength = stand_legLength;
    //mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco
    int model_nv = kinDynSolver.model_nv;

    // ini position and posture for foot-end and hand
    std::vector<double> motors_pos_des(model_nv - 6, 0);
    std::vector<double> motors_pos_cur(model_nv - 6, 0);
    std::vector<double> motors_vel_des(model_nv - 6, 0);
    std::vector<double> motors_vel_cur(model_nv - 6, 0);
    std::vector<double> motors_tau_des(model_nv - 6, 0);
    std::vector<double> motors_tau_cur(model_nv - 6, 0);
    Eigen::Vector3d fe_l_pos_L_des = {-0.018, 0.113, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des = {-0.018, -0.116, -stand_legLength};
    Eigen::Vector3d fe_l_eul_L_des = {-0.000, -0.008, -0.000};
    Eigen::Vector3d fe_r_eul_L_des = {0.000, -0.008, 0.000};
    Eigen::Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

//    hd_l_pos_L_des<<-0.02, 0.32, -0.159;
//    hd_r_pos_L_des<<-0.02, -0.32, -0.159;
//    hd_l_eul_L_des<<-1.7581, 0.2129, 2.9581;
//    hd_r_eul_L_des<<1.7581, 0.21291, -2.9581;

    Eigen::Vector3d hd_l_pos_L_des = {-0.02, 0.32, -0.159};
    Eigen::Vector3d hd_r_pos_L_des = {-0.02, -0.32, -0.159};
    Eigen::Vector3d hd_l_eul_L_des = {-1.7581, 0.2129, 2.9581};
    Eigen::Vector3d hd_r_eul_L_des = {1.7581, 0.2129, -2.9581};
    Eigen::Matrix3d hd_l_rot_des = eul2Rot(hd_l_eul_L_des(0), hd_l_eul_L_des(1), hd_l_eul_L_des(2));
    Eigen::Matrix3d hd_r_rot_des = eul2Rot(hd_r_eul_L_des(0), hd_r_eul_L_des(1), hd_r_eul_L_des(2));

    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
    auto resHand = kinDynSolver.computeInK_Hand(hd_l_rot_des, hd_l_pos_L_des, hd_r_rot_des, hd_r_pos_L_des);
    Eigen::VectorXd qIniDes = Eigen::VectorXd::Zero(model_nv + 1, 1);
    qIniDes.block(7, 0, model_nv + 1 - 7, 1) = resLeg.jointPosRes + resHand.jointPosRes;
    WBC_solv.setQini(qIniDes, RobotState.q);

    // register variable name for data logger
    logger.addIterm("motors_pos_cur", model_nv - 6);
    logger.addIterm("motors_vel_cur", model_nv - 6);
    logger.addIterm("rpy", 3);
    logger.addIterm("fL", 3);
    logger.addIterm("fR", 3);
    logger.addIterm("basePos", 3);
    logger.addIterm("baseLinVel", 3);
    logger.addIterm("baseAcc", 3);
    logger.addIterm("baseAngVel", 3);
    logger.addIterm("runTime", 1);
    logger.finishItermAdding();

    /// ----------------- sim Loop ---------------

    std::vector<double> motor_pos = {0.4551, 1.1429, 1.8946, 0.8563, 1.2360, 0.0660, -0.1173, -0.4552, -1.1427,
                                     -1.8945, 0.8563, -1.2360, 0.0661, 0.1174, -0.0000, -0.0133, 0.0031, 0.0004, 0.0000,
                                     0.0148, 0.0001, 0.3482, -0.8127, 0.4295, -0.0218, -0.0186, -0.0002, 0.3483, -0.8129,
                                     0.4297, 0.0177};
    std::vector<double> motor_vel = {0.0001, -0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0000, -0.0001, -0.0000, -0.0000,
                                     0.0000, 0.0000, -0.0000, -0.0000, 0.0000, -0.0002, -0.0000, -0.0000, -0.0000, 0.0000,
                                     -0.0001, 0.0000, -0.0006, 0.0002, 0.0003, -0.0009, 0.0002, -0.0002, -0.0002,
                                     -0.0001, 0.0008};
    const int LoopNum=10000;
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    for (int LoopCount = 0; LoopCount < LoopNum; LoopCount++) {
        RobotState.motors_pos_cur=motor_pos;
        RobotState.motors_vel_cur=motor_vel;
        RobotState.rpy[0]=0.0069;
        RobotState.rpy[1]=-0.0403;
        RobotState.rpy[2]=0;
        RobotState.fL[0]=0;
        RobotState.fL[1]=0;
        RobotState.fL[2]=0;
        RobotState.fR[0]=0;
        RobotState.fR[1]=0;
        RobotState.fR[2]=0;
        RobotState.basePos[0]=0.0039;
        RobotState.basePos[1]=-0.0054;
        RobotState.basePos[2]=1.0752;
        RobotState.baseLinVel[0]=0;
        RobotState.baseLinVel[1]=0;
        RobotState.baseLinVel[2]=0;
        RobotState.baseAcc[0]=0;
        RobotState.baseAcc[1]=0;
        RobotState.baseAcc[2]=0;
        RobotState.baseAngVel[0]=0;
        RobotState.baseAngVel[1]=0;
        RobotState.baseAngVel[2]=0;
        RobotState.updateQ();

        // update kinematics and dynamics info
        kinDynSolver.dataBusRead(RobotState);
        kinDynSolver.computeJ_dJ();
        kinDynSolver.computeDyn();
        kinDynSolver.dataBusWrite(RobotState);

        // Enter here functions to send actuator commands, like:
        // arm-l: 0-6, arm-r: 7-13, head: 14,15, waist: 16-18, leg-l: 19-24, leg-r: 25-30

        if (LoopCount>1) {
            jsInterp.setWzDesLPara(0, 1);
            jsInterp.setVxDesLPara(xv_des, 2.0); // jsInterp.setVxDesLPara(0.9,1);
        } else
            jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
        jsInterp.setWzDesLPara(0, 1);
        jsInterp.setVxDesLPara(0, 2.0); // jsInterp.setVxDesLPara(0.9,1);
        jsInterp.step();
        RobotState.js_pos_des(2) = stand_legLength + foot_height; // pos z is not assigned in jyInterp
        jsInterp.dataBusWrite(RobotState); // only pos x, pos y, theta z, vel x, vel y , omega z are rewrote.
//
        // gait scheduler
        gaitScheduler.dataBusRead(RobotState);
        if (LoopCount<1)
            gaitScheduler.step();
        gaitScheduler.dataBusWrite(RobotState);
//
        footPlacement.dataBusRead(RobotState);
        footPlacement.getSwingPos();
        footPlacement.dataBusWrite(RobotState);

        // ------------- WBC ------------
        // WBC input
        RobotState.Fr_ff = Eigen::VectorXd::Zero(12);
        RobotState.des_ddq = Eigen::VectorXd::Zero(model_nv);
        RobotState.des_dq = Eigen::VectorXd::Zero(model_nv);
        RobotState.des_delta_q = Eigen::VectorXd::Zero(model_nv);
        RobotState.base_rpy_des << 0, 0, jsInterp.thetaZ;
        RobotState.base_pos_des(2) = stand_legLength + foot_height;

        RobotState.Fr_ff << 0, 0, 370, 0, 0, 0,
                0, 0, 370, 0, 0, 0;

        // adjust des_delata_q, des_dq and des_ddq to achieve forward walking
        RobotState.des_delta_q.block<2, 1>(0, 0) << jsInterp.vx_W * timestep, jsInterp.vy_W * timestep;
        RobotState.des_delta_q(5) = jsInterp.wz_L * timestep;
        RobotState.des_dq.block<2, 1>(0, 0) << jsInterp.vx_W, jsInterp.vy_W;
        RobotState.des_dq(5) = jsInterp.wz_L;

        double k = 5;
        RobotState.des_ddq.block<2, 1>(0, 0) << k * (jsInterp.vx_W - RobotState.dq(0)), k * (jsInterp.vy_W -
                                                                                             RobotState.dq(1));
        RobotState.des_ddq(5) = k * (jsInterp.wz_L - RobotState.dq(5));


        // WBC Calculation
        WBC_solv.dataBusRead(RobotState);
        WBC_solv.computeDdq(kinDynSolver);
        WBC_solv.computeTau();
        WBC_solv.dataBusWrite(RobotState);
//
//        // get the final joint command
//
//        Eigen::VectorXd pos_des = kinDynSolver.integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
//        RobotState.motors_pos_des = eigen2std(pos_des.block(7, 0, model_nv - 6, 1));
//        RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final);
//        RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);

        pvtCtr.dataBusRead(RobotState);
        pvtCtr.setJointPD(100, 10, "J_ankle_l_pitch");
        pvtCtr.setJointPD(100, 10, "J_ankle_l_roll");
        pvtCtr.setJointPD(100, 10, "J_ankle_r_pitch");
        pvtCtr.setJointPD(100, 10, "J_ankle_r_roll");
        pvtCtr.setJointPD(1000, 100, "J_knee_l_pitch");
        pvtCtr.setJointPD(1000, 100, "J_knee_r_pitch");
        pvtCtr.calMotorsPVT();
        pvtCtr.dataBusWrite(RobotState);

        end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        start = end;


        logger.startNewLine();
        logger.recItermData("motors_pos_cur", RobotState.motors_pos_cur);
        logger.recItermData("motors_vel_cur", RobotState.motors_vel_cur);
        logger.recItermData("rpy", RobotState.rpy);
        logger.recItermData("fL", RobotState.fL);
        logger.recItermData("fR", RobotState.fR);
        logger.recItermData("basePos", RobotState.basePos);
        logger.recItermData("baseLinVel", RobotState.baseLinVel);
        logger.recItermData("baseAcc", RobotState.baseAcc);
        logger.recItermData("baseAngVel", RobotState.baseAngVel);
        logger.recItermData("runTime", duration.count());
        logger.finishLine();

//        printf("rpyVal=[%.5f, %.5f, %.5f]\n", RobotState.rpy[0], RobotState.rpy[1], RobotState.rpy[2]);
//        printf("gps=[%.5f, %.5f, %.5f]\n", RobotState.basePos[0], RobotState.basePos[1], RobotState.basePos[2]);
//        printf("vel=[%.5f, %.5f, %.5f]\n", RobotState.baseLinVel[0], RobotState.baseLinVel[1],
//               RobotState.baseLinVel[2]);
        printf("Execution time: %.6f sec. \n", duration.count() );
    }
   //std::cout<< RobotState.wbc_tauJointRes.transpose()<<std::endl;



    std::chrono::duration<double> duration = end - start;
    std::cout<<"loop time recorded to the last column of record/datalog.log"<<std::endl;

//    std::cout << "Ava Loop time: " << duration.count()/LoopNum*1000. << " ms\n";

    return 0;
}
