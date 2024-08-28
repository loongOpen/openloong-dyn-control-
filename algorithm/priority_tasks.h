/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include <Eigen/Dense>
#include "useful_math.h"
#include <utility>
#include <vector>
#include <string>
#include <iostream>
struct Task{
    std::string taskName;
    int id;
    int parentId, childId;
    Eigen::VectorXd dxDes,ddxDes;
    Eigen::VectorXd delta_q, dq, ddq;
    Eigen::MatrixXd J, dJ, Jpre;
    Eigen::MatrixXd N;
    Eigen::MatrixXd kp, kd;
    Eigen::DiagonalMatrix<double,-1> W; // weighted matrix for pseudo inverse
    Eigen::VectorXd errX, derrX;
    Task(std::string name){taskName=name;};
};
class PriorityTasks {
public:
    std::vector<Task> taskLib;
    std::vector<std::string> nameList;
    std::vector<int> idList, parentIdList, childIdList;
    Eigen::VectorXd out_delta_q, out_dq, out_ddq;
    int startId;
    void addTask(const char* name);
    int getId(const std::string& name);
    int getId(const char* name);
    void buildPriority(const std::vector<std::string> &taskOrder);
    void computeAll(const Eigen::VectorXd &des_delta_q,const Eigen::VectorXd &des_dq, const Eigen::VectorXd &des_ddq, const Eigen::MatrixXd &dyn_M
    ,const Eigen::MatrixXd &dyn_M_inv, const Eigen::VectorXd &dq);
    void printTaskInfo();
};



