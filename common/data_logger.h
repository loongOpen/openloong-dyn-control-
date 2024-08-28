/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

// Data log class based on Quill (https://github.com/odygrd/quill). It will generate a DataLogger.log file and a
// matlabReadDataScript.txt. In this txt, it gives the info about correlated column indexes of each recorded variable.
// It is recommended to use Matlab to read the log file, and use the content of the txt file to extract the recorded variable.
//
#pragma once

#include "string"
#include "vector"
#include "quill/Quill.h"
#include "fmt/format.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include "Eigen/Dense"

class DataLogger {
public:
    DataLogger(std::string fileNameIn);
    void addIterm(const std::string &name, const int & len);
    void finishItermAdding();
    void startNewLine();
    void recItermData(const std::string &name, double *dataIn);
    void recItermData(const std::string &name, double dataIn);
    void recItermData(const std::string &name, const Eigen::VectorXd &dataIn);
    void recItermData(const std::string &name, const std::vector<double> &dataIn);
    void finishLine();
private:
    int colCout{0};
    std::string filePath, fileName;
    std::string fileFolder;
    std::string tmpStr;
    std::vector<double> recValue;
    std::vector<std::string> recItemName;
    std::vector<int> recItemLen;
    std::vector<int> recItemStartCol;
    std::vector<int> recItemEndCol;
    std::vector<bool> isItemDataIn;
    quill::Logger *dl;
    quill::Handler *file_handler;
};


