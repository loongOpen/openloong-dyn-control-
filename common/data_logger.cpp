/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "data_logger.h"

DataLogger::DataLogger(std::string fileNameIn) {
    filePath=fileNameIn;
    size_t lastSlashPos = filePath.find_last_of('/');
    fileFolder=filePath.substr(0, lastSlashPos);
    fileName=filePath.substr(lastSlashPos + 1);
    file_handler = quill::file_handler(filePath, "w");
    file_handler->set_pattern(QUILL_STRING("%(message)")); // timestamp's timezone
    quill::set_default_logger_handler(file_handler);
    dl = quill::create_logger("logger", file_handler);
    quill::start();
}

void DataLogger::addIterm(const std::string &name, const int &len) {
    auto it = std::find(recItemName.begin(), recItemName.end(), name);
    if (it != recItemName.end()) {
        std::cout << name<< " has already been used!!!!!"<< std::endl;
        throw std::runtime_error("Failed to add rec item.");
    }
    recItemName.push_back(name);
    recItemLen.push_back(len);
    recItemStartCol.push_back(colCout);
    recItemEndCol.push_back(colCout+len-1);
    colCout+=len;
}

void DataLogger::finishItermAdding() {
    std::string insFileName=fileFolder+"/matlabReadDataScript.txt";
    std::ofstream outFile(insFileName);

    if (outFile.is_open()) {
        outFile << "clear variables; close all" << std::endl;
        outFile << "dataRec=load('" << fileName << "');" << std::endl;
        for (size_t i = 0; i < recItemName.size(); ++i) {
            outFile << recItemName[i] << "=dataRec(:,"<<recItemStartCol[i]+1<<":"<<recItemEndCol[i]+1<<");"<<std::endl;
        }
        outFile.close();
    } else {
        std::cerr << "unable to open matlabReadDataScript.txt\n";
    }
    recValue.resize(colCout,0.0);
    isItemDataIn.resize(recItemName.size(), false);
}

void DataLogger::startNewLine() {
    recValue.resize(colCout,0.0);
    isItemDataIn.resize(recItemName.size(), false);
}

void DataLogger::recItermData(const std::string &name, double *dataIn) {
    auto it = std::find(recItemName.begin(), recItemName.end(), name);
    if (it == recItemName.end()) {
        std::cout << name<< " has not been added!!!!!"<< std::endl;
        throw std::runtime_error("Failed to rec item.");
    }
    int curIdx=std::distance(recItemName.begin(), it);
    for (int i=0;i<recItemLen[curIdx];i++)
    {
        recValue[i+recItemStartCol[curIdx]]=dataIn[i];
    }
    isItemDataIn[curIdx]=true;
}

void DataLogger::recItermData(const std::string &name, double dataIn) {
    auto it = std::find(recItemName.begin(), recItemName.end(), name);
    if (it == recItemName.end()) {
        std::cout << name<< " has not been added!!!!!"<< std::endl;
        throw std::runtime_error("Failed to rec item.");
    }
    int curIdx=std::distance(recItemName.begin(), it);
    for (int i=0;i<recItemLen[curIdx];i++)
    {
        recValue[i+recItemStartCol[curIdx]]=dataIn;
    }
    isItemDataIn[curIdx]=true;
}

void DataLogger::recItermData(const std::string &name, const Eigen::VectorXd &dataIn) {
    auto it = std::find(recItemName.begin(), recItemName.end(), name);
    if (it == recItemName.end()) {
        std::cout << name<< " has not been added!!!!!"<< std::endl;
        throw std::runtime_error("Failed to rec item.");
    }
    int curIdx=std::distance(recItemName.begin(), it);
    for (int i=0;i<recItemLen[curIdx];i++)
    {
        recValue[i+recItemStartCol[curIdx]]=dataIn(i);
    }
    isItemDataIn[curIdx]=true;
}

void DataLogger::recItermData(const std::string &name, const std::vector<double> &dataIn) {
    auto it = std::find(recItemName.begin(), recItemName.end(), name);
    if (it == recItemName.end()) {
        std::cout << name<< " has not been added!!!!!"<< std::endl;
        throw std::runtime_error("Failed to rec item.");
    }
    int curIdx=std::distance(recItemName.begin(), it);
    for (int i=0;i<recItemLen[curIdx];i++)
    {
        recValue[i+recItemStartCol[curIdx]]=dataIn[i];
    }
    isItemDataIn[curIdx]=true;
}

void DataLogger::finishLine() {
    auto it = std::find(isItemDataIn.begin(), isItemDataIn.end(), false);
    if (it != isItemDataIn.end()) {
        std::cout << recItemName[std::distance(isItemDataIn.begin(),it)]<< " has not been recorded values!!!!!"<< std::endl;
        throw std::runtime_error("Failed to rec item.");
    }
    tmpStr = fmt::format("{:.6e}", fmt::join(recValue, ","));
    LOG_INFO(dl, "{}", tmpStr);
}


























