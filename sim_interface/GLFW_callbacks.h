/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <string>
#include <memory>

class UIctr{
public:
    GLFWwindow *window;
    // keyboard
    struct ButtonState {
        bool key_w{false};
        bool key_s{false};
        bool key_a{false};
        bool key_d{false};
        bool key_h{false};
        bool key_j{false};
        bool key_space{false};
    } buttonRead;

    // mouse interaction
    bool button_left{false};
    bool button_middle{false};
    bool button_right{false};

    bool runSim{true};
    bool isContinuous{true};
    double lastx{0};
    double lasty{0};
    mjModel* mj_model;
    mjData* mj_data;

    UIctr(mjModel *modelIn, mjData *dataIn);
    void iniGLFW();
    void createWindow(const char * windowTitle, bool saveVideo);
    void updateScene();

    // keyboard callback
    void Keyboard(int key, int scancode, int act, int mods);
    // mouse button callback
    void Mouse_button(int button, int act, int mods);
    // mouse move callback
    void Mouse_move(double xpos, double ypos);
    // scroll callback
    void Scroll(double xoffset, double yoffset);

    ButtonState getButtonState();



    void Close();

    void enableTracking();


private:
    unsigned char* image_rgb_;
    float* image_depth_;

    FILE* file;

    int width{1200};
    int height{800};
    bool save_video{false};

    bool isTrack{false};
    // UI handler
    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context
};


