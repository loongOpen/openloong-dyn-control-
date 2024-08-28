<img src="./assets/logo.png" style="zoom:50%;" />

# OpenLoong Dynamics Control

## 基于 MPC 与 WBC 的仿人机器人运动控制框架

欢迎访问 🐉 OpenLoong 开源项目代码仓库！

OpenLoong开源项目是由人形机器人（上海）有限公司、上海人形机器人制造业创新中心与开放原子开源基金会（OpenAtom Foundation）共同运营的开源项目。本仓库提供了一套基于 MPC 与 WBC 的仿人机器人控制框架，可部署在 Mujoco 仿真平台上。基于上海人形机器人创新中心“青龙”机器人模型，提供[行走](https://atomgit.com/openloong/openloong-dyn-control/blob/master/demo/walk_wbc.cpp)、[跳跃](https://atomgit.com/openloong/openloong-dyn-control/blob/master/demo/jump_mpc.cpp)、[盲踩障碍物](https://atomgit.com/openloong/openloong-dyn-control/blob/master/demo/walk_mpc_wbc.cpp)三种运动示例。

## 项目特点

- **易部署** 提供全面的代码运行环境部署解决方案，以便用户能够轻松配置其所需的工作环境，本代码仓库包含了主要依赖，无需进行众多第三方库的安装，简化整个部署过程。

- **可扩展** 控制框架结构采用分层模块化设计，旨在提高系统的可维护性和可扩展性，系统各功能模块在逻辑和功能上具有明确的界限，为二次开发提供了更加友好的环境，使开发人员能够更轻松地对系统进行功能定制和扩展。

- **易理解** 代码结构简洁，遵循针对功能进行模块封装的代码设计原则，应用总线进行模块间数据交互，减少封装冗余，有助于降低代码复杂度；算法实现采用“读取-计算-写入”的简单逻辑，提高代码的可理解性。

## 环境安装

**环境建议**

- 操作系统：Ubuntu 22.04.4 LTS
- 编译器：g++ 11.4.0

**依赖安装**

本仓库为基于 mujoco 针对“青龙”人形机器人进行制仿真测试， mujoco 的仿真引擎、pinocchio 动力学库、eigen、quill 记录工具、GLFW 图形库、jsoncpp 解析库等也包含到了仓库之中，但仿真界面需系统支持 openGL，需安装

```Bash
# Update & Install Dependencies
sudo apt-get update
sudo apt install git cmake gcc-11 g++-11
sudo apt install libglu1-mesa-dev freeglut3-dev
```

## 使用指南

**代码获取与编译**

```Bash
# Clone
git clone https://atomgit.com/openloong/openloong-dyn-control.git

# Build
cd openloong-dyn-control
mkdir build
cd build
cmake ..
make

# mujoco simulation
./walk_mpc_wbc #or ./walk_wbc or ./jump_mpc
```

**仿真效果**

<img src="assets/demo.png" alt="demo" style="zoom:50%;" />

## **代码说明**

参考本代码API接口[说明文档](https://www.openloong.org.cn/pages/api/html/index.html)及[Wiki](https://www.openloong.org.cn/pages/wiki/html/index.html)。

**主要前缀后缀指代说明**

| 前缀后缀         | 指代                       |
| ---------------- | -------------------------- |
| *_L, _W*         | 本体坐标系下、世界坐标系下 |
| *fe_*            | 足末端                     |
| *_L, _l, _R, _r* | 左侧、右侧                 |
| *swing,* *sw*    | 摆动腿                     |
| *stance,* *st*   | 支撑腿                     |
| *eul, rpy*       | 姿态角                     |
| *omega*          | 角速度                     |
| *pos*            | 位置                       |
| *vel*            | 线速度                     |
| *tor**, tau*     | 力矩                       |
| *base*           | *BaseLink*                 |
| *_des*           | 期望值                     |
| *_cur*           | 当前实际值                 |
| *_rot*           | 坐标变换矩阵               |

## 开发指南

**关键控制参数说明**

- MPC权重

```C++
//MPC.h
void    set_weight(double u_weight, Eigen::MatrixXd L_diag, Eigen::MatrixXd K_diag);
//*u_weight* ：系统输入最小权重
//*L_diag* ：系统状态与期望误差权重，顺序为eul, pos, omega, vel
//*K_diag* ：系统输入权重，顺序为fl, tl, fr, tr
```

- WBC优先级

```C++
//WBC_QP.cpp
std::vector<std::string taskOrder;
taskOrder.emplace_back("RedundantJoints");
taskOrder.emplace_back("static_Contact");
taskOrder.emplace_back("Roll_Pitch_Yaw_Pz");
taskOrder.emplace_back("PxPy");
taskOrder.emplace_back("SwingLeg");
taskOrder.emplace_back("HandTrack");
//添加优先级及调整优先级顺序
```

- WBC权重

```C++
//PriorityTasks.h
Eigen::MatrixXd Kp;                //WBC某一优先级中，位置误差权重
Eigen::MatrixXd Kd;                //WBC某一优先级中，速度误差权重
//WBC_QP.h
Eigen::MatrixXd Q1;                //外部接触力与期望误差权重,顺序为fl, tl, fr, tr
Eigen::MatrixXd Q2;                //关节加速度与期望误差权重
```

- 腾空腿轨迹

```C++
//FootPlacement.h
double kp_vx;                                 //腾空腿x方向位置落脚点调节参数
double kp_vy;                                 //腾空腿y方向位置落脚点调节参数
double kp_wz;                                 //腾空腿z方向姿态落脚点调节参数
double stepHeight;                            //抬腿高度
//FootPlacement.cpp
double    FootPlacement::Trajectory(double phase, double des1, double des2);        //腾空腿z方向轨迹
//phase：达到最高点的腾空相位
//des1：轨迹最高点位置
//des2：轨迹最终位置
```

- 步态控制

```C++
//GaitScheduler.h
double tSwing;                                         //单步时长
double FzThrehold;                                     //触地足底力阈值
//GaitScheduler.cpp
DataBus::LegState legState=DataBus::RS;                //初始腾空腿
```

- 关节参数

```json
//JointCtrConfig.json
   "Joint-ankle-l-pitch" : {
      "PVT_LPF_Fc" : 20,
      "kd" : 5.0,
      "kp" : 50.0,
      "maxPos" : 0.61087,
      "maxSpeed" : 48.8,
      "maxTorque" : 58.5,
      "minPos" : -0.43644
   }
```

**模型替换说明**

1. 模型文件

   a. **xml类型模型文件导出**

准备机器人的URDF文件和mesh文件(STL格式)，添加用于mujoco编译的标签：

```XML
<mujoco>
<compiler
        meshdir="meshes/"
        balanceinertia="true"
        discardvisual="false" />
</mujoco>
```

切换到mujoco-3.x.x/bin目录，运行指令

```Bash
./simulate
```

将urdf文件拖拽进simulate预览界面中，保存xml，注意，mesh文件目录需要对应。

可参考Mojoco官方[文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html)，设置*compiler*,*option*等标签，定义*asset*标签将STL文件导入，定义(whold)body, *actuator, sensor*等：

| 父标签      | 子标签                                                       |
| ----------- | ------------------------------------------------------------ |
| *worldbody* | 定义灯光，相机，地板，机器人(*inertial*、*joint*、*freejoint*、*geom*、*site*、*camera*、*light*)等 |
| *actuator*  | 定义执行器motor、position、velocity等                        |
| *sensor*    | 定义需要的传感器，可添加传感器噪声                           |

其中，*sensor*安装在上述定义的*site*处，*site*并不参与碰撞以及物体质量和惯性的计算，无需担心附加的site会对仿真产生不利的影响。

 b. **关于模型的修改及替换**

以此项目的青龙机器人“AzureDragon为“例：*base_link*下并联了头*Link_head_*、腰*Link_waist_*、左臂*Link**arm_l*、右臂*Link**arm_r_等四个串联分支。其中左臂、右臂分支依次串联了7个自由度，头部分支串联了2个自由度。腰分支串联了俯仰*Link_waist_pitch*、滚转*Link_waist_roll*、偏航*Link_waist_yaw*等3个自由度后，并联了左腿、右腿两个分支，每条腿上依次串联了三个髋关节*Link**hip_、一个膝关节*Link_knee_*、两个踝关节*Link_ankle_*等6个自由度。至此，完成了31个自由度的配置。

可参考该串联系统，修改其中某些自由度的配置：

```XML
<worldbody>
    <body name="base_link" pos="x x x">
        <freejoint name="float_base" />
        <body name="body1" pos="x x x">
            <inertial pos="x x x" quat="x x x" mass="x" diaginertia="x x x"/>
            <joint name="joint1" pos="0 0 0" axis="1 0 0" limited="true" range="x x"/>
            <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="x x x 1" mesh="body1"/>
            <geom type="mesh" rgba="x x x 1" mesh="body1"/>
            <body name="body2" pos="x x x">
                ...
                <joint name="joint2" .../>
                ...
            </body>
        </body>
        <body name="body3" pos="x x x">
            ...
            <joint name="joint3" .../>
            ...
        </body>
    </body>
</worldbody>
```

其中，*base_link*下并联了两个分支，其中一条分支由*body1*和*body2*串联而成，另外一条分支由*body3*构成。如果机器人为浮动基座，可以在上述名为*base_link*的*body*下添加自由关节*freejoint* 。如果机器人是固定基座，去掉*freejoint* 。可以根据需要，在模型配置阶段，选择将*freejoint* 屏蔽掉。

本项目为31个关节中的每一个都设置了motor类型的执行器。

```XML
<actuator>
    <motor name="motor1"  joint="joint1" gear="x" ctrllimited="true" ctrlrange="x x"/>
    ...
</actuator>
```

用户可以根据机器人的自由度的情况，在主动关节处定义相应执行器。

本项目配置了四元数*framequat*、速度计*velocimeter*、角速度计*gyro*、加速度计*accelerometer*等传感器，安装在*body*标签中已定义好的*site*处，可根据需要，添加 *touch*、*force*、*torque*、*jointpos*、*jointvel*、*actuatorfrc*等传感器。

```XML
<sensor>
    <framequat name="xx" objtype="site" objname="imu" />
    <velocimeter name="xx" site="imu" />
    <gyro name="xx" site="imu" />
    <accelerometer name="xx" site="imu" />
</sensor>
```

除自由度配置、执行器配置、传感器配置，其他更具体的参数修改可参考Mojoco官方[文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html)。

2. **控制代码与Mujoco接口**

可参考[文档](https://mujoco.readthedocs.io/en/stable/APIreference/index.html)中关于`mjModel`、`mjData`、`mjOption`等结构类型的定义，使用`mj_loadXML`、`mj_makeData`函数得到`mjModel`、`mjData`。

```C++
mjModel* mj_model = mj_loadXML("../Models/xxx.xml", 0, error, 1000);
mjData* mj_data = mj_makeData(mj_model);
```

其中`mj_model->nv`为广义坐标速度维度，为浮动基的线速度、角速度以及31个旋转类型关节的速度，本项目程序框架中与自由度相关变量均与`mj_model->nv-6`相对应，动力学库会根据URDF自动获取机器人自由度维数，与自由度相关变量的维数均据此定义，无需用户自己更改。

由于本项目的*body*、*joint*、*motor*等组件地址的访问方式为，依靠查询名称字符串并锁定地址，当某一组件修改时不会影响其他*body*、*joint*的数据读取及指令下发，相比直接索引编号，为修改模型提供了便利。修改模型中的某一自由度的控制参数时，只需要修改*MJ_Interface.h*的`JointName`、*Pin_KinDyn.h*的`motorName`、*PVT_Ctr.h*的`motorName`、*JointCtrConfig.json*等文件某一自由度名称对应的变量即可，例如修改`J_waist_pitch`的刚度，需修改*JointCtrConfig.json*中`J_waist_pitch`与对应的PD参数，`J_waist_pitch`名称与xml文件中的*joint name*、*motor name*相对应。

传感器数据地址的访问方式亦是通过查询名称字符串锁定地址，添加或者删除传感器只需修改*MJ_Interface.h*中对应的传感器名称即可。

## 参考文献

[1] D. Kim, J. D. Carlo, B. Katz, G. Bledt, S. Kim, Highly dynamic quadruped locomotion via whole-body impulse control and model predictive control. arXiv:1909.06586 (2019).

[2] Kim D, Jorgensen S J, Lee J, et al. Dynamic locomotion for passive-ankle biped robots and humanoids using whole-body locomotion control. arXiv:1901.08100 (2020).

[3] Di Carlo J, Wensing P M, Katz B, et al. Dynamic locomotion in the mit  cheetah 3 through convex model-predictive control[C]//2018 IEEE/RSJ  international conference on intelligent robots and systems (IROS). IEEE, 2018: 1-9.

[4] 卞泽坤, 王兴兴. 四足机器人控制算法: 建模、控制与实践[M]. 机械工业出版社, 2023

## 引用格式

若应用本开源项目中的代码，请以以下格式进行引用：

```JavaScript
@software{Robot2024OpenLoong,
  author = {Humanoid Robot (Shanghai) Co., Ltd},
  title = {{OpenLoong-DynamicsControl: Motion control framework of humanoid robot based on MPC and WBC}},
  url = {https://atomgit.com/openloong/openloong-dyn-control.git},
  year = {2024}
}
```

## 联系方式

欢迎各位开发者参与本代码库的优化与提高！

[💬 新建讨论](https://atomgit.com/openloong/openloong-dyn-control/discussions/new/choose) | [📝 反馈问题](https://atomgit.com/openloong/openloong-dyn-control/issues/create) | [📨 变更请求](https://atomgit.com/openloong/openloong-dyn-control/changes)

您可以对现有内容进行意见评价、问题反馈、贡献您的原创内容等，对本代码的任何问题及意见，请联系<web@openloong.org.cn>

## 更新日志

2024.06.29

1. 增加walk_wbc_joystick与 walk_mpc_wbc_joystick两个demo，可利用键盘控制机器人运动，并能实现转弯。

2024.08.12

1. 修改由mujoco中提取传感器数据的ID错误，感谢驯龙软件对该问题的提出；
2. 修改MPC中c矩阵定义的维数错误，感谢@geekloong、@yichuanku对该问题的提出；
3. 修改WBC优先级计算中，第一个优先级的计算错误，感谢@1190201119对该问题的提出；
4. 修改MPC的代价函数。
