clear variables;
close all;
robot=importrobot('AzureLoong_simplified.urdf');
config=homeConfiguration(robot);
m=0;
JointIniPos=[
    0.369508
     1.20714
     1.74597
    0.625543
    -1.74116+pi
  -0.0525019
 -0.00409843
   -0.368536
    -1.20662
    -1.74583
    0.623481
     1.74122-pi
  -0.0514735
   0.0051983
           0
           0
 0.0
 3.66648e-09
-3.60333e-10
0.02505/180*pi   %leg l, motor 1
9.952952/180*pi
    11.339007/180*pi
   -25.847147/180*pi
    -25.6115/180*pi
 -1.9690923/180*pi
 0.000296639
 1.99704e-06
    0.305913
   -0.670418
    0.371265
-0.000296642]*1;
for i=1:1:31
    config(i).JointPosition=JointIniPos(i);
end
for i=1:1:robot.NumBodies
    m=m+robot.Bodies{i}.Mass;
    fprintf("%d %s mass= %.3f\n",i,robot.Bodies{i}.Name,robot.Bodies{i}.Mass);
end

% print joint name
for i=1:1:robot.NumBodies
    m=m+robot.Bodies{i}.Mass;
    fprintf(' \"%s\",\n ',robot.Bodies{i}.Joint.Name);
end

endTfL_hd=getTransform(robot,config,'Link_arm_l_07');
endTfR_hd=getTransform(robot,config,'Link_arm_r_07');
eulR_hd=rotm2eul(endTfR_hd(1:3,1:3));
eulL_hd=rotm2eul(endTfL_hd(1:3,1:3));

endTfL_fe=getTransform(robot,config,'Link_ankle_l_roll');
endTfR_fe=getTransform(robot,config,'Link_ankle_r_roll');
eulR_fe=rotm2eul(endTfR_fe(1:3,1:3));
eulL_fe=rotm2eul(endTfL_fe(1:3,1:3));

%endTfL_fe=getTransform(robot,config,'Link_waist_pitch','Link_waist_roll');

tformDesL_hd=eye(4);
tformDesL_hd(1:3,4)=[-0.019, 0.34, -0.159];
tformDesL_hd(1:3,1:3)=eul2rotm([2.95811400338596	0.212912952813363	-1.75810051403098]);
tformDesR_hd=eye(4);
tformDesR_hd(1:3,4)=[-0.019, -0.34, -0.159];
tformDesR_hd(1:3,1:3)=eul2rotm([-2.95811400338596	0.212912952813363	1.75810051403098]);

% use IK
gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', {'pose','pose','pose','pose','joint'});

poseConst_fe_L = constraintPoseTarget('Link_ankle_l_roll');
poseConst_fe_L.TargetTransform=eye(4);
poseConst_fe_L.TargetTransform(1:3,1:3)=eul2rotm([0,0,0]);
poseConst_fe_L.TargetTransform(1:3,4)=[-0.02, 0.0937, -1.02];

poseConst_fe_R = constraintPoseTarget('Link_ankle_r_roll');
poseConst_fe_R.TargetTransform=eye(4);
poseConst_fe_R.TargetTransform(1:3,1:3)=eul2rotm([0,0,0]);
poseConst_fe_R.TargetTransform(1:3,4)=[-0.02, -0.0952, -1.02];

poseConst_hd_L = constraintPoseTarget('Link_arm_l_07');
poseConst_hd_L.TargetTransform=eye(4);
poseConst_hd_L.TargetTransform(1:3,1:3)=eul2rotm([2.95811400338596	0.212912952813363	-1.75810051403098]);
poseConst_hd_L.TargetTransform(1:3,4)=[-0.02, 0.32, -0.159];

poseConst_hd_R = constraintPoseTarget('Link_arm_r_07');
poseConst_hd_R.TargetTransform=eye(4);
poseConst_hd_R.TargetTransform(1:3,1:3)=eul2rotm([-2.95811400338596	0.212912952813363	1.75810051403098]);
poseConst_hd_R.TargetTransform(1:3,4)=[-0.02, -0.32, -0.159];

% disable head and waist joints
limitJoint = constraintJointBounds(robot);
limitJoint.Bounds(15,1)=0;limitJoint.Bounds(15,2)=0;
limitJoint.Bounds(16,1)=0;limitJoint.Bounds(16,2)=0;
limitJoint.Bounds(17,1)=0;limitJoint.Bounds(17,2)=0;
limitJoint.Bounds(18,1)=0;limitJoint.Bounds(18,2)=0;
limitJoint.Bounds(19,1)=0;limitJoint.Bounds(19,2)=0;

initialguess = homeConfiguration(robot);
initialguess(2).JointPosition=pi/9;
initialguess(3).JointPosition=pi/9;
initialguess(4).JointPosition=pi/9;
initialguess(8).JointPosition=-pi/6;
initialguess(9).JointPosition=-pi/9;
initialguess(10).JointPosition=-pi/9;
initialguess(11).JointPosition=pi/9;
initialguess(23).JointPosition=-pi/6;
initialguess(29).JointPosition=-pi/6;
[configSol,solInfo]=gik(initialguess,poseConst_fe_L,poseConst_fe_R, ...
    poseConst_hd_L, poseConst_hd_R, limitJoint);

res1=getTransform(robot,configSol,'Link_arm_l_01');
disp(res1(1:3,4)');
res2=getTransform(robot,configSol,'Link_arm_l_07','Link_arm_l_01');
disp(res1(1:3,1:3)*res2(1:3,4));

res1=getTransform(robot,configSol,'Link_arm_r_01');
disp(res1(1:3,4)');
res2=getTransform(robot,configSol,'Link_arm_r_07','Link_arm_r_01');
disp(res1(1:3,1:3)*res2(1:3,4));

figure('Name','New');
%configSol=homeConfiguration(robot);
show(robot,config,'visual','on','collision','off');





















%rgba="0.89804 0.91765 0.92941 1" 
