% ROBOJOINT_TEST Test roboJoint class
clc, clearvars, close all

%% Constructor test
joints = roboJoint('j1');
joints(end+1) = roboJoint('j2', 'revolute');
joints(end+1) = roboJoint('j3', 'revolute', [-pi, pi]);
joints(end+1) = roboJoint('j4', 'revolute', [-pi, pi], pi/3);

joint = rigidBodyJoint('joint', 'prismatic');
joints(end+1) = roboJoint(joint);

%% Set joint axis
joints(1).setJointAxis('y')
joints(2).setJointAxis([1, 0, 0])