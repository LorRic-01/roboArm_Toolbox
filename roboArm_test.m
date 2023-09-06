% ROBOARM_TEST Test roboArm class
clc, clearvars, close all

%% Creation test

arm = roboArm;

joint1 = roboJoint('j1');
DHParams = [0, pi/2, 0.5, 0];
joint1.setFixedTransform(DHParams, 'j2p', 'dh')

DHParams = [0.2, -pi/2, 0.15, 0];
joint1.setFixedTransform(DHParams, 'c2j', 'dh');

% Rotation/translation
joint1.setJointAxis([0, 0, 1])
joint1.Type = 'prismatic';

link1 = roboLink('l1', joint1);

arm.addBody(link1)