% ROBOARM_TEST Test roboArm class
clc, clearvars, close all

%% Creation test

arm = roboArm;

% Joint 1
joint1 = roboJoint('j1');
DHParams = [0, 0, 0, 0];
joint1.setFixedTransform(DHParams, 'j2p', 'dh')

DHParams = [0, 0, 1, 0];
joint1.setFixedTransform(DHParams, 'c2j', 'dh');

% Rotation/translation
joint1.setJointAxis([0, 0, 1])
joint1.Type = 'revolute';

link1 = roboLink('l1', joint1);

arm.addBody(link1)

% Joint 2
joint2 = roboJoint('j2');
DHParams = [0, 0, 0, 0];
joint2.setFixedTransform(DHParams, 'j2p', 'dh')

DHParams = [0, 0, 0.5, 0];
joint2.setFixedTransform(DHParams, 'c2j', 'dh');

% Rotation/translation
joint2.setJointAxis([0, 0, 1])
joint2.Type = 'revolute';

link2 = roboLink('l2', joint2);

arm.addBody(link2, 'l1')