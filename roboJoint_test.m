% ROBOJOINT_TEST Test roboJoint class
clc, clearvars, close all

%% Creation test
joint1 = roboJoint('j1');

% Joint2parent
DHParams = [0, pi/2, 0.5, 0];
joint1.setFixedTransform(DHParams, 'j2p', 'dh')

DHParams = [0.2, -pi/2, 0.15, 0];
joint1.setFixedTransform(DHParams, 'c2j', 'dh');

% Rotation/translation
joint1.setJointAxis([0, 0, 1])
joint1.Type = 'prismatic';


figure, joint1.plot(1, 'all')
axis equal, axis padded, grid on, view(3)
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')

%% Copy test
jnt2 = rigidBodyJoint('jnt2');

joint2 = roboJoint(jnt2);
jnt2, joint2

%% Symbolic matrix test
joint1.setJointAMatrix