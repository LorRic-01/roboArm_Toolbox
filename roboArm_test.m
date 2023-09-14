% ROBOARM_TEST Test roboArm class
% clc, clearvars, close all

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
    DHParams = [0, 0, 1, 0];
    joint2.setFixedTransform(DHParams, 'c2j', 'dh');
    % Rotation/translation
    joint2.setJointAxis([0, 0, 1])
    joint2.Type = 'revolute';
link2 = roboLink('l2', joint2);

arm.addBody(link2, 'l1')

% EE
ee = roboJoint('ee');
ee_link = roboLink('l_ee', ee);

arm.addBody(ee_link, 'l2')

%% Plot
figure(1), clf
arm.plot([pi/3, pi/4])
axis equal, axis padded, grid on, view(3)
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')

%% Comparison
arm = roboArm;
dhparams = [0   	pi/2	0   	0;
            0.4318	0       0       0
            0.0203	-pi/2	0.15005	0;
            0   	pi/2	0.4318	0;
            0       -pi/2	0   	0;
            0       0       0       0]; dhparams = [dhparams(:, 3:4), dhparams(:, 1:2)];

joint1 = roboJoint('j1'); joint1.setFixedTransform(dhparams(1, :), 'c2j', 'dh');
joint2 = roboJoint('j2'); joint2.setFixedTransform(dhparams(2, :), 'c2j', 'dh');
joint3 = roboJoint('j3'); joint3.setFixedTransform(dhparams(3, :), 'c2j', 'dh');
joint4 = roboJoint('j4'); joint4.setFixedTransform(dhparams(4, :), 'c2j', 'dh');
joint5 = roboJoint('j5'); joint5.setFixedTransform(dhparams(5, :), 'c2j', 'dh');
joint6 = roboJoint('j6'); joint6.setFixedTransform(dhparams(6, :), 'c2j', 'dh');

joint1.Type = 'revolute'; link1 = roboLink('l1', joint1); arm.addBody(link1)
joint2.Type = 'revolute'; link2 = roboLink('l2', joint2); arm.addBody(link2, 2)
joint3.Type = 'revolute'; link3 = roboLink('l3', joint3); arm.addBody(link3, 3)
joint4.Type = 'revolute'; link4 = roboLink('l4', joint4); arm.addBody(link4, 4)
joint5.Type = 'revolute'; link5 = roboLink('l5', joint5); arm.addBody(link5, 5)
joint6.Type = 'revolute'; link6 = roboLink('l6', joint6); arm.addBody(link6, 6)

ee = roboJoint('ee');
ee_link = roboLink('l_ee', ee); arm.addBody(ee_link, 7)

figure(2), clf
arm.plot
axis equal, axis padded, grid on, view(3)
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')