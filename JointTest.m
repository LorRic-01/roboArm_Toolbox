% JOINTTEST Test Joint class
clc, clearvars, close all

%% Constructor
joint = Joint('joint1'); joint.toString
joint = Joint('joint2', 'revolute'); joint.toString
joint = Joint('joint3', 'prismatic', [-1, 1].'); joint.toString
joint = Joint('joint4', 'revolute', [-pi, pi].', pi/4); joint.toString
joint = Joint('joint5', 'revolute', [-pi, pi].', pi/4, [0, 1, 0].'); joint.toString

rigidBodyJointObj = rigidBodyJoint('rBJ', 'prismatic');
joint = Joint(rigidBodyJointObj); joint.toString

%% Set joint axis
joint = Joint('joint6');
joint.jointAxis = 'y'; joint.toString

%% Set rot-tra between frames
DHParams = [1, pi/4, 1, 0];
joint = Joint('joint7');
joint.setFixedTR(DHParams); joint.toString

%% Plot
joint = Joint('joint7', 'revolute', [-pi, pi].', pi/4, [1, 0, 0].');
DHParams = [0, 0, 1, pi/3]; joint.setFixedTR(DHParams, 'j2p');
DHParams = [1, pi/6, 0, 0]; joint.setFixedTR(DHParams, 'c2j');
figure(1), clf, grid on, axis equal, joint.plot([], 'all'), view(3)