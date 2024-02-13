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
