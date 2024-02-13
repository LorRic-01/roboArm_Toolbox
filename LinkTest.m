% LINKTEST Test Link class
clc, clearvars, close all

%% Constructor
link = Link('link'); link.toString

joint = Joint('joint', 'revolute', [-pi, pi].', pi/4, [0, 1, 0].');
link = Link('link', joint); link.toString

rigidBodyJointObj = rigidBodyJoint('rBJ', 'prismatic');
rigidBodyObj = rigidBody('rB'); rigidBodyObj.Joint = rigidBodyJointObj;
link = Link(rigidBodyObj); link.toString

%% Set dynamic parameters
link = Link('link');
link.mass = [1, 2]; link.CoM = [[1, 2, 3].', [4, 5, 6].'];
link.I = [[1, 2, 3, 0, 0, 0].', [4, 5, 6, 0, 0, 0].'];
link.toString