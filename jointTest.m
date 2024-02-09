% JOINTTEST Test Joint class
clc, clearvars, close all

%% Constructor
joint = Joint('joint');
joint = Joint('joint', 'revolute');
joint = Joint('joint', 'prismatic', [-1, 1].');
joint = Joint('joint', 'revolute', [-pi, pi].', pi/4);
joint = Joint('joint', 'revolute', [-pi, pi].', pi/4, [0, 1, 0].');