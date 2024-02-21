% LINKTEST Test Link class
clc, clearvars, close all

%% Constructor
link = Link('link1'); link.toString

joint = Joint('joint2', 'revolute', [-pi, pi].', pi/4, [0, 1, 0].');
link = Link('link2', joint); link.toString

rigidBodyJointObj = rigidBodyJoint('rBJ', 'prismatic');
rigidBodyObj = rigidBody('rB'); rigidBodyObj.Joint = rigidBodyJointObj;
link = Link(rigidBodyObj); link.toString

%% Set dynamic parameters
link = Link('link3');
link.mass = [1, 2]; link.CoM = [[1, 2, 3].', [4, 5, 6].'];
link.I = [[1, 2, 3, 0, 0, 0].', [4, 5, 6, 0, 0, 0].'];
link.toString

%% Add visual geometry
link = Link('link4'); link.addVisual('empty')
link = Link('link5'); link.addVisual('box', 'j2p', [1, 1, 1])
link = Link('link6'); link.addVisual('cyl', 'j2p', [1, 1, 1], ...
    trvec2tform([0, 0, 0])*eul2tform(pi/180*[10 0 0], 'XYZ'));
% link = Link('link'); link.addVisual('stl', 'j2p', 'EndEffector_merge_fixed.stl', eye(4))

%% Plot
joint = Joint('joint7', 'revolute', [-pi, pi].', pi/4, [0, 1, 0].');
link = Link('link7', joint); link.toString
link.addVisual('cyl', 'j2p', [1, 1], trvec2tform([0, 0, 0])*eul2tform(pi/180*[10 0 0], 'XYZ'));
link.addVisual('box', 'c2j', [1, 1, 1], trvec2tform([0, 0, 2])*eul2tform(pi/180*[0 0 0], 'XYZ'));

figure(1), clf, grid on, axis equal, link.plot(0, 'all'), view(3)

link.joint.A = trvec2tform([1, 2, 3])*eul2tform(pi/180*[45 90 0], 'XYZ');
figure(2), clf, grid on, axis equal, link.plot(0, 'all'), view(3)

%% Compute dynamical parameters from visual
joint = Joint('joint8', 'revolute', [-pi, pi].', pi/4, [0, 1, 0].');
DHParams = [0, 0, 1, pi/3]; joint.setFixedTR(DHParams, 'j2p');
DHParams = [1, pi/6, 0, 0]; joint.setFixedTR(DHParams, 'c2j');
link = Link('link8', joint);
link.addVisual('cyl', 'j2p', [1, 1], trvec2tform([0, 0, 0])*eul2tform(pi/180*[0 0 0], 'XYZ'));
link.addVisual('box', 'c2j', [1, 1, 1], trvec2tform([0, 0, 0])*eul2tform(pi/180*[0 0 0], 'XYZ'));

link.mass = [1, 1];
link.cmpDynParam(struct('cycle', 1000, 'n_p', 1000, 'verbose', true)), link.toString
figure(3), clf, grid on, axis equal, link.plot(0, 'all'), view(3)