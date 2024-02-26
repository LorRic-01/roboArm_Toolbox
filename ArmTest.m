% ARMTEST Test Arm class
clc, clearvars, close all

%% Constructor
arm = Arm('arm1');

joint = Joint('joint1', 'revolute', [-pi, pi].', pi/4, [0, 1, 0].');
DHParams = [0, 0, 1, pi/3]; joint.setFixedTR(DHParams, 'j2p');
link = Link('link1', joint);
link.addVisual('cyl', 'j2p', [1, 1], trvec2tform([0, 0, 0])*eul2tform(pi/180*[0 0 0], 'XYZ'));
link.addVisual('box', 'c2j', [1, 1, 1], trvec2tform([0, 0, 0])*eul2tform(pi/180*[0 0 0], 'XYZ'));
link.mass = [1, 1];
link.cmpDynParam(struct('cycle', 100, 'n_p', 1000, 'verbose', false));
arm = Arm('arm2', link);

%% Arm construction
arm = Arm('arm3', Link('link1', Joint('joint1'))); arm.toString
link = Link('link2', Joint('joint2'));
arm.addLink(link); arm.toString
link = Link('link3', Joint('joint3'));
arm.addLink(link, 'link1'); arm.toString

%% Arm remove procedure
arm = Arm('arm4');
arm.addLink(Link('link_1'))
arm.addLink(Link('link_2'), 'link_1')
arm.addLink(Link('link_3'), 'link_1')
arm.addLink(Link('link_4'), 'link_3')
arm.addLink(Link('link_5'), 'link_3')
arm.addLink(Link('link_6'), 'link_4')
arm.addLink(Link('link_7'), 'link_5')
arm.addLink(Link('link_8'), 'link_5')
arm.toString