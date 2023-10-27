% ROBOLINK_TEST Test roboLink class
clc, clearvars, close all

%% Constructor test
% No initialization
link = roboLink('l');

% Link with custom joint
joints = roboJoint('j');
link = roboLink('l', joints);

% Link conversion from rigidBody to roboLink
link = roboLink(rigidBody('link'));

%% Set dynamic parameters

link = roboLink('l', roboJoint('j'));
link.setDynParams('c2j', 'Mass', 1, 'CoM', [1, 2, 3], 'I', diag(rand(1, 3)))






%%
% %% Constructor test
% links = roboLink('l1');
% 
% joints = roboJoint('j2');
% links(end+1) = roboLink('l2', joints);
% 
% link = rigidBody('link');
% links(end+1) = roboLink(link);
% 
% %% Set dynamic parameters
% joints = roboJoint('j3');
% links(end+1) = roboLink('l3', joints);
% 
% links(end).setDynParams('Mass', 1, 'CoM', [1, 2, 3], 'I', 0.5*eye(3))
% 
% %% Add viusalization
% joints = roboJoint('j3', 'revolute');
% DHparams = [1, pi/3, 1, 0];
% joints.setFixedTransform(DHparams, "c2j", "dh")
% 
% links(end+1) = roboLink('l4', joints);
% 
% links(end).setDynParams('Mass', 1, 'CoM', [1, 2, 3], 'I', 0.5*eye(3))
% links(end).addVisual('box')
% links(end).plot
% axis equal, grid on, view(3)