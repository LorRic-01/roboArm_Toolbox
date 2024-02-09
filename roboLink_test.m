% ROBOLINK_TEST Test roboLink class
clc, clearvars, close all

%% Constructor test
% No initialization
link = roboLink('l');

% Link with custom joint
joint = roboJoint('j');
link = roboLink('l', joint);

% Link conversion from rigidBody to roboLink
link = roboLink(rigidBody('link'));

%% Set dynamic parameters

link = roboLink('l', roboJoint('j'));
link.setDynParams('c2j', 'Mass', 1, 'CoM', [1, 2, 3], 'I', diag(rand(1, 3)))

%% Add visual

DHparams = [1, pi/3, 1, 0];
joint = roboJoint('j');
joint.setFixedTransform(DHparams, "c2j", "dh")

DHparams = [1, 0, 0, 0];
joint.setFixedTransform(DHparams, "j2p", "dh")

link = roboLink('l', joint);
link.addVisual('cyl', 'both', 'auto', [eye(3), [0; 0; 5]; 0, 0, 0, 1])

pdegplot(link.Visual{1}), hold on, pdegplot(link.Visual{2}), joint.plot


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