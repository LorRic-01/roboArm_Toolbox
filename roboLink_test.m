% ROBOLINK_TEST Test roboLink class
clc, clearvars, close all

%% Constructor test
links = roboLink('l1');

joints = roboJoint('j2');
links(end+1) = roboLink('l2', joints);

link = rigidBody('link');
links(end+1) = roboLink(link);

