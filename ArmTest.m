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

%% Constructor via copy
robot = rigidBodyTree;

dhparams = [0   	pi/2	0   	0;
            0.4318	0       0       0
            0.0203	-pi/2	0.15005	0;
            0   	pi/2	0.4318	0;
            0       -pi/2	0   	0;
            0       0       0       0];

body1 = rigidBody('body1'); jnt1 = rigidBodyJoint('jnt1','revolute');
body2 = rigidBody('body2'); jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3'); jnt3 = rigidBodyJoint('jnt3','revolute');
body4 = rigidBody('body4'); jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5'); jnt5 = rigidBodyJoint('jnt5','revolute');
body6 = rigidBody('body6'); jnt6 = rigidBodyJoint('jnt6','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh'); setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh'); setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh'); setFixedTransform(jnt6,dhparams(6,:),'dh');

body1.Joint = jnt1; body2.Joint = jnt2;
body3.Joint = jnt3; body4.Joint = jnt4;
body5.Joint = jnt5; body6.Joint = jnt6;

addBody(robot,body1,'base'),  addBody(robot,body2,'body1')
addBody(robot,body3,'body2'), addBody(robot,body4,'body3')
addBody(robot,body5,'body4'), addBody(robot,body6,'body5')

config = homeConfiguration(robot);
config(1).JointPosition = pi/3;

arm = Arm(robot);
figure(1), clf, arm.plot([config.JointPosition], 'parent')
axis equal, axis padded, grid on, view(3)
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')

figure(2), clf, show(robot, config);
axis equal, axis padded, grid on, view(3)
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')

figure, arm.plotGraph

%% Arm construction
arm = Arm('arm3', Link('link1', Joint('joint1'))); arm.toString
link = Link('link2', Joint('joint2'));
arm.addLink(link); arm.toString
link = Link('link3', Joint('joint3'));
arm.addLink(link, 'link1'); arm.toString

%% Arm remove procedure
arm = Arm('arm4');
linkConnection = {'', 'link_1', 'link_1', 'link_3', 'link_3', 'link_4', 'link_5', 'link_6'};
type = {'fixed', 'revolute', 'prismatic', 'prismatic', 'prismatic', 'fixed', 'revolute', 'revolute'};
for k = 1:8
    joint = Joint(['joint_', num2str(k)], type{k}, [-pi, pi].', 0, [0, 0, 1].');
    DHParams = rand(1, 4); joint.setFixedTR(DHParams, 'c2j'); 

    link = Link(['link_', num2str(k)], joint); link.mass = [1, 1];

    A_c2j = Tools.rotTra(DHParams); vec = A_c2j(1:3, 4);
    link.addVisual('box', 'c2j', [0.1, 0.1, norm(vec)], [axang2rotm(Tools.vrrotvec([0, 0, 1].', vec/norm(vec))), [0, 0, 0].'; [0, 0, 0, 1]]);
    link.cmpDynParam

    arm.addLink(link, linkConnection{k})
end
arm.toString
figure, arm.plotGraph, figure, arm.plot([], {'parent', 'child'}, true)
axis equal, axis tight, grid on, view(3)
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')
arm.removeLink('link_4')
figure, arm.plotGraph, figure, arm.plot([], {'parent', 'child'})
axis equal, axis tight, grid on, view(3)
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')

