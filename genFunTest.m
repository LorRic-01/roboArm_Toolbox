% GENFUNTEST Generate function from arm model
clc, clearvars, close all

%% Arm
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

arm = Arm(robot); clearvars dhparams body* jnt*
% Add visual
for k = arm.links
    vec = k.joint.c2j(1:3, 4);
    if norm(vec) > 1e-2
        k.addVisual('box', 'c2j', [0.1, 0.1, norm(vec)], [axang2rotm(Tools.vrrotvec([0, 0, 1].', vec/norm(vec))), [0, 0, 0].'; [0, 0, 0, 1]]);
    end
end

config = homeConfiguration(robot);

figure(1), clf, arm.plot
axis equal, axis tight, grid on, view(3)