classdef Joint < handle_light
    % JOINT Robotic arm manupulator joint class
    %   Generate robotic arm joint object
    %
    % Joint Properties:
    %   name - Joint name
    %   type - Type of joint
    %   positionLimits - Minimum and maximum values reachable by the joint
    %   homePosition - Home position of the joint
    %   jointAxis - Axis of rotation or translation of the joint
    %   j2p - Roto-translation of the current joint axis w.r.t. parent (previous joint axis)
    %   c2j - Roto-translation of the next joint(s) w.r.t. current joint
    %
    % Joint Methods:
    %   Joint - Class constructor
    %   copyRigidBodyJoint - Copy rigidBodyJoint in a Joint object (Static)
    %   setFixedTR - Set fixed transformation between joint's frames
    %   toString - Plot in the command window object data

    properties
        % name - Joint name
        %   char array or string
        name {mustBeTextScalar, mustBeNonempty} = ' '

        % type - Type of joint
        %   in {'revolute', 'prismatic', 'fixed'} | defult = 'fixed' | char array or string
        type {mustBeMember(type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'

        % positionLimits - Minimum and maximum values reachable by the joint
        %   rad or m | default = [0, 0] | double(2, 1)
        positionLimits {mustBeNonNan, mustBeNumeric, Tools.mustHaveSize(positionLimits, [2, 1])} = [0, 0].'

        % homePosition - Home position of the joint
        %   rad or m | default = 0 | double(1, 1)
        homePosition {mustBeNonNan, mustBeNumeric, Tools.mustHaveSize(homePosition, [1, 1])} = 0

        % jointAxis - Axis of rotation or translation of the joint
        %   [automatic conversion from {'x', 'y', 'z'} to double(3, 1)]
        %   default = [0, 0, 1].' | double(3, 1)
        jointAxis {Tools.mustBeAxis} = [0, 0, 1].'
    end

    % ------------------------- %

    properties (SetAccess = {?Joint, ?Link})
        % j2p - Roto-translation of the current joint axis w.r.t. parent (previous joint axis)
        %   belong to SE(3) | default = eye(4) | double(4, 4)
        j2p {Tools.mustBeSE3} = eye(4)
        
        % c2j - Roto-translation of the next joint(s) w.r.t. current joint
        %   belong to SE(3) | default = eye(4) | double(4, 4)
        c2j {Tools.mustBeSE3} = eye(4)
    end

    % -------------------------------------------------- %

    methods
        % ----- Constructor ----- %
        function obj = Joint(name, type, positionLimits, homePosition, jointAxis)
            % Joint - Class contructor
            %
            % Syntax
            %   Joint(name)
            %   Joint(name, type)
            %   Joint(name, type, positionLimits)
            %   Joint(name, type, positionLimits, homePosition)
            %   Joint(name, type, positionLimits, homePosition, jointAxis)
            %   Joint(rigidBodyJointObj)
            %
            % Input:
            %   rigidBodyJointObj - rigidBodyJoint object
            %       rigidBodyJoint
            %   
            % See also name, type, positionLimits, homePosition, jointAxis, rigidBodyJoint
    
            arguments (Input)
                name {mustBeNonempty, Tools.mustOr(name, {'mustBeTextScalar'}, {'mustBeA', 'rigidBodyJoint'})}
                type {mustBeMember(type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'
                positionLimits {mustBeNonNan, mustBeNumeric, Tools.mustHaveSize(positionLimits, [2, 1])} = [0, 0].'
                homePosition {mustBeNonNan, mustBeNumeric, Tools.mustHaveSize(homePosition, [1, 1])} = 0
                jointAxis {Tools.mustBeAxis} = [0, 0, 1].'
            end
            arguments (Output), obj Joint, end

            
            Tools.checkCasADi   % Check casadi folder
            if isa(name, 'rigidBodyJoint')
                obj = Joint.copyRigidBodyJoint(name);
                return
            end

            obj.name = name; obj.type = type;
            obj.positionLimits = positionLimits; obj.homePosition = homePosition;
            obj.jointAxis = jointAxis/norm(jointAxis);
        end

        % ------------------------- %

        function setFixedTR(obj, data, frame2frame)
            % setFixedTR - Set fixed transformation between joint's frames
            % Set transformation between:
            %   - 'c2j': child(ren) frame w.r.t. joint frame
            %   - 'j2p': joint frame w.r.t. parent frame
            % Available rototranslation representation
            %   - homogeneous matrix A (in SE(3))
            %   - Denavit-Hartenberg parameters ([d, theta, a, alpha])
            %
            % See also Joint.c2j and Joint.j2p
            %
            % Syntax
            %   setFixedTR(data)
            %   setFixedTR(data, frame2frame)
            %
            % Input:
            %   data - Roto-transaltion data
            %       in SE(3) or Danavit-Hartenberg parms | double(4, 4) or double(1, 4)
            %       Denavit-Hartenberg params: [d, theta, a, alpha]
            %   frame2frame - Tranformation frame
            %       in {'c2j', 'j2p'} | default = 'c2j' | char array or string

            arguments, obj Joint
                data {mustBeNonempty, mustBeFinite, Tools.mustOr(data, {'mustBeSE3'}, {'mustHaveSize', [1, 4]})}
                frame2frame {mustBeMember(frame2frame, {'c2j', 'j2p'})} = 'c2j'
            end

            switch frame2frame
                case 'j2p', obj.j2p = Tools.rotTra(data);
                otherwise, obj.c2j = Tools.rotTra(data);
            end

        end

        % ------------------------- %

        function set.jointAxis(obj, axis)            
            % Input:
            %   axis - Axis of rotation/transaltion
            %       in {'x', 'y', 'z'} or double(3, 1)
            
            arguments, obj Joint, axis {Tools.mustBeAxis}, end
            obj.jointAxis = Tools.isAxis(axis);
        end

        % ------------------------- %

        function toString(obj, prefix)
            % toString - Plot in the command window object data
            %
            % Syntax
            %   toString
            %   toString(prefix)
            %
            % Input;
            %   prefix - Text before print
            %       char array or string

            arguments, obj Joint, prefix {mustBeTextScalar} = '', end

            fprintf([prefix, ' ----- Joint object ----- \n' ...
                     prefix, '  Name:              %s \n' ...
                     prefix, '  Type:              %s \n' ...
                     prefix, '  Pos. lim [lb, ub]: [%s]\n' ...
                     prefix, '  Home pos.:         %s\n' ...
                     prefix, '  Axis:              [%s]^T\n' ...
                     prefix, '  Joint-to-parent:   RPY: [%s] deg, T: [%s]^T m\n' ...
                     prefix, '  Child-to-joint:    RPY: [%s] deg, T: [%s]^T m\n' ...
                     prefix, ' ------------------------- \n\n'], ...
                obj.name, obj.type, num2str(obj.positionLimits.'), ...
                num2str(obj.homePosition), num2str(obj.jointAxis.'), ...
                num2str(rotm2eul(obj.j2p(1:3, 1:3), "XYZ")*180/pi), num2str(obj.j2p(1:3, 4).'), ...
                num2str(rotm2eul(obj.c2j(1:3, 1:3), "XYZ")*180/pi), num2str(obj.c2j(1:3, 4).'))
        end
    end

    % -------------------------------------------------- %

    methods (Static)
        function jointObj = copyRigidBodyJoint(rigidBodyJointObj)
            % copyRigidBodyJoint - Copy rigidBodyJoint in a Joint object
            %
            % Syntax
            %   copyRigidBodyJoint(rigidBodyJointObj)
            %
            % Input:
            %   rigidBodyJointObj - rigidBodyJoint object
            %       rigidBodyJoint
            % Output:
            %   jointObj - Joint object
            %       Joint

            arguments (Input)
                rigidBodyJointObj {mustBeA(rigidBodyJointObj, 'rigidBodyJoint')}
            end
            arguments (Output), jointObj Joint, end

            jointObj = Joint(rigidBodyJointObj.Name, rigidBodyJointObj.Type, ...
                rigidBodyJointObj.PositionLimits.', rigidBodyJointObj.HomePosition);
            jointObj.jointAxis = rigidBodyJointObj.JointAxis.';
            jointObj.j2p = rigidBodyJointObj.JointToParentTransform;
            jointObj.c2j = rigidBodyJointObj.ChildToJointTransform;
        end
    end
end