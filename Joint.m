classdef Joint < handle_light
    % JOINT Robotic arm manupulator joint class
    %   Generate robotic arm joint object
    %
    % Joint Properties:
    %
    % Joint Methods:
    %   Joint - Class constructor

    properties
        % Name - Joint name
        %   char array or string
        Name {mustBeTextScalar} = ''

        % Type - Type of joint
        %   in {'revolute', 'prismatic', 'fixed'} | defult = 'fixed' | char array or string
        Type {mustBeMember(Type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'

        % PositionLimits - Minimum and maximum values reachable by the joint
        %   rad or m | default = [0, 0] | double(2, 1)
        PositionLimits {mustBeNonNan, mustBeNumeric, Tools.mustBeSizeVector(PositionLimits, [2, 1])} = [0, 0].'

        % HomePosition - Home position of the joint
        %   rad or m | default = 0 | double(1, 1)
        HomePosition {mustBeNonNan, mustBeNumeric, Tools.mustBeSizeVector(HomePosition, [1, 1])} = 0
    end

    % ------------------------- %

    properties (SetAccess = {?Joint, ?Link})
        % JointAxis - Axis of rotation or translation of the joint
        %   default = [0, 0, 1].' | double(3, 1)
        JointAxis {Tools.mustBeNonzeroNorm, mustBeNonNan, Tools.mustBeSizeVector(JointAxis, [3, 1])} = [0, 0, 1].'

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
        function obj = Joint(Name, Type, PositionLimits, HomePosition, JointAxis)
            % Joint - Class contructor
            %
            % Syntax
            %   Joint(Name)
            %   Joint(Name, Type)
            %   Joint(Name, Type, PositionLimits)
            %   Joint(Name, Type, PositionLimits, HomePosition)
            %   Joint(Name, Type, PositionLimits, HomePosition, JointAxis)
            %   Joint(rigidBodyJointObj)
            %
            % Input:
            %   rigidBodyJointObj - rigidBodyJoint object
            %       rigidBodyJoint
            %   
            % See also NAME, TYPE, POSITIONLIMITS, HOMEPOSITION, RIGIDBODYJOINT
    
            arguments (Input)
                Name {mustBeTextScalar, mustBeNonempty}
                Type {mustBeMember(Type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'
                PositionLimits {mustBeNonNan, mustBeNumeric, Tools.mustBeSizeVector(PositionLimits, [2, 1])} = [0, 0].'
                HomePosition {mustBeNonNan, mustBeNumeric, Tools.mustBeSizeVector(HomePosition, [1, 1])} = 0
                JointAxis {Tools.mustBeNonzeroNorm, mustBeNonNan, Tools.mustBeSizeVector(JointAxis, [3, 1])} = [0, 0, 1].'
            end
            arguments (Output), obj Joint, end

            
            Tools.checkCasADi   % Check casadi folder
            if isa(Name, 'rigidBodyJoint')
                obj = Joint.copyRigidBodyJoint(Name);
                return
            end

            obj.Name = Name; obj.Type = Type;
            obj.PositionLimits = PositionLimits; obj.HomePosition = HomePosition;
            obj.JointAxis = JointAxis/norm(JointAxis);
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
                rigidBodyJointObj rigidBodyJoint {mustBeA(rigidBodyJointObj, 'rigidBodyJoint')}
            end
            arguments (Output), jointObj Joint, end

            jointObj = Joint(rigidBodyJointObj.Name, rigidBodyJointObj.Type, ...
                rigidBodyJointObj.PositionLimits, rigidBodyJointObj.HomePosition);
            jointObj.JointAxis = rigidBodyJointObj.JointAxis;
            jointObj.j2p = rigidBodyJointObj.JointToParentTransform;
            jointObj.c2j = rigidBodyJointObj.ChildToJointTransform;
        end
    end
end