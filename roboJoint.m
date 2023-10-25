classdef roboJoint < handle_light
    % ROBOJOINT Robotic arm manipulator joint
    %   Generate robotic arm joint object
    %
    % roboJoint Properties:
    %   Name           - Joint name
    %   Type           - Type of joint
    %   JointAxis      - Rotation or translation joint's axis
    %   PositionLimits - Joint limit
    %   HomePosition   - Standard position of the joint
    %   j2p            - Roto-translation joint-to-parent
    %   c2j            - Roto-translation child-to-joint
    %
    % roboJoint Methods:
    %   roboJoint          - Class constructor
    %   setJointAxis       - Set joint axis of rotation/translation
    %   setFixedTransform  - Set fixed transformation between joint's frames
    %   plot               - Plot joint frames
    %   copyRigidBodyJoint - Copy a rigidBodyJoint object to roboJoint one

    properties
        % Name - Joint name
        %   char array or string
        Name {mustBeTextScalar, mustBeNonempty} = ' '

        % Type - Type of joint
        %   in {'revolute', 'prismatic', 'fixed'} | default = 'fixed' | char array or string
        Type {mustBeMember(Type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'

        % JointAxis - Axis of rotation or translation of the joint
        %   default = [0, 0, 1] | double(1, 3)
        JointAxis {mustBeNonZeroNorm, mustBeNonNan, mustBeSizeVector(JointAxis, [1, 3])} = [0, 0, 1]

        % PositionLimits - Minimum and maximum values reachable by the joint
        %   rad or m | default = [0, 0] | double(1, 2)
        PositionLimits {mustBeNonNan, mustBeSizeVector(PositionLimits, [1, 2])} = [0, 0]

        % HomePosition - Standard position of the joint
        %   rad or m | default = 0 | double
        HomePosition {mustBeNonNan, mustBeFinite, mustBeSizeVector(HomePosition, [1, 1])} = 0
    end

    % ------------------------------------------------------------------- %

    properties (SetAccess = {?roboLink})
        % j2p - Roto-translation of the current joint axis w.r.t. parent (previous joint axis)
        %   belong to SE(3) | default = eye(4) | double(4, 4)
        j2p {mustBeSE3, mustBeNonNan} = eye(4)

        % c2j - Roto-translation of next joint(s) w.r.t. current joint
        %   belong to SE(3) | default = eye(4) | double(4, 4)
        c2j {mustBeSE3, mustBeNonNan} = eye(4)
    end

    % ------------------------------------------------------------------- %

    methods
        % --- Constructor --- %
        function obj = roboJoint(Name, Type, PositionLimits, HomePosition)
            % roboJoint - Class constructor
            %
            % Syntax
            %   roboJoint(Name)
            %   roboJoint(Name, Type)
            %   roboJoint(Name, Type, PositionLimits)
            %   roboJoint(Name, Type, PositionLimits, HomePosition)
            %
            %   roboJoint(rigidBodyJointObj)
            %
            % Input:
            %   rigidBodyJointObj - rigidBodyJoint object
            %       rigidBodyJoint
            %
            % Se also NAME, TYPE, POSITIONLIMITS, HOMEPOSITION

            arguments
                Name {mustBeA(Name, {'char', 'string', 'rigidBodyJoint'}), mustBeNonempty}
                Type {mustBeMember(Type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'
                PositionLimits {mustBeNumeric, mustBeNonNan} = [0, 0]        
                HomePosition {mustBeNumeric, mustBeNonNan, mustBeFinite} = 0
            end
            
            % Add casadi to path
            try
                tools.addCasadiToPath
            catch E
                if ismember(E.identifier, {'tools:MissingFolder', 'tools:pathError'}) && ...
                        strcmp(tools.tim.Running, 'off')
                    tools.tim.start
                    warning('off', 'backtrace'), warning(getReport(E, 'extended')) % basic
                    warning('Future computation might be affected if not corrected manually');
                    warning('on', 'backtrace')
                end
                if ~ismember(E.identifier, {'tools:MissingFolder', 'tools:pathError'}), throw(E), end
            end
            
            % Object creation
            if isa(Name, 'rigidBodyJoint')
                obj = roboJoint.copyRigidBodyJoint(Name);
            else
                obj.Name = Name; obj.Type = Type;
                obj.PositionLimits = sort(PositionLimits);
                obj.HomePosition = HomePosition;
            end
        end

        % --------------------------------------------------------------- %

        function setJointAxis(obj, JointAxis)
            % setJointAxis - Set joint axis of rotation/translation
            %
            % Syntax
            %   setJointAxis(JointAxis)
            %   setJointAxis(in {'x', 'y', 'z'})
            %
            % See also JOINTAXIS

            arguments
                obj roboJoint
                JointAxis {mustBeAxis}
            end

            if ischar(JointAxis), JointAxis = double(JointAxis == ['x', 'y', 'z']); end
            obj.JointAxis = JointAxis; obj.JointAxis = JointAxis/norm(JointAxis);
        end

        % --------------------------------------------------------------- %

        function setFixedTransform(obj, data, frame2frame, type)
            % setFixedTransform - Set fixed transformation between joint's frames
            % Set transformation between:
            %   - 'c2j': child(ren) frame w.r.t. joint frame
            %   - 'j2p': joint frame w.r.t. parent frame
            % Available types:
            %   - 'A': homogeneous matrix
            %   - 'dh', 'DH': Denavit-Hartenberg parameters
            %
            % See also roboJoint.c2j and roboJoint.j2p
            %
            % Syntax
            %   setFixedTransform(data)
            %   setFixedTransform(data, frame2frame)
            %   setFixedTransform(data, frame2frame, type)
            %
            % Input:
            %   data - Data used for setting the transformation
            %       [belong to SE(3) | double(4,4)] or [[d, theta, a, alpha] | double(1, 4)]
            %   frame2frame - Define transformation frame
            %       in {'c2j', 'j2p'} | default = 'c2j' | char array or string
            %   type - Type of description used to define the transformation
            %       in {'A', 'dh'} | default = 'dh' | char array or string
            
            arguments
                obj roboJoint
                data {mustBeNonempty, mustBeNumeric}
                frame2frame {mustBeMember(frame2frame, {'c2j', 'j2p'})} = 'c2j'
                type {mustBeMember(type, {'A', 'dh', 'DH'})} = 'dh'
            end

            switch type
                case 'A', A = data;
                case 'dh', A = tools.ADHparams(data);
                case 'DH', A = tools.ADHparams(data);
            end
            switch frame2frame
                case 'c2j', obj.c2j = A;
                case 'j2p', obj.j2p = A;
            end
        end

        % --------------------------------------------------------------- %

        function [Ajoint2B_m] = plot(obj, jointValue, Ajoint2B, specifics)
            % plot - Plot joint frames
            % Frame legend
            %   - dotted line: joint frame without movement ('fixed')
            %   - full line: joint frame with movement ('moved')
            %   - dashed line: child(ren) base frame ('child')
            %
            % Syntax
            %   Ajoint2B_m = plot
            %   ... = plot(Ajoint2B)
            %   ... = plot(jointValue, Ajoint2B)
            %   ... = plot(jointValue, Ajoint2B, specifics)
            %
            % Input:
            %   jointValue - Joint value
            %       rad or m | default = HomePosition | double
            %   Ajoint2B - Joint parent frame w.r.t. base frame
            %       belong to SE(3) | default = eye(4) | double(4, 4)
            %   specifics - Frame(s) to show
            %       in {'all', 'fixed', 'moved', 'child'} | default = {'fixed', 'child'} | char cell array or string array
            %
            % Output:
            %   Ajoint2B_m - Joint frame with movement ('moved')
            %       belong to SE(3) | double(4, 4)

            arguments (Input)
                obj roboJoint
                jointValue {mustBeSizeVector(jointValue, [1, 1]), mustBeNumeric} = obj.HomePosition
                Ajoint2B {mustBeSE3} = obj.j2p
                specifics {mustBeMember(specifics, {'all', 'fixed', 'moved', 'child'})} = {'fixed', 'child'}
            end
            arguments (Output)
                Ajoint2B_m {mustBeSE3}
            end
            
            R = eye(3); T = [0, 0, 0].';
            switch obj.Type
                case 'revolute', R = axang2rotm([obj.JointAxis, jointValue]);
                case 'prismatic', T = jointValue*obj.JointAxis.';
            end
            A = [R, T; 0, 0, 0, 1];

            if ismember({'fixed'}, specifics) || ismember({'all'}, specifics)
                tools.plotFrame(Ajoint2B, [obj.Name, '_{fixed}'], ":")
            end
            if ismember({'moved'}, specifics) || ismember({'all'}, specifics)
                tools.plotFrame(Ajoint2B*A, [obj.Name, '_{moved}'], "-")
            end
            if ismember({'child'}, specifics) || ismember({'all'}, specifics)
                tools.plotFrame(Ajoint2B*A*obj.c2j, [obj.Name, '_{child}'], "--")
            end
            
            Ajoint2B_m = Ajoint2B*A;
        end
    end

    % ------------------------------------------------------------------- %

    methods (Static)
        function roboJointObj = copyRigidBodyJoint(rigidBodyJointObj)
            % copyRigidBodyJoint - Copy rigidBodyJoint object in a
            % roboJoint object
            %
            % Syntax
            %   copyRigidBodyJoint(rigidBodyJointObj)
            %
            % Input:
            %   rigidBodyJointObj - rigidBodyJoint object
            %       rigidBodyJoint
            %
            % Output:
            %   roboJointObj - roboJoint object
            %       roboJoint
            
            arguments
                rigidBodyJointObj {mustBeA(rigidBodyJointObj, 'rigidBodyJoint')}
            end

            roboJointObj = roboJoint(rigidBodyJointObj.Name, rigidBodyJointObj.Type, ...
                rigidBodyJointObj.PositionLimits, rigidBodyJointObj.HomePosition);
            if all(~isnan(rigidBodyJointObj.JointAxis))
                roboJointObj.JointAxis = rigidBodyJointObj.JointAxis;
            end
            roboJointObj.j2p = rigidBodyJointObj.JointToParentTransform;
            roboJointObj.c2j = rigidBodyJointObj.ChildToJointTransform;
        end
    end
end

% --- Validating function --- %
function mustBeNonZeroNorm(data)
    % mustBeAxis - Validate that data has non zero norm
    if norm(data), return
    else, throw(MException('roboJoint:WrongValue', 'Must have non zero norm'));
    end
end

function mustBeSE3(data)
    % mustBESE3 - Validate that data is in the SE(3) group
    mustBeSizeVector(data, [4, 4])
    R = data(1:3, 1:3);
    if (~all(data(4, :) == [0, 0, 0, 1]) || ~(max(abs(R.'*R - eye(3)), [], 'all') < 1e-5) ...
            || ~(abs(det(R) - 1) < 1e-5))
        throw(MException('roboJoint:WrongValue', 'Must belong to SE(3) group'));
    end
end

function mustBeAxis(data)
    % mustBeAxis - Validate that data is an admissible axis
    %   Namely, it is member of {'x', 'y', 'z'} or is a numerical triplet
    wrong = false;
    if ~isnumeric(data)
        if numel(data) ~= 1, wrong = true; end
        try mustBeMember(data, {'x', 'y', 'z'}), catch, wrong = true; end
    elseif numel(data) ~= 3, wrong = true;
    end

    if wrong, throw(MException('roboJoint:WrongValue', ['Must be member of ' ...
            '{''x'', ''y'', ''z''} or numeric triplet'])), end
end

function mustBeSizeVector(data, vec_size)
    % mustBeSizeVector - Validate that data is a vector with specific size
    mustBeNumeric(data)
    if isequal(size(data), vec_size), return
    else, throw(MException('roboJoint:WrongDimension', 'Must have size [%s]', num2str(vec_size)));
    end
end