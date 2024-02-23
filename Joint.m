classdef Joint < handle_light
    % JOINT Robotic arm manupulator joint class
    %   Generate robotic arm joint object
    %
    % Joint Properties:
    %   name - Joint name
    %   type - Joint type
    %   positionLimits - Minimum and maximum values reachable by joint
    %   homePosition - Joint home position
    %   jointAxis - Axis of rotation/translation of the joint
    %   j2p - Roto-transaltion of the current joint axis w.r.t. parent (previous child axis)
    %   c2j - Roto-transaltion of the child(ren) axis w.r.t. current joint
    %   A - Homogeneous matrix representing the (moved) joint
    %
    % Joint Methods:
    %   Joint - Class constructor
    %   setFixedTR - Set fixed transformation between joint's frames
    %   toString - Plot in the command window object data
    %   plot - Plot Joint object
    %   copyRigidBodyJoint - Copy rigidBodyJoint in a Joint object


    % ---------------- Properties ---------------------- %

    properties
        % name - Joint name
        %   char array or string
        %   Validation: mustBeTextScalar, mustBeNonempty
        name {mustBeTextScalar, mustBeNonempty} = ' '

        % type - Joint type
        %   in {'revolute', 'prismatic', 'fixed'} | defaul = 'fixed' | char array or string
        %   Validation: mustBeTextScalar, mustBeMember(..., {'revolute', 'prismatic', 'fixed'})
        type {mustBeTextScalar, mustBeMember(type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'
        
        % positionLimits - Minimum and maximum values reachable by joint
        %   rad or m | default = [0; 0] | double(2, 1)
        %   Validation: mustBeNonNan, mustBeReal, mustBeNumeric, Tools.mustHaveSize(..., [2, 1])
        positionLimits {mustBeNonNan, mustBeReal, mustBeNumeric, Tools.mustHaveSize(positionLimits, [2, 1])} = [0; 0]

        % homePosition - Joint home position
        %   rad or m | default = 0 | double(1, 1)
        %   Validation: mustBeNonNan, mustBeReal, mustBeNumeric, Tools.mustHaveSize(..., [2, 1]) 
        homePosition {mustBeNonNan, mustBeReal, mustBeNumeric, Tools.mustHaveSize(homePosition, [1, 1])} = 0

        % jointAxis - Axis of rotation/translation of the joint
        %   [automatic conversion from {'x', 'y', 'z'} to double(3, 1)]
        %   default = [0, 0, 1].' | double(3, 1)
        %   Validation: Tools.mustBeAxis
        jointAxis {Tools.mustBeAxis} = [0, 0, 1].'
    end

    % ------------------------- %

    properties (SetAccess = {?Joint, ?Link, ?Arm})
        % j2p - Roto-transaltion of the current joint axis w.r.t. parent (previous child axis)
        %   belong to SE(3) | default = eye(4) | double(4, 4)
        %   Validation: Tools.mustBeSE3
        j2p {Tools.mustBeSE3} = eye(4)

        % c2j - Roto-transaltion of the child(ren) axis w.r.t. current joint
        %   belong to SE(3) | default = eye(4) | double(4, 4)
        %   Validation: Tools.mustBeSE3
        c2j {Tools.mustBeSE3} = eye(4)
    end

    % ------------------------- %

    properties (Dependent)
        % A - Homogeneous matrix representing the (moved) joint
        %   Set: set the frame of the parent (child of the previous joint) [SE3(4, 4) | MX.sym(4, 4)]
        %   Get: get the joint (moved) frame [MX.sym(4, 4)]
        %   Validation: Tools.mustHaveSize(..., [4, 4]), mustBeA(..., 'casadi.MX') OR Tools.mustBeSE3
        A {Tools.mustHaveSize(A, [4, 4]), ...
            Tools.mustAndOr('or', A, 'mustBeA', 'casadi.MX', 'mustBeSE3', [])}
    end

    % ------------------------- %

    properties (Access = {?Joint, ?Link, ?Arm}, Hidden)
        % Ab - Hidden homogeneous matrix containing the parent (child of the previous joint) frame
        %   defualt = eye(4) | double(4, 4) or MX.sym(4, 4)
        %   Validation: Tools.mustHaveSize(..., [4, 4]), mustBeA(..., 'casadi.MX') OR Tools.mustBeSE3
        Ab {Tools.mustHaveSize(Ab, [4, 4]), ...
            Tools.mustAndOr('or', Ab, 'mustBeA', 'casadi.MX', 'mustBeSE3', [])} = eye(4)
    end


    % ----------------- Functions ---------------------- %

    methods
        % ----- Constructor ----- %
        function obj = Joint(name, type, positionLimits, homePosition, jointAxis)
            % Joint - Class constructor
            %
            % Syntax
            %   Joint(name)
            %   Joint(name, type)
            %   Joint(name, type, positionLimits)
            %   Joint(name, type, positionLimits, homePosition)
            %   Joint(name, type, positionLimits, homePosition, jointAxis)
            %   
            %   Joint(rigidBodyJointObj)
            %
            % Input:
            %   rigidBodyJointObj - rigidBodyJoint object
            %       rigidBodyJoint
            %       Validation: mustBeA(..., 'rigidBodyJoint'), mustBeNonempty
            %
            % See also name, type, positionLimits, homePosition, jointAxis

            arguments (Input)
                name {mustBeNonempty, Tools.mustAndOr('or', name, 'mustBeA', 'rigidBodyJoint', 'mustBeTextScalar', [])}
                type {mustBeTextScalar, mustBeMember(type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'
                positionLimits {mustBeNonNan, mustBeReal, mustBeNumeric, Tools.mustHaveSize(positionLimits, [2, 1])} = [0; 0]
                homePosition {mustBeNonNan, mustBeReal, mustBeNumeric, Tools.mustHaveSize(homePosition, [1, 1])} = 0
                jointAxis {Tools.mustBeAxis} = [0, 0, 1].'
            end

            Tools.checkCasADi   % Check casadi folder
            if isa(name, 'rigidBodyJoint')
                obj = Joint.copyRigidBodyJoint(name); return
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
            % Syntax
            %   setFixedTR(data)
            %   setFixedTR(data, frame2frame)
            %
            % Input:
            %   data - Roto-transaltion data
            %       in SE(3) or Danavit-Hartenberg parms | double(4, 4) or double(1, 4)
            %       Denavit-Hartenberg params: [d, theta, a, alpha]
            %       Validation: mustBeReal, mustBeFinite, mustBeNumeric, Tools.mustBeSE3 OR Tools.mustHaveSize(..., [1, 4]) OR Tools.mustHaveSize(..., [4, 1])
            %   frame2frame - Tranformation frame
            %       in {'c2j', 'j2p'} | default = 'c2j' | char array or string
            %       Validation: mustBeTextScalar, mustBeMember(..., {'c2j', 'j2p'})
            %
            % See also Joint.c2j, Joint.j2p

            arguments, obj Joint
                data {mustBeReal, mustBeFinite, mustBeNumeric, ...
                    Tools.mustAndOr('or', data, 'mustBeSE3', [], 'mustHaveSize', [1, 4], 'mustHaveSize', [4, 1])}
                frame2frame {mustBeTextScalar, mustBeMember(frame2frame, {'c2j', 'j2p'})} = 'c2j'
            end

            switch frame2frame
                case 'j2p', obj.j2p = Tools.rotTra(data);
                otherwise, obj.c2j = Tools.rotTra(data);
            end
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
            %       Validation: mustBeTextScalar

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
                num2str(rotm2eul(obj.j2p(1:3, 1:3), 'XYZ')*180/pi), num2str(obj.j2p(1:3, 4).'), ...
                num2str(rotm2eul(obj.c2j(1:3, 1:3), 'XYZ')*180/pi), num2str(obj.c2j(1:3, 4).'))
        end

        % ------------------------- %

        function plot(obj, jointValue, specifics)
            % plot - Plot Joint object
            % Frame legend:
            %   - dotted line: parent joint frame ('parent')
            %   - full line: (moved) joint frame ('joint')
            %   - dashed line: child(ren) frame ('child')
            %
            % Syntax
            %   plot(jointValue)
            %   plot(jointValue, specific_1, specific_2, ...)
            %
            % Input:
            %   jointValue - Joint value
            %       rad or m | default = homePosition | empty or double(1, 1)
            %       Validation: mustBeReal, mustBeNumeric, mustBeFinite, mustBeScalarOrEmpty
            %   specifics - Frame(s) to show
            %       in {'parent', 'joint', 'child', 'all'} | default = 'joint' | char array or string
            %       Validation: mustBeMember(..., {'parent', 'joint', 'child', 'all'})
            
            arguments
                obj Joint,
                jointValue {mustBeReal, mustBeNumeric, mustBeFinite, mustBeScalarOrEmpty} = obj.homePosition
            end
            arguments (Repeating), specifics {mustBeMember(specifics, {'parent', 'joint', 'child', 'all'})}, end

            if isempty(jointValue), jointValue = obj.homePosition; end
            if isempty(specifics), specifics = {'joint'}; end

            if isa(obj.Ab, 'casadi.MX'), warning(['Cannot plot the joint since ' ...
                    'there is a further dependency on the joint base frame.\n' ...
                    'Resort to plot of Arm class'], []), return
            end

            A_fun = obj.A; A_val = full(A_fun(jointValue));

            if any(ismember(specifics, 'all')), specifics = {'parent', 'joint', 'child'}; end
            specifics = unique(specifics);
            for k = 1:numel(specifics)
                switch specifics{k}
                    case 'parent', Tools.plotFrames(obj.Ab, [obj.name, '_p'], ':');
                    case 'joint', Tools.plotFrames(A_val, [obj.name, '_m'], '-');
                    case 'child', Tools.plotFrames(A_val*obj.c2j, [obj.name, '_c'], '--');
                end
            end
        end
    end


    % ---------------- Get/set fun. -------------------- %

    methods
        function set.jointAxis(obj, axis)            
            % Input:
            %   axis - Axis of rotation/transaltion
            %       in {'x', 'y', 'z'} or double(3, 1)
            %       Validation: Tools.mustBeAxis
            
            arguments, obj Joint, axis {Tools.mustBeAxis}, end
            obj.jointAxis = Tools.isAxis(axis);
        end

        % ------------------------- %

        function set.A(obj, Ab)
            % Save homogeneous matrix of the joint base frame
            obj.Ab = Ab;
        end
        
        % ------------------------- %

        function A = get.A(obj)
            % Return (moved) joint symbolic expression
            import casadi.*
            x = MX.sym('x', [1, 1]);
            switch obj.type
                case 'revolute'
                    K = Tools.skew(obj.jointAxis); R = MX.eye(3) + sin(x)*K + (1 - cos(x))*K^2;
                    Aj = [R, zeros(3, 1); 0, 0, 0, 1];
                case 'prismatic', Aj = [MX.eye(3), x*obj.jointAxis; 0, 0, 0, 1];
                otherwise, Aj = MX.eye(4);
            end
            A = Function('A', {x}, {obj.Ab*obj.j2p*Aj});
        end
    end


    % ------------------- Static ----------------------- %

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
            %       Vaidation: mustBeNonempty, mustBeA(..., 'rigidBodyJoint')
            % Output:
            %   jointObj - Joint object
            %       Joint

            arguments (Input)
                rigidBodyJointObj {mustBeNonempty, mustBeA(rigidBodyJointObj, 'rigidBodyJoint')}
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