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
    %   A - Homogeneous matrix representing the (moved) joint (casadi function)
    %
    % Joint Methods:
    %   Joint - Class constructor    
    %   setFixedTR - Set fixed transformation between joint's frames
    %   toString - Plot in the command window object data
    %   plot - Plot Joint object
    %   copyRigidBodyJoint - Copy rigidBodyJoint in a Joint object (Static)
    

    % ---------------- Properties ---------------------- %

    properties
        % name - Joint name
        %   char array or string
        name {mustBeTextScalar, mustBeNonempty} = ' '

        % type - Type of joint
        %   in {'revolute', 'prismatic', 'fixed'} | defult = 'fixed' | char array or string
        type {mustBeMember(type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'

        % positionLimits - Minimum and maximum values reachable by the joint
        %   rad or m | default = [0, 0] | double(2, 1)
        positionLimits {mustBeNonNan, mustBeReal, Tools.mustHaveSize(positionLimits, [2, 1])} = [0, 0].'

        % homePosition - Home position of the joint
        %   rad or m | default = 0 | double(1, 1)
        homePosition {mustBeNonNan, mustBeReal, Tools.mustHaveSize(homePosition, [1, 1])} = 0

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

    % ------------------------- %
    
    properties (Dependent)
        % A - Homogeneous matrix representing the (moved) joint
        %   Set: set the frame of the previous joint
        %   Get: get the joint (moved) frame
        %       defualt = eye(4) | double(4, 4) or MX.sym(4, 4)
        A {Tools.mustHaveSize(A, [4, 4]), Tools.mustOr(A, {'mustBeA', 'casadi.MX'}, {'mustBeReal'})}
    end

    % ------------------------- %

    properties (SetAccess = {?Joint, ?Link}, GetAccess = {?Joint, ?Link}, Hidden)
        % Ab - Hidden variable containing the frame of the previous joint
        %       defualt = eye(4) | double(4, 4) or MX.sym(4, 4)
        Ab {Tools.mustHaveSize(Ab, [4, 4]), Tools.mustOr(Ab, {'mustBeA', 'casadi.MX'}, {'mustBeReal'})} = eye(4)
    end


    % ----------------- Functions ---------------------- %

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
                positionLimits {mustBeNonNan, mustBeReal, Tools.mustHaveSize(positionLimits, [2, 1])} = [0, 0].'
                homePosition {mustBeNonNan, mustBeReal, Tools.mustHaveSize(homePosition, [1, 1])} = 0
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
            %   specifics - Frame(s) to show
            %       in {'parent', 'joint', 'child', 'all'} | default = 'joint' | char array or string
            
            arguments
                obj Joint,
                jointValue {mustBeReal, Tools.mustOr(jointValue, {'mustHaveSize', [1, 1]}, {'mustBeEmpty'})} = obj.homePosition
            end
            arguments (Repeating), specifics {mustBeMember(specifics, {'parent', 'joint', 'child', 'all'})}, end

            if isempty(jointValue), jointValue = obj.homePosition; end

            if isa(obj.Ab, 'casadi.MX'), warning(['Cannot plot the joint since ' ...
                    'there is a further dependency on the joint base frame.\n' ...
                    'Resort to plot of Arm class'], []), return
            end

            if isempty(specifics), specifics = {'joint'}; end
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