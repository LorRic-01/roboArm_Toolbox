classdef roboJoint < handle_light
    % ROBOJOINT - Robotic arm manipulator joint
    %   Generate robotic arm joint object
    %
    % roboJoint Properties:
    %   Name            - Joint name
    %   Type            - Type of joint
    %   JointAxis       - Axis of rotation or translation of the joint
    %   PositionLimits  - Joint limits
    %   HomePosition    - Standard position of the joint
    %   Parent          - Roto-translation from the previous joint
    %   Child           - Roto-transaltion of the child(ren)
    %   Ajoint2B        - Roto-translation homogeneous matrix from joint frame to base frame
    %   Ajoint2B_fun    - Ajoint2B casADi function
    %   Ajoint          - Joint frame roto-translation homogeneous matrix 
    %
    % roboJoint Methods:
    %   roboJoint           - Constructor
    %   setJointAxis        - Set the axis of rotation/translation of the joint
    %   setFixedTransform   - Set transformation between frames
    %   plot                - show joint frames
    %   copyRigidBodyJoint  - Copy a rigidBodyJoint object to a roboJoint object
    %   genJointAMatrix     - Generate joint roto-translation simbolic homogeneous matrix 

    properties
        % Name - Joint name
        %   char array
        Name (1, :) char

        % Type - Type of joint
        %   in {'revolute', 'prismatic', 'fixed'} | default = 'fixed' | char array
        Type (1, :) char {mustBeMember(Type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'

        % JointAxis - Axis of rotation or translation of the joint
        %   default = [0, 0, 1] | double(1, 3)
        JointAxis (1, 3) double {mustBeAxis(JointAxis), mustBeNonNan} = [0, 0, 1]

        % PositionLimits - Minimum and maximum values reachable by the joint
        %   rad or m | default = [0, 0] | double(1, 2)
        PositionLimits (1, 2) double {mustBeNumeric, mustBeVector} = [0, 0]

        % HomePosition - Standard position of the joint
        %   rad or m | default = 0 | double
        HomePosition (1, 1) double {mustBeNumeric} = 0
    end


    % ------------------------------------------------------------------- %


    properties (SetAccess = {?roboLink, ?roboArm})
        % Parent - Roto-translation of the current axis joint w.r.t. previous joint axis
        %   belong to SE(3) | default = eye(4) | double(4, 4)
        Parent (4, 4) double {mustBeSE3(Parent), mustBeNonNan} = eye(4)
        
        % Child - Roto-translation of the next joint(s) axis w.r.t. the current joint axis
        %   belong to SE(3) | default = eye(4) | double(4, 4)
        Child (4, 4) double {mustBeSE3(Child), mustBeNonNan} = eye(4)

        % Ajoint2B - Roto-translation homogeneous matrix from joint frame to base frame
        %   The roto-translation is referred to the fixed frame to which
        %   the joint is attached to (bafore joint motion frame)
        %   belong to SE(3) | default = eye(4) | double(4, 4) or casadi.SX or casadi.MX
        Ajoint2B (4, 4) = eye(4)

        % Ajoint2B_fun - Roto-translation function from joint frame to base frame
        %   casADi function of the symbolic varible Ajoint2B
        %   See also roboJoint.Ajoint2B
        %   casadi.Function
        Ajoint2B_fun

        % Ajoint - Joint frame roto-translation homogeneous matrix 
        %   belong to SE(3) | default = eye(4) | double(4, 4) or casadi.SX or casadi.MX
        Ajoint (4, 4) = eye(4)
    end


    % ------------------------------------------------------------------- %


    methods
        % --- Constructor --- %
        function obj = roboJoint(varargin)
            % roboJoint - Class constructor
            %
            % Syntax
            %   roboJoint(rigidBodyJointObj)
            %   roboJoint(Name)
            %   roboJoint(Name, Type)
            %   roboJoint(Name, Type, PositionLimits)
            %   roboJoint(Name, Type, PositionLimits, HomePosition)
            %
            % Input:
            %   rigidBodyJointObj - rigidBodyJoint object
            %       rigidBodyJoint
            % See also NAME, TYPE, POSITIONLIMITS, HOMEPOSITION

            tools.addCasadiToPath
            
            if (nargin == 0) || (nargin > 4)
                error("Wrong number of parameters passed, check constructor syntax")
            end

            if (nargin > 0) && (~isempty(varargin{1}))
                switch class(varargin{1})
                    case 'rigidBodyJoint', obj.copyRigidBodyJoint(varargin{1})
                    case 'char', obj.Name = varargin{1};
                    otherwise, error("Wrong passed parameter, check constructor syntax")
                end
            end

            if (nargin > 1) && (~isempty(varargin{2}))
                obj.Type = varargin{2};
            end

            if (nargin > 2) && (~isempty(varargin{3}))
                obj.PositionLimits = varargin{3}; obj.PositionLimits = sort(varargin{3});
            end

            if (nargin > 3) && (~isempty(varargin{4}))
                obj.HomePosition = varargin{4};
            end
        end

        % --------------------------------------------------------------- %

        function setJointAxis(obj, JointAxis)
            % setJointAxis - Set the axis of rotation/translation of the joint
            %
            % Syntax
            %   setJointAxis(char) | char in {'x', 'y', 'z'}
            %   setJointAxis(double(1, 3))
            
            if ischar(JointAxis)
                switch JointAxis
                    case 'x', JointAxis = [1, 0, 0];
                    case 'y', JointAxis = [0, 1, 0];
                    case 'z', JointAxis = [0, 0, 1];
                    otherwise, warning("JointAxis must be char in {'x', 'y', 'z'} or double(1, 3)"), return;
                end
            end

            obj.JointAxis = JointAxis; obj.JointAxis = JointAxis/norm(JointAxis);
        end
        
        % --------------------------------------------------------------- %

        function setFixedTransform(obj, data, varargin)
            % setFixedTransform - Set fixed transformation between frames
            % Set transformation between:
            %   - 'c2j': child(ren) frame w.r.t. joint frame
            %   - 'j2p': joint frame w.r.t. parent frame
            % Available descriptions:
            %   - 'A': homogeneous matrix
            %   - 'dh': Denavit-Hartenberg parameters
            %
            % Syntax
            %   setFixedTransform(data)
            %   setFixedTransform(data, frame2frame)
            %   setFixedTransform(data, frame2frame, descriptor)
            %
            % Input:
            %   data - data used for setting the transformation
            %       [belong to SE(3) | double(4,4)] or [[d, theta, a, alpha] | double(1, 4)]
            %   frame2frame - define frames of the transformation
            %       in {'c2j', 'j2p'} | default = 'c2j' | char array
            %   descriptor - type of description used to define the transformation
            %       in {'A', 'dh'} | default = 'dh' | char array
            
            if (nargin > 2) && ~isempty(varargin{1})
                if ismember(varargin{1}, {'c2j', 'j2p'}), frame2frame = varargin{1};
                else, frame2frame = 'c2j'; end
            else, frame2frame = 'c2j';
            end

            if (nargin > 3) && ~isempty(varargin{2})
                if ismember(varargin{2}, {'A', 'dh'}), descriptor = varargin{2};
                else, descriptor = 'dh'; end
            else, descriptor = 'dh';
            end

            switch descriptor
                case 'A', A = data;
                case 'dh', A = roboJoint.setDHParams(data);
            end
            switch frame2frame
                case 'j2p', obj.Parent = A;
                case 'c2j', obj.Child = A;
            end
        end

        % --------------------------------------------------------------- %

        function plot(obj, varargin)
            % plot - Plot the joint frames
            % FRame legend:
            %   - dotted line: parent frame (joint frame without joint movement)
            %   - full line: current joint frame
            %   - dashed line: children base frame
            %
            % Syntax
            %   plot()
            %   plot(jointValue)
            %   plot(jointValue, Ajoint2B_eval)
            %   plot(..., specifics)
            %
            % Input:
            %   jointValue - joint value
            %       rad or m | default = HomePosition | double
            %   Ajoint2B_eval - roto-translation of the joint w.r.t. arm base (neglet specifics variable)
            %       belong to SE(3) | double(4, 4)
            %   specifics - plot specifics
            %       in {'all', 'parent', 'joint', 'child'} | default = 'joint' | char array
            
            if (nargin > 1) && ischar(varargin{end}), mode = varargin{1}; nargin_num = 1;
            else, mode = 'joint'; nargin_num = 0; end

            if (nargin > 1 + nargin_num) && ~isempty(varargin{1})
                jointValue = varargin{1};
            else, jointValue = obj.HomePosition;
            end
            if (nargin > 2 + nargin_num) && tools.isSE3(varargin{2})
                Ajoint2B_eval = varargin{2}; mode = 'armJoint';
            end

            switch obj.Type
                case 'revolute', R = axang2rotm([obj.JointAxis, jointValue]); T = [0, 0, 0]';
                case 'prismatic', R = eye(3); T = jointValue*obj.JointAxis';
                case 'fixed', R = eye(3); T = [0, 0, 0]';
            end

            A = [R, T; 0, 0, 0, 1];

            switch mode
                case 'parent', tools.framePlot(obj.Parent, [obj.Name, '_{Parent}'], ':')
                case 'joint', tools.framePlot(obj.Parent*A, obj.Name, '-')
                case 'child', tools.framePlot(obj.Parent*A*obj.Child, [obj.Name, '_{Child}'], '--')
                case 'all'
                    tools.framePlot(obj.Parent, [obj.Name, '_{Parent}'], ':')
                    tools.framePlot(obj.Parent*A, obj.Name, '-')
                    tools.framePlot(obj.Parent*A*obj.Child, [obj.Name, '_{Child}'], '--')
                case 'armJoint', tools.framePlot(Ajoint2B_eval*A, obj.Name, '-')
                otherwise
                    warning('Wrong display char passed. Display only joint frame')
                    tools.framePlot(obj.Parent*A, obj.Name, '-')
            end
        end
        
        % --------------------------------------------------------------- %

        function copyRigidBodyJoint(obj, rigidBodyJointObj)
            % copyRigidBodyJoint - Copy a rigidBodyJoint object to a
            % roboJoint object
            %
            % Syntax
            %   copyRigidBodyJoint(rigidBodyJointObj)
            %
            % Input:
            %   rigidBodyJointObj - rigidBodyJoint object
            %       rigidBodyJoint

            if ~isa(rigidBodyJointObj, 'rigidBodyJoint')
                error("Wrong passed parameter, check syntax")
            end

            obj.Name = rigidBodyJointObj.Name;
            obj.Type = rigidBodyJointObj.Type;
            if all(~isnan(rigidBodyJointObj.JointAxis))
                obj.JointAxis = rigidBodyJointObj.JointAxis;
            end
            obj.PositionLimits = rigidBodyJointObj.PositionLimits;
            obj.HomePosition = rigidBodyJointObj.HomePosition;
            obj.Parent = rigidBodyJointObj.JointToParentTransform;
            obj.Child = rigidBodyJointObj.ChildToJointTransform;
        end

        % --------------------------------------------------------------- %

        function var = genJointAMatrix(obj, varargin)
            % genJointAMatrix - Generate joint roto-translation simbolic homogeneous matrix
            %
            % Syntax
            %   genJointAMatrix
            %   genJointAMatrix(var)
            %   genJointAMatrix(num)
            %
            % Input:
            %   var - symbolic joint variables
            %       default = casadi.SX.sym('x') | casadi.SX.sym or casadi.MX.sym
            %   num - numbered suffix to identify the joint variable
            %       default = none | double

            if (nargin > 1) && ~isempty(varargin{1})
                switch class(varargin{1})
                    case 'casadi.SX', var = varargin{1};
                    case 'casadi.MX', var = varargin{1};
                    case 'double', var = casadi.SX.sym(['x',num2str(varargin{1})]);
                    otherwise, error("Cannot perform assignment. Necessary casADi symbolic variable")
                end
            else
                var = casadi.SX.sym('x');
            end

            switch obj.Type
                case 'revolute', R = tools.axang2rotm([obj.JointAxis, var]); T = [0, 0, 0]';
                case 'prismatic', R = eye(3); T = var*obj.JointAxis';
                case 'fixed', R = eye(3); T = [0, 0, 0]'; var = [];
            end

            obj.Ajoint = [R, T; 0, 0, 0, 1];
            switch class(var)
                case 'casadi.SX', obj.Ajoint = simplify(obj.Ajoint);
                case 'casadi.MX', obj.Ajoint = simplify(obj.Ajoint);
            end
        end

    end

    
    % ------------------------------------------------------------------- %


    methods (Static = true)
        
        function A = setDHParams(DHParams, varargin)
            % setDHParams - Use Denavit-Hartenberg params representation to
            % generate the homogeneous matrix
            %
            % Syntax
            %   setDHParam(DHParams)
            %   setDHParam(theta, d, a, alpha)
            %
            % Input:
            %   DHParams - Denavit-Hartenberg params representation ([d, theta, a, alpha])
            %       [m, rad, m, rad] | double(1, 4)
            %   d - translation along z-axis
            %       m | double
            %   theta - rotation around z-axis
            %       rad | double
            %   a - translation along x-axis
            %       m | double
            %   alpha - rotation around x-axis
            %       rad | double
            %
            % Output:
            %   A - homogeneous matrix
            %       belong to SE(3) | double(4, 4)
            
            if (nargin > 3) && ~isempty(DHParams) && all(~isempty(varargin{:}))
                d = DHParams; theta = varargin{1}; a = varargin{2}; alpha = varargin{3};
            else, d = DHParams(1); theta = DHParams(2); a = DHParams(3); alpha = DHParams(4);
            end

            A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta);
                 sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                     0,              sin(alpha),              cos(alpha),          d;
                     0,                  0,                      0,                1];

        end

    end

end


% --- Vaidating function --- %
function mustBeSE3(data)
    % mustBeSE3 - Validate that data is in the SE(3) group
    R = data(1:3, 1:3);    
    if (~all(data(4, :) == [0, 0, 0, 1])) || ~(max(abs(R.'*R - eye(3)), [], 'all') < 1e-5) || ~(abs(det(R) - 1) < 1e-5)
        error("Value of property must be a matrix belonging to SE(3) group")
    end
end

function mustBeAxis(data)
    % mustBeAxis - Validate that data is an admissible axis
    if ~norm(data), error("Value of property must have norm greater than 0"), end
end