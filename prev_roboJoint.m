classdef roboJoint < handle_light
    % ROBOJOINT - Robotic arm manipulator joint
    %   Generate robotic arm joint object
    
    properties
        Type            (1, :)          {mustBeMember(Type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'
        Name            (1, :)  char
        PositionLimits  (1, 2)  double  {mustBeNumeric, mustBeVector} = [0, 0]
        HomePosition    (1, 1)  double  {mustBeNumeric}
        JointAxis       (1, 3)  double  {mustBeReal} = [nan, nan, nan]
    end

    properties (SetAccess = protected)
        Parent                  cell = {};
        Child                   cell = {};
    end

    methods
        % --- Constructors --- %
        function obj = roboJoint(Name, varargin)
            % roboJoint - Class constructor
            % Syntax
            %   roboJoint(Name)
            %
            %   roboJoint(Name, Type)
            %   roboJoint(Name, Type, PositionLimits)
            %   roboJoint(Name, Type, PositionLimits, HomePosition)
            %   roboJoint(Name, Type, PositionLimits, HomePosition, JointAxis)
            %
            % Input
            %   Name - joint name
            %       char
            %   Type - type of joint 
            %       in {'revolute', 'prismatic', 'fixed'} | char
            %   PositionLimits - joint range 
            %       defualt = [0, 0] | double(1, 2)
            %   HomePosition - home position
            %       double
            %   JointAxis - joint axis of rotation 
            %       if set Type default = [0, 0, 1] | double(1, 3)
            
            obj.Name = Name;
            if (nargin > 1) && (~isempty(varargin{1}))
                obj.Type = varargin{1}; obj.JointAxis = [0, 0, 1];
            end
            
            if (nargin > 2) && (~isempty(varargin{2}))
                obj.PositionLimits = varargin{2};
                if obj.PositionLimits(1) > obj.PositionLimits(2), obj.PositionLimits = flip(obj.PositionLimits); end
            end

            if nargin > 3
                if (~isempty(varargin{3})) && (obj.PositionLimits(1) <= varargin{3}) && ...
                        (obj.PositionLimits(2) >= varargin{3})
                obj.HomePosition = varargin{3};
                else, error('Home position not comprehended in the position limits')
                end
            end
            
            if nargin > 4
                if ~norm(varargin{4}), warning('Wrong rotation axis format, check value'), return, end
                obj.JointAxis = varargin{4};
                obj.JointAxis = obj.JointAxis./norm(obj.JointAxis);
            end
        end


        % --- Set position w.r.t. previous body --- %
        function obj = setJointFrame(obj, data, varargin)
            % setJointFrame - Set joint position and orientation w.r.t.
            % parent's position and orientation (parent frame)
            % Syntax
            %   setJointFrame(tform)
            %   setJointFrame(dhparams, 'dh')
            %
            % Input
            %   tform - homogeneous transformation
            %       double(4, 4) | se3
            %   dhparams - Denavit-Hartenberg parameters [a, alpha, d, theta]
            %       double(1, 4)

            if nargin < 2, error('Wrong number of passed parameters'), end
            if isempty(varargin)
                % Homogeneous transformation
                if any(size(data) ~= [4, 4]), error('Wrong homogeneous transformation matrix dimension'), end
                if (abs(det(data(1:3, 1:3)) - 1) > 1e-3) || (any(data(4, 1:4) ~= [0, 0, 0, 1]))
                    error('Homogeneous transformation not corresponding to any roto-translation')
                end
                
                obj.Parent = {data}; return
            end

            if length(varargin) > 1, error('Wrong number of passed parameters'), end

            switch varargin{1}
                case 'dh'
                    % Denavit-Hartenberg representation
                    if length(data) ~= 4, error('Wrong number of parameter passed to DH representation'), end
                    obj.Parent = ...
                        {[cos(data(4)), -sin(data(4))*cos(data(2)), sin(data(4))*sin(data(2)), data(1)*cos(data(4));
                        sin(data(4)), cos(data(4))*cos(data(2)), -cos(data(4))*sin(data(2)),  data(1)*sin(data(4));
                             0,             -sin(data(2)),                cos(data(2)),          data(3);
                             0,                  0,                           0,                    1]};
                otherwise
                    error('Wrong type of parameters settings, check available syntax')
            end

        end
    end
end

