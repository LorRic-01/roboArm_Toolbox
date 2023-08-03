classdef roboJoint < handle_light
%ROBOJOINT - Robotic arm manipulator joint
%   Generate robotic arm joint object
    
    properties
        Type            (1, :)          {mustBeMember(Type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'
        Name            (1, :)  char
        PositionLimits  (1, 2)  double  {mustBeNumeric, mustBeVector} = [0, 0]
        HomePosition    (1, 1)  double  {mustBeNumeric}
        JointAxis       (1, 3)  double  {mustBeNumeric} = [nan, nan, nan] % 'z' axis
        Parent                  cell = {};
        Child                   cell = {};
    end

    methods
        % --- Constructors --- %
        function obj = roboJoint(varargin)
            if nargin > 0, obj.Name = varargin{1}; end
            if nargin > 1, obj.Type = varargin{2}; end
            if nargin > 2, obj.PositionLimits = varargin{3}; end
            if nargin > 3
                mustBeInRange(varargin{4}, obj.PositionLimits(1), obj.PositionLimits(2))
                obj.HomePosition = varargin{4};
            end
            if nargin > 4, obj.Type = varargin{5}; end
        end
    end
end

