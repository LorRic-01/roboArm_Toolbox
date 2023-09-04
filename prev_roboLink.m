classdef roboLink
    % ROBOLINK - Robotic arm manipulator link
    %   Generate robotic arm link object used to generate roboTree objects
    %   It is possible to associate to each roboLink object multiple
    %   roboJoint in order to compose complex structures

    properties
        Name            (1, :)  char
        Joint
    end

    properties (SetAccess = protected)
        Parent                  cell = {};
        Child                   cell = {};
    end

    methods
        % --- Constructors --- %
        function obj = roboLink(Name, varargin)
            % roboLink - Class constructor
            % Syntax
            %   roboLink(Name)
            %
            % Input
            %   Name - joint name
            %       char

            obj.Name = Name;
        end

        
    end
end

