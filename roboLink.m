classdef roboLink < handle_light
    % ROBOLINK - Robotic arm manipulator link
    %   Generate robotic arm link object
    %
    % robolink Properties:
    %   Name - Link name
    %   Parent - index 
    %
    % roboLink Methods:
    %   roboLink - Constructor

    properties
        % Name - Link name
        %   char array
        Name (1, :) char

        % Joint - Joint object associated to the link
        %   roboJoint
        Joint roboJoint

% Mass
% CoM
% Inertia
    end

    % ------------------------------------------------------------------- %

    properties (SetAccess = protected)
        % Parent - index of the link's parent in the roboArm object
        %   default = [] | double
        Parent double = []
        
        % Child - index(ces) of the link's child(ren) in the roboArm object
        %   default = [] | double(1, :)
        Child (1, :) double {mustBeVector} = []
    end

    % ------------------------------------------------------------------- %

    methods
        % --- Constructor --- %
        function obj = roboLink(Name, varargin)
            % roboLink - Class constructor
            %
            % Syntax
            %   roboLink(Name)
            %   roboLink(Name, Joint)
            %
            % See also NAME, JOINT

            obj.Name = Name;
            if nargin > 1, obj.Joint = varargin{1}; end
        end
    end

end