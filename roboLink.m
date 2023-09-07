classdef roboLink < handle_light
    % ROBOLINK - Robotic arm manipulator link
    %   Generate robotic arm link object
    %
    % robolink Properties:
    %   Name - Link name
    %   Parent - index 
    %
    % roboLink Methods:
    %   roboLink        - Constructor
    %   copyRigidBody   - Copy a rigidBody object to a roboLink object
    %   plot            - Wrapper of the roboJoint.plot function
    %   genJointAMatrix - Wrapper of the roboJoint.genJointAMatrix function
    %
    % See also roboJoint.plot, roboJoint.genJointAMatrix

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

    properties (SetAccess = {?roboArm})
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
        function obj = roboLink(varargin)
            % roboLink - Class constructor
            %
            % Syntax
            %   roboLink(rigidBodyObj)
            %   roboLink(Name)
            %   roboLink(Name, Joint)
            %
            % Input:
            %   rigidBodyObj - rigidBody object
            %       rigidBody
            % See also NAME, JOINT

            tools.addCasadiToPath

            if (nargin == 0) || (nargin > 2)
                error("Wrong number of parameters passed, check constructor syntax")
            end

            if (nargin > 0) && (~isempty(varargin{1}))
                switch class(varargin{1})
                    case 'rigidBody', obj.copyRigidBody(varargin{1})
                    case 'char', obj.Name = varargin{1};
                    otherwise, error("Wrong passed parameter, check constructor syntax")
                end
            end

            if (nargin > 1) && (~isempty(varargin{2}))
                obj.Joint = varargin{2};
            end
        end

        % --------------------------------------------------------------- %

        function copyRigidBody(obj, rigidBodyObj)
            % copyRigidJoint - Copy a rigidBody object to a roboLink object
            %
            % Syntax
            %   copyRigidBody(rigidBodyObj)
            %
            % Input:
            %   rigidBodyObj - rigidBody object
            %       rigidBody

            if ~isa(rigidBodyObj, 'rigidBody')
                error("Wrong passed parameter, check syntax")
            end

            obj.Name = rigidBodyObj.Name;
            obj.Joint = roboJoint(rigidBodyObj.Joint);
        end

        % --------------------------------------------------------------- %

        function plot(obj, varargin)
            % plot - Plot the joint frames
            % If all the frames are requested:
            %   - dotted line: parent frame (joint frame without joint movement)
            %   - full line: current joint frame
            %   - dashed line: children base frame
            %
            % Syntax
            %   plot()
            %   plot(jointValue)
            %   plot(..., char array) | char array in {'all', 'joint'} (default = 'joint')
            %
            % Input:
            %   jointValue - joint value
            %       rad or m | default = HomePosition | double
            
            obj.Joint.plot(varargin{:})
        end

        % --------------------------------------------------------------- %

        function genJointAMatrix(obj, varargin)
            % genJointAMatrix - Generate joint roto-translation simbolic homogeneous matrix
            %
            % Syntax
            %   genJointAMatrix
            %   genJointAMatrix(var)
            %
            % Input:
            %   var - Symbolic joint variables. If omitted, defined as 'x'
            %       casadi.SX.sym or casadi.MX.sym
            
            obj.Joint.genJointAMatrix(varargin{:})
        end
    end

end