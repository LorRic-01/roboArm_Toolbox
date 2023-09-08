classdef roboArm < handle_light
    % ROBOARM - Robotic arm manipulator
    %   Generate robotic arm manipulator object and models
    %
    % roboArm Properties:
    %   Gravity     - Gravitational acceleration experienced by the robot
    %   Bodies      - List of roboLink objects composing the manipulator
    %   BodiesName  - List of roboLink names
    %
    % roboArm Methods:
    %   roboArm - Constructor
    %   


    properties
        % Gravity - Gravitational acceleration experienced by the robot
        %   m/s^2 | default = [0, 0, 9.8067] | double(1, 3)
        Gravity (1, 3) double = [0, 0, 9.8067]
    end


    % ------------------------------------------------------------------- %


    properties (SetAccess = protected)
        % Bodies - List of roboLink objects composing the manipulator
        %   default = {wordFixedFictitiousLink} | {roboLink, ...}
        Bodies {mustBeRoboLinkCell(Bodies)} = {}

        % BodiesName - List of roboLink names
        %   default = {'world'} | {char(1, :), ...}
        BodiesName {mustBeCharCell(BodiesName), mustBeUnique(BodiesName)} = {}
    end


    % ------------------------------------------------------------------- %


    properties (SetAccess = protected, Hidden = true)
        % casadiVars - casADi variables used in the robot descriptions
        %   default = [] | casadi.SX or casadi.MX
        casadiVars = [];
    end


    % ------------------------------------------------------------------- %


    
    methods
        % --- Constructor --- %
        function obj = roboArm(varargin)
            % roboArm - Class constructor
            %
            % Syntax
            %   roboArm
            %   roboArm(baseName)
            %
            % Input:
            %   baseName - Base name
            %       char(1, :)

            tools.addCasadiToPath

            if nargin > 0, obj.BodiesName = varargin(1);
            else, obj.BodiesName = {'world'}; end

            baseJoint = roboJoint(obj.BodiesName{1}, 'fixed');
            baseJoint.genJointAMatrix,
            baseLink = roboLink(obj.BodiesName{1}, baseJoint); baseLink.Parent = 0;
            obj.Bodies = {baseLink};
        end

        % --------------------------------------------------------------- %

        function addBody(obj, roboLinkObj, varargin)
            % addBody - Add the roboLinkObj as child of the already
            % existing-in-the-arm roboLink object. If no further parameters
            % are passed the parent object is the base (1st element).
            %
            % Syntax
            %   addBody(roboLinkObj)
            %   addBody(roboLinkObj, parentName)
            %   addBody(roboLinkObj, parentIndex)
            
            obj.BodiesName{end + 1} = roboLinkObj.Name;

            index = [];
            if (nargin > 2) && ~isempty(varargin{1})
                if isnumeric(varargin{1}), index = varargin{1}; end
                if ischar(varargin{1}), [~, index] = ismember(varargin{1}, obj.BodiesName); end
            end

            if isempty(index) || isnan(index) || (index < 0)
                index = 1;
            end
            
            roboLinkObj.Parent = index;
            obj.Bodies{index}.Child(end + 1) = length(obj.Bodies) + 1;
            obj.Bodies{end + 1} = roboLinkObj;

            % Generate joint 2 base frame transformation
            roboLinkObj.genJointAMatrix
            index = roboLinkObj.Parent; A = roboLinkObj.Joint.Parent;
            while (index ~= 0)
                A = obj.Bodies{index}.Joint.Parent * obj.Bodies{index}.Joint.Ajoint * obj.Bodies{index}.Joint.Child * A;
                index = obj.Bodies{index}.Parent;
            end
            roboLinkObj.Joint.Ajoint2B = A;
        end

        % --------------------------------------------------------------- %

        function getTransform(obj, varargin)
            % getTransform - Get the roto-translation homogeneous matrix
        end
    end

end

% --- Vaidating function --- %
function mustBeRoboLinkCell(data)
    % mustBeRoboLinkCell - Validate that data is a roboLink object cell array
    if isempty(data), return, end
    for k = 1:length(data)
        if ~isa(data{k}, 'roboLink')
            error("Value of property must be a cell array of roboLink object")
        end
    end
end

function mustBeCharCell(data)
    % mustBeCharCell - Vaidate that data is a char cell array
    if isempty(data), return, end
    for k = 1:length(data)
        if ~isa(data{k}, 'char')
            error("Value of property must be a char cell array")
        end
    end
end

function mustBeUnique(data)
    % mustBeUnique - Validate that data is a vector of unique elements
    if isempty(data), return, end
    if numel(data) ~= numel(unique(data))
        error("Value of property must have unique elements")
    end
end