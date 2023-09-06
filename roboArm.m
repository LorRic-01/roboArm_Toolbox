classdef roboArm < handle_light
    % ROBOARM - Robotic arm manipulator
    %   Generate robotic arm manipulator object and models
    %
    % roboArm Properties:
    %   
    %
    % roboArm Methods:
    %   

    properties
        % Bodies - list of roboLink objects composing the manipulator
        %   default = {wordFixedFictitiousLink} | {roboLink, ...}
        Bodies {mustBeRoboLinkCell(Bodies)} = {}

        % Bodies - list of roboLink objects composing the manipulator
        %   default = {'world'} | {char(1, :), ...}
        BodiesName {mustBeCharCell(BodiesName), mustBeUnique(BodiesName)} = {}

        % Gravity - Gravitational acceleration experienced by the robot
        %   m/s^2 | default = [0, 0, 9.8067] | double(1, 3)
        Gravity (1, 3) double = [0, 0, 9.8067]
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

            if nargin > 0, obj.BodiesName = varargin(1);
            else, obj.BodiesName = {'world'}; end

            baseJoint = roboJoint(obj.BodiesName{1}, 'fixed');
            obj.Bodies = {roboLink(obj.BodiesName{1}, baseJoint)};
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
                if ischar(varargin{1}), index = find(ismember(varargin{1}, obj.BodiesName)); end
            end

            if isempty(index) || isnan(index) || (index < 0)
                index = 1;
            end

            roboLinkObj.Parent = index;
            obj.Bodies{index}.Child(end + 1) = length(obj.Bodies) + 1;
            obj.Bodies{end + 1} = roboLinkObj;
        end

        % --------------------------------------------------------------- %

        function getTransform(obj, varargin)
            % getTransform 
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