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
        BodiesName {mustBeCharCell} = {}

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

            if nargin > 0, obj.BodiesName = {varargin{1}};
            else, obj.BodiesName = {'world'}; end

            baseJoint = roboJoint(obj.BodiesName{1}, 'fixed');
            obj.Bodies = {roboLink(obj.BodiesName{1}, baseJoint)};
        end

        % --------------------------------------------------------------- %

        % function addBody(obj, roboLinkObj, )
        % 
        % end
    end

end

% --- Vaidating function --- %
function mustBeRoboLinkCell(data)
    % mustBeRoboLinkCell - Validate that values is a roboLink object cell array
    if isempty(data), return, end
    for k = 1:length(data)
        if ~isa(data{k}, 'roboLink')
            error("Value of property must be a cell array of roboLink object")
        end
    end
end

function mustBeCharCell(data)
    % mustBeCharCell - Vlaidate that values is a char cell array
    if isempty(data), return, end
    for k = 1:length(data)
        if ~isa(data{k}, 'char')
            error("Value of property must be a char cell array")
        end
    end
end
