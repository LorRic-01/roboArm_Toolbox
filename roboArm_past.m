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
        %   default = [wordFixedFictitiousLink] | {roboLink, ...}
        Bodies {mustBeRoboLinkArray(Bodies)} = []

        % BodiesName - List of roboLink names
        %   default = {'world'} | {char(1, :), ...}
        BodiesName {mustBeCharCell(BodiesName), mustBeUnique(BodiesName)} = {}
    end


    % ------------------------------------------------------------------- %


    properties % (SetAccess = protected, Hidden = true)
        % casadiVars - casADi variables used in the robot descriptions
        %   default = casadi.SX.sym('casadiVars', 1, 0) | casadi.SX or casadi.MX
        casadiVars

        % jointIndex - non-fixed joint index
        %   default = false | double(1, num_link)
        jointIndex = false
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
            
            obj.casadiVars = casadi.SX.sym('casadiVars', 1, 0);
            if nargin > 0, obj.BodiesName = varargin(1);
            else, obj.BodiesName = {'world'}; end

            baseJoint = roboJoint(obj.BodiesName{1}, 'fixed');
            baseJoint.genJointAMatrix;
            baseLink = roboLink(obj.BodiesName{1}, baseJoint); baseLink.Parent = 0;
            obj.Bodies = baseLink;
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
            if strcmp(roboLinkObj.Joint.Type, 'fixed'), obj.jointIndex(end + 1) = false;
            else, obj.jointIndex(end + 1) = true; end

            index = [];
            if (nargin > 2) && ~isempty(varargin{1})
                if isnumeric(varargin{1}), index = varargin{1}; end
                if ischar(varargin{1}), [~, index] = ismember(varargin{1}, obj.BodiesName); end
            end

            if isempty(index) || isnan(index) || (index < 0)
                index = 1;
            end
            
            roboLinkObj.Parent = index;
            obj.Bodies(index).Child(end + 1) = length(obj.Bodies) + 1;
            obj.Bodies(end + 1) = roboLinkObj;

            % Generate joint 2 base frame transformation
            var = roboLinkObj.genJointAMatrix(length(obj.BodiesName) - 1);
            if ~isempty(var)
                if numel(obj.casadiVars), obj.casadiVars = [obj.casadiVars, var];
                else, obj.casadiVars = var; end
            end
            index = roboLinkObj.Parent; A = roboLinkObj.Joint.Parent;
            while (index ~= 0)
                A = obj.Bodies(index).Joint.Parent * obj.Bodies(index).Joint.Ajoint * obj.Bodies(index).Joint.Child * A;
                index = obj.Bodies(index).Parent;
            end
            roboLinkObj.Joint.Ajoint2B = A;
        end

        % --------------------------------------------------------------- %

        function plot(obj, varargin)
            % plot - Plot the arm
            %
            % Syntax
            %   plot()
            %   plot(armJointValues)
            %   plot(..., specifics)
            %
            % Input:
            %   armJointValues - arm joint values
            %       Note: the joint values MUST be ordered accordingly to
            %       casadiVars property
            %       rad or m array | default = jointHomePosition | double(1, num_joint)
            %   specifics - plot specifics
            %       in {'all', 'joints', 'links', 'CoM'} | default = 'joints' | char array

            if (nargin > 1) && ischar(varargin{end}), mode = varargin{end}; nargin_num = 1;
            else, mode = 'joints'; nargin_num = 0; end
            
            armJointValues = zeros(1, length(obj.BodiesName));
            if (nargin > 1 + nargin_num) && ~isempty(varargin{1}) && (length(varargin{1}) == length(obj.casadiVars))
                armMobileJointValues = varargin{1};
                armJointValues(obj.jointIndex) = varargin{1};
            else
                for k = 1:length(armJointValues), armJointValues(k) = obj.Bodies(k).Joint.HomePosition; end
                armMobileJointValues = armJointValues(obj.jointIndex);
            end

            % Update/Generate function
            for k = 1:length(armJointValues)
                obj.Bodies(k).Joint.Ajoint2B_fun = casadi.Function('Ajoint2B_fun', ...
                    {obj.casadiVars}, {obj.Bodies(k).Joint.Ajoint2B});
            end
        

            for k = 1:length(armJointValues)
                % For each link
                switch mode
                    case 'joints'
                        obj.Bodies(k).plot(armJointValues(k), ...
                            full(obj.Bodies(k).Joint.Ajoint2B_fun(armMobileJointValues)));
                        if k > 2
                            L = full([obj.Bodies(k).Joint.Ajoint2B_fun(armMobileJointValues), obj.Bodies(k-1).Joint.Ajoint2B_fun(armMobileJointValues)]);
                            L = L(1:3, [4, 8]); hold on, plot3(L(1, :), L(2, :), L(3, :), 'LineWidth', 1, 'Color', 'k'), hold off
                        end
                end
            end

        end
    end

end

% --- Vaidating function --- %
function mustBeRoboLinkArray(data)
    % mustBeRoboLinkCell - Validate that data is a roboLink object cell array
    if isempty(data), return, end
    for k = 1:length(data)
        if ~isa(data(k), 'roboLink')
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