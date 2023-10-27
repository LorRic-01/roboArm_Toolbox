classdef roboLink < handle_light
    % ROBOLINK Robotic arm manipulator link
    %   Generate robotic arm link object
    %
    % roboLink Properties:
    %   Name   - Link name
    %   Mass   - Link mass
    %   CoM    - Center of mass, expressed w.r.t. fixed joint frame
    %   I      - Inertia matrix w.r.t. link's CoM
    %   Parent - Index of link which the current link is attached to
    %   Child  - Index of link(s) attache to the current link
    %
    % roboLink Methods:
    %   roboLink           - Class constructor
    %   setDynParams       - Set link dyamic parameters
    %   addVisual          - Add the visual geometry
    %   plot               - Plot link and joint frames
    %   copyRigidBody      - Copy rigidBody object in a roboLink object

    properties
        % Name - Link name
        %   char array or string
        Name {mustBeTextScalar, mustBeNonempty} = ' '
        
        % Joint - roboJoint object associated to the link
        %   default = roboJoint([Name, '_jnt']) | roboJoint
        Joint {mustBeAorEmpty(Joint, {'roboJoint'})}

        % Mass - Link mass
        %   kg | default = 0 | double(1, 1)
        Mass {mustBeSizeVector(Mass, [1, 1])} = 0
        
        % CoM - Center of mass, expressed w.r.t. fixed joint frame
        %   [m, m, m] | default = [0, 0, 0] | double(1, 3)
        CoM {mustBeSizeVector(CoM, [1, 3])} = zeros(1, 3)

        % I - Inertia matrix w.r.t. link's CoM
        %   kg m^2 (3x3) | default = eye(3, 3) | double(3, 3)
        I {mustBeInertiaorZero} = zeros(3, 3)
    end

    % ------------------------------------------------------------------- %

    properties (SetAccess = private)
        % Parent - Index of link which the current link is attached to
        %   default = [] | double(1, 1)
        Parent {mustBeInteger, mustBeScalarOrEmpty} = []

        % Child - Index of link(s) attache to the current link
        %   default = [] | double(1, :)
        Child {mustBeInteger} = zeros(1, 0)

        % Visual_j2p - Visual geometry of the link specified as pde.DiscreteGeometry (j2p link)
        %   pde.DiscreteGeometry
        Visual_j2p {mustBeAorEmpty(Visual_j2p, 'pde.DiscreteGeometry')} = [];

        % Visual_c2j - Visual geometry of the link specified as pde.DiscreteGeometry (c2j link)
        %   pde.DiscreteGeometry
        Visual_c2j {mustBeAorEmpty(Visual_c2j, 'pde.DiscreteGeometry')} = [];
    end

    % ------------------------------------------------------------------- %
    
    methods
        % --- Constructor --- %
        function obj = roboLink(Name, Joint)
            % roboLink - Class constructor
            % 
            % Syntax
            %   roboLink(Name)
            %   roboLink(Name, Joint)
            %
            %   roboLink(rigidBodyObj)
            %
            % Input:
            %   rigidBodyObj - rigidBody object
            %       rigidBody
            %
            % See also NAME, JOINT

            arguments
                Name {mustBeA(Name, {'char', 'string', 'rigidBody'}), mustBeNonempty}
                Joint {mustBeA(Joint, {'roboJoint'})} = roboJoint.empty
            end

            % Add casadi to path
            try
                tools.addCasadiToPath
            catch E
                if ismember(E.identifier, {'tools:MissingFolder', 'tools:pathError'}) && ...
                        strcmp(tools.tim.Running, 'off')
                    tools.tim.start
                    warning('off', 'backtrace'), warning(getReport(E, 'extended')) % basic
                    warning('Future computation might be affected if not corrected manually');
                    warning('on', 'backtrace')
                end
                if ~ismember(E.identifier, {'tools:MissingFolder', 'tools:pathError'}), throw(E), end
            end

            % Object creation
            if isa(Name, 'rigidBody'), obj = roboLink.copyRigidBody(Name);
            else
                obj.Name = Name;
                if ~isempty(Joint), obj.Joint = Joint;
                else, obj.Joint = roboJoint([Name, '_jnt']); end
            end
        end
        
        % --------------------------------------------------------------- %

        function setDynParams(obj, parameters, values)
            % setDynParams - Set link dyamic parameters such as mass, CoM and
            % inertia matrix
            %
            % Syntax
            %   addDynParams(parameter1, value1, parameter2, value2, ...)
            %
            % Input:
            %   parameter - Link dynamic parameter
            %       in {'Mass', 'CoM', 'I'} | char array or string
            %   value - Link dynamic parameter value
            %       double(:, :)
            %
            % See also MASS, COM, I

            arguments
                obj roboLink
            end
            arguments (Repeating)
                parameters {mustBeMember(parameters, {'Mass', 'CoM', 'I'})}
                values {mustBeNumeric}
            end           

            for k = 1:numel(parameters)
                switch parameters{k}
                    case 'Mass', obj.Mass = values{k};
                    case 'CoM', obj.CoM = values{k};
                    case 'I', obj.I = values{k};
                end
            end

        end

        % --------------------------------------------------------------- %

        function addVisual(obj, type, data, frames, parameters, values)
            % addVisual - Add the visual geometry
            % Predefined geometries have starting point in the link's fixed joint center
            % (customizable)
            %
            % Syntax
            %   addVisual(type, data)
            %   addVisual()

            arguments
                obj roboLink
                type {mustBeMember(type, {'empty', 'box', 'cyl'})} = 'box'
                data {mustBeNumericTextScalarorMember(data, {'auto'})} = 'auto'
                frames {mustBeMember(frames, {'j2p', 'c2j', 'both'})} = 'both'
            end
            arguments (Repeating)
                parameters
                values
            end

            if strcmp(type, 'empty'), obj.Visual_j2p = []; end
            if strcmp(frames, 'both'), frames = {tools.invA(obj.Joint.j2p), obj.Joint.c2j};
            else
                switch frames
                    case 'j2p', frames = {tools.invA(obj.Joint.j2p)}; % p2j 
                    case 'c2j', frames = {obj.Joint.c2j};
                end
            end

            for k = 1:length(frames)
                A = eye(4);
                if strcmp(data, 'auto')
                    l = norm(frames(1:3, 4));
                    A(1:3, 1:3) = tools.vrrotvec([0, 0, 1], frames(1:3, 4));
                    switch type
                        case 'box',
                        case 'cyl', 
                        case 'mesh', 
                    end

            end            

        end

        % --------------------------------------------------------------- %

        function plot(obj, jointValue, Ajoint2B, parameters, values)
            % plot - Plot link and joint frames
            % Frame legend
            %   - dotted line: joint frame without movement ('fixed')
            %   - full line: joint frame with movement ('moved')
            %   - dashed line: child(ren) base frame ('child')
            %
            % Syntax
            %   plot
            %   plot(jointValue)
            %   plot(jointValue, Ajoint2B)
            %   plot(jointValue, Ajoint2B, parameter1, value1, parameter2, value2, ...)
            %
            % Input:
            %   jointValue - Joint value
            %       rad or m | default = HomePosition | double
            %   Ajoint2B - Joint parent frame w.r.t. base frame
            %       belong to SE(3) | default = eye(4) | double(4, 4)
            %   parameters - Frame(s) to show
            %       in {'Joint', 'CoM'} | char array or string
            %   values - Frame(s) to show value
            %       'Joint' -> see roboJoint.plot specifics characteristic | default = {'fixed', 'child'}
            %       'CoM' -> in {'on', 'off'} | default = 'on'
            %
            % See also roboJoint.plot

            arguments
                obj roboLink
                jointValue {mustBeSizeVector(jointValue, [1, 1])} = obj.Joint.HomePosition
                Ajoint2B {mustBeSE3} = obj.Joint.j2p
            end
            arguments (Repeating)
                parameters {mustBeMember(parameters, {'Joint', 'CoM'})}
                values {mustBeMember(values, {'all', 'fixed', 'moved', 'child', 'on', 'off'})}
            end

            % Update frame specs
            jointSpecs = {'fixed', 'child'}; CoMSpecs = 'on';
            if ~isempty(parameters)
                for k = 1:length(parameters)
                    switch parameters{k}
                        case 'Joint', jointSpecs = values{k};
                        case 'CoM', mustBeTextScalar(values{k}), CoMSpecs = values{k};
                    end
                end
            end

            % Plot frames
            Ajoint2B_m = obj.Joint.plot(jointValue, Ajoint2B, jointSpecs);
            
            % Plot visual
            if ~isempty(obj.Visual_j2p)
                tmp_Visual = obj.Visual_j2p; axang = rotm2axang(Ajoint2B_m(1:3, 1:3));
                tmp_Visual = translate(tmp_Visual, Ajoint2B_m(1:3, 4));
                tmp_Visual = rotate(tmp_Visual, rad2deg(axang(4)), Ajoint2B_m(1:3, 4), Ajoint2B_m(1:3, 4)' + axang(1:3));
                hold on, pdegplot(tmp_Visual, 'FaceAlpha', 0.1, 'Lighting', 'off'); hold off
            end

            % Plot CoM
            if strcmp(CoMSpecs, 'on')
                A_CoM = Ajoint2B_m*[eye(3), obj.CoM'; 0, 0, 0, 1];
                tools.plotCoM(A_CoM(1:3, 4));
            end
        end
    end

    % ------------------------------------------------------------------- %

    methods (Static)
        function roboLinkObj = copyRigidBody(rigidBodyObj)
            % copyRigidBodyJoint - Copy rigidBody object in a roboLink object
            %
            % Syntax
            %   copyRigidBody(rigidBodyObj)
            %
            % Input:
            %   rigidBodyObj - rigidBody object
            %       rigidBody
            %
            % Output:
            %   roboLinkObj - roboLink object
            %       roboLink
            
            arguments
                rigidBodyObj {mustBeA(rigidBodyObj, 'rigidBody')}
            end

            roboLinkObj = roboLink(rigidBodyObj.Name, ...
                roboJoint.copyRigidBodyJoint(rigidBodyObj.Joint));
            roboLinkObj.Mass = rigidBodyObj.Mass;
            roboLinkObj.CoM = rigidBodyObj.CenterOfMass;
            roboLinkObj.I = diag(rigidBodyObj.Inertia(1:3)) + ...
                squareform(flip(rigidBodyObj.Inertia(4:6)));
        end
    end
end

% --- Validating function --- %
function mustBeSizeVector(data, vec_size)
    % mustBeSizeVector - Validate that data is a vector with specific size
    mustBeNumeric(data)
    if isequal(size(data), vec_size), return
    else, throw(MException('roboLink:WrongDimension', 'Must have size [%s]', num2str(vec_size)));
    end
end

function mustBeAorEmpty(data, className)
    % mustBeAorEmpty - Validate that class of data is inside the admissible class name or empty
    if isempty(data), return, end
    mustBeA(data, className);
end

function mustBeNumericTextScalarorMember(data, Names)
    % mustBeNumericTextScalarorMember - Validate that data is numeric, 
    % text scalar or is inside the admissible char array or string
    error = true;
    try mustBeTextScalar(data), error = false; catch, end
    try mustBeNumeric(data), error = false; catch, end
    try mustBeMember(data, Names), error = false; catch, end

    if error
        Names = strcat(Names, '''');
        Names = strcat('''', Names);
        throw(MException('roboLink:WrongValue', ['Must be text scalar, numeric data ' ...
            'or member of {%s}'], strjoin(Names, ', ')));
    end
end

function mustBeNumericorTextScalar(data)
    % mustBeNumericTextScalar - Validate that data is numeric or text scalar
    error = true;
    try mustBeTextScalar(data), error = false; catch, end
    try mustBeNumeric(data), error = false; catch, end

    if error
        throw(MException('roboLink:WrongValue', 'Must be text scalar or numeric data'));
    end
end

function mustBeSE3(data)
    % mustBESE3 - Validate that data is in the SE(3) group
    mustBeSizeVector(data, [4, 4])
    R = data(1:3, 1:3);
    if (~all(data(4, :) == [0, 0, 0, 1]) || ~(max(abs(R.'*R - eye(3)), [], 'all') < 1e-5) ...
            || ~(abs(det(R) - 1) < 1e-5))
        throw(MException('roboLink:WrongValue', 'Must belong to SE(3) group'));
    end
end

function mustBeInertiaorZero(data)
    % mustBeInertiaorZero - Validate that data is an inertia matrix or zero matrix
    mustBeSizeVector(data, [3, 3])
    if (all(abs(data) < 1e-5, 'all')), return, end
    if (~all((data - data.') < 1e-3, 'all') || (det(data) <= 0))
        throw(MException('roboLink:WrongValue', 'Must be a positive definite symmetric matrix'));
    end
end