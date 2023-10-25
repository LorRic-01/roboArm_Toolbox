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
        I {mustBeInertia} = eye(3, 3)
    end

    % ------------------------------------------------------------------- %

    properties (SetAccess = private)
        % Parent - Index of link which the current link is attached to
        %   default = [] | double(1, 1)
        Parent {mustBeInteger, mustBeScalarOrEmpty} = []

        % Child - Index of link(s) attache to the current link
        %   default = [] | double(1, :)
        Child {mustBeInteger} = zeros(1, 0)

        % Visual - Visual geometry of the link specified as pde.DiscreteGeometry
        %   pde.DiscreteGeometry
        Visual {mustBeAorEmpty(Visual, 'pde.DiscreteGeometry')}
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

        function addVisual(obj, type, data, parameters, values)
            % addVisual - Add the visual geometry
            % Predefined geometries have  in the link's fixed joint center
            % (customizable)
            % 
            % Syntax
            %   addVisual(type, data)
            %   addVisual(type, data, parameter1, value1, parameter2, value2, ...)
            %
            % Input:
            %   type - Type of geometry
            %       in {'box', 'cylinder', 'sphere', 'mesh'} | default = 'box' | char array or string
            %   data - Visual data for each 'type' object
            %       'box'       -> [x, y, z]
            %       'cylinder'  -> [radius, length]
            %       'sphere'    -> radius
            %       'mesh'      -> pde.DiscreteGeometry or STL file name as char array or string
            %       all         -> 'auto': compute automatically dimensions
            %       default = auto
            %   parameters - Specific parameters for modifying the element
            %       in {'scale', 'rot', 'tra'} | char array or string
            %   values - Specific parameters values
            %       'scale' -> double(1, 1) > 0
            %       'rot'   -> axang representation [ax_x, ax_y, ax_z, alpha] | doule(1, 4)
            %       'tra'   -> double(1, 3)

            arguments
                obj roboLink
                type {mustBeMember(type, {'box', 'cylinder', 'sphere', 'mesh', 'empty'})} = 'box'
                data {mustBeNumericTextScalarorMember(data, {'auto'})} = 'auto'
            end
            arguments (Repeating)
                parameters {mustBeMember(parameters, {'scale', 'rot', 'tra'})}
                values {mustBeNumeric}
            end

            if strcmp(type, 'empty'), obj.Visual = []; return, end

            % Compute parameters
            Rot = eye(3);
            if strcmp('auto', data)
                l = norm(obj.Joint.c2j(1:3, 4)); Rot = tools.vrrotvec([0, 0, 1], obj.Joint.c2j(1:3, 4));
                switch type
                    case 'box', data = [l/10, l/10, l];
                    case 'cylinder', data = [l/10, l];
                    case 'sphere', data = l;
                end
            end

            % Import visual
            switch type
                case 'box', mustBeSizeVector(data, [1, 3])
                    obj.Visual = multicuboid(data(1), data(2), data(3));
                    obj.Visual = rotate(obj.Visual, rad2deg(Rot(4)), [0, 0, 0], Rot(1:3));
                case 'cylinder', mustBeSizeVector(data, [1, 2]),
                    obj.Visual = multicylinder(data(1), data(2));
                    obj.Visual = rotate(obj.Visual, rad2deg(Rot(4)), [0, 0, 0], Rot(1:3));
                case 'sphere', mustBeSizeVector(data, [1, 1]),
                    obj.Visual = multisphere(data(1));
                case 'mesh', mustBeTextScalar(data)
                    if isa(data, 'pde.DiscreteGeometry'), obj.Visual = data;
                    else, obj.Visual = importGeometry(data);
                    end
            end

            % Modify visual
            if ~isempty(parameters)
                for k = 1:length(parameters)
                    switch parameters{k}
                        case 'scale', mustBeSizeVector(values{k}, [1, 1]), mustBeGreaterThan(values{k}, 0)
                            obj.Visual = scale(obj.Visual, values{k});
                        case 'rot', mustBeSizeVector(values{k}, [1, 4])
                            obj.Visual = rotate(obj.Visual, rad2deg(values{k}(4)), values{k}(1:3));
                        case 'tra', mustBeSizeVector(values{k}, [1, 3])
                            obj.Visual = translate(obj.Visual, values{k});
                    end
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
            if ~isempty(obj.Visual)
                tmp_Visual = obj.Visual; axang = rotm2axang(Ajoint2B_m(1:3, 1:3));
                tmp_Visual = translate(tmp_Visual, Ajoint2B_m(1:3, 4));
                tmp_Visual = rotate(tmp_Visual, rad2deg(axang(4)), Ajoint2B_m(1:3, 4), Ajoint2B_m(1:3, 4)' + axang(1:3));
                hold on, pdegplot(tmp_Visual, 'FaceAlpha', 0.1, 'Lighting', 'off'); hold off
            end

            % Plot CoM
            if strcmp(CoMSpecs, 'on'), tools.plotCoM(obj.CoM + Ajoint2B_m(1:3, 4)'); end % missing rotation of the element
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
    % mustBeNumericorMember - Validate that data is numeric, text scalar or
    % is inside the admissible char array or string
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

function mustBeSE3(data)
    % mustBESE3 - Validate that data is in the SE(3) group
    mustBeSizeVector(data, [4, 4])
    R = data(1:3, 1:3);
    if (~all(data(4, :) == [0, 0, 0, 1]) || ~(max(abs(R.'*R - eye(3)), [], 'all') < 1e-5) ...
            || ~(abs(det(R) - 1) < 1e-5))
        throw(MException('roboLink:WrongValue', 'Must belong to SE(3) group'));
    end
end

function mustBeInertia(data)
    % mustBeInertia - Validate that data is an inertia matrix
    mustBeSizeVector(data, [3, 3])
    if (~all((data - data.') < 1e-3, 'all') || (det(data) <= 0))
        throw(MException('roboLink:WrongValue', 'Must be a positive definite symmetric matrix'));
    end
end