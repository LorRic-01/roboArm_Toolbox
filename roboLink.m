classdef roboLink < handle_light
    % ROBOLINK Robotic arm manipulator link
    %   Generate robotic arm link object
    %
    % roboLink Properties:
    %   
    %
    % roboLink Methods:
    %   roboLink - Class constructor
    %   setDynParams - Set link(s) dynamic parameters
    %   copyRigidBody - Copy rigidBody object in a roboLink object

    properties
        % Name - Link name
        %   char array or string
        Name {mustBeTextScalar, mustBeNonempty} = ' '

        % Joint - roboJoint object associated to the link
        %   default = roboJoint('jnt') | roboJoint
        Joint {mustBeA(Joint, 'roboJoint'), mustBeNonempty} = roboJoint('jnt')

        % Mass - Link mass ([j2p_mass; c2j_mass])
        %   kg (2X1) | default = zeros(2, 1) | double (2, 1)
        Mass {mustBeNumeric, mustHaveSize(Mass, [2, 1])} = zeros(2, 1)

        % CoM - Center of Mass position, expressed w.r.t. fixed joint frame ([j2p_CoM; c2j_CoM])
        %   m (2X3) | defualt = zeros(2, 3) | double(2, 3)
        CoM {mustBeNumeric, mustHaveSize(CoM, [2, 3])} = zeros(2, 3)

        % I - Inertia matrix expressed w.r.t. link's CoM (j2p_I (:, :, 1), c2j_I (:, :, 2))
        %   kg m^2 (3x3x2) | defualt = zeros(3, 3, 2) | double(3, 3, 2)
        I {mustOr(I, 'mustBeZero', 'mustBeInertia')} = zeros(3, 3, 2)
    end

    % ------------------------------------------------------------------- %

    properties (SetAccess = private)
        % Visual -Link's visual geometry associated to specific frame ([j2p, c2j])
        %   cell(1,2) pde.DiscreteGeometry
        Visual {mustBeCellA(Visual, 'pde.DiscreteGeometry'), ...
            mustHaveSize(Visual, [1, 2])} = {pde.DiscreteGeometry.empty, pde.DiscreteGeometry.empty}
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
                Name {mustOr(Name, 'mustBeTextScalar', {'mustBeA', 'rigidBody'}), mustBeNonempty}
                Joint {mustOr(Joint, 'mustBeNonempty', {'mustBeA', 'roboJoint'})} = roboJoint.empty
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

        function setDynParams(obj, frames, params, values)
            % setDynParams - Set link(s) dynamic parameters such as mass, CoM
            % and inertia matrix
            %
            % Syntax
            %   setDynParams(param1, value1, param2, value2)
            %
            % Input:
            %   frames - Which link the dyn params are referred to
            %       in {'j2p', 'c2j', 'both'} | default = 'c2j' | char array or string
            %   param - Link dynamic parameter
            %       in {'Mass', 'CoM', 'I'} | char array or string
            %   value - Link dynamic parameter value
            %       double(:, :)
            %
            % See also MASS, COM, I

            arguments, obj roboLink
                frames {mustBeMember(frames, {'c2j', 'j2p', 'both'})} = 'c2j'
            end
            arguments (Repeating)
                params {mustBeMember(params, {'Mass', 'CoM', 'I'})}
                values {mustBeNumeric}
            end

            switch frames
                case 'j2p', frames = 1; case 'c2j', frames = 2;
                case 'both', frames = [1, 2];
            end

            for k = 1:numel(params)
                for n = frames
                    switch params{k}
                        case 'Mass'
                            mustHaveSize(values{k}, [1, 1], 'mass')
                            obj.Mass(n) = values{k};
                        case 'CoM'
                            mustHaveSize(values{k}, [1, 3], 'CoM')
                            obj.CoM(n, :) = values{k};
                        case 'I'
                            mustHaveSize(values{k}, [3, 3], 'inertia matrix')
                            obj.I(:, :, n) = values{k};
                    end
                end
            end        
        end

        % --------------------------------------------------------------- %

        function addVisual(obj, type, data, frames)
            % addVisual - Add the visual geometry
            % 
            % Syntax
            %   addVisual(type, data)
            %
            % Input:
            %   type - Type of geometry
            %       in {'empty', 'box', 'cyl', 'mesh'} | defualt = 'box' | char array or string
            %   data - Visual data for each 'type' object
            %       box -> [l_x, l_y, l_z]
            %       cyl -> [radius, l_z]
            %       mesh -> (n x 3) matrix of vertices or STL file
            %       empty -> no visual
            %   frames 

            arguments, obj roboLink
                type {mustBeMember(type, {'empty', 'box', 'cyl', 'mesh'})} = 'box'
                data {mustOr(data, {'mustBeMember', 'auto'}, 'mustBeNumeric', 'mustBeFile')} = 'auto'
                frames {mustBeMember(frames, {'j2p', 'c2j', 'both'})}= 'c2j'
            end
            arguments (Repeating)

            end

        end
    end

    % ------------------------------------------------------------------- %

    methods (Static)
        function roboLinkObj = copyRigidBody(rigidBodyObj, frame)
            % copyRigidBody - Copy rigidBody object in a roboLink object
            %
            % Syntax
            %   copyRigidBody(rigidBodyObj)
            %
            % Input:
            %   rigidBodyObj - rigidBody object
            %       rigidBody
            %   frame - Frame in which copying the object
            %       in  {'j2p', 'c2j'} | defualt = 'c2j' | char array or string
            %
            % Output:
            %   roboLinkObj - roboLink object
            %       roboLink

            arguments (Input)
                rigidBodyObj {mustBeA(rigidBodyObj, 'rigidBody')}
                frame {mustBeMember(frame, {'j2p', 'c2j'})} = 'c2j'
            end
            arguments (Output)
                roboLinkObj {mustBeA(roboLinkObj, 'roboLink')}
            end

            switch frame, case 'j2p', frame = 1; case 'c2j', frame = 2; end

            roboLinkObj = roboLink(rigidBodyObj.Name, ...
                roboJoint.copyRigidBodyJoint(rigidBodyObj.Joint));
            roboLinkObj.Mass(frame) = rigidBodyObj.Mass;
            roboLinkObj.CoM(frame, :) = rigidBodyObj.CenterOfMass;
            roboLinkObj.I(:, :, frame) = diag(rigidBodyObj.Inertia(1:3)) + ...
                squareform(flip(rigidBodyObj.Inertia(4:end)));

            % TODO - Add visual
        end
    end
end

% --- Validating function --- %
function mustOr(data, checks)
    % mustOr - Validate that data is ... or ...
    arguments, data, end
    arguments (Repeating)
        checks
    end

    if isempty(checks), return, end

    error = true; errorReport = '';
    for k = 1:length(checks)
        try
            if ~isa(checks{k}, 'cell'), checks{k} = checks(k); end
            switch checks{k}{1}
                case 'mustBePositive', mustBePositive(data), error = false;
                case 'mustBeNonpositive', mustBeNonpositive(data), error = false;
                case 'mustBeNonnegative', mustBeNonnegative(data), error = false;
                case 'mustBeNegative', mustBeNegative(data), error = false;
                case 'mustBeFinite', mustBeFinite(data), error = false;
                case 'mustBeNonNan', mustBeNonNan(data), error = false;
                case 'mustBeNonzero', mustBeNonzero(data), error = false;
                case 'mustBeNonsparse', mustBeNonsparse(data), error = false;
                case 'mustBeSparse', mustBeSparse(data), error = false;
                case 'mustBeReal', mustBeReal(data), error = false;
                case 'mustBeInteger', mustBeInteger(data), error = false;
                case 'mustBeNonmissing', mustBeNonmissing(data), error = false;

                case 'mustBeGreaterThan', mustBeGreaterThan(data, checks{k}{2}), error = false;
                case 'mustBeLessThan', mustBeLessThan(data, checks{k}{2}), error = false;
                case 'mustBeGreaterThanOrEqual', mustBeGreaterThanOrEqual(data, checks{k}{2}), error = false;
                case 'mustBeLessThanOrEqual', mustBeLessThanOrEqual(data, checks{k}{2}), error = false;
                
                case 'mustBeA', mustBeA(data, checks{k}{2}), error = false;
                case 'mustBeNumeric', mustBeNumeric(data), error = false;
                case 'mustBeNumericOrLogical', mustBeNumericOrLogical(data), error = false;
                case 'mustBeFloat', mustBeFloat(data), error = false;
                case 'mustBeUnderlyingType', mustBeUnderlyingType(data, checks{k}{2}), error = false;

                case 'mustBeNonempty', mustBeNonempty(data), error = false;
                case 'mustBeScalarOrEmpty', mustBeScalarOrEmpty(data), error = false;
                case 'mustBeVector', mustBeVector(data), error = false;

                case 'mustBeMember', mustBeMember(data, checks{k}{2}), error = false;

                case 'mustBeFile', mustBeFile(data), error = false;
                case 'mustBeFolder', mustBeFolder(data), error = false;
                case 'mustBeNonzeroLengthText', mustBeNonzeroLengthText(data), error = false;
                case 'mustBeText', mustBeText(data), error = false;
                case 'mustBeTextScalar', mustBeTextScalar(data), error = false;

                % Custom
                case 'mustHaveSize', mustHaveSize(data, checks{k}{2}), error = false;
                case 'mustBeInertia', mustBeInertia(data), error = false;
                case 'mustBeZero', mustBeZero(data), error = false;
            end
        catch
            if numel(checks{k}) > 1
                checks{k}{2} = strcat(checks{k}{2}, ''', '); checks{k}{2} = strcat('''', checks{k}{2});
                if ~isa(checks{k}{2}, 'cell'), checks{k}{2} = checks{k}(2); end
                checks{k}{2} = strjoin(checks{k}{2}); checks{k}{2} = checks{k}{2}(1:end-1);
                stringTmp = [checks{k}{1}, '(..., {', checks{k}{2}, '}), '];
            else, stringTmp = [checks{k}{1}, ', '];
            end

            errorReport(end + (1:length(stringTmp))) = stringTmp;
        end
    end

    if error, throw(MException('roboLink:WrongValue', ['Validation not passed.' ...
            '\nValidation to pass %s'], errorReport(1:end-2))); end
end

% ---------- %

function mustHaveSize(data, sizeCheck, text)
    % mustHaveSize - Validate that data has specific size
    arguments
        data, sizeCheck, text {mustBeTextScalar} = ''
    end
    if ~isempty(text), text = [text, ' assignement']; end

    if isequal(size(data), sizeCheck), return
    else, throw(MException('roboLink:WrongDimension', [ text, ': ' ...
            'Must have size %s'], ['[', num2str(sizeCheck), '] and ' ...
            'not [', num2str(size(data)), ']']));
    end
end

% ---------- %

function mustBeInertia(data)
    % mustBeInertia - Validate that data is an inertia matrix
    mustBeNumeric(data), mustHaveSize(data(:, :, 1), [3, 3])
    for k = 1:size(data, 3)
        if ~iszero(data(:, :, k) - data(:, :, k).') || (det(data(:, :, k)) < 0)
            throw(MException('roboLink:WrongValue', 'Must be an inertia matrix (symmetric and positive)'));
        end
    end
end

% ---------- %

function mustBeZero(data)
    % mustBeInertia - Validate that data is zero matrix
    mustBeNumeric(data)
    if iszero(data), return, end
    throw(MException('roboLink:WrongValue', 'Must be zero matrix'));
end
function check = iszero(data), check = all(abs(data) < 1e-5, 'all'); end

% ---------- %

function mustBeCellA(data, check)
    % mustBeCellA - Validate that cell contains all element with specific
    % class names
    
    for k = 1:numel(data)
        try mustBeA(data{k}, check)
        catch E, throw(MException('roboLink:WrongClass', ['Cell array with different type of element.\n', E.message]));
        end
    end
end