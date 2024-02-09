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
    %   addVisual - Add the visual geometry
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
        Mass {mustBeNumeric, mustHaveSize(Mass, [2, 1], 'Mass')} = zeros(2, 1)

        % CoM - Center of Mass position, expressed w.r.t. fixed joint frame ([j2p_CoM; c2j_CoM])
        %   m (2X3) | defualt = zeros(2, 3) | double(2, 3)
        CoM {mustBeNumeric, mustOr(CoM, {'mustHaveSize', [2, 3], 'CoM'}, 'mustBeEmpty')} = []

        % I - Inertia matrix expressed w.r.t. link's CoM (j2p_I (:, :, 1), c2j_I (:, :, 2))
        %   kg m^2 (3x3x2) or empty | defualt = zeros(3, 3, 2) | double(3, 3, 2)
        I {mustOr(I, 'mustBeZero', 'mustBeInertia', 'mustBeEmpty')} = []
    end

    % ------------------------------------------------------------------- %

    properties (SetAccess = private)
        % Visual -Link's visual geometry associated to specific frame ([j2p, c2j])
        %   cell(1,2) pde.DiscreteGeometry
        Visual {mustBeCellA(Visual, 'pde.DiscreteGeometry'), ...
            mustHaveSize(Visual, [1, 2], 'Visual')} = {pde.DiscreteGeometry.empty, pde.DiscreteGeometry.empty}
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
            %   setDynParams(frames, param1, value1, param2, value2)
            %
            % Input:
            %   frames - Which link the dyn params are referred to
            %       in {'c2j', 'j2p', 'both'} | default = 'c2j' | char array or string
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
                            mustOr(values{k}, {'mustHaveSize', [1, 1], 'Mass'}, 'mustBeEmpty')
                            obj.Mass(n) = values{k};
                        case 'CoM'
                            mustOr(values{k}, {'mustHaveSize', [1, 3], 'CoM'}, 'mustBeEmpty')
                            obj.CoM(n, :) = values{k};
                        case 'I'
                            mustOr(values{k}, {'mustHaveSize', [3, 3], 'Inertia'}, 'mustBeEmpty')
                            obj.I(:, :, n) = values{k};
                    end
                end
            end        
        end

        % --------------------------------------------------------------- %

        function cmpDynParams(obj, frames)
            % cmpDynParams - Compute link(s) dynamic parameters (CoM and I)
            % given the link(s) mass(es)
            % Uses a triangular mesh simplification of the figure, with
            % equal weights to each node and compute discrete CoM and I
            %
            % Syntax
            %   cmpDynParams
            %   cmpDynParams(frames)
            %
            % Input:
            %   frames - Compute dynmaic params of the specific link
            %       in {'c2j', 'j2p', 'both'} | default = 'c2j' | char array or string

            arguments, obj roboLink
                frames {mustBeMember(frames, {'c2j', 'j2p', 'both'})} = 'c2j'
            end

            switch frames
                case 'j2p', frames = 1; case 'c2j', frames = 2;
                case 'both', frames = [1, 2];
            end

            for k = frames
                obj.Visual{k}.Vertices
            end
        end

        % --------------------------------------------------------------- %

        function addVisual(obj, type, frames, data, tform)
            % addVisual - Add the visual geometry
            %   The object will be attached to the related frame from the
            %   [0, 0, 0] coordinate
            % 
            % Syntax
            %   addVisual(type, frames)
            %   addVisual(type, frames, data)
            %   addVisual(type, frames, data, tform)
            %
            % Input:
            %   type - Type of geometry
            %       in {'empty', 'box', 'cyl', 'mesh'} | defualt = 'box' | char array or string
            %   frames - Frames to attach the visual to
            %       in {'c2j', 'j2p', 'both'} | default = 'c2j' | char array or string
            %   data - Visual data for each 'type' object
            %       box -> [l_x, l_y, l_z]
            %       cyl -> [radius, l_z]
            %       mesh -> (n x 3) matrix of vertices, pde.DiscreteGeometry or STL file
            %       empty -> no visual
            %       auto -> 
            %   tform - Rototranslation homogeneous matrix (applied after 'auto' computation)
            %       belong to SE(3) | defualt = eye(4) | 


            arguments, obj roboLink
                type {mustBeMember(type, {'empty', 'box', 'cyl', 'mesh'})} = 'box'
                frames {mustBeMember(frames, {'c2j', 'j2p', 'both'})} = 'c2j'
                data {mustOr(data, {'mustBeMember', 'auto'}, 'mustBeNumeric', ...
                    'mustBeFile', {'mustBeA', 'pde.DiscreteGeometry'})} = 'auto'
                tform {mustBeNumeric, mustBeSE3} = eye(4)
            end

            switch frames
                case 'j2p', frames = 1; case 'c2j', frames = 2;
                case 'both', frames = [1, 2];
            end

            % Empty object
            if strcmp(type, 'empty')
                obj.Visual(frames) = {pde.DiscreteGeometry.empty}; return
            end

            for k = frames
                switch k
                    case 1, frame = obj.Joint.j2p;
                    case 2, frame = obj.Joint.c2j;
                end

                % Compute auto params
                A = eye(4); % homogeneous matrix
                if strcmp(data, 'auto') % automatic data computation
                    l = norm(frame(1:3, 4)); A(1:3, 1:3) = axang2rotm(tools.vrrotvec([0, 0, 1], frame(1:3, 4)));
                    switch type
                        case 'box', dataFig = [l/10, l/10, l];
                        case 'cyl', dataFig = [l/10, l];
                    end
                else, dataFig = data;
                end


                % Import visual
                switch type
                    case 'box', mustHaveSize(dataFig, [1, 3])
                        visualObj = multicuboid(dataFig(1), dataFig(2), dataFig(3));
                    case 'cyl', mustHaveSize(dataFig, [1, 2])
                        visualObj = multicylinder(dataFig(1), dataFig(2));
                    case 'mesh'
                        if isfile(data), visualObj = importGeometry(data);
                        elseif isa(data, 'pde.DiscreteGeometry'), visualObj = data;
                        else
                            mustHaveSize(data(1, :), [1, 3], 'data row')
    
                            visualObj = createpde();
                            convexHull = convhull(data(:, 1), data(:, 2), data(:, 3));
                            geometryFromMesh(visualObj, data, convexHull);
                        end
                end           
                
                A(1:3, 1:3) = A(1:3, 1:3)*tform(1:3, 1:3); A(1:3, 4) = A(1:3, 4) + tform(1:3, 4);
                Rot = rotm2axang(A(1:3, 1:3)); 
                rotate(visualObj, rad2deg(Rot(4)), [0, 0, 0], Rot(1:3));
                translate(visualObj, A(1:3, 4));
                obj.Visual{k} = visualObj;
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

                case 'mustBeGreaterThan', mustBeGreaterThan(data, checks{k}{2:end}), error = false;
                case 'mustBeLessThan', mustBeLessThan(data, checks{k}{2:end}), error = false;
                case 'mustBeGreaterThanOrEqual', mustBeGreaterThanOrEqual(data, checks{k}{2:end}), error = false;
                case 'mustBeLessThanOrEqual', mustBeLessThanOrEqual(data, checks{k}{2:end}), error = false;
                
                case 'mustBeA', mustBeA(data, checks{k}{2:end}), error = false;
                case 'mustBeNumeric', mustBeNumeric(data), error = false;
                case 'mustBeNumericOrLogical', mustBeNumericOrLogical(data), error = false;
                case 'mustBeFloat', mustBeFloat(data), error = false;
                case 'mustBeUnderlyingType', mustBeUnderlyingType(data, checks{k}{2:end}), error = false;

                case 'mustBeNonempty', mustBeNonempty(data), error = false;
                case 'mustBeScalarOrEmpty', mustBeScalarOrEmpty(data), error = false;
                case 'mustBeVector', mustBeVector(data), error = false;

                case 'mustBeMember', mustBeMember(data, checks{k}{2:end}), error = false;

                case 'mustBeFile', mustBeFile(data), error = false;
                case 'mustBeFolder', mustBeFolder(data), error = false;
                case 'mustBeNonzeroLengthText', mustBeNonzeroLengthText(data), error = false;
                case 'mustBeText', mustBeText(data), error = false;
                case 'mustBeTextScalar', mustBeTextScalar(data), error = false;

                % Custom
                case 'mustHaveSize', mustHaveSize(data, checks{k}{2:end}), error = false;
                case 'mustBeInertia', mustBeInertia(data), error = false;
                case 'mustBeZero', mustBeZero(data), error = false;
                case 'mustBeEmpty', mustBeEmpty(data), error = false;
            end
        catch
            % Error description
            if numel(checks{k}) > 1
                if isnumeric(checks{k}{2}), checks{k}{2} = ['[', num2str(checks{k}{2}), ']']; end
                checks{k}{2} = strcat(checks{k}{2}, ''', '); checks{k}{2} = strcat('''', checks{k}{2});
                if ~isa(checks{k}{2}, 'cell'), checks{k}{2} = checks{k}(2); end
                checks{k}{2} = strjoin(checks{k}{2}); checks{k}{2} = checks{k}{2}(1:end-1);
                stringTmp = [checks{k}{1}, '(..., {', checks{k}{2}, '}), '];
            else, stringTmp = [checks{k}{1}, ', '];
            end

            errorReport(end + (1:length(stringTmp))) = stringTmp;
        end
    end

    if error
        throw(MException('roboLink:WrongValue', ['Validation not passed.' ...
            '\nValidation to pass %s'], errorReport(1:end-2)));
    end
end

% ---------- %

function mustHaveSize(data, sizeCheck, text)
    % mustHaveSize - Validate that data has specific size
    arguments
        data, sizeCheck, text {mustBeTextScalar} = ''
    end

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

% ---------- %

function mustBeEmpty(data)
    % mustBeEmpty - Validate that data is empty

    if isempty(data), return, end
    throw(MException('roboLink:WrongValue', 'Must be empty'));
end

% ---------- %

function mustBeSE3(data)
    % mustBESE3 - Validate that data is in the SE(3) group
    mustHaveSize(data, [4, 4], 'Homogeneous matrix')
    R = data(1:3, 1:3);
    if (~all(data(4, :) == [0, 0, 0, 1]) || ~(max(abs(R.'*R - eye(3)), [], 'all') < 1e-5) ...
            || ~(abs(det(R) - 1) < 1e-5))
        throw(MException('roboJoint:WrongValue', 'Must belong to SE(3) group'));
    end
end