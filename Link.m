classdef Link < handle_light
    % LINK Robotic arm manupulator link class
    %   Generate robotic arm link object
    %
    % Link Properties:
    %
    % Link Methods:
    %

    % ---------------- Properties ---------------------- %

    properties
        % name - Link name
        %   char array or string
        %   Validation: mustBeTextScalar, mustBeNonempty
        name {mustBeTextScalar, mustBeNonempty} = ' '

        % joint - Joint object associated with the link
        %   default = Joint('jnt') | Joint
        %   Validation: mustBeNonempty, mustBeA(..., 'Joint')
        % See also Joint
        joint {mustBeNonempty, mustBeA(joint, 'Joint')} = Joint('jnt')

        % mass - Link mass ([j2p_mass, c2j_mass])
        %   kg (1x2) | default = zeros(1, 2) | duble(1, 2)
        %   Validation: mustBeNumeric, mustBeFinite, mustBeReal, Tools.mustHaveSize(..., [1, 2])
        mass {mustBeNumeric, mustBeFinite, mustBeReal, Tools.mustHaveSize(mass, [1, 2])} = zeros(1, 2)

        % CoM - Center of Mass position, expressed w.r.t. previous frame ([j2p_CoM, c2j_CoM])
        %   m (3x2) | default = zeros(3, 2) | double(3, 2)
        %   Validation: mustBeNumeric, mustBeFinite, mustBeReal, Tools.mustHaveSize(..., [3, 2])
        CoM {mustBeNumeric, mustBeFinite, mustBeReal, Tools.mustHaveSize(CoM, [3, 2])} = zeros(3, 2)

        % I - Inertia matrix expressed w.r.t. link's CoM ([Ixx, Iyy, Izz, Iyz, Ixz, Ixy].')
        %   kg m^2 (6x2) | defualt = zeros(6, 2) | double(6, 2)
        %   Validation: mustBeNumeric, mustBeFinite, mustBeReal, Tools.mustHaveSize(..., [6, 2])
        I {mustBeNumeric, mustBeFinite, mustBeReal, Tools.mustHaveSize(I, [6, 2])} = zeros(6, 2)
    end

    % ------------------------- %

    properties (SetAccess = {?Link, ?Arm})
        % Visual -Link's visual geometry associated to specific frame ([j2p, c2j])
        %   cell(1,2) {triangulation}
        %   Validation: Tools.mustBeCellA(..., 'triangulation'), Tools.mustHaveSize(..., [1, 2])
        visual {Tools.mustBeCellA(visual, 'triangulation'), Tools.mustHaveSize(visual, [1, 2])} = cell(1, 2)

        % parent - Index of link which the current link is attached to
        %   default = [] | double(1, 1)
        %   Validation: mustBeInteger, mustBePositive, mustBeScalarOrEmpty
        parent {mustBeInteger, mustBePositive, mustBeScalarOrEmpty} = []

        % child - Index of link(s) attached to the current link
        %   default = [] | double(1, :)
        %   Validation: mustBeVector, mustBePositive, mustBeInteger
        child {mustBeVector, mustBePositive, mustBeInteger} = zeros(1, 0)
    end


    % ----------------- Functions ---------------------- %

    methods
        % ----- Constructor ----- %
        function obj = Link(name, joint)
            % Link - Class constructor
            %
            % Syntax
            %   Link(name)
            %   Link(name, joint)
            %
            %   Link(rigidBodyObj)
            %
            % Input:
            %   rigidBodyObj - rigidBody object
            %       rigidBody
            %       Validation: mustBeA(..., 'rigidBody'), mustBeNonempty
            %
            % See also name, joint, rigidBody

            arguments (Input)
                name {mustBeNonempty, Tools.mustAndOr('or', name, 'mustBeA', 'rigidBody', 'mustBeTextScalar', [])}
                joint {mustBeA(joint, 'Joint')} = Joint('jnt')
            end
            arguments (Output), obj Link, end

            Tools.checkCasADi   % Check casadi folder
            if isa(name, 'rigidBody')
                obj = Link.copyRigidBody(name); return
            end
            
            obj.name = name; obj.joint = joint;        
        end

        % ------------------------- %

        function toString(obj, prefix)
            % toString - Plot in the command window object data
            %
            % Syntax
            %   toString
            %   toString(prefix)
            %
            % Input;
            %   prefix - Text before print
            %       char array or string
            %       Validation: mustBeTextScalar

            arguments, obj Link, prefix {mustBeTextScalar} = '', end

            fprintf([prefix, ' ----- Link object ----- \n' ...
                     prefix, '  Name: %s \n' ...
                     prefix, '  Joint: \n'], obj.name)
            obj.joint.toString([prefix, '   |'])
            fprintf([prefix, '\b  Mass:\n' ...
                     prefix, '   |  j2p: %s kg\n', ...
                     prefix, '   |  c2j: %s kg\n', ...
                     prefix, '  CoM:\n' ...
                     prefix, '   |  j2p: [%s]^T m\n', ...
                     prefix, '   |  c2j: [%s]^T m\n', ...
                     prefix, '  Inertia [Ixx, Iyy, Izz, Iyz, Ixz, Ixy]:\n' ...
                     prefix, '   |  j2p: [%s]^T kg m^2\n', ...
                     prefix, '   |  c2j: [%s]^T kg m^2\n', ...
                     ' ------------------------- \n\n'], ...
                     num2str(obj.mass(:, 1)), num2str(obj.mass(:, 2)), ...
                     num2str(obj.CoM(:, 1).'), num2str(obj.CoM(:, 2).'), ...
                     num2str(obj.I(:, 1).'), num2str(obj.I(:, 2).'))
        end

        % ------------------------- %

        function addVisual(obj, type, frames, data, tform)
            % addVisual - Add visual geometries
            %   Visual geometry frame ([0, 0, 0], xyz-axis) attached to the indicated frame
            %
            % Syntax
            %   addVisual(type)
            %   addVisual(type, frames)
            %   addVisual(type, frames, data)
            %   addVisual(type, frames, data, tform)
            %
            % Input:
            %   type - Type of passed geometry
            %       in {'empty', 'box', 'cyl', 'pde', 'stl', 'tri'} | default = 'empty' | char array or string
            %       Validation: mustBeTextScalar, mustBeMember(..., {'empty', 'box', 'cyl', 'pde', 'stl', 'tri'})
            %   frames - Frames to attach the visual to
            %       in {'c2j', 'j2p', 'both'} | deafult = 'c2j' | char array or string
            %       Validation: mustBeTextScalar, mustBeMember(..., {'c2j', 'j2p', 'both'})
            %   data - Visual data to pass for each 'type' of object
            %       empty -> no visual
            %       box   -> [l_x, l_y, l_z]
            %       cyl   -> [radius, l_z]
            %       pde   -> pde.DiscreteGeometry object
            %       stl   -> STL file name
            %       tri   -> triangulation
            %       Validation: mustBeA(..., {'triangulation', 'pde.DiscreteGeometry') OR ...
            %           mustBeFile OR (mustBeVector AND mustBeNumeric AND mustBeFinite)
            %   tform - Roto-translation homogeneous matrix w.r.t 'frame' frame
            %       blong to SE(3) | default = eye(4) | double(4, 4)
            %       Validation: Tools.mustBeSE3
            
            arguments, obj Link
                type {mustBeTextScalar, mustBeMember(type, {'empty', 'box', 'cyl', 'pde', 'stl', 'tri'})} = 'empty'
                frames {mustBeTextScalar, mustBeMember(frames, {'c2j', 'j2p', 'both'})} = 'c2j'
                data {Tools.mustAndOr('or', data, 'mustBeA', {'triangulation', 'pde.DiscreteGeometry'}, ...
                    'mustAndOr', {'and', 'mustBeVector', [], 'mustBeNumeric', [], 'mustBeFinite', []}, ...
                    'mustBeFile', [])} = zeros(1, 0)
                tform {Tools.mustBeSE3} = eye(4)
            end

            switch frames
                case 'j2p', frames = 1; case 'c2j', frames = 2;
                case 'both', frames = [1, 2];
            end

            % Empty obj
            if strcmp(type, 'empty'), obj.visual(frames) = cell(length(frames), 1); return, end

            % Compute visual
            for k = frames
                switch type
                    case 'box', mustBeReal(data),
                        if numel(data) ~= 3
                            warning('Wrong dimension for box visual element.\nPassed: %s, Desired: %s, Used: %s', ...
                                Tools.convertToString(size(data)), Tools.convertToStringnum2str([2, 1]), Tools.convertToString(data(1)*ones(1, 3)))
                            data = data(1)*ones(1, 3);
                        end
                        model = multicuboid(data(1), data(2), data(3));
                    case 'cyl', mustBeReal(data)
                        if numel(data) ~= 2
                            warning('Wrong dimension for box visual element.\nPassed: %s, Desired: %s, Used: %s', ...
                                Tools.convertToString(size(data)), Tools.convertToString([2, 1]), Tools.convertToString(data(1)*ones(1, 2)))
                            data = data(1)*ones(1, 2);
                        end
                        model = multicylinder(data(1), data(2));
                    case 'pde', mustBeA(data, 'pde.DiscreteGeometry')
                        model = data;
                    case 'stl', mustBeFile(data)
                        if ~strcmp(data(end-2:end), 'stl'), throw(MException('Link:WrongData', 'Methods only accept .stl files')), end
                        model = createpde; model.Geometry = importGeometry(data);
                    case 'tri', mustBeA(data, 'triangulation'), Faces = data.Faces; Vertices = data.Vertices;
                end

                if ~strcmp(type, 'tri')
                    h1 = figure; h = pdegplot(model); Faces = h(1).Faces; Vertices = h(1).Vertices; close(h1)
                end
                
                % Apply tform
                Vertices = tform*[Vertices.'; ones(1, size(Vertices, 1))];
                Vertices = Vertices(1:3, :).';
                obj.visual{k} = triangulation(Faces, Vertices);
            end
        end

        % ------------------------- %

        function cmpDynParam(obj, params)
            % cmpDynParam - Compute dynamics parameters (CoM and I) using
            %   visual information
            %
            % Syntax
            %   cmpDynParam
            %   cmpDynParam(parms)
            %
            % Input:
            %   params - Struct with the following parameters
            %       verbose - Print Command Window info and plots
            %           default = false | logical
            %       cycle - Number of optimization cycles
            %           deafult = 100 | double(1, 1)
            %       n_p - Number of particles approximating the component
            %           defualt = max(1.1*# of vertices, 100) | double(1, 1)
            %       cost - Cost function to optimize during the particle positioning
            %           deafult = @(x, swarm) min(sum((swarm - x).^2, 2)) | function_handle
            %       Validation: mustBeA(..., 'struct')
            
            arguments
                obj Link
                params {mustBeA(params, 'struct')} = struct()
            end

            for k = 1:length(obj.visual)
                if isempty(obj.visual{k})
                    obj.CoM(:, k) = zeros(3, 1); obj.I(:, k) = zeros(6, 1);
                else
                    [obj.CoM(:, k), obj.I(:, k), ~, obj.visual{k}] = ...
                        Tools.cmpDynParams(obj.visual{k}, [], params);
                    
                    obj.I(:, k) = Tools.inertiaConv(Tools.inertiaConv(obj.I(:, k)) + norm(obj.CoM(:, k)).^2*eye(3) - (obj.CoM(:, k))*(obj.CoM(:, k).'));
                    if obj.mass(:, k), obj.I(:, k) = obj.I(:, k)*obj.mass(:, k);
                    else, obj.I(:, k) = zeros(6, 1);
                    end
                end
            end
        end

        % ------------------------- % MISSING PIECES -------------------------------------------------------------------------------------------------

        function plot(obj, jointValue, specifics)
            % plot - Plot Link object, namely Joint and visual
            % Frame legend:
            %   - dotted line: parent joint frame ('parent')
            %   - full line: (moved) joint frame ('joint')
            %   - dashed line: child(ren) frame ('child')
            %
            % Syntax
            %   plot(jointValue)
            %   plot(jointValue, specific_1, specific_2, ...)
            %
            % Input:
            %   jointValue - Joint value
            %       rad or m | default = homePosition | empty or double(1, 1)
            %   specifics - Frame(s) to show
            %       in {'parent', 'joint', 'child', 'all'} | default = 'joint' | char array or string
            
            arguments
                obj Link,
                jointValue {mustBeReal, mustBeScalarOrEmpty} = obj.joint.homePosition
            end
            arguments (Repeating), specifics {mustBeMember(specifics, {'parent', 'joint', 'child', 'all'})}, end

            if isempty(jointValue), jointValue = obj.joint.homePosition; end
            if isempty(specifics), specifics = {'joint'}; end

            % Plot joint
            obj.joint.plot(jointValue, specifics{:})
            if isa(obj.joint.Ab, 'casadi.MX'), return, end

            if any(ismember(specifics, 'all')), specifics = {'parent', 'joint', 'child'}; end
            specifics = unique(specifics);

            A_fun = obj.joint.A; A_val = full(A_fun(jointValue));
            for k = 1:length(specifics)
                if strcmp(specifics{k}, 'child')
                    if isempty(obj.visual{2}), continue, end
                    Tools.plotTri(obj.visual{2}, A_val, 'FaceColor', [0.8500 0.3250 0.0980], 'EdgeColor', [0.8500 0.3250 0.0980], 'FaceAlpha', 0.1)
                else
                    if isempty(obj.visual{1}), continue, end
                    Tools.plotTri(obj.visual{1}, obj.joint.Ab, 'FaceColor', [0 0.4470 0.7410], 'EdgeColor', [0 0.4470 0.7410], 'FaceAlpha', 0.1)
                end
            end
        end
    end


    % ---------------- Get/set fun. -------------------- %

    methods
        function set.I(obj, inertia)
            % Check that data represents an inertia matrix during inesertion
            % Input:
            %   inertia - Inertia matrix [Ixx, Iyy, Izz, Iyz, Ixz, Ixy]
            %       double(6, 2)

            arguments, obj Link, inertia {mustBeReal}, end

            for k = 1:size(inertia, 2), Tools.inertiaConv(inertia(:, k)); end
            obj.I = inertia;
        end
    end


    % ------------------- Static ----------------------- %

    methods (Static)
        function linkObj = copyRigidBody(rigidBodyObj, frame)
            % copyRigidBody - Copy rigidBody in a Link object
            %
            % Syntax
            %   copyRigidBody(rigidBodyObj)
            %
            % Input:
            %   rigidBodyObj - rigidBody object
            %       rigidBody
            %       Vaidation: mustBeNonempty, mustBeA(..., 'rigidBody')
            %   frame - Body connected to which frame
            %       in {'j2p', 'c2j'} | default = 'c2j' | char array or string
            %       mass and inertia w.r.t. the frame:
            %           - 'c2j' => joint
            %           - 'j2p' => parent (prev. joint child)
            %       Vaidation: mustBeTextScalar, mustBeMember(..., {'j2p', 'c2j'})
            % Output:
            %   linkObj - Link object
            %       Link

            arguments (Input)
                rigidBodyObj {mustBeNonempty, mustBeA(rigidBodyObj, 'rigidBody')}
                frame {mustBeTextScalar, mustBeMember(frame, {'j2p', 'c2j'})} = 'c2j'
            end
            arguments (Output), linkObj Link, end
            
            if isa(rigidBodyObj.Joint, 'Joint'), joint = rigidBodyObj.Joint;
            else, joint = Joint.copyRigidBodyJoint(rigidBodyObj.Joint);
            end
            linkObj = Link(rigidBodyObj.Name, joint);
            switch frame
                case 'c2j', index = 2;
                otherwise, index = 1;
            end

            linkObj.mass(:, index) = rigidBodyObj.Mass;
            linkObj.CoM(:, index) = rigidBodyObj.CenterOfMass;
            linkObj.I(:, index) = rigidBodyObj.Inertia;

            % ---------------------------------------------------------------------------------------------------- TO DO: Add visual
        end
    end
end