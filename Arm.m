classdef Arm < handle_light
    % ARM Robotic arm manipulator arm class
    %   Generate robotic arm object
    %
    % Arm Properties:
    %
    %
    % Arm Methods:
    %   Arm - Class constructor
    %


    % ---------------- Properties ---------------------- %
    
    properties
        % name - Arm name
        %   char array or string
        %   Validation: mustBeTextScalar, mustBeNonempty
        name {mustBeTextScalar, mustBeNonempty} = ' '

        % gravity - Gravitational acceleration experienced by the robot
        %   m/s^2 | default = [0, 0, 9.8067].' | double(3, 1)
        %   Validation: mustBeNumeric, mustBeReal, mustBeFinite, mustHaveSize(..., [3, 1])
        gravity {mustBeNumeric, mustBeReal, mustBeFinite, Tools.mustHaveSize(gravity, [3, 1])} = [0, 0, 9.8067].'
    end

    % ------------------------- %

    properties (SetAccess = {?Arm})
        % links - List of Link object composing the arm
        %   defualt = empty | Link array
        %   Validation: mustBeA(..., 'Link'), mustBeVector OR Tools.mustBeEmpty
        links {mustBeA(links, 'Link'), Tools.mustAndOr('or', links, 'mustBeVector', [], 'mustBeEmpty', [])} = Link.empty

        % bodiesName - List of Link name
        %   default = empty | cell of char array or string
        %   Validation: mustBeText, Tools.mustBeUnique OR Tools.mustBeEmpty
        linksName {mustBeText, Tools.mustAndOr('or', linksName, 'mustBeUnique', [], 'mustBeEmpty', [])} = {}

        % A_genFun - A joint's homogeneous matrix funtions (casADi) w.r.t. all controllable joints
        %   defualt = empty | cell array(casadi.Function)
        %   Validation: Tools.mustBeEmpty OR Tools.mustBeCellA(..., 'casadi.Function')
        A_genFun {Tools.mustAndOr('or', A_genFun, 'mustBeEmpty', [], 'mustBeCellA', 'casadi.Function')}

        % A_CoMgenFun - A CoM's homogeneous matrix funtions (casADi) w.r.t. all controllable joints
        %   defualt = empty | cell array(casadi.Function)
        %   Validation: Tools.mustBeEmpty OR Tools.mustBeCellA(..., 'casadi.Function')
        A_CoMgenFun {Tools.mustAndOr('or', A_CoMgenFun, 'mustBeEmpty', [], 'mustBeCellA', 'casadi.Function')}
    end


    % ----------------- Functions ---------------------- %
    
    methods
        % ----- Constructor ----- %
        function obj = Arm(name, link)
            % Arm - Class constructor
            %
            % Syntax
            %   Arm(name)
            %   Arm(name, link)
            %
            % Input:
            %   link - Link object
            %       Link
            %       Validation: mustBeA(..., 'Link')
            %
            % See also name, bodies, Link

            arguments
                name {mustBeNonempty, Tools.mustAndOr('or', name, 'mustBeA', 'rigidBodyTree', 'mustBeTextScalar', [])}
                link {mustBeA(link, 'Link')} = Link.empty
            end

            Tools.checkCasADi   % Check casadi folder
            
            if isa(name, 'rigidBodyTree')
                obj = Arm.copyRigidBodyTree(name); return
            end

            obj.name = name; if isempty(link), return, end
            obj.links(1) = link; obj.linksName{1} = link.name;
        end

        % ------------------------- %

        function addLink(obj, link, connection)
            % addLink - Add link to the arm
            %
            % Syntax
            %   addLink(link)
            %   addLink(link, parentName)
            %   addLink(link, parentIndex)
            %
            % Input:
            %   link - Link object to add to the arm
            %       Link
            %       Validation: mustBeNonempty, mustBeA(..., 'Link')
            %   parentName - Name of the link which is attached to
            %       deafult = linksName{end} | char array or string
            %       Validation: mustBeTextScalar
            %   parentIndex - Index of the link which is attached to
            %       deafult = length(linksName) | Integer
            %       Validation: mustBeInteger, mustBeReal, mustBePositive

            arguments
                obj Arm, link {mustBeNonempty, mustBeA(link, 'Link')}
                connection {Tools.mustAndOr('or', connection, ...
                    'mustAndOr', {'and', 'mustBeInteger', [], 'mustBeReal', [], 'mustBePositive', [], 'mustBeScalarOrEmpty', []}, ...
                    'mustBeTextScalar', [])} = ''
            end

            if isempty(obj.links), obj.links = link; obj.linksName = {link.name}; return, end

            if isempty(connection), connection = length(obj.linksName);
            elseif ~isnumeric(connection)
                index = find(strcmp(connection, obj.linksName), 1);
                if isempty(index), throw(MException('Arm:noLinkFound', ...
                        'No link with name %s was found.\nAvailable links: %s', ...
                        Tools.convertToString(connection), Tools.convertToString(obj.linksName)))
                else, connection = index;
                end
            elseif connection > length(obj.linksName)
                throw(MException('Arm:noLinkFound', 'No match between index and numer of links.'))
            end
            
            obj.links = [obj.links, link];
            obj.linksName{end + 1} = link.name;
            
            obj.links(connection).child(end + 1) = length(obj.links);
            obj.links(end).parent = connection;
        end

        % ------------------------- %

        function removeLink(obj, connection)
            % removeLink - Remove link and everything attached to it from the arm
            %
            % Syntax
            %   removeLink(linkName)
            %   removeLink(linkIndex)
            %
            % Input:
            %   linkName - Name of the link to remove
            %       char array or string
            %       Validation: mustBeTextScalar, mustBeNonempty
            %   linkIndex - Index of the link to remove
            %       Integer
            %       Validation: mustBeNonempty, mustBeInteger, mustBeReal, mustBePositive

            arguments
                obj Arm,
                connection {mustBeNonempty, Tools.mustAndOr('or', connection, ...
                    'mustAndOr', {'and', 'mustBeInteger', [], 'mustBeReal', [], 'mustBePositive', [], 'mustBeScalarOrEmpty', []}, ...
                    'mustBeTextScalar', [])} = ''
            end

            if isnumeric(connection)
                if connection > length(obj.linksName)
                    warning('No match. Cannot remove element.'), return
                end
                index = connection;
            else
                index = find(strcmp(connection, obj.linksName), 1);
                if isempty(index), warning('No match. Cannot remove element.'), return, end
            end

            % Remove recursively children
            if ~isempty(obj.links(index).child)
                nameLink = obj.linksName(obj.links(index).child);
                if ~iscell(nameLink), nameLink = {nameLink}; end
                for k = nameLink, obj.removeLink(k{1}), end
            end

            % Remove element
            index = find(strcmp(connection, obj.linksName), 1);
            if ~isempty(obj.links(index).parent)
                parent = obj.links(obj.links(index).parent); parent.child(parent.child == index) = [];
            end
            for k = 1:length(obj.linksName)
                obj.links(k).parent = obj.links(k).parent - (obj.links(k).parent > index);
                obj.links(k).child = obj.links(k).child - (obj.links(k).child > index);
            end
            obj.links(index) = [];
            obj.linksName(index) = [];
        end

        % ------------------------- %
        
        function cmpKin(obj)
            % cmpKin - Compute arm kinematic model w.r.t base frame (link with no parent)
            %
            % Syntax
            %   cmpKin

            arguments, obj Arm, end
            
            A_fun = obj.A_genFun;
            
            
        end

        % ------------------------- %

        function plot(obj, jointValue, frameSpec, visual)
            % plot - Plot Arm object, namely Link, Joint and visual
            % Frame legend:
            %   - dotted line: parent joint frame ('parent')
            %   - full line: (moved) joint frame ('joint')
            %   - dashed line: child(ren) frame ('child')
            % Color legend:
            %   - same colour = same Link object
            %
            % Syntax
            %   plot(jointValue)
            %   plot(jointValue, frameSpec)
            %   plot(jointValue, frameSpec, visual)
            %
            % Input:
            %   jointValue - Joint value
            %       rad or m | default = links.joint.homePosition | empty or double(1, 1)
            %       Validation: (mustBeReal AND mustBeNumeric AND mustBeFinite AND mustBeVector) OR Tools.mustBeEmpty
            %   frameSpec - Frame(s) to show
            %       in {'parent', 'joint', 'child', 'all', 'none'} | default = 'all' | char array or string
            %       Validation: Tools.mustBeEmpty OR mustBeMember(..., {'parent', 'joint', child', 'all', 'none'})
            %   visual - Plot visual objects
            %       default = true | logical
            %       Validation: mustBeA(..., 'logical')

            arguments, obj Arm,
                jointValue {Tools.mustAndOr('or', jointValue, ...
                    'mustAndOr', {'and', 'mustBeReal', [], 'mustBeNumeric', [], 'mustBeFinite', [], 'mustBeVector', []}, ...
                    'mustBeEmpty', [])} = []
                frameSpec {Tools.mustAndOr('or', frameSpec, ...
                    'mustBeMember', {'parent', 'joint', 'child', 'all', 'none'}, 'mustBeEmpty', [])} = 'all'
                visual {mustBeA(visual, 'logical')} = true
            end

            if any(strcmp(frameSpec, 'none')), return, end
            obj.cmpATree(1); % compute A homogeneus matrix
            joints = [obj.links.joint]; moving_joints = ~strcmp({joints.type}, 'fixed');

            % If jointValue does not match with neither # of moving joint
            % and all joints, use the homePosition of every joint
            if isempty(jointValue), jointValue = [joints.homePosition];
            elseif all(length(jointValue) ~= [length(obj.linksName), sum(moving_joints)])
                warning(['Cannot plot the joint since ' ...
                    'there is a further dependency on other joints.\n' ...
                    'Resort to plot of Arm class or check jointValue'], [])
                jointValue = [joints.homePosition];
            end

            % Padd joint values
            if length(jointValue) ~= length(obj.linksName)
                tmp = zeros(length(obj.linksName), 1);
                tmp(moving_joints) = jointValue; jointValue = tmp;
            end

            colors = orderedcolors("gem12");
            for k = 1:length(obj.linksName)
                % Plot each link
                count = length(obj.linksName);
                indeces = zeros(size(obj.linksName)); indeces(end) = k;
                iterLink = obj.links(k);
                while ~isempty(iterLink.parent)
                    indeces(count - 1) = iterLink.parent; count = count - 1;
                    iterLink = obj.links(iterLink.parent);
                end

                if visual, surfSpec = {'FaceColor', colors(k, :), 'EdgeColor', colors(k, :), 'FaceAlpha', 0.1};
                else, surfSpec = {};
                end
                obj.links(k).plot(jointValue(indeces(count:end)), frameSpec, ...
                    surfSpec{:});
            end
        end

        % ------------------------- %

        function plotGraph(obj)
            % plotGraph - Plot Arm graph representing the link connection (digraph)
            %
            % Syntax
            %   plotGraph

            % Graph representing connections
            t = [obj.links.child]; s = [obj.links.parent];
            G = digraph(s, t, nan(size(s)), obj.linksName);
            plot(G,'Layout', 'layered')
        end

        % ------------------------- %

        function cmpATree(obj, index)
            % cmpATree - Compute the A homogeneous matrix of each joint
            % w.r.t. base frame (link with no parent)
            %
            % Syntax
            %   cmpATree
            %   cmpTree(1)
            %       (Despite working with othr numbers the correct
            %       behaviours is obtained when called with 1)

            arguments, obj Arm, index {mustBeInteger, mustBePositive} = 1, end

            import casadi.*
            
            % Recursively compute A homogeneous matrix
            x_prev = casadi.MX.get_input(obj.links(index).joint.A); x_prev = x_prev{1};
            for k = obj.links(index).child
                obj.links(k).joint.A = Function('A', {x_prev}, ...
                    {obj.links(index).joint.A(x_prev)*obj.links(index).joint.c2j});
                cmpATree(obj, k)
            end
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
            %       default = '' | char array or string
            %       Validation: mustBeTextScalar

            arguments, obj Arm, prefix {mustBeTextScalar} = '', end

            fprintf([prefix, ' ----- Arm object ----- \n' ...
                     prefix, '  Name: %s \n' ...
                     prefix, '  Links'' names: %s \n' ...
                     prefix, '  Links: \n'], ...
                     obj.name, Tools.convertToString(obj.linksName));

            for k = 1:length(obj.links)
                obj.links(k).toString([prefix, '  ', '#', num2str(k)], true)
            end
            fprintf(['\b', prefix, ' ------------------------- \n\n'])
        end
    end


    % ---------------- Get/set fun. -------------------- %

    methods
        function A = get.A_genFun(obj)
            % Return A joint's homogeneous matrix funtions (casADi) w.r.t. controllable joints
            
            arguments, obj Arm, end
            
            import casadi.*
            A = cell(size(obj.linksName));
            obj.cmpATree(1)
            
            children = obj.links(1).child;
            mask = eye(length(obj.linksName));
            while ~isempty(children)
                for k = 1:length(children)
                    mask(children(k), obj.links(children(k)).parent) = true;
                    mask(children(k), :) = mask(children(k), :) | mask(obj.links(children(k)).parent, :);
                end
                children = [obj.links(children).child];
            end
            
            joints = [obj.links.joint];
            x = MX.sym('x', [sum(~strcmp({joints.type}, 'fixed')), 1]);
            x_ext = MX.zeros([length(obj.linksName), 1]);
            x_ext(find(~strcmp({joints.type}, 'fixed'), length(x))) = x;

            for k = 1:length(obj.linksName)
                A{k} = Function(['A_fun', num2str(k)], {x}, ...
                    {obj.links(k).joint.A(x_ext(find(mask(k, :), sum(mask(k, :)))))});
            end
        end

        % ------------------------- %

        function A_CoM = get.A_CoMgenFun(obj)
            % Return A CoM's homogeneous matrix funtions (casADi) w.r.t. controllable joints
            
            arguments, obj Arm, end
            
            import casadi.*
            A_CoM = cell(length(obj.linksName), 2);
            obj.cmpATree(1)
            
            children = obj.links(1).child;
            mask = eye(length(obj.linksName));
            while ~isempty(children)
                for k = 1:length(children)
                    mask(children(k), obj.links(children(k)).parent) = true;
                    mask(children(k), :) = mask(children(k), :) | mask(obj.links(children(k)).parent, :);
                end
                children = [obj.links(children).child];
            end
            mask_p = mask - eye(length(obj.linksName));
            
            joints = [obj.links.joint];
            x = MX.sym('x', [sum(~strcmp({joints.type}, 'fixed')), 1]);
            x_ext = MX.zeros([length(obj.linksName), 1]);
            x_ext(find(~strcmp({joints.type}, 'fixed'), length(x))) = x;

            for k = 1:length(obj.linksName)
                A_CoMp = obj.links(k).A_CoM{1}; A_CoMj = obj.links(k).A_CoM{2};
                if sum(mask_p(k, :))
                A_CoM{k, 1} = Function(['A_CoMfun_', num2str(k)], {x}, ...
                    {A_CoMp(x_ext(find(mask_p(k, :), sum(mask_p(k, :)))))});
                else, A_CoM{k, 1} = Function(['A_CoMfun_', num2str(k)], {x}, {A_CoMp});
                end
                A_CoM{k, 2} = Function(['A_CoMfun_', num2str(k)], {x}, ...
                    {A_CoMj(x_ext(find(mask(k, :), sum(mask(k, :)))))});
            end
        end
    end


    % ------------------- Static ----------------------- %

    methods (Static)
        function armObj = copyRigidBodyTree(rigidBodyTreeObj)
            % copyRigidBodyTree - Copy rigidBodyTree in Arm object
            %
            % Syntax
            %   copyRigidBodyTree(rigidBodyTreeObj)
            %
            % Input:
            %   rigidBodyTreeObj - rigidBodyTree object
            %       rigidBodyTree
            %       Vaidation: mustBeNonempty, mustBeA(..., 'rigidBodyTree')
            % Output:
            %   armObj - Arm object
            %       Arm

            arguments (Input)
                rigidBodyTreeObj {mustBeNonempty, mustBeA(rigidBodyTreeObj, 'rigidBodyTree')}
            end
            arguments (Output), armObj Arm, end
            
            armObj = Arm('arm');
            armObj.gravity = reshape(rigidBodyTreeObj.Gravity, [3, 1]);

            links = cell(size(rigidBodyTreeObj.Bodies));
            parent = cell(size(rigidBodyTreeObj.Bodies));
            for k = 1:length(rigidBodyTreeObj.Bodies)
                if isa(rigidBodyTreeObj.Bodies{k}, 'rigidBody'), links{k} = Link.copyRigidBody(rigidBodyTreeObj.Bodies{k});
                else, links{k} = rigidBodyTreeObj.Bodies{k}; end
                if ~strcmp(rigidBodyTreeObj.Bodies{k}.Parent.Name, rigidBodyTreeObj.BaseName)
                    parent{k} = rigidBodyTreeObj.Bodies{k}.Parent.Name;
                end
            end

            base = false;
            while ~isempty(parent)
                for k = 1:length(parent)
                    if base
                        if any(strcmp(parent{k}, armObj.linksName))
                            armObj.addLink(links{k}, parent{k});
                            break
                        end
                    else
                        if ~isempty(parent{k}), continue, end
                        base = true;
                        armObj.addLink(links{k});
                        break
                    end
                end
                parent(k) = [];
                links(k) = [];
            end
        end
    end

end