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
    end

    % ------------------------- %

    properties (SetAccess = {?Arm})
        % casadiVars - casADi variables used in the robot descriptions
        %   default = empty | casadi.MX
        casadiVars {Tools.mustAndOr('or', casadiVars, 'mustBeA', 'casadi.MX', 'mustBeEmpty', [])} = []
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
                name {mustBeNonempty, mustBeTextScalar}
                link {mustBeA(link, 'Link')} = Link.empty
            end

            obj.name = name;
            if isempty(link), return, end

            obj.links(1) = link;
            obj.linksName{1} = link.name;
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

        function plot(obj, jointValue, frameSpec, surfSpec)
            % plot - Plot Link object, namely Joint and visual
            % Frame legend:
            %   - dotted line: parent joint frame ('parent')
            %   - full line: (moved) joint frame ('joint')
            %   - dashed line: child(ren) frame ('child')

            % Graph representing connections
            t = [obj.links.child]; s = [obj.links.parent];
            G = digraph(s, t, nan(size(s)), obj.linksName);
            plot(G,'Layout', 'layered')
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

    end


    % ------------------- Static ----------------------- %

    methods (Static)

    end

end