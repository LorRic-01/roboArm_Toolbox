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
        % gravity - Gravitational acceleration experienced by the robot
        %   m/s^2 | default = [0, 0, 9.8067].' | double(3, 1)
        gravity {mustBeReal, Tools.mustHaveSize(gravity, [3, 1])} = [0, 0, 9.8067].'
    end

    % ------------------------- %

    properties (SetAccess = {?Arm})
        % bodies - Link of Link object composing the manipulator
        %   defualt = empty | Link array
        bodies {mustBeA(bodies, 'Link'), Tools.mustOr(bodies, 'mustBeVector', 'mustBeEmpty')} = Link.empty

        % bodiesName - List of Link name
        %   default = empty | cell of char array or string
        bodiesName {mustBeText, Tools.mustOr(bodiesName, 'mustBeUnique', 'mustBeEmpty')} = ''
    end


    % ----------------- Functions ---------------------- %
    
    methods
        % ----- Constructor ----- %
        function obj = Arm(link)
            % Arm - Class constructor
            %
            % Syntax
            %   Arm
            %   Arm(link)
            %
            % Input:
            %   link - Link object
            %       Link

            arguments, link {}, end



        end

        % ------------------------- %


    end


    % ---------------- Get/set fun. -------------------- %

    methods

    end


    % ------------------- Static ----------------------- %

    methods (Static)

    end

end