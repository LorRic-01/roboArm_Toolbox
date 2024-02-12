classdef Link < handle_light
    % LINK Robotic arm manipulator link class
    %   Generate robotic arm link object
    %   For completeness, it is possible to define masses and inertia
    %   before (j2p) and after (c2j) the joint
    %
    % Link Properties:
    %   name - Joint name
    %   joint - Joint object associated with the link
    %   mass - Link mass
    %   CoM - Center of Mass position, expressed w.r.t. fixed joint frame
    %   I - Inertia matrix expressed w.r.t. link's CoM
    %
    % Joint Methods:
    %   Link - Class constructor
    %   copyRigidBody - Copy rigidBody in a Link object
    %

    properties
        % name - Joint name
        %   char array or string
        name {mustBeTextScalar, mustBeNonempty} = ' '

        % joint - Joint object associated with the link
        %   default = Joint('jnt') | Joint
        % See also Joint
        joint {mustBeA(joint, 'Joint'), mustBeNonempty} = Joint('jnt')

        % mass - Link mass ([j2p_mass; c2j_mass])
        %   kg (2x1) | default = zeros(2, 1) | duble(2, 1)
        mass {mustBeNumeric, Tools.mustHaveSize(mass, [2, 1], 'mass')} = zeros(2, 1)

        % CoM - Center of Mass position, expressed w.r.t. fixed joint frame ([j2p_CoM; c2j_CoM])
        %   m (2x3) | default = zeros(2, 3) | double(2, 3)
        CoM {mustBeNumeric, Tools.mustHaveSize(CoM, [2, 3], 'CoM')} = zeros(2, 3)

        % I - Inertia matrix expressed w.r.t. link's CoM ([Ixx, Iyy, Izz, Iyz, Ixz, Ixy])
        %   kg m^2 (2x6) | defualt = zeros(2, 6) | double(2, 6)
        I {mustBeNumeric, Tools.mustHaveSize(I, [2, 6], 'Inertia')} = zeros(2, 6)
    end

    % ------------------------- %

    properties (SetAccess = {?Link})

    end

    % -------------------------------------------------- %

    methods
        % ----- Constructor ----- %
        function obj = Link(name, joint)
            % Link - Class constructor
            %
            % Syntax
            %   Link(name)
            %   Link(name, joint)
            %
            % See also name, joint, rigidBody

            arguments (Input)
                name {mustBeNonempty, Tools.mustOr(name, {'mustBeTextScalar'}, {'mustBeA', 'rigidBody'})}
                joint {mustBeA(joint, 'Joint')} = Joint('jnt')
            end
            arguments (Output), obj Link, end

            Tools.checkCasADi   % Check casadi folder
            if isa(name, 'rigidBody')
                obj = Link.copyRigidBody(name);
                return
            end
            obj.name = name; obj.joint = joint;
        end

        % ------------------------- %

        function toString(obj)
            % toString - Plot in the command window object data
            %
            % Syntax
            %   toString

            fprintf([' ----- Link object ----- \n' ...
                     '  Name: %s \n' ...
                     '  Joint: \n'], obj.name)
            obj.joint.toString('   |')
            fprintf(['\b  Mass:\n' ...
                     '   |  j2p: %s kg\n', ...
                     '   |  c2j: %s kg\n', ...
                     '  CoM:\n' ...
                     '   |  j2p: [%s] m\n', ...
                     '   |  c2j: [%s] m\n', ...
                     '  Inertia [Ixx, Iyy, Izz, Iyz, Ixz, Ixy]:\n' ...
                     '   |  j2p: [%s] kg m^2\n', ...
                     '   |  c2j: [%s] kg m^2\n', ...
                     ' ------------------------- \n\n'], ...
                     num2str(obj.mass(1, :)), num2str(obj.mass(2, :)), ...
                     num2str(obj.CoM(1, :)), num2str(obj.CoM(2, :)), ...
                     num2str(obj.I(1, :)), num2str(obj.I(2, :)))
        end
    end

    % -------------------------------------------------- %

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
            %   frame - Body connected to which frame
            %       in {'c2j', 'j2p'} | default = 'c2j' | char array or string
            %       [mass and inertia w.r.t. the joint: 'c2j' -> after, 'j2p' -> before]
            % Output:
            %   linkObj - Link object
            %       Link

            arguments (Input)
                rigidBodyObj rigidBody {mustBeA(rigidBodyObj, 'rigidBody')}
                frame {mustBeMember(frame, {'c2j', 'j2p'})} = 'c2j'
            end
            arguments (Output), linkObj Link, end

            linkObj = Link(rigidBodyObj.Name, rigidBodyObj.Joint);
            switch frame
                case 'c2j', index = 2;
                otherwise, index = 1;
            end

            linkObj.mass(index, :) = rigidBodyObj.Mass;
            linkObj.CoM(index, :) = rigidBodyObj.CoM;
            linkObj.I(index, :) = rigidBodyObj.I;

            % ------------------------------------------------------------- TO DO: Add visual
        end
    end
end