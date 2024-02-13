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

        % mass - Link mass ([j2p_mass, c2j_mass])
        %   kg (1x2) | default = zeros(2, 1) | duble(2, 1)
        mass {mustBeReal, Tools.mustHaveSize(mass, [1, 2], 'mass')} = zeros(1, 2)

        % CoM - Center of Mass position, expressed w.r.t. fixed joint frame ([j2p_CoM, c2j_CoM])
        %   m (3x2) | default = zeros(2, 3) | double(2, 3)
        CoM {mustBeReal, Tools.mustHaveSize(CoM, [3, 2], 'CoM')} = zeros(3, 2)

        % I - Inertia matrix expressed w.r.t. link's CoM ([Ixx, Iyy, Izz, Iyz, Ixz, Ixy].')
        %   kg m^2 (6x2) | defualt = zeros(6, 2) | double(6, 2)
        I {mustBeReal, Tools.mustHaveSize(I, [6, 2], 'Inertia')} = zeros(6, 2)
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
                     '   |  j2p: [%s]^T m\n', ...
                     '   |  c2j: [%s]^T m\n', ...
                     '  Inertia [Ixx, Iyy, Izz, Iyz, Ixz, Ixy]:\n' ...
                     '   |  j2p: [%s]^T kg m^2\n', ...
                     '   |  c2j: [%s]^T kg m^2\n', ...
                     ' ------------------------- \n\n'], ...
                     num2str(obj.mass(:, 1)), num2str(obj.mass(:, 2)), ...
                     num2str(obj.CoM(:, 1).'), num2str(obj.CoM(:, 2).'), ...
                     num2str(obj.I(:, 1).'), num2str(obj.I(:, 2).'))
        end

        % ------------------------- %

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
                rigidBodyObj {mustBeA(rigidBodyObj, 'rigidBody')}
                frame {mustBeMember(frame, {'c2j', 'j2p'})} = 'c2j'
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

            % ------------------------------------------------------------- TO DO: Add visual
        end
    end
end