classdef roboLink < handle_light
    % ROBOLINK Robotic arm manipulator link
    %   Generate robotic arm link object
    %
    % roboLink Properties:
    %   Name - Link name
    %   Mass - Link mass
    %   CoM - Center of mass, expressed w.r.t. fixed joint frame
    %   I - Inertia matrix w.r.t. link's CoM
    %   Parent - Index of link which the current link is attached to
    %   Child - Index of link(s) attache to the current link
    %
    % roboLink Methods:
    %   roboLink - Class constructor
    

    properties
        % Name - Link name
        %   char array or string
        Name (1, :) {mustBeA(Name, {'char', 'string'}), mustBeNonempty} = ' '
        
        % Joint - roboJoint object associated to the link
        %   default = roboJoint([Name, '_jnt']) | roboJoint
        Joint roboJoint

        % Mass - Link mass
        %   kg | default = 0 | double(1, 1)
        Mass (1, 1) {mustBeNumeric} = 0
        
        % CoM - Center of mass, expressed w.r.t. fixed joint frame
        %   [m, m, m] | default = [0, 0, 0] | double(1, 3)
        CoM (1, 3) {mustBeNumeric} = zeros(1, 3)

        % I - Inertia matrix w.r.t. link's CoM
        %   kg m^2 (3x3) | default = zeros(3, 3) | double(3, 3)
        I (3, 3) {mustBeNumeric} = zeros(3, 3)
    end

    % ------------------------------------------------------------------- %

    properties (SetAccess = {?roboLink})
        % Parent - Index of link which the current link is attached to
        %   default = [] | double(1, 1)
        Parent {mustBeInteger, mustBeScalarOrEmpty} = []

        % Child - Index of link(s) attache to the current link
        %   default = [] | double(1, :)
        Child (1, :) {mustBeInteger} = zeros(1, 0)
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
                Name {mustBeNonempty}
                Joint roboJoint = roboJoint.empty
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