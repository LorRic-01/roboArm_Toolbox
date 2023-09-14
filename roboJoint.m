classdef roboJoint < handle_light
    % ROBOJOINT - Robotic arm manipulator joint
    %   Generate robotic arm joint object
    
    properties
        % Name - Joint name
        %   char array or string
        Name (1, :) {mustBeCharString}

        % Type - Type of joint
        %   in {'revolute', 'prismatic', 'fixed'} | default = 'fixed' | char array
        Type (1, :) char {mustBeMember(Type, {'revolute', 'prismatic', 'fixed'})} = 'fixed'

        % JointAxis - Axis of rotation or translation of the joint
        %   default = [0, 0, 1] | double(1, 3)
        JointAxis (1, 3) double {mustBeAxis} = [0, 0, 1]

    end

end



% --- Validating function --- %
function mustBeCharString(data)
    % mustBeCharString - Validate that data is char array or string
    switch class(data)
        case 'string', return
        case 'char', return
        otherwise
            throwAsCaller(...
                createValidatorExceptionWithValue(createPrintableList(B), ...
                'MATLAB:validators:mustBeMemberGenericText',...
                'MATLAB:validators:mustBeMember')...
                );
    end
end
