classdef tools
    % TOOLS - Auxiliary static class
    %   Implements all the general tools needed in various implementations
    
    properties (Access = private, Hidden = true)
        A (4, 4) double {mustBeSE3(A)} = eye(4)
        colorAplha (1, 1) double {mustBeNonNan} = 1
    end

    methods (Static)
        function framePlot(A, varargin)
            % framePlot - Plot the frame passed as homogeneous matrix
            % Syntax
            %   framePlot(A)
            %   framePlot(A, frameName)
            %   framePlot(A, frameName, quiverLineStyle)
            %
            % Input:
            %   A - homogeneous matrix representing the frame
            %       belong to SE(3) | double(4, 4)
            %   frameName - name of the frame
            %       default = '' | char array
            %   quiverLineStyle - style of the quiver
            %       default = '-' | LineStyle char array
            %       See also matlab.graphics.chart.primitive.Area/LineStyle

            % Frame font dimension
            font = 10; dim = 2.54/72.272/10*font;
            % Lenght of frame axis
            L = 0.5;

            if (nargin > 0) && ~(isempty(varargin{1}))
                frameName = varargin{1};
            else, frameName = '';
            end

            if (nargin > 1) && ~(isempty(varargin{2}))
                quiverLineStyle = varargin{2};
            else, quiverLineStyle = '-';
            end

            R = L*A(1:3, 1:3); T = A(1:3, 4);
            text(T(1) - dim, T(2) - dim, T(3) - dim, frameName, 'FontSize', font);
            colorAxis = 'rgb'; labelAxis = 'xyz'; hold on
            for k = 1:3
                quiver3(T(1), T(2), T(3), R(1, k), R(2, k), R(3, k), ...
                    'Color', colorAxis(k), 'LineWidth', 1, 'LineStyle', quiverLineStyle);
                text(T(1) + 0.95*R(1, k), T(2) + 0.95*R(2, k), T(3) + 0.95*R(3, k), labelAxis(k))
            end
            hold off
        end
    end
end

% --- Vaidating function --- %
function mustBeSE3(data)
    % mustBeSE3 - Validate that values is in the SE(3) group
    R = data(1:3, 1:3);    
    if (~all(data(4, :) == [0, 0, 0, 1])) || ~(max(abs(R.'*R - eye(3)), [], 'all') < 1e-5) || ~(abs(det(R) - 1) < 1e-5)
        error("Value of property must be a matrix belonging to SE(3) group")
    end
end