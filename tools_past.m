classdef tools
    % TOOLS - Auxiliary static class
    %   Implements all the general tools needed in various implementations

    methods (Static)
        function framePlot(A, varargin)
            % framePlot - Plot the frame passed as homogeneous matrix
            %
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
            colorAxis = 'rgb'; hold on
            for k = 1:3
                q = quiver3(T(1), T(2), T(3), R(1, k), R(2, k), R(3, k), ...
                    'Color', colorAxis(k), 'LineWidth', 1, 'LineStyle', quiverLineStyle, ...
                    'LineWidth', 2);
                q.DataTipTemplate.DataTipRows = [dataTipTextRow('Frame: ', {frameName}), ...
                        dataTipTextRow('Pos:   ', {T.'}), dataTipTextRow('RPY:   ', {rotm2eul(R)})];
                if k == 1, q.Marker = '.'; q.MarkerSize = 15; q.MarkerEdgeColor = 'k'; end
            end
            hold off
        end

        % --------------------------------------------------------------- %

        function addCasadiToPath()
            % addCasadiToPath - Add the casADi folder to path
            % Folder must be found in the common MATLAB folder (Documents folder)
            %
            % Syntax
            %   addCasadiToPath

            if ispc % Windows
                [~, user] = system('echo %username%'); user = user(1:end-1);
                path = ['C:\Users\', user, '\Documents\MATLAB'];
                % Complete implementation
            elseif isunix
                path = '~/Documents/MATLAB';
                [~, files] = system(['ls ', path]); files = strsplit(files, {' ', '\n'});
                for k = 1:length(files)
                    if startsWith(files{k}, 'casadi'), break, end
                    if k == length(files), error("No casADi folder found in ~/Documents/MATLAB"), end
                end
                addpath(['~/Documents/MATLAB/', files{k}])
            % elseif ismac % missing implementation
            else, error("Not supported platform available")
            end
        end

        % --------------------------------------------------------------- %

        function R = axang2rotm(data)
            % axang2rotm - Convert axis-angle representation to rotation matrix
            %
            % Syntax
            %   axang2rotm([axis, angle])
            %
            % Input:
            %   axis - Axis of rotation
            %       double(1, 3)
            %   angle - Angle of rotation
            %       double or casadi.SX.sym or casadi.MX.sym
            %
            % Output:
            %   R - rotation matrix
            %       belong to SO(3) | double(3, 3) or casadi.SX.sym(3, 3) or casadi.MX.sym(3, 3)

            if all(size(data) ~= [1, 4]), error("Wrong dimension of axis-angle representation"), end

            if isnumeric(data(end))
                I = eye(3);
            else
                switch class(data(end))
                    case 'casadi.SX', I = casadi.SX.eye(3);
                    case 'casadi.MX', I = casadi.MX.eye(3);
                end
            end

            K = [0, -data(3), data(2); data(3), 0, -data(1); -data(2), data(1), 0];
            R = I + sin(data(end))*K + (1 - cos(data(end)))*K*K;
            switch class(R)
                case 'SX.sym', R = simplify(R);
                case 'MX.sym', R = simplify(R);
            end
        end

        % --------------------------------------------------------------- %

        function out = isSE3(data)
            % mustBeSE3 - Return true if data is in the SE(3) group
            R = data(1:3, 1:3);
            out = ~((~all(data(4, :) == [0, 0, 0, 1])) || ~(max(abs(R.'*R - eye(3)), [], 'all') < 1e-5) || ~(abs(det(R) - 1) < 1e-5));
        end

    end
end