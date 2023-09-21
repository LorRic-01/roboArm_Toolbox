classdef tools
    % TOOLS Static class for utilities implementation
    %   Implements all general tools useful for problem definition and
    %   visualization
    %
    % tools Methods:
    %   addCasadiToPath - Add the casadi folder to Matlab path
    %   ADHparams - Generate roto-translation given DH params

    properties (Constant = true, Hidden = true)
        % tim - Timer used to avoid multiple error report
        %   timer('StartDelay', 3, 'TimerFcn', @(~, ~) tim.stop) | timer
        tim = timer('StartDelay', 3, 'TimerFcn', @(~, ~) fprintf(''))
    end

    methods (Static)
        function addCasadiToPath(searchPath)
            % addCasadiToPath - Add the casadi folder to Matlab path
            %   Two options:
            %   - Search in the userpath MATLAB specific path (or in the
            %       passed path) for a folder named 'casadi*'
            %   - Use UI to select directly the casadi folder
            %
            % Syntax
            %   addCasadiToPath
            %   addCasadiToPath(searchPath)
            %   addCasadiToPath('UI')
            %
            % Input:
            %   searchPath - folder containing the 'casadi*' folder (absolute path)
            %       default = userpath() | char array or string

            arguments
                searchPath {mustBeTextScalar} = userpath
            end

            % Already existing included path containingcontain
            if contains(path, 'casadi', 'IgnoreCase', ispc), return, end

            switch searchPath
                case 'UI' % use User Interface to add casadi folder
                    searchPath = uigetdir(userpath(), 'Select casADi folder');
                otherwise
                    % Different system call to extract folders in the path
                    if ispc                                                 % Windows % --- TO IMPLEMENT --- %
                    elseif isunix, [status, files] = system(['ls ', searchPath]); % Linux
                    elseif ismac                                            % Mac % --- TO IMPLEMENT --- %
                    else, throw(MException('tools:WrongSystem', ...
                            'Function available only on Windows, Linux and Mac systems'));
                    end

                    % Extract if exists the first casadi folder
                    if status, throw(MException('tools:MissingFolder', 'Missing casadi folder in path %s', searchPath)); end
                    files = strsplit(files, {' ', '\n'}); found = false;
                    for k = 1:length(files)
                        if startsWith(files{k}, 'casadi'), files = files{k}; found = true; break, end
                    end
                    if ~found, throw(MException('tools:MissingFolder', 'Missing casadi folder in path %s', searchPath)); end
                    searchPath = [searchPath, '/', files];
            end

            % Add casdi* folder path
            lastwarn(''); addpath(searchPath)
            if ~isempty(lastwarn), throw(MException('tools:pathError', 'Error during addpath of %s', searchPath)); end
        end

        % --------------------------------------------------------------- %

        function A = ADHparams(DHParams)
            % ADHparams - Generate constant roto-translation homogeneous
            % matrix from Denavit-Hartenberg params representation
            %
            % Syntax
            %   ADHparams(DHParams)
            %
            % Input:
            %   DHParams - Denavit-Hartenberg params representation ([d, theta, a, alpha])
            %       [m, rad, m, rad] | double(1, 4)
            %
            % Output:
            %   A - homogeneous matrix
            %       belong to SE(3) | double(4, 4)

            arguments
                DHParams (1, 4) {mustBeNumeric, mustBeNonNan, mustBeFinite}
            end
            d = DHParams(1); theta = DHParams(2); a = DHParams(3); alpha = DHParams(4);

            A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta);
                 sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                     0,              sin(alpha),              cos(alpha),          d;
                     0,                  0,                      0,                1];
        end

        % --------------------------------------------------------------- %

        function plotFrame(A, frameName, quiverLineStyle)
            % plotFrame - Plot homogeneous matrix as a frame
            % 
            % Syntax
            %   plotFrame(A)
            %   plotFrame(A, frameName)
            %   plotFrame(A, frameName, quiverLineStyle)
            %
            % Input:
            %   A - homogeneous matrix
            %       belong to SE(3) | double(4, 4)
            %   frameName - name of the frame
            %       default = '' | char array or string
            %   quiverLineStyle - style of arrow stem
            %       in {'-', '--', ':', '-.', 'none'} | defualt = '-' | char(1, 1) or string
            %       See also matlab.graphics.chart.primitive.Area.LineStyle

            arguments
                A {mustBeSE3}
                frameName {mustBeA(frameName, {'char', 'string'})} = ''
                quiverLineStyle {mustBeMember(quiverLineStyle, {'-', '--', ':', '-.', 'none'})} = '-'
            end
            
            % Length of frame axis
            L = 0.5;
            R = L*A(1:3, 1:3); T = A(1:3, 4);
            colorQuiver = 'rgb'; hold on
            for k = 1:3
                q = quiver3(T(1), T(2), T(3), R(1, k), R(2, k), R(3, k), ...
                    'Color', colorQuiver(k), 'LineWidth', 2, 'LineStyle', quiverLineStyle);
                q.DataTipTemplate.DataTipRows = [dataTipTextRow('Frame: ', {frameName}), ...
                        dataTipTextRow('Pos:   ', {T.'}), dataTipTextRow('RPY:   ', {rotm2eul(R)})];
                if k == 1, q.Marker = '.'; q.MarkerSize = 15; q.MarkerEdgeColor = 'k'; end
            end
            hold off
        end
    end
end

% --- Validating function --- %
function mustBeSE3(data)
    % mustBESE3 - Validate that data is in the SE(3) group
    R = data(1:3, 1:3);
    if (~all(data(4, :) == [0, 0, 0, 1]) || ~(max(abs(R.'*R - eye(3)), [], 'all') < 1e-5) ...
            || ~(abs(det(R) - 1) < 1e-5))
        throw(MException('roboJoint:WrongValue', 'Must belong to SE(3) group'));
    end
end