classdef tools
    % TOOLS Static class for utilities implementation
    %   Implements all general tools useful for problem definition and
    %   visualization
    %
    % tools Methods:
    %   addCasadiToPath - Add the casadi folder to Matlab path
    %   ADHparams - Generate roto-translation given DH params
    %   plotFrame - Plot homogeneous matrix as a frame
    %   plotCoM - Plot CoM element identifier
    %   vrrotvec - Calculate a rotation between two vectors (Matlab copy)

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
            %   searchPath - Folder containing the 'casadi*' folder (absolute path)
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
            %   A - Homogeneous matrix
            %       belong to SE(3) | double(4, 4)
            %   frameName - Name of the frame
            %       default = '' | char array or string
            %   quiverLineStyle - Style of arrow stem
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
                        dataTipTextRow('Pos:   ', {T.'}), dataTipTextRow('RPY:   ', {rad2deg(rotm2eul(R, 'XYZ'))})];
                if k == 1, q.Marker = '.'; q.MarkerSize = 15; q.MarkerEdgeColor = 'k'; end
            end
            hold off
        end

        % --------------------------------------------------------------- %

        function plotCoM(pos)
            % plotCoM - Plot CoM element identifier
            %
            % Syntax
            %   plotCoM(pos)
            %
            % Input:
            %   pos - Position o fthe Center of Mass (CoM)
            %       [m, m, m] | double(1, 3)
            
            r = 0.1; hold on; [x, y, z] = sphere(8);
            z_tmp = z; z_tmp((sign(x).*sign(y) > 0) & (abs(x) > 1e-3)) = nan;
            surf(r*x + pos(1), r*y + pos(2), r*z_tmp + pos(3), 'FaceColor', 'k', 'EdgeColor', 'none')
            
            z_tmp = z; z_tmp((sign(x).*sign(y) < 0) & (abs(x) > 1e-3)) = nan;
            surf(r*x + pos(1), r*y + pos(2), r*z_tmp + pos(3), 'FaceColor', 'y', 'EdgeColor', 'none')
            hold off
        end

        % --------------------------------------------------------------- %

        function r = vrrotvec(a, b, options)
            %VRROTVEC Calculate a rotation between two vectors.
            %   R = VRROTVEC(A, B) calculates a rotation needed to transform 
            %   a 3d vector A to a 3d vector B.
            %
            %   R = VRROTVEC(A, B, OPTIONS) calculates the rotation with the default 
            %   algorithm parameters replaced by values defined in the structure
            %   OPTIONS.
            %
            %   The OPTIONS structure contains the following parameters:
            %
            %     'epsilon'
            %        Minimum value to treat a number as zero. 
            %        Default value of 'epsilon' is 1e-12.
            %
            %   The result R is a 4-element axis-angle rotation row vector.
            %   First three elements specify the rotation axis, the last element
            %   defines the angle of rotation.
            %
            %   See also VRROTVEC2MAT, VRROTMAT2VEC, VRORI2DIR, VRDIR2ORI.
            
            %   Copyright 1998-2018 HUMUSOFT s.r.o. and The MathWorks, Inc.
            
            % test input arguments
            narginchk(2, 3);
            
            if ~isnumeric(a) || ~isreal(a)
              error(message('sl3d:vrdirorirot:argnotreal'));
            end
            
            if (length(a) ~= 3)
              error(message('sl3d:vrdirorirot:argbaddim', 3));
            end
            
            if ~isnumeric(b) || ~isreal(b)
              error(message('sl3d:vrdirorirot:argnotreal'));
            end
            
            if (length(b) ~= 3)
              error(message('sl3d:vrdirorirot:argbaddim', 3));
            end
            
            if nargin == 2
              % default options values
              epsilon = 1e-12;
            else
              if ~isstruct(options)
                 error(message('sl3d:vrdirorirot:optsnotstruct'));
              else
                % check / read the 'epsilon' option
                if ~isfield(options,'epsilon') 
                  error(message('sl3d:vrdirorirot:optsfieldnameinvalid')); 
                elseif (~isreal(options.epsilon) || ~isnumeric(options.epsilon) || options.epsilon < 0)
                  error(message('sl3d:vrdirorirot:optsfieldvalueinvalid'));   
                else
                  epsilon = options.epsilon;
                end
              end
            end
            
            % compute the rotation, vectors must be normalized
            an = sl3dnormalize(a, epsilon);
            bn = sl3dnormalize(b, epsilon);
            
            % test for zero input argument magnitude after normalize to take epsilon 
            % into account
            if (~any(an) || ~any(bn))
              error(message('sl3d:vrdirorirot:argzeromagnitude'));
            end
            
            ax = sl3dnormalize(cross(an, bn), epsilon);
            % min to eliminate possible rounding errors that can lead to dot product >1
            angle = acos(min(dot(an, bn), 1));
            
            % if cross(an, bn) is zero, vectors are parallel (angle = 0) or antiparallel
            % (angle = pi). In both cases it is necessary to provide a valid axis. Let's
            % select one that satisfies both cases - an axis that is perpendicular to
            % both vectors. We find this vector by cross product of the first vector 
            % with the "least aligned" basis vector.
            if ~any(ax)
                absa = abs(an);
                [~, mind] = min(absa);
                c = zeros(1,3);
                c(mind) = 1;
                ax = sl3dnormalize(cross(an, c), epsilon);
            end
            
            % Be tolerant to column vector arguments, produce a row vector
            r = [ax(:)' angle];
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

% --- Fucntions --- %
function vec_n = sl3dnormalize(vec, maxzero)
    %SL3DNORMALIZE Normalize a vector.
    %   Y = SL3DNORMALIZE(X,MAXZERO) returns a unit vector Y parallel to the 
    %   input vector X. Input X can be vector of any size. If the modulus of
    %   the input vector is <= MAXZERO, the output is set to zeros(size(X)).
    %
    %   Not to be called directly.
    
    %   Copyright 1998-2008 HUMUSOFT s.r.o. and The MathWorks, Inc.
    
    norm_vec = norm(vec);
    if (norm_vec <= maxzero)
      vec_n = zeros(size(vec));
    else
      vec_n = vec ./ norm_vec;
    end
end