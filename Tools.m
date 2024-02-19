classdef Tools
    % TOOLS Static class for utilities implementation
    %   Implements all general tools useful for folder management, 
    %   problem definition and object visualization
    %
    % Tools Methods:
    %   addCasADiToPath - Add CasADi folder to Matlab path
    %   checkCasADi - Check if CasADi folder is already in Matlab path
    %   cmpDynParams - Compute dyn. params (I and CoM) from triangulation
    %   inertiaConv - Inertia matrix-vector conversion
    %   isAxis - Check if data is member of {'x', 'y', 'z'} group or
    %   mustBeAxis - Validate that data is axis
    %   mustBeEmpty - Validate that data is empty
    %   mustBeNonzeroNorm - Validate that data has non zero norm
    %   mustBeSE3 - Validate that data is an homogeneous matrix (SE(3)    
    %   mustHaveField - Validate that data has the desired filed
    %   mustHaveSize - Validate that data has specific dimension
    %   mustOr - Validate if data is ... or ...
    %   plotFrames - Plot frame, passed as homogeneous matrix
    %   rotTra - Check or generate roto-translation from data
    %   skew - Convert a 3d vector in its skew symmetric matrix representation

    properties (Constant = true, Access = private, Hidden = true)
        % tim - Timer used to avoid multiple error report
        %   timer('StartDelay', 1, 'TimerFcn', @(~, ~) tim.stop) | timer
        tim = timer('StartDelay', 1, 'TimerFcn', @(~, ~) fprintf(''))
    end

    % -------------------------------------------------- %

    methods (Static)
        function addCasADiToPath(searchPath)
            % addCasADiToPath - Add CasADi folder to Matlab path
            %   Two available options:
            %   - Search in the userpath Matlab specific path (or in the 
            %      passed path) for a folder name 'casadi*'
            %   - Use UI (User Interface) to select directly the CasADi folder
            %
            % Syntax
            %   addCasADiToPath
            %   addCasADiToPath(searchPath)
            %   addCasADiToPath('UI')
            %
            % Input:
            %   searchPath - Folder containing the 'casadi*' folder (absolute path)
            %       default = userpath | char array or string

            arguments 
                searchPath {Tools.mustOr(searchPath, 'mustBeFolder', {'mustBeMember', 'UI'})} = userpath
            end

            % Avoid re-search if already in path
            if contains(path, 'casadi', 'IgnoreCase', ispc), return, end

            switch searchPath
                case 'UI' % User Interface
                    searchPath = uigetdir(userpath, 'Select CasADi folder');
                otherwise % Different system calls to extract folder in path
                    if ispc % Windows                                           % ----- TO IMPLEMENT ----- %
                    elseif isunix % Linux
                        [status, files] = system(['ls ', searchPath]);
                    elseif ismac % Mac                                          % ----- TO IMPLEMENT ----- %
                    else, throw(MException('Tools:NotimplementedSystem', ...
                            'Not recognized system. Available systems: Windows, Linux, Mac'));
                    end

                    % Extract if exists casadi folder
                    if status % fail system call
                        throw(MException('Tools:MissingFolder', ...
                            'Missing ''casadi*'' folder in path %s', searchPath))
                    end
                    files = strsplit(files, {' ', '\n'});
                    for k = 1:length(files)
                        if startsWith(files{k}, 'casadi'), files = files{k}; break, end
                    end
                    if isa(files, 'cell') % missing folder
                        throw(MException('Tools:MissingFolder', ...
                            'Missing ''casadi*'' folder in path %s', searchPath))
                    end
                    searchPath = [searchPath, '/', files];
            end

            % Add casadi* folder path
            lastwarn(''); addpath(searchPath)
            if ~isempty(lastwarn), throw(MException('Tools:addpathError', ...
                    'Error during addpath(%s)', searchPath)), end
        end

        % ------------------------- %

        function checkCasADi
            % checkCasADi - Check if CasADi folder is already in Matlab path
            %
            % Syntax
            %   checkCasADi

            if contains(path, 'casadi', 'IgnoreCase', ispc), return, end
            if strcmp(Tools.tim.Running, 'off')
                Tools.tim.start
                warning('Required CasADi toolbox. See Tools.addCasADiToPath to help adding CasADi folder.')
            end
        end

        % ------------------------- %

        function [CoM, I, swarm, tri] = cmpDynParams(tri, swarm, params)
            % cmpDynParams - Compute dynamics parameters (inertia and CoM)
            %   from triangulation throught max optimization (density = const., mass = 1kg)
            %
            % Syntax
            %   [CoM, I, tri] = cmpDynParams(tri)
            %   ... = cmpDynParams(tri, swarm)
            %   ... = cmpDynParams(tri, swarm, params)
            %
            % Input:
            %   tri - Triangulation representing the component
            %       triangulation
            %   swarm - Problem solution
            %           Used to continue the iteratie convergence of the algorithm
            %       double(:, 3)
            %   params - Struct with the following parameters
            %       verbose - Progression and plots
            %           default = false | logical
            %       cycle - Number of optimization cycles
            %           deafult = 100 | double(1, 1)
            %       n_particle - Number of particles approximating the component
            %           defualt = max(1.1*# of vertices, 100) | double(1, 1)
            %       cost - Cost function to optimize during the particle positioning
            %           deafult = @(x, swarm) min(sum((swarm - x).^2, 2)) (max(min(distances))) | function_handle
            % Output:
            %   CoM - Center of Mass
            %       m (3x1) | double(3, 1)
            %   I - Inertia matrix ([Ixx, Iyy, Izz, Iyz, Ixz, Ixy].')
            %       kg m^2 (6x2) | double(6, 2)
            %   swarm - Problem solution
            %           Used to continue the iteratie convergence of the algorithm
            %       double(:, 3)
            %   tri - Triangulation representing the component
            %       If the component presents some points not used, the algorithm will delete them
            %       triangulation

            arguments (Input)
                tri {mustBeA(tri, 'triangulation')}
                swarm {mustBeReal} = []
                params {mustBeA(params, 'struct')} = struct('verbose', false, ...
                    'cycle', 100, 'n_particles', )
            end           

            % Delete non-connected points
            P = tri.Points; K = tri.ConnectivityList;
            uniqueP = unique(K);
            if size(uniqueP, 1) ~= size(P, 1)
                for k = 1:size(uniqueP, 1), P(k, :) = P(uniqueP(k), :); K(K == uniqueP(k)) = k; end
                tri = trangulation(K, P);
            end

            % Settings
            if isempty(cycle), cycle = 100; end
            try Tools.mustHaveSize(swarm, [n_particle, size(tri.Points, 2)]), catch, swarm = []; end
            if verbose, fprintf(' --- Compute dynamic parmas. --- \n'), end

            % Initialization
            direction = rand(3, 1);
            if isempty(swarm), swarm = tri.Points(randi(size(tri.Points, 1), n_particle, 1), :); end
            cost_s = zeros(size(swarm, 1), 1);
            for k = 1:length(cost_s), cost_s(k) = cost(swarm(k, :), swarm([1:k-1, k+1:end], :)); end
            still = zeros(size(swarm, 1), 1); stillIter = 20; % --------------------------------------------------

            if verbose
                h = figure;
                trimesh(tri, 'EdgeColor', [0.8500 0.3250 0.0980], 'FaceAlpha', 0.1)
                grid on, axis equal, axis padded, hold on, plot3(swarm(:, 1), swarm(:, 2), swarm(:, 3), '.', ...
                    'MarkerSize', 15, 'Color', [0 0.4470 0.7410]), hold off; drawnow
            end

            % Decomposition
            if verbose, fprintf('Compute boundary conditions: '), strLenght = 0; end
            dA = {size(tri.ConnectivityList, 1), 1};
            for k = 1:size(tri.ConnectivityList, 1)
                if verbose
                    fprintf(repmat('\b', 1, strLenght))
                    strLenght = fprintf('Iter %d of %d', k, size(tri.ConnectivityList, 1)); 
                end
                dA{k} = decomposition([[tri.Points(tri.ConnectivityList(k, :), :).'; 1, 1, 1], [-direction; 0]], 'auto');
            end
            if verbose, fprintf(repmat('\b', 1, strLenght)), fprintf('done\n'); end
            
            % Optimization procedure
            if verbose, fprintf('Compute optimization: '), strLenght = 0; end
            for iter = 1:cycle
                if verbose
                    fprintf(repmat('\b', 1, strLenght))
                    strLenght = fprintf('Iter %d of %d', iter, cycle); 
                end
                
                % New elements
                sum_faces = zeros(size(swarm, 1), 1); swarm_new = swarm;
                for k = 1:size(swarm, 1)
                    if (still(k) > stillIter) && (cost_s(k) == 0)
                        % Resample if too static
                        still(k) = 0; swarm_new(k, :) = tri.Points(randi(size(tri.Points, 1), 1, 1), :);
                    end
                    index_1 = randi(size(swarm, 1)-1, 1, 1); index_1 = index_1 + (index_1 >= k);
                    index_2 = randi(size(swarm, 2), 1, 1);
                    swarm_new(k, index_2) = swarm_new(k, index_2) + ...
                        (2*rand - 1)*(swarm_new(k, index_2) - swarm(index_1, index_2));
                end
                
                % Check faces
                for k = 1:length(dA)
                    sum_faces = sum_faces + all((dA{k}\[swarm_new.'; ones(1, size(swarm_new, 1))]) > 0, 1).';
                end
                change = (mod(sum_faces, 2) == 0); swarm_new(change, :) = swarm(change, :);

                % Update policy
                for k = 1:size(swarm ,1)
                    cost_new = cost(swarm_new(k, :), swarm([1:k-1, k+1:end], :));
                    if cost_new >= cost_s(k)
                        swarm(k, :) = swarm_new(k, :); cost_s(k) = cost_new;
                    end
                end

                % Resample if static
                still(cost_s == 0) = still(cost_s == 0) + 1;
            end
            if verbose, fprintf(repmat('\b', 1, strLenght)), fprintf('done\n'); end

            % Figure
            if verbose
                figure(h)
                trimesh(tri, 'EdgeColor', [0.8500 0.3250 0.0980], 'FaceAlpha', 0.1)
                grid on, axis equal, axis padded, hold on, plot3(swarm(:, 1), swarm(:, 2), swarm(:, 3), '.', ...
                    'MarkerSize', 15, 'Color', [0 0.4470 0.7410]), hold off
            end

            CoM = mean(swarm).'; xyz = swarm - CoM.';
            I = [sum(xyz(:, [2, 3]).^2, 'all'), sum(xyz(:, [1, 3]).^2, 'all'), sum(xyz(:, [1, 2]).^2, 'all'), ...
                -sum(xyz(:, 2).*xyz(:, 3), 'all'), -sum(xyz(:, 1).*xyz(:, 3), 'all'), -sum(xyz(:, 1).*xyz(:, 2), 'all')];
            I = I/size(swarm, 1);
        end


        % ------------------------- %

        function I_conv = inertiaConv(I)
            % inertiaConv - Inertia matrix-vector conversion. Check also if
            % data represents an admissible inertia matrix
            %   (Inertia vector [Ixx, Iyy, Izz, Iyz, Ixz, Ixy])
            %
            % Syntax
            %   I_conv = inertiaConv(I)
            %
            % Input:
            %   I - Inertia matrix (vector)
            %       double(3, 3) (double(6, 1))
            % Output:
            %   I_conv - Inertia vector (matrix)
            %       double(6, 1) (double(3, 3))
            
            arguments, I {mustBeReal, Tools.mustOr(I, {'mustHaveSize', [6, 1]}, ...
                    {'mustHaveSize', [3, 3]})}, end

            if isequal(size(I), [3, 3])
                if det(I) < 0, throw(MException('Tools:WrongData', ...
                        'Inertia matrices shold be positive semi-definite. Check data'))
                end
                I_conv = [diag(I), I(2, 3), I(1, 3), I(1, 2)];
            else
                I_conv = diag(I(1:3)) + squareform(flip(I(4:end)));
                if det(I_conv) < 0, throw(MException('Tools:WrongData', ...
                        'Inertia matrices must be positive semi-definite. Check data'))
                end
            end
        end

        % ------------------------- %

        function A = skew(w)
            % skew - Convert a 3d vector in its skew symmetric matrix representation
            %
            % Syntax
            %   A = skew(w)
            %
            % Input:
            %   w - Vector
            %       ...(3, 1) or ...(1, 3)
            % Output:
            %   A - Skew symmetric matrix
            %       ...(3, 3)

            arguments, w {mustBeVector}, end
            if size(w, 1) ~= 1, w = w.'; end
            Tools.mustHaveSize(w, [1, 3])

            A = [[0 -w(3), w(2)]; [w(3), 0, -w(1)]; [-w(2), w(1), 0]];
        end

        % ------------------------- %

        function plotFrames(A, text, specifics)
            % plotFrames - Plot frame, passed as homogeneous matrix
            %
            % Syntax
            %   plotFrames(A)
            %   plotFrames(A, text)
            %   plotFrames(A, text, specific1, specific2, ...)
            %
            % Input:
            %   A - Homogeneous matrix frame
            %       belong to SE(3) | double(4, 4)
            %   text - Frame's name
            %       default = '' | char array or string
            %   specifics - Quiver style
            %       default = '-' | char array
            %       See also matlab.graphics.chart.primitive.Quiver
            
            arguments, A {Tools.mustBeSE3}, text {mustBeTextScalar} = '', end
            arguments (Repeating), specifics, end

            if isempty(specifics), specifics = {'-'}; end

            % Frame axis length
            L = 0.5;

            R = L*A(1:3, 1:3); T = A(1:3, 4);
            colorAxis = 'rgb'; hold on
            for k = 1:3
                q = quiver3(T(1), T(2), T(3), R(1, k), R(2, k), R(3, k), ...
                    specifics{:}, 'Color', colorAxis(k), 'LineWidth', 2);
                q.DataTipTemplate.DataTipRows = [dataTipTextRow('Frame: ', {text}), ...
                        dataTipTextRow('Pos [m]: ', {T.'}), dataTipTextRow('RPY [deg]: ', {rotm2eul(R, 'XYZ')*180/pi})];
                if k == 1, q.Marker = '.'; q.MarkerSize = 15; q.MarkerEdgeColor = 'k'; end
            end
            xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]'), axis equal, hold off
        end


        % ------------------------- %

        function mustOr(data, checks)
            % mustOr - Validate if data is ... or ...
            %
            % Syntax
            %   mustOr(data, check1, check2, ...)
            %
            % Input:
            %   data - Data to check
            %   check - Cell array containing name of the check and other params
            %       e.g. {'mustBeMember', {'a', 'b', 'c'}} | {'mustBeNumeric'}

            arguments, data, end
            arguments (Repeating), checks {mustBeA(checks, {'cell', 'char', 'string'})}, end

            if isempty(checks), return, end

            errorReport = '';
            for k = 1:length(checks)
                try
                    if ~isa(checks{k}, 'cell'), checks{k} = checks(k); end
                    multiInput = false;
                    switch checks{k}{1}
                        % Numeric Value Attributes
                        case 'mustBePositive', mustBePositive(data)
                        case 'mustBeNonpositive', mustBeNonpositive(data)
                        case 'mustBeNonnegative', mustBeNonnegative(data)
                        case 'mustBeNegative', mustBeNegative(data)
                        case 'mustBeFinite', mustBeFinite(data)
                        case 'mustBeNonNan', mustBeNonNan(data)
                        case 'mustBeNonzero', mustBeNonzero(data)
                        case 'mustBeNonsparse', mustBeNonsparse(data)
                        case 'mustBeSparse', mustBeSparse(data)
                        case 'mustBeReal', mustBeReal(data)
                        case 'mustBeInteger', mustBeInteger(data)
                        case 'mustBeNonmissing', mustBeNonmissing(data)
                        % mustHaveField - Validate that data has the desired filed
                        % Comparison with Other Values
                        case 'mustBeGreaterThan', multiInput = true; mustBeGreaterThan(data, checks{k}{2})
                        case 'mustBeLessThan', multiInput = true; mustBeLessThan(data, checks{k}{2})
                        case 'mustBeGreaterThanOrEqual', multiInput = true; mustBeGreaterThanOrEqual(data, checks{k}{2})
                        case 'mustBeLessThanOrEqual', multiInput = true; mustBeLessThanOrEqual(data, checks{k}{2})
                        
                        % Data Types
                        case 'mustBeA', multiInput = true; mustBeA(data, checks{k}{2})
                        case 'mustBeNumeric', mustBeNumeric(data)
                        case 'mustBeNumericOrLogical', mustBeNumericOrLogical(data)
                        case 'mustBeFloat', mustBeFloat(data)
                        case 'mustBeUnderlyingType', multiInput = true; mustBeUnderlyingType(data, checks{k}{2})

                        % Size
                        case 'mustBeNonempty', mustBeNonempty(data)
                        case 'mustBeScalarOrEmpty', mustBeScalarOrEmpty(data)
                        case 'mustBeVector', mustBeVector(data)

                        % Membership and Range
                        case 'mustBeMember', multiInput = true; mustBeMember(data, checks{k}{2}) 
                        case 'mustBeInRange', multiInput = true; mustBeInRange(data, checks{k}{2}, checks{k}{3}, checks{k}{4}, checks{k}{5})
                        
                        % Text
                        case 'mustBeFile', mustBeFile(data)
                        case 'mustBeFolder', mustBeFolder(data)
                        case 'mustBeNonzeroLengthText', mustBeNonzeroLengthText(data)
                        case 'mustBeText', mustBeText(data) 
                        case 'mustBeTextScalar', mustBeTextScalar(data)
                        case 'mustBeValidVariableName', mustBeValidVariableName(data) 
                        

                        % Custom
                        case 'mustBeAxis', Tools.mustBeAxis(data)
                        case 'mustBeNonzeroNorm', Tools.mustBeNonzeroNorm(data)
                        case 'mustBeSE3', Tools.mustBeSE3(data)
                        case 'mustHaveSize', multiInput = true; Tools.mustHaveSize(data, checks{k}{2})
                        case 'mustBeEmpty', Tools.mustBeEmpty(data)
                        case 'mustHaveField', Tools.mustHaveField(data, checks{k}{2})    
                        % case '',

                        otherwise
                            % Check feasibility of 
                            try mustBeTextScalar(checks{k}{1})
                                fprintf('%s: not inside the implemented checks\n', checks{k}{1});
                            catch, throw(MException('Tools:NotParamCheck', ...
                                'Required variable check not feasible. Pass char array or string as identifier'));
                            end
                    end
                    return
                catch
                    % Error description
                    if multiInput
                        if isnumeric(checks{k}{2}), checks{k}{2} = ['[', num2str(checks{k}{2}), ']']; end
                        checks{k}{2} = strcat(checks{k}{2}, ''', '); checks{k}{2} = strcat('''', checks{k}{2});
                        if ~isa(checks{k}{2}, 'cell'), checks{k}{2} = checks{k}(2); end
                        checks{k}{2} = strjoin(checks{k}{2}); checks{k}{2} = checks{k}{2}(1:end-1);
                        stringTmp = [checks{k}{1}, '(..., {', checks{k}{2}, '}), '];
                    else, stringTmp = [checks{k}{1}, ', '];
                    end

                    errorReport(end + (1:length(stringTmp))) = stringTmp;
                end
            end

            throw(MException('Tools:MustOr', ['Validation not passed.' ...
                '\nValidation to pass %s'], errorReport(1:end-2)));            
        end

        % ------------------------- %

        function mustHaveField(data, field)
            % mustHaveField - Validate that data has the desired filed
            %
            % Syntax
            %   mustHaveField(data, field)
            %
            % Input:
            %   data - Data to validate
            %       struct
            %   field - Cell of char arry/string containing the fileds
            %       cell(char array or string)

            arguments
                data {mustBeA(data, 'struct')}
                field {mustBeText}
            end


            if all(isfield(data, field)), return
            else
                str = strcat(field, ''', '); str = strcat('''', str);
                str = strjoin(str); str = str(1:end-1);
                str1 = fieldnames(data); str1 = strcat(str1, ''', '); str1 = strcat('''', str1);
                str1 = strjoin(str1); str1 = str1(1:end-1);
                throw(MException('Tools:WrongData', 'Required fileds: {%s}\nActual fields: {%s}', ...
                    str, str1))
            end
        end

        % ------------------------- %

        function mustBeEmpty(data)
            % mustBeEmpty - Validate that data is empty
            %
            % Syntax
            %   mustBeEmpty(data)
            %
            % Input:
            %   data - Data to validate

            try
                mustBeNonempty(data)
                throw(MException('Tools:WrongData', 'Data must be empty'))
            catch, return
            end
        end

        % ------------------------- %

        function mustHaveSize(data, dim, text)
            % mustHaveSize - Validate that data has specific dimension
            %
            % Syntax
            %   mustHaveSize(data, dim)
            %
            % Input:
            %   data - Data to validate
            %       double(n1, n2, ...) or char(n1, n2, ...)
            %   dim - Dimension to check with
            %       Integer(1, m)

            arguments, data, dim {mustBeInteger, mustBeVector}, text {mustBeTextScalar} = '', end
            
            if size(dim, 1) ~= 1, dim = dim.'; end

            if isequal(size(data), dim), return
            else
                if isempty(text), text = '''data'''; else, text = ['', text, '']; end
                throw(MException('Tools:WrongDim', [text, ' does not match the required dimension.' ...
                    '\nActual dimension = [%s], Desired = [%s]'], num2str(size(data)), num2str(dim)))
            end
        end

        % ------------------------- %

        function mustBeNonzeroNorm(data)
            % mustBeNonzeroNorm - Validate that data has non zero norm
            %   Calls mustBeVector, mustBeReal
            %
            % Syntax
            %   mustBeNonzeroNorm(data)
            %
            % Input:
            %   data - Data to validate
            %       double()

            mustBeVector(data), mustBeReal(data)
            if norm(data), return, end
            throw(MException('Tools:ZeroNorm', 'Data must have non-zero norm'))
        end

        % ------------------------- %

        function mustBeSE3(data)
            % mustBeSE3 - Validate that data is an homogeneous matrix (SE(3))
            %   Calls mustBeReal, mustBeNonNan, Tools.mustHaveSize(data, [4, 4])
            %
            % Syntax
            %   mustBeSE3(data)
            %
            % Input:
            %   data - Data to validate
            %       double()

            threshold = 1e-5;

            mustBeReal(data), mustBeNonNan(data), Tools.mustHaveSize(data, [4, 4])
            R = data(1:3, 1:3);
            if any(abs(data(4, :) - [0, 0, 0, 1]) > threshold, 'all') || ...
                any(abs(R*(R.') - eye(3)) > threshold, 'all') || any(abs((R.')*R - eye(3)) > threshold, 'all') || ...
                (abs(det(R) - 1) > threshold)
                throw(MException('Tools:WrongValue', 'Must belong to SE(3) group'))
            end
        end

        % ------------------------- %

        function mustBeAxis(data)
            % mustBeAxis - Validate that data is axis
            %   Calls Tools.isAxis
            %
            % Syntax
            %   mustBeAxis(data)
            %
            % Input:
            %   data - Data to validate
            %       double()

            if any(isnan(Tools.isAxis(data)))
                throw(MException('Tools:WrongValue', ['Data must be member of ' ...
                    '{''x'', ''y'', ''z''} or numeric triplet double(3, 1)']))
            end
        end
        
        % ------------------------- %

        function axis = isAxis(data)
            % isAxis - Check if data is member of {'x', 'y', 'z'} group or
            %   is a vector triplet and return the correspondednt normalized
            %   vector
            %   Calls {Tools.mustBeNonzeroNorm, mustBeNonNan, Tools.mustHaveSize(..., [3, 1])}
            %       or {mustBeMember(..., {'x', 'y', 'z'})}
            %       
            % Syntax
            %   axis = isAxis(data)
            %
            % Input:
            %   data - Data to validate
            %       double(3, 1) or {char array or string}
            % Output:
            %   axis - Nomralized corresponding axis (nan if fail some checks)
            %       double(3, 1)
            arguments (Output)
                axis {Tools.mustHaveSize(axis, [3, 1])}
            end

            try
                Tools.mustBeNonzeroNorm(data), mustBeNonNan(data)
                Tools.mustHaveSize(data, [3, 1])
                axis = data/norm(data);
                return
            catch
                try
                    mustBeMember(data, {'x', 'y', 'z'})
                    switch data
                        case 'x', axis = [1, 0, 0].';
                        case 'y', axis = [0, 1, 0].';
                        case 'z', axis = [0, 0, 1].';
                    end
                    return
                catch
                end
            end
            axis = nan(3, 1);
        end

        % ------------------------- %

        function A = rotTra(data)
            % rotTra - Check or generate roto-translation from data
            % 
            % Syntax
            %   rotTra(data)
            %   
            % Input:
            %   data - Roto-transaltion data
            %       in SE(3) or Danavit-Hartenberg parms | double(4, 4) or double(1, 4)
            %       Denavit-Hartenberg params: [d, theta, a, alpha]
            % Output:
            %   A - HOmogeneous matrix representing the roto-translation
            %       in SE(3) | double(4, 4)

            arguments (Input)
                data {mustBeNonempty, mustBeFinite, Tools.mustOr(data, {'mustBeSE3'}, {'mustHaveSize', [1, 4]})}
            end
            arguments (Output), A {Tools.mustBeSE3}, end
            
            if all(size(data) == [1, 4])
                d = data(1); theta = data(2); a = data(3); alpha = data(4);

                A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta);
                     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                         0,              sin(alpha),              cos(alpha),          d;
                         0,                  0,                      0,                1];
            else, A = data;
            end
        end
    end
end