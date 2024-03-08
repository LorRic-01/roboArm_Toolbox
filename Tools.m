classdef Tools
    % TOOLS Static class for utilities implementation
    %   Implements all general tools useful for folder management, 
    %   problem definition and object visualization
    %
    % Tools Methods:
    %   addCasADiToPath - Add CasADi folder to Matlab path
    %   checkCasADi - Check if CasADi folder is already in Matlab path
    %   cmpDynParams - Compute dynamics parameters (CoM and inertia w.r.t CoM)
    %   convertToString - Convert data into string/char array
    %   inertiaConv - Inertia matrix-vector conversion
    %   invA - Compute inverse of homogeneous matrix
    %   isAxis - Check if data is an admissible axis
    %   isUnique - Check if data is composed only of unique data
    %   mustAndOr - Validate if data is ... {and, or} ...
    %   mustBeAxis - Validate that data represents axis
    %   mustBeEmpty - Validate that data is empty
    %   mustBeLogical - Validate that data is logical
    %   mustBeNonzeroNorm - Validate that data has non zero norm
    %   mustBeSE3 - Validate that data is an homogeneous matrix (SE(3))
    %   mustBeUnique - Validate that data is composed only of unique data
    %   mustHaveField - Validate that data has the desired filed
    %   mustHaveSize - Validate that data has specific dimension
    %   plotCoM - Plot CoM element identifier
    %   plotFrames - Plot frame, passed as homogeneous matrix
    %   plotTri - Plot triangulation
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
            %       Validation: mustBeTextScalar, mustBeFolder OR mustBeMember(..., 'UI')
            %
            % See also Tools.mustAndOr, mustBeTextScalar, mustBeFolder, mustBeMember
        
            arguments 
                searchPath {mustBeTextScalar, Tools.mustAndOr('or', searchPath, 'mustBeFolder', [], 'mustBeMember', 'UI')} = userpath
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
                    else, throw(MException('Tools:notImplementedSystem', ...
                            'Not recognized system. Available systems: Windows, Linux, Mac'));
                    end
        
                    % Extract if exists casadi folder
                    if status % fail system call
                        throw(MException('Tools:missingFolder', ...
                            'Missing ''casadi*'' folder in path %s', searchPath))
                    end
                    files = strsplit(files, {' ', '\n'});
                    for k = 1:length(files)
                        if startsWith(files{k}, 'casadi'), files = files{k}; break, end
                    end
                    if isa(files, 'cell') % missing folder
                        throw(MException('Tools:missingFolder', ...
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
                warning('Missing CasADi toolbox/folder. See Tools.addCasADiToPath to help adding CasADi folder.')
            end
        end

        % ------------------------- %

        function [CoM, I, swarm, tri] = cmpDynParams(tri, swarm, params)
            % cmpDynParams - Compute dynamics parameters (CoM and inertia w.r.t CoM)
            %   from triangulation throught max optimization
            %   ---------------------------------------------------------------------------------------------------- TO ADJUST
            %
            % Syntax
            %   [CoM, I, tri] = cmpDynParams(tri)
            %   ... = cmpDynParams(tri, swarm)
            %   ... = cmpDynParams(tri, swarm, params)
            %
            % Input:
            %   tri - Triangulation representing the component
            %       triangulation
            %       Validation: mustBeA(..., 'triangulation')
            %   swarm - Problem solution
            %           Used to continue the iteratie convergence of the algorithm
            %       double(:, 3)
            %       Validation: mustBeReal, mustBeFinite, mustBeNonNan
            %   params - Struct with the following parameters
            %       verbose - Print Command Window info and plots
            %           default = false | logical
            %       cycle - Number of optimization cycles
            %           deafult = 100 | double(1, 1)
            %       n_p - Number of particles approximating the component
            %           defualt = max(1.1*# of vertices, 100) | double(1, 1)
            %       cost - Cost function to optimize during the particle positioning
            %           deafult = @(x, swarm) min(sum((swarm - x).^2, 2)) | function_handle
            %       Validation: mustBeA(..., 'struct')
            % Output:
            %   CoM - Center of Mass
            %       m (3x1) | double(3, 1)
            %   I - Inertia matrix ([Ixx, Iyy, Izz, Iyz, Ixz, Ixy].')
            %       kg m^2 (6x2) | double(6, 2)
            %   swarm - Problem solution
            %           Used to continue the iteratie convergence of the algorithm
            %       double(:, 3)
            %   tri - Triangulation representing the component
            %       If the component has some points not used, the algorithm will delete them
            %       in the triangulation

            arguments (Input)
                tri {mustBeA(tri, 'triangulation')}
                swarm {mustBeReal, mustBeFinite, mustBeNonNan} = []
                params {mustBeA(params, 'struct')} = struct('verbose', false, ...
                    'cycle', 100, 'n_p', max(round(1.1*size(tri.Points, 1)), 100), ...
                    'cost', @(x, swarm) max(mink(sum((swarm - x).^2, 2), 2)))
            end           

            % --- Delete non-connected points or non-triangle --- %
            P = tri.Points; K = tri.ConnectivityList;
            mask = true(size(K, 1), 1);
            for k = 1:size(K, 1) % delete non triangular points
                v = diff(P(K(k, :), :)); mask(k) = norm(cross(v(1, :), v(2, :))) > 0;
            end
            K = K(mask, :); uniqueP = unique(K);
            if size(uniqueP, 1) ~= size(P, 1) % delete multiple points
                for k = 1:size(uniqueP, 1), P(k, :) = P(uniqueP(k), :); K(K == uniqueP(k)) = k; end
            end
            tri = triangulation(K, P(1:size(uniqueP, 1), :));

            % --- Params initialization --- %
            params_all = struct('verbose', false, ...
                    'cycle', 100, 'n_p', max(round(1.1*size(tri.Points, 1)), 100), ...
                    'cost', @(x, swarm) max(mink(sum((swarm - x).^2, 2), 2)));
            fields = fieldnames(params_all);
            for k = 1:length(fields)
                if ~isfield(params, fields(k)) || isempty(params.(fields{k}))
                    params.(fields{k}) = params_all.(fields{k});
                end
            end
            verbose = params.verbose; cycle = params.cycle; n_p = params.n_p; cost = params.cost;

            % --- Initialization --- %
            try Tools.mustHaveSize(swarm, [n_p, size(tri.Points, 2)]), catch, swarm = []; end
            if isempty(swarm), swarm = tri.Points(randi(size(tri.Points, 1), n_p, 1), :); end
            if verbose, fprintf(' --- Compute dynamic parmas. --- \n'), end

            direction = rand(3, 1);     % checking direction
            cost_s = zeros(size(swarm, 1), 1);
            for k = 1:length(cost_s), cost_s(k) = cost(swarm(k, :), swarm); end

            % --- Plot --- %
            if verbose
                h = figure; clf
                trimesh(tri, 'EdgeColor', [0.8500 0.3250 0.0980], 'FaceAlpha', 0.1, 'LineWidth', 0.1)
                grid on, axis equal, axis padded, hold on
                plot3(swarm(:, 1), swarm(:, 2), swarm(:, 3), '.', 'MarkerSize', 15, 'Color', [0 0.4470 0.7410])
                drawnow
            end

            % --- Bound matrix decomposition --- %
            if verbose, fprintf('Compute boundary conditions: '), strLenght = 0; end
            dA = cell(size(tri.ConnectivityList, 1), 1);
            for k = 1:size(tri.ConnectivityList, 1)
                if verbose, strLenght = fprintf([repmat('\b', 1, strLenght), 'Iter %d of %d'], k, size(dA, 1)) - strLenght; end
                dA{k} = decomposition([[tri.Points(tri.ConnectivityList(k, :), :).'; 1, 1, 1], [-direction; 0]], 'auto');
            end
            if verbose, fprintf(repmat('\b', 1, strLenght)), fprintf('done\n'); end
            
            % --- Optimization procedure --- %
            if verbose, fprintf('Compute optimization: '), strLenght = 0; end
            for iter = 1:cycle
                if verbose, strLenght = fprintf([repmat('\b', 1, strLenght), 'Iter %d of %d'], iter, cycle) - strLenght; end
                
                % New elements
                sum_faces = zeros(size(swarm, 1), 1); swarm_new = swarm;
                for k = 1:size(swarm, 1)
                    if (cost_s(k) == 0) && (iter > 1)
                        % Resample if too static
                        swarm_new(k, :) = tri.Points(randi(size(tri.Points, 1), 1, 1), :);
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
                    cost_new = cost(swarm_new(k, :), swarm);
                    if cost_new >= cost_s(k)
                        swarm(k, :) = swarm_new(k, :); cost_s(k) = cost_new;
                    end
                end

                % Permuting array
                perm = randperm(size(swarm, 1));
                swarm = swarm(perm, :); cost_s = cost_s(perm);
            end
            if verbose, fprintf(repmat('\b', 1, strLenght)), fprintf('done\n'); end

            % --- Plot --- %
            if verbose
                figure(h); clf
                trimesh(tri, 'EdgeColor', [0.8500 0.3250 0.0980], 'FaceAlpha', 0.1, 'LineWidth', 0.1)
                grid on, axis equal, axis padded, hold on
                plot3(swarm(:, 1), swarm(:, 2), swarm(:, 3), '.', 'MarkerSize', 15, 'Color', [0 0.4470 0.7410])
                drawnow
            end

            % --- Compute params --- %
            CoM = mean(swarm).'; xyz = swarm - CoM.';
            I = [sum(xyz(:, [2, 3]).^2, 'all'), sum(xyz(:, [1, 3]).^2, 'all'), sum(xyz(:, [1, 2]).^2, 'all'), ...
                -sum(xyz(:, 2).*xyz(:, 3), 'all'), -sum(xyz(:, 1).*xyz(:, 3), 'all'), -sum(xyz(:, 1).*xyz(:, 2), 'all')];
            I = I/size(swarm, 1);
        end

        % ------------------------- %

        function str = convertToString(data)
            % convertToString - Convert data into string/char array
            %
            % Syntax
            %   str = convertToString(data)
            %
            % Input:
            %   data - Data to convert
            % Output:
            %   str - String representing the data
            %       char array or string
           
            str = '';
            if isnumeric(data) || islogical(data)
                if isscalar(data), str = num2str(data); return
                elseif isvector(data)
                    if size(data, 1) ~= numel(data), str = ['[', num2str(data), ']']; return
                    else, str = ['[', num2str(data.'), ']^T']; return
                    end
                else
                    for k = 1:size(data, 1), str = [str, Tools.convertToString(data(k, :)), ';']; end
                    str = ['[', str(1:end-1), ']'];
                end
                return
            end
            if isstring(data), str = ['"', data, '"']; return, end
            if ischar(data), str = ['''', data, '''']; return, end
        
            if iscell(data)
                for k = 1:length(data)
                    data{k} = Tools.convertToString(data{k});
                end
                data = strcat(data, ', ');
                str = strjoin(data); str = ['{', str(1:end-1), '}']; return
            end
        
            warning('Not supported data conversion')
        end
        
        % ------------------------- %

        function I_conv = inertiaConv(I)
            % inertiaConv - Inertia matrix-vector conversion. Check also if
            % data represents an admissible inertia matrix
            %   (Inertia vector [Ixx, Iyy, Izz, Iyz, Ixz, Ixy])
            %
            % Syntax
            %   inertiaConv(I)
            %   I_conv = inertiaConv(I)
            %
            % Input:
            %   I - Inertia matrix (vector)
            %       double(3, 3) (double(6, 1) or double(1, 6))
            %       Validation: mustBeNumeric, mustBeReal, ...
            %           mustHaveSize(..., [6, 1]) OR mustHaveSize(..., [1, 6]) OR mustHaveSize(..., [3, 3])
            % Output:
            %   I_conv - Inertia vector (matrix)
            %       double(6, 1) (double(3, 3))
            %
            % See also mustBeNumeric, mustBeReal, Tools.mustAndOr, Tools.mustHaveSize
            
            arguments, I {mustBeNumeric, mustBeReal ...
                    Tools.mustAndOr('or', I, 'mustHaveSize', [6, 1], 'mustHaveSize', [1, 6], ...
                    'mustHaveSize', [3, 3])}, end

            if isequal(size(I), [3, 3])
                I_conv = [diag(I); I(2, 3); I(1, 3); I(1, 2)];
                if det(I) < 0, throw(MException('Tools:notInertia', ...
                        ['Inertia matrices shold be positive semi-definite. Check data' ...
                        '\nI = [Ixx, Iyy, Izz, Iyz, Ixz, Ixy] = %s'], Tools.convertToString(I_conv)))
                end
            else
                I_conv = diag(I(1:3)) + squareform(flip(I(4:end)));
                if det(I_conv) < 0, throw(MException('Tools:notInertia', ...
                        ['Inertia matrices shold be positive semi-definite. Check data' ...
                        '\nI = [Ixx, Iyy, Izz, Iyz, Ixz, Ixy] = %s'], Tools.convertToString(I)))
                end
            end
        end

        % ------------------------- %

        function A_inv = invA(A)
            % invA - Compute inverse of homogeneous matrix
            %
            % Syntax
            %   A_inv = invA(A)
            %
            % Input:
            %   A - Homogeneous matrix
            %       belong to SE(3)
            %       Validation: no validation but the inversion is based on
            %       the homogeneous matrix structure
            % Output:
            %   A_inv - Inverse homogeneous matrix
            %       belong to SE(3)
            
            A_inv = [A(1:3, 1:3).', (-A(1:3, 1:3).')*A(1:3, 4);
                0, 0, 0, 1];
        end

        % ------------------------- %
        
        function axis = isAxis(data)
            % isAxis - Check if data is member of {'x', 'y', 'z'} group or
            %   is a non-zero norm triplet and return the correspondednt normalized
            %   vector    
            %       
            % Syntax
            %   isAxis(data)
            %   axis = isAxis(data)
            %
            % Input:
            %   data - Data to validate
            %       double(3, 1) or {char array or string}
            %       Validation (mustBeNonzeroNorm AND mustHaveSize(..., [3, 1])) OR ...
            %           (mustBeTextScalar AND mustBeMember(..., {'x', 'y', 'z'}))
            % Output:
            %   axis - Normalized axis
            %       double(3, 1)
            %
            % See also Tools.mustAndOr, Tools.mustBeNonzeroNorm, Tools.mustHaveSize, mustBeMember, mustBeTextScalar
        
            arguments
                data {Tools.mustAndOr('or', data, ...
                    'mustAndOr', {'and', 'mustBeNonzeroNorm', [], 'mustHaveSize', [3, 1]}, ...
                    'mustAndOr', {'and', 'mustBeTextScalar', [], 'mustBeMember', {'x', 'y', 'z'}})}
            end
            
            if isnumeric(data)
                axis = data/norm(data);
            else
                switch data
                    case 'x', axis = [1, 0, 0].';
                    case 'y', axis = [0, 1, 0].';
                    case 'z', axis = [0, 0, 1].';
                end
            end
        end

        % ------------------------- %

        function [check, data_u] = isUnique(data)
            % isUnique - Check if data is composed only of unique data
            %   (Use unique function)
            %
            % Syntax
            %   check = isUnique(data)
            %   [check, data_u] = isUnique(data)
            %
            % Input:
            %   data - Data to validate
            % Output:
            %   check - True if data is composed only of unique data
            %       logic
            %   data_u - Unique data
            %
            % See also unique

            data_u = unique(data);
            if isequal(size(data_u), size(data))
                check = true; return
            end
            check = false;
        end

        % ------------------------- %

        function mustAndOr(andOr, data, checks, params)
            % mustAndOr - Validate that data is ... {and, or} ...
            %
            % Syntax
            %   mustAndOr(andOr, data, check1, param1, check2, param2, ...)
            %
            % Input:
            %   andOr - Selection between 'and'/'or' operator
            %       in {'and', 'or'} | char array or string
            %       Validation: mustBeMember(..., {'and', 'or'}), mustBeTextScalar
            %   data - Data to check
            %   check - Validation method passed as string
            %       e.g. 'mustBeMember', 'mustBeNumeric' | char array or string
            %       Validation: mustBeTextScalar
            %   param - Cell array containing name of the check and other params
            %       e.g. {'a', 'b', 'c'}
            %
            % See also mustBeMember, mustBeTextScalar
        
            arguments, andOr {mustBeMember(andOr, {'and', 'or'}), mustBeTextScalar}, data, end
            arguments (Repeating), checks {mustBeTextScalar}, params, end
        
            if isempty(checks), return, end
        
            errorReport = '';
            for k = 1:length(checks)
                try
                    done = true;
                    switch checks{k}
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
                        
                        % Comparison with Other Values
                        case 'mustBeGreaterThan', mustBeGreaterThan(data, params{k})
                        case 'mustBeLessThan', mustBeLessThan(data, params{k})
                        case 'mustBeGreaterThanOrEqual', mustBeGreaterThanOrEqual(data, params{k})
                        case 'mustBeLessThanOrEqual', mustBeLessThanOrEqual(data, params{k})
                        
                        % Data Types
                        case 'mustBeA', mustBeA(data, params{k})
                        case 'mustBeNumeric', mustBeNumeric(data)
                        case 'mustBeNumericOrLogical', mustBeNumericOrLogical(data)
                        case 'mustBeFloat', mustBeFloat(data)
                        case 'mustBeUnderlyingType', mustBeUnderlyingType(data, params{k})
        
                        % Size
                        case 'mustBeNonempty', mustBeNonempty(data)
                        case 'mustBeScalarOrEmpty', mustBeScalarOrEmpty(data)
                        case 'mustBeVector', mustBeVector(data)
        
                        % Membership and Range
                        case 'mustBeMember', mustBeMember(data, params{k}) 
                        case 'mustBeInRange', mustBeInRange(data, params{k})
                        
                        % Text
                        case 'mustBeFile', mustBeFile(data)
                        case 'mustBeFolder', mustBeFolder(data)
                        case 'mustBeNonzeroLengthText', mustBeNonzeroLengthText(data)
                        case 'mustBeText', mustBeText(data) 
                        case 'mustBeTextScalar', mustBeTextScalar(data)
                        case 'mustBeValidVariableName', mustBeValidVariableName(data) 
                        
                        % Custom
                        case 'mustAndOr',Tools.mustAndOr(params{k}{1}, data, params{k}{2:end})
                        case 'mustBeAxis', Tools.mustBeAxis(data)
                        case 'mustBeEmpty', Tools.mustBeEmpty(data)
                        case 'mustBeNonzeroNorm', Tools.mustBeNonzeroNorm(data)
                        case 'mustBeSE3', Tools.mustBeSE3(data)
                        case 'mustBeUnique', Tools.mustBeUnique(data)
                        case 'mustHaveField', Tools.mustHaveField(data, params{k})
                        case 'mustHaveSize', Tools.mustHaveSize(data, params{k})
                        case 'mustBeCellA', Tools.mustBeCellA(data, params{k})
                        case 'mustBeLogical', Tools.mustBeLogical(data)
                        % case '',
        
                        otherwise
                            warning('%s: not inside the implemented checks\n', checks{k});
                            done = false;
                    end
                    if done && strcmp(andOr, 'or'), return, end
                catch
                    % Error description
                    if ~isempty(params{k})
                        stringTmp = [checks{k}, '(..., ', Tools.convertToString(params{k}), '), '];
                    else, stringTmp = [checks{k}, ', '];
                    end
                    errorReport(end + (1:length(stringTmp))) = stringTmp;
                end
            end
            
            if strcmp(andOr, 'and') && isempty(errorReport), return, end
            if strcmp(andOr, 'and'), andOr = '''and'' yet'; else, andOr = '''or'''; end
            throw(MException('Tools:failCheck', ['Validation not passed.' ...
                '\nValidation %s to pass %s'], andOr, errorReport(1:end-2)));            
        end

        % ------------------------- %

        function mustBeAxis(data)
            % mustBeAxis - Validate that data represents axis
            %
            % Syntax
            %   mustBeAxis(data)
            %
            % Input:
            %   data - Data to validate
            %
            % See also Tools.isAxis
        
            if Tools.isAxis(data)
                throw(MException('Tools:notAxis', ['Data must be member of ' ...
                    '{''x'', ''y'', ''z''} or non-zero norm vector double(3, 1)']))
            end
        end

        % ------------------------- %

        function mustBeCellA(data, type)
            % mustBeCellA - Validate that data is a cell array of all the
            %   same type or empty
            %
            % Syntax
            %   mustBeCellA(data, type)
            %
            % Input
            %   data - Data to check
            %       cell
            %       Validation: mustBeA(..., 'cell')
            %   type - Type to check with
            %       char array or string
            %       Validation: mustBeTextScalar

            arguments, data {mustBeA(data, 'cell')}, type {mustBeTextScalar}, end

            if isempty(data), return, end
            try
                for k = 1:numel(data)
                    if isempty(data{k}), continue, end
                    mustBeA(data{k}, type)
                end
            catch, throw(MException('Tools:wrongType', 'All data must be of type %s', type))
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

            if ~isempty(data)
                throw(MException('Tools:notEmpty', 'Data must be empty'))
            end
        end

        % ------------------------- %
        
        function mustBeLogical(data)
            % mustBeLogical - Validate that data is logical
            %
            % Syntax
            %   mustBeLogical(data)
            %
            % Input:
            %   data - Data to validate

            if ~islogical(data)
                throw(MException('Tools.notLogical', 'Data must be logical'))
            end
        end

        % ------------------------- %

        function mustBeNonzeroNorm(data)
            % mustBeNonzeroNorm - Validate that data has non zero norm
            %
            % Syntax
            %   mustBeNonzeroNorm(data)
            %
            % Input:
            %   data - Data to validate
            %       double(:, :)
            %       Validation: mustBeNumeric, mustBeVector, mustBeReal, mustBeFinite
            %
            % See also mustBeNumeric, mustBeVector, mustBeReal, mustBeFinite
        
            arguments, data {mustBeNumeric, mustBeVector, mustBeReal, mustBeFinite}, end
        
            if norm(data) > 0, return, end
            throw(MException('Tools:zeroNorm', 'Data must have non-zero norm'))
        end

        % ------------------------- %

        function mustBeSE3(data)
            % mustBeSE3 - Validate that data is an homogeneous matrix (SE(3))
            %
            % Syntax
            %   mustBeSE3(data)
            %
            % Input:
            %   data - Data to validate
            %       double(4, 4)
            %       Validation: mustBeReal, mustBeFinite, mustBeNumeric, mustHaveSize(..., [4, 4])
            %
            % See also mustBeReal, mustBeFinite, mustBeNumeric, Tools.mustHaveSize

            arguments, data {mustBeReal, mustBeFinite, mustBeNumeric, Tools.mustHaveSize(data, [4, 4])}, end
            threshold = 1e-5;

            R = data(1:3, 1:3);
            if any(abs(data(4, :) - [0, 0, 0, 1]) > threshold, 'all') || ...
                any(abs(R*(R.') - eye(3)) > threshold, 'all') || any(abs((R.')*R - eye(3)) > threshold, 'all') || ...
                (abs(det(R) - 1) > threshold)
                throw(MException('Tools:notSE3', 'Must belong to SE(3) group'))
            end
        end

        % ------------------------- %

        function mustBeUnique(data)
            % mustBeUnique - Validate that data is composed only of unique data
            %
            % Syntax
            %   mustBeUnique(data)
            %
            % Input:
            %   data - Data to validate
            %
            % See also Tools.isUnique

            if ~Tools.isUnique(data)
                throw(MException('Tools:notUNique', 'Data must have unique data.\nData: %s%s', ...
                    Tools.convertToString(data), ''))
            end
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
            %       Validation: mustBeA(..., 'struct')
            %   field - Cell of char arry/string containing the fileds
            %       cell(char array or string)
            %       Validation: mustBeText
            %
            % See also mustBeA, mustBeText

            arguments
                data {mustBeA(data, 'struct'), mustBeNonempty}
                field {mustBeText}
            end

            if isempty(field), return, end
            if all(isfield(data, field)), return
            else
                throw(MException('Tools:missingFileds', 'Required fileds: %s\nActual fields: %s', ...
                    Tools.convertToString(field), Tools.convertToString(fieldnames(data))))
            end
        end

        % ------------------------- %

        function mustHaveSize(data, dim)
            % mustHaveSize - Validate that data has specific dimension
            %
            % Syntax
            %   mustHaveSize(data, dim)
            %
            % Input:
            %   data - Data to validate
            %   dim - Dimension to check with
            %       Integer(1, m)
            %       Validation: mustBeInteger, mustBeVector, mustBePositive
            %
            % See also mustBeInteger, mustBeVector, mustBePositive
        
            arguments, data, dim {mustBeInteger, mustBeVector, mustBePositive}, end
            
            if size(dim, 1) ~= 1, dim = dim.'; end
        
            if isequal(size(data), dim), return
            else
                throw(MException('Tools:wrongSize', ['Data does not match the required dimension.' ...
                    '\nActual dimension = %s, Desired = %s'], Tools.convertToString(size(data)), Tools.convertToString(dim)))
            end
        end

        % ------------------------- %

        function plotCoM(A, r)
            % plotCoM - Plot CoM element identifier
            %
            % Syntax
            %   plotCoM(pos)
            %   plotCoM(A)
            %   plotCoM(..., r)
            %
            % Input:
            %   A - Homogeneous matrix representing pose of Center of Mass (CoM)
            %       belong to SE(3) | double(4, 4)
            %   pos - Position of Center of Mass (CoM)
            %       m (3, 1) | double(3, 1)
            %       Validation: mustBeFinite, mustBeReal, Tools.mustHaveSize(..., [3, 1])
            %   r - Radius of the sphere representing CoM
            %       m | default = 0.05 | double(1, 1)
            %       Validation: mustBeFinite, mustBeReal, mustBeScalarOrEmpty

            arguments
                A {Tools.mustAndOr('or', A, 'mustBeSE3', [], ...
                    'mustAndOr', {'and', 'mustBeFinite', [], 'mustBeReal', [],...
                    'mustHaveSize', [3, 1]})}
                r {mustBeFinite, mustBeReal, mustBeScalarOrEmpty} = 0.05
            end
            
            if isempty(r), r = 0.05; end
            if ~isvector(A), A = A(1:3, 4); end

            hold on; [x, y, z] = sphere(8);
            z_tmp = z; z_tmp((sign(x).*sign(y) > 0) & (abs(x) > 1e-3)) = nan;
            surf(r*x + A(1), r*y + A(2), r*z_tmp + A(3), 'FaceColor', 'k', 'EdgeColor', 'none')
           

            z_tmp = z; z_tmp((sign(x).*sign(y) < 0) & (abs(x) > 1e-3)) = nan;
            surf(r*x + A(1), r*y + A(2), r*z_tmp + A(3), 'FaceColor', 'y', 'EdgeColor', 'none')
            hold off
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
            %       Validation: mustBeSE3
            %   text - Frame's name
            %       default = '' | char array or string
            %       Validation: mustBeTextScalar
            %   specifics - Quiver style
            %       default = '-' | char array
            %       See also matlab.graphics.chart.primitive.Quiver
            %
            % See also Tools.mustBeSE3, mustBeTextScalar
            
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

        function plotTri(tri, A, specifics)
            % plotTri - Plot triangulation
            %
            % Syntax
            %   plotTri(tri)
            %   plotTri(tri, A)
            %   plotTri(tri, A, specifics)
            %
            % Input:
            %   tri - Triangulation object
            %       triangulation
            %       Validation: mustBeA(..., 'triangulation')
            %   A - Homogeneous matrix frame
            %       belong to SE(3) | default = eye(4) | double(4, 4)
            %       Validation: mustBeSE3
            %   specifics - Surface style
            %       default = {'FaceColor', [0 0.4470 0.7410], 'EdgeColor', [0 0.4470 0.7410], 'FaceAlpha', 0.1}
            %       See also matlab.graphics.chart.primitive.Surface
            %
            % See also mustBeA, Tools.mustBeSE3
            
            arguments, tri {mustBeA(tri, 'triangulation')}, A {Tools.mustBeSE3} = eye(4), end
            arguments (Repeating), specifics, end

            if isempty(specifics), specifics = {'FaceColor', [0 0.4470 0.7410], ...
                    'EdgeColor', [0 0.4470 0.7410], 'FaceAlpha', 0.1}; end

            if ~isequal(A, eye(4))
                P = tri.Points; P = A*[P.'; ones(1, size(P, 1))];
                tri = triangulation(tri.ConnectivityList, P(1:3, :).');
            end

            hold on, trimesh(tri, specifics{:});
            xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]'), axis equal, hold off
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
            %       in SE(3) or Danavit-Hartenberg parms | double(4, 4) or double(1, 4) or double(4, 1)
            %       Denavit-Hartenberg params: [d, theta, a, alpha]
            %       Validation: mustBeFinite, mustBeSE3 OR mustHaveSize(..., [1, 4]) OR mustHaveSize(..., [4, 1])
            % Output:
            %   A - HOmogeneous matrix representing the roto-translation
            %       in SE(3) | double(4, 4)

            arguments (Input)
                data {mustBeFinite, Tools.mustAndOr('or', data, ...
                    'mustBeSE3', [], 'mustHaveSize', [1, 4], 'mustHaveSize', [4, 1])}
            end
            arguments (Output), A {Tools.mustBeSE3}, end
            
            if all(size(data) == [1, 4]) || all(size(data) == [4, 1])
                d = data(1); theta = data(2); a = data(3); alpha = data(4);

                A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta);
                     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                         0,              sin(alpha),              cos(alpha),          d;
                         0,                  0,                      0,                1];
            else, A = data;
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
            %   Validation: mustHaveSize(..., [3, 1]) OR mustHaveSize(..., [1, 3])
            % Output:
            %   A - Skew symmetric matrix
            %       ...(3, 3)
            %
            % See also Tools.mustHaveSize, Tools.mustAndOr

            arguments, w {Tools.mustAndOr('or', w, 'mustHaveSize', [3, 1], 'mustHaveSize', [1, 3])}, end
            if size(w, 1) ~= 1, w = w.'; end
            Tools.mustHaveSize(w, [1, 3])

            A = [[0 -w(3), w(2)]; [w(3), 0, -w(1)]; [-w(2), w(1), 0]];
        end

        % ------------------------- %

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
            an = a;
            bn = b;
            
            % test for zero input argument magnitude after normalize to take epsilon 
            % into account
            if (~any(an) || ~any(bn))
              error(message('sl3d:vrdirorirot:argzeromagnitude'));
            end
            
            ax = cross(an, bn);
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
                ax = cross(an, c);
            end
            
            % Be tolerant to column vector arguments, produce a row vector
            r = [ax(:)' angle];
        end
    end
end