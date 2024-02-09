classdef Tools
    % TOOLS Static class for utilities implementation
    %   Implements all general tools useful for folder management, 
    %   problem definition and object visualization
    %
    % Tools Methods:
    %   addCasADiToPath - Add CasADi folder to Matlab path
    %   checkCasADi - Check if CasADi folder is already in Matlab path
    %   mustOr - Validate if data is ... or ...

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
                        case 'mustBeSizeVector', multiInput = true; Tools.mustBeSizeVector(data, checks{k}{2})
                        case 'mustBeNonzeroNorm', Tools.mustBeNonzeroNorm(data)
                        % case '',

                        otherwise
                            % Check feasibility of 
                            try mustBeTextScalar(checks{k}{1})
                                fprintf('%s: not inside the implemented checks', checks{k}{1});
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

        function mustBeSizeVector(data, dim)
            % mustBeSizeVector - Validate that data has specific dimension
            %
            % Syntax
            %   mustBeSizeVector(data, dim)
            %
            % Input:
            %   data - Data to validate
            %       double(n1, n2, ...) or char(n1, n2, ...)
            %   dim - Dimension to check with
            %       Integer(1, m)

            arguments, data, dim {mustBeInteger, mustBeVector}, end
            
            if size(dim, 1) ~= 1, dim = dim.'; end

            % Check array match
            if numel(size(data)) ~= numel(dim)
                throw(MException('Tools:WrongDim', ['Mismatch between ' ...
                    'numel(size(data)) (= %s) and # of dim to check (= %s)'], num2str(numel(size(data))), num2str(numel(dim))))
            end

            if any(size(data) ~= dim)
                throw(MException('Tools:WrongDim', ['''data'' does not match the required dimension.' ...
                    '\nActual dimension = [%s], Desired = [%s]'], num2str(size(data)), num2str(dim)))
            end
        end

        % ------------------------- %

        function mustBeNonzeroNorm(data)
            % mustBeNonzeroNorm - Validate that data has non zero norm
            %   Calls mustBeVector, mustBeNumeric
            %
            % Syntax
            %   mustBeNonzeroNorm(data)
            %
            % Input:
            %   data - Data to validate
            %       double()

            mustBeVector(data), mustBeNumeric(data)
            if norm(data), return, end
            throw(MException('Tools:ZeroNorm', 'Data must have non-zero norm'))
        end

        % ------------------------- %

        function mustBeSE3(data)
            % mustBeSE3 - Validate that data is an homogeneous matrix (SE(3))
            %   Calls mustBeNumeric, mustBeNonNan, mustBeSizeVector(data, [4, 4])
            %
            % Syntax
            %   mustBeSE3(data)
            %
            % Input:
            %   data - Data to validate
            %       double()

            threshold = 1e-5;

            mustBeNumeric(data), mustBeNonNan(data), Tools.mustBeSizeVector(data, [4, 4])
            R = data(1:3, 1:3);
            if any(abs(data(4, :) - [0, 0, 0, 1]) > threshold, 'all') || ...
                any(abs(R*(R.') - eye(3)) > threshold, 'all') || any(abs((R.')*R - eye(3)) > threshold, 'all') || ...
                (abs(det(R) - 1) > threshold)
                throw(MException('Tools:WrongValue', 'Must belong to SE(3) group'))
            end
        end
    end
end