classdef tools
    % TOOLS Static class for utilities implementation
    %   Implements all general tools useful for problem definition and
    %   visualization
    %
    % tools Methods:
    %   addCasadiToPath - Add the casadi folder to Matlab path

    methods (Static)

        % --------------------------------------------------------------- %

        function addCasadiToPath(path)
            % addCasadiToPath - Add the casadi folder to Matlab path
            %   Two options:
            %   - Search in the userpath MATLAB specific path or in the
            %       passed path for a folder named 'casadi*'
            %   - Use UI to select directly the casadi folder
            %
            % Syntax
            %   addCasadiToPath
            %   addCasadiToPath(folderName)
            %   addCasadiToPath('UI')
            %
            % Input:
            %   path - folder containing the 'casadi*' folder (absolute path)
            %       default = userpath() | char array or string

            arguments
                path {mustBeTextScalar} = userpath
            end

            switch path
                case 'UI' % use User Interface to add casadi folder
                    path = uigetdir(userpath(), 'Select casADi folder');
                otherwise
                    % Different system call to extract folders in the path
                    if ispc                                                 % Windows % --- TO IMPLEMENT --- %
                    elseif isunix, [status, files] = system(['ls ', path]); % Linux
                    elseif ismac                                            % Mac % --- TO IMPLEMENT --- %
                    else, throw(MException('tools:WrongSystem', ...
                            'Function available only on Windows, Linux and Mac systems'));
                    end
                    
                    % Extract if exists the first casadi folder
                    if status, throw(MException('tools:MissingFolder', 'Missing casadi folder in path %s', path)); end
                    files = strsplit(files, {' ', '\n'}); found = false;
                    for k = 1:length(files)
                        if startsWith(files{k}, 'casadi'), files = files{k}; found = true; break, end
                    end
                    if ~found, throw(MException('tools:MissingFolder', 'Missing casadi folder in path %s', path)); end
                    path = [path, '/', files];
            end

            % Add casdi* folder path
            lastwarn(''); addpath(path)
            if ~isempty(lastwarn), throw(MException('tools:pathError', 'Error during addpath of %s', path)); end
        end
    end
end