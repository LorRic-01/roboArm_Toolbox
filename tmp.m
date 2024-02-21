Pi = rand(100000, 3);
K = convhull(Pi(:, 1), Pi(:, 2), Pi(:, 3));
tri = triangulation(K,Pi);

[x,y,z] = sphere(10);
fvc = surf2patch(x,y,z,z,'triangles');
tri = triangulation(fvc.faces, fvc.vertices);

Pi = [[0, 0, 0].', [3, 0, 0].', [3, 0, 2].', [2, 0, 2].', [2, 0, 1].',...
    [1, 0, 1].', [1, 0, 2].', [0, 0, 2].', ...
    [0, 1, 0].', [3, 1, 0].', [3, 1, 2].', [2, 1, 2].', [2, 1, 1].',...
    [1, 1, 1].', [1, 1, 2].', [0, 1, 2].'].';
K = [1, 8, 7;
    1, 7, 6;
    1, 6, 5;
    1, 5, 2;
    2, 4, 5;
    2, 3, 4;
    9, 16, 15;
    9, 15, 14;
    9, 14, 13;
    9, 13, 10;
    10, 12, 13;
    10, 11, 12;
    1, 8, 9;
    9, 8, 16;
    2, 10, 3;
    3, 10, 11;
    11, 4, 3;
    11, 4, 12;
    5, 13, 4;
    4, 13, 12;
    5, 6, 13;
    6, 13, 14;
    6, 14, 7;
    7, 14, 15;
    7, 8, 15;
    8, 15, 16;
    1, 2, 10;
    1, 10, 9];
tri = triangulation(K,Pi);

P = 1.5*rand(3, 1);

dir = rand(3, 1);
A1 = zeros(numel(tri.ConnectivityList) + size(tri.ConnectivityList, 1), numel(tri.ConnectivityList) + size(tri.ConnectivityList, 1));
for k = 1:size(tri.ConnectivityList, 1)
    A1(((k-1)*4 + 1):(4*k), ((k-1)*4 + 1):(4*k)) = [[tri.Points(tri.ConnectivityList(k, :), :).'; 1, 1, 1], [-dir; 0]];
end
P_f = repmat([P; 1], [size(tri.ConnectivityList, 1), 1]);
%%
clc

sol = A1\P_f;
count = sum(all(reshape(sol > 0, 4, []), 1));

%%
% vecs = zeros(size(tri.ConnectivityList, 1), 4);
% done = false;
% while ~done
%     count = 0;
% 
%     for k = 1:size(tri.ConnectivityList, 1)
%         A = [[tri.Points(tri.ConnectivityList(k, :), :).'; 1, 1, 1], [-dir; 0]];
%         lastwarn(''), vec = A\[P; 1];
% 
%         [warnMsg, warnId] = lastwarn;
%         if ~isempty(warnMsg)
%             done = false; fprinf('Fail \n')
%             break
%         end
% 
%         points = [tri.Points(tri.ConnectivityList(k, :), :).', tri.Points(tri.ConnectivityList(k, 1), :).'];
%         figure(2), clf, plot3(points(1, :), points(2, :), points(3, :), '.-', 'MarkerSize', 15)
%         grid on, axis equal, hold on
%         plot3(P(1), P(2), P(3), '.', 'MarkerSize', 15), quiver3(P(1), P(2), P(3), dir(1), dir(2), dir(3)), view(2)
%         xlabel('x'), ylabel('y'), zlabel('z')
%         vecs(k, :) = vec.';
%         if (vec(4) > 0) && all(vec(1:3) > 0)
%             count = count + 1;
%         end
%     end
%     done = true;
% end
% 
% count
% figure(1), clf
% trimesh(tri, 'EdgeColor', 'b')
% grid on, axis equal, hold on, plot3(P(1), P(2), P(3), '.', 'MarkerSize', 15)
% quiver3(P(1), P(2), P(3), dir(1), dir(2), dir(3))
% xlabel('x'), ylabel('y'), zlabel('z')

%%
Tools.cmpDynParams(tri, 1, true)

%% Particle distribution

n_part = 1000;
particle = 0.1*rand(n_part, 3) + 0.5;
cost = @(x, swarm) min(sum((swarm - x).^2, 2));

figure(3), clf
trimesh(tri, 'EdgeColor', 'b', 'FaceAlpha', 0.1)
grid on, axis equal, hold on, plot3(particle(:, 1), particle(:, 2), particle(:, 3), '.', 'MarkerSize', 15)
xlabel('x'), ylabel('y'), zlabel('z'), drawnow

cost_p = zeros(n_part, 1);
for k = 1:n_part
    cost_p(k) = cost(particle(k, :), particle);
end

dA = decomposition(A1, 'auto');

tic
cycle = 100;
for iter = 1:cycle
    fprintf('Iter: %i\n', iter)
    for n = 1:n_part
        % Mix
        j = n;
        while j == n
            j = randi(n_part, [1, 1]);
            k = randi(3, [1, 1]);
        end
        newP = particle(n, :);
        newP(:, k) = particle(n, k) + (2*rand - 1)*(particle(n, k) - particle(j, k));
        cost_new = cost(newP, particle);
        
        % Check cost improvement
        P_f = repmat([newP.'; 1], [size(tri.ConnectivityList, 1), 1]);
        count = sum(all(reshape((dA\P_f) > 0, 4, []), 1));
        if (cost_new > cost_p(n)) && (mod(count, 2) ~= 0)
            particle(n, :) = newP;
            cost_p(n) = cost_new;
        end
    end
end
toc

figure(3), clf
trimesh(tri, 'EdgeColor', 'b', 'FaceAlpha', 0.1)
grid on, axis equal, hold on, plot3(particle(:, 1), particle(:, 2), particle(:, 3), '.', 'MarkerSize', 15)
xlabel('x'), ylabel('y'), zlabel('z')


%% Test

in2m = 1/39.3701;
model = createpde;
model.Geometry = importGeometry('EndEffector_merge_fixed.stl');
scale(model.Geometry, in2m);

h = pdegplot(model);
reducepatch(h(1), 0.5)

tri = triangulation(h(1).Faces, h(1).Vertices);

dA = {};
for k = 1:size(tri.ConnectivityList, 1)
    dA{end + 1} = decomposition([[tri.Points(tri.ConnectivityList(k, :), :).'; 1, 1, 1], [-dir; 0]], ...
        'auto');
end

%%
Pi = tri.Points;
K = tri.ConnectivityList;
C = unique(K);

Pi_new = zeros(size(C, 1), size(Pi, 2));
for k = 1:size(C, 1)
    Pi(k, :) = Pi(C(k), :);
    K(K == C(k)) = k;
end

tri_new = triangulation(K, Pi(1:size(C, 1), :));

%%
params = struct('verbose', true, ...
                    'cycle', 100, 'n_p', 100);
[CoM, I, swarm, tri] = Tools.cmpDynParams(tri, swarm, params);
CoM, I{:}
cost = @(x, swarm) min(sum((swarm - x).^2, 2));
cost_s = zeros(size(swarm, 1), 1);

for k = 1:size(swarm, 1)
    cost_s(k) = cost(swarm(k, :), swarm([1:k-1, k+1:end], :));
end
% 
% figure
%  trimesh(tri, 'EdgeColor', [0.8500 0.3250 0.0980], 'FaceAlpha', 0.1)
%                 grid on, axis equal, axis padded, hold on, plot3(swarm(:, 1), swarm(:, 2), swarm(:, 3), '.', ...
%                     'MarkerSize', 15, 'Color', [0 0.4470 0.7410])
% plot3(CoM(1), CoM(2), CoM(3), '.r', 'MarkerSize', 15)