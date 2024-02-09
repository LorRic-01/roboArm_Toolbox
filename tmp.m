in2m = 1/39.3701;

model = createpde;
model.Geometry = importGeometry('Pri1_merge_fixed.stl');
scale(model.Geometry, in2m)

figure(1), clf, pdegplot(model), hold on

figure(2), clf, nexttile, plot3(model.Geometry.Vertices(:, 1), ...
    model.Geometry.Vertices(:, 2), model.Geometry.Vertices(:, 3), '.', 'MarkerSize', 10)
axis equal, hold on

convexHull = convhull(model.Geometry.Vertices(:, 1), model.Geometry.Vertices(:, 2), model.Geometry.Vertices(:, 3));
TR = triangulation(convexHull, model.Geometry.Vertices);
% trimesh(TR)

used = unique(convexHull(:));
DTR = delaunayTriangulation(model.Geometry.Vertices(used, 1),model.Geometry.Vertices(used, 2), model.Geometry.Vertices(used, 3));
nexttile, tetramesh(DTR)

% DTR2 = delaunayTriangulation(model.Geometry.Vertices(:, 1),model.Geometry.Vertices(:, 2), model.Geometry.Vertices(:, 3));
% tetramesh(DTR2)

% [index, c, score] = kmeans(model.Geometry.Vertices, floor(size(model.Geometry.Vertices, 1)/100));
% plot3(c(:, 1), c(:, 2), c(:, 3), '.', 'MarkerSize', 10)

% %% New pde
% model2 = createpde;
% model2.Geometry = geometryFromMesh(model2, )

% generateMesh(model,"Hmin", 0.25, 'GeometricOrder','linear')
% figure, plot3(model.Mesh.Nodes(1, :), model.Mesh.Nodes(2, :), model.Mesh.Nodes(3, :), '.', 'MarkerSize', 10)

%%

points = model.Geometry.Vertices;
uniquePoints = [];

for n = 1:5
    convexHull = convhull(points(:, 1), points(:, 2), points(:, 3));
    uniquePoints = [uniquePoints; points(unique(convexHull(:)), :)];
    
    nearPoint = false(size(points, 1), 1);
    
    for convexHull = 1:size(uniquePoints, 1)
        d = sum((points - uniquePoints(convexHull, :)).^2, 2);
        nearPoint(d < 0.05) = true;
    end
    
    points = points(~nearPoint, :);
    if isempty(points), break, end
end

figure(3), clf
plot3(model.Geometry.Vertices(:, 1), model.Geometry.Vertices(:, 2), model.Geometry.Vertices(:, 3), '.', 'MarkerSize', 5), hold on
plot3(uniquePoints(:, 1), uniquePoints(:, 2), uniquePoints(:, 3), '.g', 'MarkerSize', 10)
% plot3(farPoints(:, 1), farPoints(:, 2), farPoints(:, 3), '.r', 'MarkerSize', 10)
axis equal


%
DTR3 = delaunayTriangulation(uniquePoints(:, 1), uniquePoints(:, 2), uniquePoints(:, 3));
figure(4), clf, tetramesh(DTR3)


%% Final results

in2m = 1/39.3701;

model = createpde;
model.Geometry = importGeometry('Pri1_merge_fixed.stl');
scale(model.Geometry, in2m);

figure(1), clf, h = pdegplot(model); hold on

for k = 1:length(h)
    if isa(h(k), 'matlab.graphics.primitive.Patch')
        reducepatch(h(k), 0.1)
        TR = triangulation(h(k).Faces, h(k).Vertices(:, 1), h(k).Vertices(:, 2), h(k).Vertices(:, 3));
        trimesh(TR)
        break;
    end
end

model2 = createpde;
geometryFromMesh(model2, TR.Points.', TR.ConnectivityList.');


%%

cube = multicuboid(1, 1, 1);
% DTR = delaunayTriangulation(cube.Vertices);
figure(1), clf, h = pdegplot(cube); hold on
TR = triangulation(h(1).Faces, h(1).Vertices(:, 1), h(1).Vertices(:, 2), h(1).Vertices(:, 3));

trimesh(TR)

inner3D([0, 0, 0], TR.Points, TR.ConnectivityList)


function check = inner3D(point, points, connectivity)
    % compute if a point is inside the 3d object
    check = false;

    for k = 1:size(connectivity, 1)
        (abs(det([points(connectivity(1, :), :).', point.'; 1, 1, 1, 0])) > 1e-5)
    end

end

function Ainv = inv4(A)
    % invert 4x4 matix
    Ainv = []
end