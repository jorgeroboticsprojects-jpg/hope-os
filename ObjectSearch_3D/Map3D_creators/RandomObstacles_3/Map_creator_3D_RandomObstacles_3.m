% Parameters
mapSizeX = 90; mapSizeY = 90; mapSizeZ = 70;
mapSize = [mapSizeX, mapSizeY, mapSizeY];
map3D = occupancyMap3D(1);
occMatrix = zeros(mapSize);

%% Floor
occMatrix = placeBlock(occMatrix, 1, 1, 1, mapSize(1), mapSize(2), 1);

%% --- Cubes (scaled ~30%)
cubeList = [
    15 15 1 10 10 10;
    60 60 30 13 13 13;
    40 10 20 9 9 12;   % NEW cube
];

for i = 1:size(cubeList,1)
    occMatrix = placeBlock(occMatrix, cubeList(i,1), cubeList(i,2), cubeList(i,3), ...
                                      cubeList(i,4), cubeList(i,5), cubeList(i,6));
end

%% --- Spheres (~30% bigger radius)
sphereList = [
    40 40 20 8;
    20 70 45 5;
    70 75 25 6;  % NEW sphere
];

for i = 1:size(sphereList,1)
    cx = sphereList(i,1);
    cy = sphereList(i,2);
    cz = sphereList(i,3);
    r  = sphereList(i,4);
    for x = -r:r
        for y = -r:r
            for z = -r:r
                if norm([x y z]) <= r
                    vx = cx + x;
                    vy = cy + y;
                    vz = cz + z;
                    if all([vx vy vz] > 0) && ...
                       vx <= mapSize(1) && vy <= mapSize(2) && vz <= mapSize(3)
                        occMatrix(vx, vy, vz) = 1;
                    end
                end
            end
        end
    end
end

%% --- Donut (rotated vertical, scaled)
center = [25, 60, 35];  % center of torus
rMajor = 10;   % major radius (in YZ plane now)
rMinor = 3;    % minor radius (tube)

for theta = linspace(0, 2*pi, 60)
    for phi = linspace(0, 2*pi, 30)
        y = (rMajor + rMinor * cos(phi)) * cos(theta);
        z = (rMajor + rMinor * cos(phi)) * sin(theta);
        x = rMinor * sin(phi);
        px = round(center(1) + x);
        py = round(center(2) + y);
        pz = round(center(3) + z);
        if all([px py pz] > 0) && ...
           px <= mapSize(1) && py <= mapSize(2) && pz <= mapSize(3)
            occMatrix(px, py, pz) = 1;
        end
    end
end

%% Convert to occupancy map
[x, y, z] = ind2sub(size(occMatrix), find(occMatrix == 1));
occupiedVoxels = [x y z] - 1;
setOccupancy(map3D, occupiedVoxels, 1);

%% Save Occupancy Matrix
save('../../Maps_ROI_Object_Situations/RandomObstacles_3_situations/randomOccMatrix_3.mat','occMatrix');
disp('saved ')

%% Visualize
figure;
h=show(map3D);
axis equal; grid on;
view(135, 30);
set(h, 'XLim', [0 mapSizeX], 'YLim', [0 mapSizeY], 'ZLim', [0 mapSizeZ]);
title('Custom Desk & Monitor Placement');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% Utility function
function occMatrix = placeBlock(occMatrix, ox, oy, oz, sx, sy, sz, fill)
    % Default fill = 1 (occupied), use fill = 0 to carve holes
    if nargin < 8
        fill = 1;
    end
    [maxX, maxY, maxZ] = size(occMatrix);
    if ox < 1 || oy < 1 || oz < 1 || ...
       ox+sx-1 > maxX || oy+sy-1 > maxY || oz+sz-1 > maxZ
        warning('Skipping block at [%d %d %d] (out of bounds)', ox, oy, oz);
        return;
    end
    occMatrix(ox:ox+sx-1, oy:oy+sy-1, oz:oz+sz-1) = fill;
end