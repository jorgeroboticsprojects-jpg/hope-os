% Parameters
mapSizeX = 90; mapSizeY = 90; mapSizeZ = 70;
mapSize = [mapSizeX, mapSizeY, mapSizeY];
map3D = occupancyMap3D(1);
occMatrix = zeros(mapSize);

%% Add floor
occMatrix = placeBlock(occMatrix, 1, 1, 1, mapSize(1), mapSize(2), 1);  % base layer

%% Large hollow block with inner corridor
occMatrix = placeBlock(occMatrix, 30, 30, 1, 20, 20, 35);  % outer cube
occMatrix = placeBlock(occMatrix, 33, 33, 5, 14, 14, 25, 0); % carve out core
occMatrix = placeBlock(occMatrix, 33, 30, 5, 14, 3, 25, 0);  % entrance tunnel (front)

%% Tall pillar block
occMatrix = placeBlock(occMatrix, 70, 15, 1, 10, 10, 40);

%% Wall with archway
occMatrix = placeBlock(occMatrix, 10, 70, 1, 30, 3, 30);
occMatrix = placeBlock(occMatrix, 20, 70, 2, 10, 3, 15, 0);  % open arch in center

%% Stacked blocks in steps
occMatrix = placeBlock(occMatrix, 10, 10, 1, 6, 6, 10);
occMatrix = placeBlock(occMatrix, 19, 10, 1, 6, 6, 20);
occMatrix = placeBlock(occMatrix, 28, 10, 1, 6, 6, 30);
occMatrix = placeBlock(occMatrix, 37, 10, 1, 6, 6, 40);

%% Corner structures
occMatrix = placeBlock(occMatrix, 5, 5, 1, 5, 5, 25);
occMatrix = placeBlock(occMatrix, 80, 5, 1, 5, 5, 25);
occMatrix = placeBlock(occMatrix, 5, 80, 1, 5, 5, 25);
occMatrix = placeBlock(occMatrix, 80, 80, 1, 5, 5, 25);

%% Convert to occupancy map
[x, y, z] = ind2sub(size(occMatrix), find(occMatrix == 1));
occupiedVoxels = [x y z] - 1;
setOccupancy(map3D, occupiedVoxels, 1);

%% Define WorkSpace
WorkSpace = ones(mapSize); % 1=> reachbale workspace | 0=> not reachable

%% Save Occupancy Matrix
save('../../Maps_ROI_Object_Situations/RandomObstacles_1_situations/RandomObstacles_1OccMatrix.mat','occMatrix');
disp('saved .mat ')

%% Save WorkSpace
save('../../Maps_ROI_Object_Situations/RandomObstacles_1_situations/RandomObstacles_1WorkSpace.mat','WorkSpace');
disp('saved as Office_2WorkSpace.mat ')

%% Visualize
figure;
h=show(map3D);
axis equal; grid on;
view(135, 30);
set(h, 'XLim', [0 mapSizeX], 'YLim', [0 mapSizeY], 'ZLim', [0 mapSizeZ]);
title('');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% Utility function
function occMatrix = placeBlock(occMatrix, ox, oy, oz, sx, sy, sz, fill)
    % Default: fill = 1 (occupied), fill = 0 (carve/hole)
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