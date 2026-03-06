% Parameters
mapSizeX = 90; mapSizeY = 90; mapSizeZ = 70;
mapSize = [mapSizeX, mapSizeY, mapSizeY];
map3D = occupancyMap3D(1);
occMatrix = zeros(mapSize);

%% Floor
occMatrix = placeBlock(occMatrix, 1, 1, 1, mapSize(1), mapSize(2), 1);  % ground

%% Add scattered blocks (obstacles)
% Format: [x, y, sx, sy, sz]
blocks = [
    15 15 8 8 20;    % low block
    30 25 6 6 40;    % tall block
    45 45 10 10 10;  % wide but short
    60 20 5 5 65;    % thin and tall
    70 70 8 8 30;    % mid-high
    25 70 6 6 55;    % corner obstacle
    50 10 5 5 25;    % small box
    75 35 6 6 45;    % near edge
];

% Add all blocks
for i = 1:size(blocks,1)
    bx = blocks(i,1);
    by = blocks(i,2);
    sx = blocks(i,3);
    sy = blocks(i,4);
    sz = blocks(i,5);
    occMatrix = placeBlock(occMatrix, bx, by, 2, sx, sy, sz);  % start at z=2 (over floor)
end

%% Convert to occupancy map
[x, y, z] = ind2sub(size(occMatrix), find(occMatrix == 1));
occupiedVoxels = [x y z] - 1;
setOccupancy(map3D, occupiedVoxels, 1);

%% Define WorkSpace
WorkSpace = ones(mapSize); % 1=> reachbale workspace | 0=> not reachable

%% Save Occupancy Matrix
save('../../Maps_ROI_Object_Situations/RandomObstacles_2_situations/RandomObstacles_2OccMatrix.mat','occMatrix');
disp('saved randomOccMatrix_2.mat')

%% Save WorkSpace
save('../../Maps_ROI_Object_Situations/RandomObstacles_2_situations/RandomObstacles_2WorkSpace.mat','WorkSpace');
disp('saved as RandomObstacles_2WorkSpace.mat ')

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