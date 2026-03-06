% Parameters
mapSizeX = 90; mapSizeY = 90; mapSizeZ = 70;
voxelSize = 0.05;
mapResolution = 1;
mapSize = [mapSizeX, mapSizeY, mapSizeY];
map3D = occupancyMap3D(mapResolution);
occMatrix = zeros(mapSize);

%% Set environment distribution
occMatrix = placeBlock(occMatrix, 1, 1, 1, mapSize(1), mapSize(2), 1);%floor

% Format: [x, y, rotation, length, height, thickness] Only 0 & 90 degrees.
 walls = [
%     90 1   90  90 70 1;    % horizontal wall along X at bottom
%     1 90  0  90 70 1;    % vertical wall along Y at left
 ];

% Define desk placements: [x, y, rotation, monitor, monitor side]
% rotation: 0 = horizontal (long in X), 1 = vertical (long in Y)
% monitor 1=yes, 0=no, monitor side (0,1)
desks = [
    25 55 1 1 0;  % Desk at [30,55], horizontal, with monitor
    45 55 1 1 1  % Desk at [30,55], horizontal, with monitor
];

% Format: [x, y, rotation]
% rotation: 0 = facing North (Y+), 90 = East (X+), 180 = South (Y-), 270 = West (X-)
chairs = [
    13 66 270;    % Chair just behind the desk
    68 66 90
];

% Format: [x, y, rotation, width, height, levels, depth]
shelves = [
     1 50 270  30 60  5 10;   % Left shelf
    81 21  90  30 60  7 10;   % Right shelf
    1 20 270  20 20  1 15;   % Small Left shelf
];

% Format: [x, y, height, radius]
trashBins = [
    80 10 12 4   % Trash bin in lower-right corner
];



%% Add walls
% Format: [x, y, rotation, length, height, thickness]
% rotation: 0 = along X, 90 = along

for i = 1:size(walls,1)
    px = walls(i,1);
    py = walls(i,2);
    rot = walls(i,3);
    len = walls(i,4);
    h   = walls(i,5);
    t   = walls(i,6);

    if rot == 0  % horizontal (along X)
        occMatrix = placeBlock(occMatrix, px, py, 1, len, t, h);
    elseif rot == 90  % vertical (along Y)
        occMatrix = placeBlock(occMatrix, px, py, 1, t, len, h);
    else
        warning('Unsupported wall rotation: %d°', rot);
    end
end



%% Add desks and monitors
tableH = 16;   % height of table surface
legH = 15;     % height of table legs

% Define desk placements: [x, y, rotation, monitor, monitor side]
% rotation: 0 = horizontal (long in X), 1 = vertical (long in Y)
% monitor 1=yes, 0=no

% Desk positions are set using bottom left cortern in XY plane
% desks = [
%     10 35 0 0;
%     50 20 1 1;
% ];
% Add desks and monitors
for i = 1:size(desks,1)
    px = desks(i,1);
    py = desks(i,2);
    rot = desks(i,3);
    mon = desks(i,4);
    monSide = desks(i,5);  % NEW

    % Desk dimensions
    if rot == 0
        deskL = 30; deskW = 20;
    else
        deskL = 20; deskW = 30;
    end
    deskH = 1;

    % Table surface
    occMatrix = placeBlock(occMatrix, px, py, tableH, deskL, deskW, deskH);

    % Table legs (2x2 base)
    occMatrix = placeBlock(occMatrix, px, py, 1, 2, 2, legH);  % front-left
    occMatrix = placeBlock(occMatrix, px+deskL-2, py, 1, 2, 2, legH);  % front-right
    occMatrix = placeBlock(occMatrix, px, py+deskW-2, 1, 2, 2, legH);  % back-left
    occMatrix = placeBlock(occMatrix, px+deskL-2, py+deskW-2, 1, 2, 2, legH);  % back-right

    % Monitor base + screen
    if mon == 1
        if rot == 0
            mx = px + round(deskL/2) - 7;
            if monSide == 0
                my = py + deskW - 5;  % default front
            else
                my = py + 5;          % opposite side
            end
            occMatrix = placeBlock(occMatrix, mx, my, tableH+5, 16, 1, 10);    % screen
            occMatrix = placeBlock(occMatrix, mx+7, my, tableH+1, 2, 1, 9);    % stand
        else
            my = py + round(deskW/2) - 7;
            if monSide == 0
                mx = px + deskL - 5;  % default front
            else
                mx = px + 5;          % opposite side
            end
            occMatrix = placeBlock(occMatrix, mx, my, tableH+5, 1, 16, 10);    % screen (rotated)
            occMatrix = placeBlock(occMatrix, mx, my+7, tableH+1, 1, 2, 9);    % stand
        end
    end
end

%% Add chairs
% Format: [x, y, rotation]
% rotation: 0 = facing North (Y+), 90 = East (X+), 180 = South (Y-), 270 = West (X-)
% chairs = [
%     30 60 180
% ];

seatH = 11;        % Height of the seat from the floor
legH = seatH-1;         % Height of the chair legs
seatSize = 10;     % Width/length of the seat (in voxels)
backH = 15;       % Height of the backrest (optional)

for i = 1:size(chairs,1)
    px = chairs(i,1);
    py = chairs(i,2);
    rot = chairs(i,3);

    % Chair seat (6x6x1)
    occMatrix = placeBlock(occMatrix, px, py, seatH, seatSize, seatSize, 2);

    % 4 chair legs (2x2x6)
    occMatrix = placeBlock(occMatrix, px, py, 1, 2, 2, legH);  % front-left
    occMatrix = placeBlock(occMatrix, px+seatSize-2, py, 1, 2, 2, legH);  % front-right
    occMatrix = placeBlock(occMatrix, px, py+seatSize-2, 1, 2, 2, legH);  % back-left
    occMatrix = placeBlock(occMatrix, px+seatSize-2, py+seatSize-2, 1, 2, 2, legH);  % back-right

    % Chair backrest (oriented by rotation)
    switch rot
        case 0 % facing North (Y+)
            occMatrix = placeBlock(occMatrix, px, py+seatSize-1, seatH+1, seatSize, 1, backH);
        case 90 % facing East (X+)
            occMatrix = placeBlock(occMatrix, px+seatSize-1, py, seatH+1, 1, seatSize, backH);
        case 180 % facing South (Y-)
            occMatrix = placeBlock(occMatrix, px, py, seatH+1, seatSize, 1, backH);
        case 270 % facing West (X-)
            occMatrix = placeBlock(occMatrix, px, py, seatH+1, 1, seatSize, backH);
    end
end

%% Add shelves (improved version with back panel and top cover)
% Format: [x, y, rotation, width, height, levels]
% rotation: 0 = North (Y+), 90 = East (X+), etc.
% shelves = [
%     20 20 0 8 24 3;
%     40 40 90 10 30 4;
% ];

for i = 1:size(shelves,1)
    px = shelves(i,1);
    py = shelves(i,2);
    rot = shelves(i,3);
    unitW = shelves(i,4);       % Width (left to right)
    unitH = shelves(i,5);       % Total height (bottom to top)
    numShelves = shelves(i,6);  % Number of shelf levels
    shelfDepth = shelves(i,7);  % now variable

    %shelfDepth = 4;             % Depth (front to back)
    shelfThickness = 1;         % Thickness of shelves
    sideThickness = 1;          % Thickness of side panels
    topThickness = 1;           % Thickness of top cover

    % Shelf spacing
    shelfSpacing = floor((unitH - topThickness) / (numShelves + 1));

    switch rot
        case 0
            % Side panels
            occMatrix = placeBlock(occMatrix, px, py, 1, sideThickness, shelfDepth, unitH);                     % Left
            occMatrix = placeBlock(occMatrix, px+unitW-sideThickness, py, 1, sideThickness, shelfDepth, unitH); % Right

            % Back panel
            occMatrix = placeBlock(occMatrix, px, py+shelfDepth-1, 1, unitW, 1, unitH);

            % Top panel
            occMatrix = placeBlock(occMatrix, px, py, unitH, unitW, shelfDepth, topThickness);

            % Horizontal shelves
            for s = 1:numShelves
                z = 1 + s * shelfSpacing;
                occMatrix = placeBlock(occMatrix, px, py, z, unitW, shelfDepth, shelfThickness);
            end

        case 90
            occMatrix = placeBlock(occMatrix, px, py, 1, shelfDepth, sideThickness, unitH);                     % Left
            occMatrix = placeBlock(occMatrix, px, py+unitW-sideThickness, 1, shelfDepth, sideThickness, unitH); % Right

            occMatrix = placeBlock(occMatrix, px+shelfDepth-1, py, 1, 1, unitW, unitH);  % Back
            occMatrix = placeBlock(occMatrix, px, py, unitH, shelfDepth, unitW, topThickness); % Top

            for s = 1:numShelves
                z = 1 + s * shelfSpacing;
                occMatrix = placeBlock(occMatrix, px, py, z, shelfDepth, unitW, shelfThickness);
            end

        case 180
            occMatrix = placeBlock(occMatrix, px, py, 1, sideThickness, shelfDepth, unitH);
            occMatrix = placeBlock(occMatrix, px+unitW-sideThickness, py, 1, sideThickness, shelfDepth, unitH);

            occMatrix = placeBlock(occMatrix, px, py, 1, unitW, 1, unitH);  % Back (Y-)
            occMatrix = placeBlock(occMatrix, px, py, unitH, unitW, shelfDepth, topThickness); % Top

            for s = 1:numShelves
                z = 1 + s * shelfSpacing;
                occMatrix = placeBlock(occMatrix, px, py, z, unitW, shelfDepth, shelfThickness);
            end

        case 270
            occMatrix = placeBlock(occMatrix, px, py, 1, shelfDepth, sideThickness, unitH);
            occMatrix = placeBlock(occMatrix, px, py+unitW-sideThickness, 1, shelfDepth, sideThickness, unitH);

            occMatrix = placeBlock(occMatrix, px, py, 1, 1, unitW, unitH);  % Back (X-)
            occMatrix = placeBlock(occMatrix, px, py, unitH, shelfDepth, unitW, topThickness); % Top

            for s = 1:numShelves
                z = 1 + s * shelfSpacing;
                occMatrix = placeBlock(occMatrix, px, py, z, shelfDepth, unitW, shelfThickness);
            end
    end
end

%% Add trash bins
% Format: [x, y, height, radius]
% x, y = center base position in voxels
% trashBins = [
%     30 30 12 4;
%     60 20 16 5
% ];

for i = 1:size(trashBins,1)
    cx = trashBins(i,1);
    cy = trashBins(i,2);
    h = trashBins(i,3);
    r = trashBins(i,4);

    for x = -r:r
        for y = -r:r
            % Check if on the border (hollow cylinder)
            d = sqrt(x^2 + y^2);
            if d >= r - 0.5 && d <= r + 0.5
                for z = 1:h
                    vx = cx + x;
                    vy = cy + y;
                    vz = z;

                    % Bounds check
                    if vx > 0 && vy > 0 && vz > 0 && ...
                       vx <= size(occMatrix,1) && ...
                       vy <= size(occMatrix,2) && ...
                       vz <= size(occMatrix,3)
                        occMatrix(vx, vy, vz) = 1;
                    end
                end
            end
        end
    end
end


%% Convert to occupancy map
[x, y, z] = ind2sub(size(occMatrix), find(occMatrix == 1));
occupiedVoxels = [x y z] - 1;
setOccupancy(map3D, occupiedVoxels, 1);

%% Define WorkSpace
WorkSpace = ones(mapSize); % 1=> reachable workspace | 0=> not reachable

%% Save Occupancy Matrix
save('../../Maps_ROI_Object_Situations/Office_1_situations/Office_1OccMatrix.mat','occMatrix');
disp('saved as officeOccMatrix_1.mat ');

%% Save Workspace
save('../../Maps_ROI_Object_Situations/Office_1_situations/Office_1WorkSpace.mat','WorkSpace');
disp('saved workspace as LHCWorkSpace.mat');


%% Visualization
figure;
h=show(map3D);
axis equal; grid on; hold on;
view(135, 30);
set(h, 'XLim', [0 mapSizeX], 'YLim', [0 mapSizeY], 'ZLim', [0 mapSizeZ]);
title('');
xlabel('X'); ylabel('Y'); zlabel('Z');

% % draw WorkSpace
% [wx, wy, wz] = ind2sub(size(WorkSpace), find(WorkSpace == 1)); 
% scatter3(wx-0.5, wy-0.5, wz-0.5, 100, [0.0, 0.4, 1], 'filled', 'MarkerFaceAlpha', 0.3);

%% Utility function
function occMatrix = placeBlock(occMatrix, ox, oy, oz, sx, sy, sz)
    [maxX, maxY, maxZ] = size(occMatrix);
    if ox < 1 || oy < 1 || oz < 1 || ...
       ox+sx-1 > maxX || oy+sy-1 > maxY || oz+sz-1 > maxZ
        warning('Skipping block at [%d %d %d] (out of bounds)', ox, oy, oz);
        return;
    end
    occMatrix(ox:ox+sx-1, oy:oy+sy-1, oz:oz+sz-1) = 1;
end

