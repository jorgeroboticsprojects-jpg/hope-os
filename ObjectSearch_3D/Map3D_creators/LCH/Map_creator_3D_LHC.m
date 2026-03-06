%% Parameters
voxelEdgeSize = 0.05;  % Real voxel edge size in meters (5cm)
mapResolution = 1;     % Map resolution (1 voxel = 1 axis unit)

% Map dimensions
mapSizeX = 90; mapSizeY = 90; mapSizeZ = 70;
mapSize = [mapSizeX, mapSizeY, mapSizeZ];

%% Create empty occupancy map and matrix
map3D = occupancyMap3D(mapResolution);
occMatrix = zeros(mapSize);

%% Add LHC Pipe and Walls
for x = 1:mapSizeX
    for y = 1:mapSizeY
        for z = 1:mapSizeZ
            % Main pipe shape (tube)
            dist1 = sqrt((x - 38)^2 + (z - 26)^2);
            if dist1 < 9 && (dist1 > 7 || y == 1 || y == mapSizeY)
                occMatrix(x, y, z) = 1;
            end

            % LHC wall shape (big cylinder)
            dist2 = sqrt((x - 45)^2 + (z - 28)^2);
            if dist2 >= 37 && dist2 <= 39 && (z > 8 || x < 30) && (z <= 28 || x <= mapSizeX)
                occMatrix(x, y, z) = 1;
            end
        end
    end
end

%% Add floor and structures
occMatrix(24:76, :, 8) = 1;       % Main floor
occMatrix(24, :, 1:8) = 1;        % Vertical wall/floor
occMatrix(19:23, :, 1) = 1;       % Gap floor
occMatrix(13:24, :, 46) = 1;      % Platform 1
occMatrix(16:32, :, 52) = 1;      % Platform 2

% Additional boxes / structures
occMatrix(9:20, :, 37) = 1;       % Stant box
occMatrix(20, :, 37:45) = 1;      % Top box

%% Convert to occupancy map
[occX, occY, occZ] = ind2sub(size(occMatrix), find(occMatrix == 1));
occupiedVoxels = [occX, occY, occZ] - 1;

% Set occupied voxels
setOccupancy(map3D, occupiedVoxels, 1);

%% Define WorkSpace
WorkSpace = ones(mapSize); % 1=> reachable workspace | 0=> not reachable

% WorkSpace = zeros(mapSize);
% for x = 1:mapSizeX
%     for y = 1:1%mapSizeY
%         for z = 1:mapSizeZ
%             % Define the robots workspace, in this casem is a tubular
%             % workspace (Tims robot)         
%             dist1 = sqrt((x - 49)^2 + (z - 40)^2);
%             if dist1 < 25 && (dist1 > 7 || y == 1 || y == mapSizeY)
%                 WorkSpace(x, y, z) = 1;
%             end   
%         end
%     end
% end    

%% Save Occupancy Matrix
save('../../Maps_ROI_Object_Situations/LHC_situations/LHCOccMap.mat','occMatrix');
disp('saved occupancy matrix as LHCOccMap.mat')

%% Save Workspace
save('../../Maps_ROI_Object_Situations/LHC_situations/LHCWorkSpace.mat','WorkSpace');
disp('saved workspace as LHCWorkSpace.mat')

%% Visualize the occupancy map
figure;
hold on;
h = show(map3D);
axis equal; grid on;
set(h, 'XLim', [0 mapSizeX], 'YLim', [0 mapSizeY], 'ZLim', [0 mapSizeZ]);
view(135, 30);
title('');
xlabel('X'); ylabel('Y'); zlabel('Z');

% draw WorkSpace
% [wx, wy, wz] = ind2sub(size(WorkSpace), find(WorkSpace == 1)); 
% scatter3(wx-0.5, wy-0.5, wz-0.5, 100, [0.0, 0.4, 1], 'filled', 'MarkerFaceAlpha', 0.3);

