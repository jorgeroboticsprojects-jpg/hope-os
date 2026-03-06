%% Map dimensions (MAKE SURE THEY ARE THE SAME AS THE MAP CREATOR)
mapSizeX = 90; mapSizeY = 90; mapSizeZ = 70;
mapSize = [mapSizeX, mapSizeY, mapSizeZ];

%% Generate ROI and Object positions
% We store environment state in this matrix with this values:
% ---- 0 = free | 1= Occupied | 2 = unexplored | 3 = Frontier | 4 = ROI | 
% ---- 5 = Object
voxelStates = 2*ones(mapSize);

%% DEFINE ROI SETTINGS
voxelStates(45:80, 5:40, 50:70) = 4;

%% DEFINE OBJECTIVE LOCATION
objectX=62; objectY=22; objectZ=68;

% Define the object value
objectValue = 5;
% Compute 3x3x3 block centered on the object position
for dx = -1:1
    for dy = -1:1
        for dz = -1:1
            x = objectX + dx;
            y = objectY + dy;
            z = objectZ + dz;

            % Check bounds before assigning
            if x >= 1 && x <= size(voxelStates,1) && ...
               y >= 1 && y <= size(voxelStates,2) && ...
               z >= 1 && z <= size(voxelStates,3)
                voxelStates(x,y,z) = objectValue;
            end
        end
    end
end

figure;             
hold on; 
axis([0 mapSize(1) 0 mapSize(2) 0 mapSize(3)]);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');


[rx, ry, rz] = ind2sub(size(voxelStates), find(voxelStates == 4));
scatter3(rx-0.5, ry-0.5, rz-0.5, 50, [1, 0.8, 0], 'filled', 'MarkerFaceAlpha', 0.1);

[gx, gy, gz] = ind2sub(size(voxelStates), find(voxelStates == 5));
scatter3(gx-0.5, gy-0.5, gz-0.5, 50, [0.6, 0.2, 0.8], 'filled', 'MarkerFaceAlpha', 1);

view(135, 30);

%% Save Voxel Initial States Matrix
save('../../Maps_ROI_Object_Situations/RandomObstacles_2_situations/RandomObstacles_2_voxelStates_3.mat','voxelStates');
disp('saved voxelStates_3.mat')