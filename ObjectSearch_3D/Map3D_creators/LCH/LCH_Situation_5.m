%% Map dimensions (MAKE SURE THEY ARE THE SAME AS THE MAP CREATOR)
mapSizeX = 90; mapSizeY = 90; mapSizeZ = 70;
mapSize = [mapSizeX, mapSizeY, mapSizeZ];

%% Generate ROI and Object positions
% We store environment state in this matrix with this values:
% ---- 0 = free | 1= Occupied | 2 = unexplored | 3 = Frontier | 4 = ROI | 
% ---- 5 = Object
voxelStates = 2*ones(mapSize);



%% DEFINE ROI  SETTTINGS  
for x=1:mapSize(1)
    for y=1:mapSize(2)
        for z=1:mapSize(3)
            dist= sqrt((x-(38))^2 + (z-(26))^2);
            if dist < 13 && dist >= 10
                voxelStates(x,y,z)=4;
            end
        end 
    end
end


%% DEFINE OBJECTIVE LOCATION
objectX=31; objectY=15; objectZ=18;

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

%% Save Voxel Initial States Matrix
save('../../Maps_ROI_Object_Situations/LHC_situations/LHC_voxelStates_5.mat','voxelStates');
disp('saved voxelStates')