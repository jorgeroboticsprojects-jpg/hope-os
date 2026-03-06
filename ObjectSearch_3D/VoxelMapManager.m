classdef VoxelMapManager 
    % VoxelMapManager - Manages a 3D voxel occupancy map with
    % visualization, sensor update and raytracing
    % This class encapsulates voxel state updates, frontier detection, and 3D visualization.
    methods (Static)    
        %% Movements - Update map sequence
        function [voxelStates, MapExp_Iterations, ROIExp_Iterations, posOrientations_done, objectFound] = Movements(voxelStates, poses, maxrange, numRaysXY,...
                 numRaysZ, anglesXY, anglesZ, directions, GoalPosition, plotting, ROIsize, MapExp_Iterations, ...
                 ROIExp_Iterations, posOrientations_done, objectFound, mapSize,occMatrix)

            for k = 1:size(poses, 1)
                sensorPose = poses(k, :);
                sensorPose(1:3) = sensorPose(1:3) - 0.5;  % Shift X, Y, Z to voxel centers
                posOrientations_done = [posOrientations_done; sensorPose];
        
                if plotting == 1
                    cla; pause(0.001); hold on;
                end
        
                ArrowPlotting = 0;
        
                voxelStates_copy = VoxelMapManager.updateMap(voxelStates,sensorPose,maxrange,numRaysXY,...
                        numRaysZ, anglesXY, anglesZ, directions,occMatrix,mapSize,plotting,ArrowPlotting);

                voxelStates = VoxelMapManager.markFrontierVoxels(voxelStates,voxelStates_copy);
        
                if plotting == 1
                    VoxelMapManager.showVoxelMap(voxelStates);
                end
        
                [mapExp, ROIExp] = VoxelMapManager.mapExpPercentage(voxelStates, mapSize, ROIsize);
                MapExp_Iterations = [MapExp_Iterations, mapExp];
                ROIExp_Iterations = [ROIExp_Iterations, ROIExp];
                
                for i = 1:size(GoalPosition, 1)
                    if voxelStates(GoalPosition(i,1), GoalPosition(i,2), GoalPosition(i,3)) ~= 5
                        objectFound = 1;
                        disp('  [INFO] Object Found! :)')
                        break;
                    end
                end
                if objectFound==1
                    break;
                end
        
                if plotting == 1
                    pause(0.1);
                end
            end
        end

        %% mapExpPercentage - Function to calculate exploration proportion of the map and ROI 
        function [mapExp, ROIExp] = mapExpPercentage(voxelStates,mapSize, ROIsize)
            % Calculates percentage of explored map and ROI

            countUnexplored = 0;
            countUnexploredROI = 0;

            for i = 1:mapSize(1)
                for j = 1:mapSize(2)
                    for k = 1:mapSize(3)
                        if voxelStates(i,j,k) == 2
                            countUnexplored = countUnexplored + 1;
                        end
                        if voxelStates(i,j,k) == 4 || voxelStates(i,j,k) == 5
                            countUnexplored = countUnexplored + 1;
                            countUnexploredROI = countUnexploredROI + 1;
                        end
                    end
                end
            end

            totalVoxels = prod(mapSize);
            mapExp = 1 - (countUnexplored / totalVoxels);
            ROIExp = 1 - (countUnexploredROI / ROIsize);
        end


        %% markFrontierVoxels - Mark frontier cells as unexplroed
        function voxelStates_copy = markFrontierVoxels(voxelStates,voxelStates_copy)
            % Marks frontier voxels based on unexplored neighbors
            % we mark these frontier as unexplored to stay pessimist.
            % A frontier voxel is a free voxel next to unexplored ones

            % Create logical matrices to identify free and unexplored voxels
            freeVoxels = (voxelStates_copy == 0); % Free voxels
            unexploredVoxels = (voxelStates_copy == 2 | voxelStates_copy == 4 | voxelStates_copy == 5);  % Unexplored voxels

            % Create a 3D kernel that checks neighbors in all three dimensions
            kernel3D = zeros(3, 3, 3);
            kernel3D(2,2,1) = 1; % Neighbor in the -Z direction
            kernel3D(2,2,3) = 1; % Neighbor in the +Z direction
            kernel3D(2,1,2) = 1; % Neighbor in the -Y direction
            kernel3D(2,3,2) = 1; % Neighbor in the +Y direction
            kernel3D(1,2,2) = 1; % Neighbor in the -X direction
            kernel3D(3,2,2) = 1; % Neighbor in the +X direction;
            
            % Apply 3D convolution to count unexplored neighbors for each free voxel
            neighborCount3D = convn(double(unexploredVoxels), kernel3D, 'same');

            % Identify frontier voxels where there are free voxels and at least one unexplored neighbor
            frontierVoxels = freeVoxels & (neighborCount3D > 0);

            % Update voxelStates to mark frontier voxels
            % If it is frontier we put again the value that had before
            voxelStates_copy(frontierVoxels) = voxelStates(frontierVoxels);
        end
        
        %% updateMap - Ray tracing and occupancy information update
        function voxelStates = updateMap(voxelStates,sensorPose,maxrange,numRaysXY, numRaysZ,...
                         anglesXY, anglesZ, directions,occMatrix,mapSize,plotting,ArrowPlotting)
            % Uses raytracing to simulate the field FoV, marks free and
            % occupied voxels.

            if plotting == 1
                hold on; axis equal; grid on;
            end
            
            % Build a 3D occupancy map from occMatrix
            map3D = occupancyMap3D(1);
            [x, y, z] = ind2sub(size(occMatrix), find(occMatrix == 1));
            occupiedVoxels = [x, y, z]-1;
            
            % Stablish each occupied possition in the matrix
            for i = 1:size(occupiedVoxels, 1)
                setOccupancy(map3D, occupiedVoxels(i, :), 1);
            end

            if plotting == 1
                h = show(map3D);
                axis equal; grid on;
                set(h, 'XLim', [0 size(voxelStates, 1)], 'YLim', [0 size(voxelStates, 2)], 'ZLim', [0 size(voxelStates, 3)]);
                view(135,30);
                title('3D Map');
                pause(0.1);
            end
            
            % Calculate all directions from yaw/pitch grids
            index = 1;
            for i = 1:numRaysXY
                for j = 1:numRaysZ
                    directions(index, 1) = cos(anglesXY(i)) * cos(anglesZ(j)); % Component X
                    directions(index, 2) = sin(anglesXY(i)) * cos(anglesZ(j)); % Component Y
                    directions(index, 3) = sin(anglesZ(j));                    % Component Z
                    index = index + 1;
                end
            end
            
            % Perform raycasting, compute intersections with obstacles
            [intersectionPts, isOccupied] = rayIntersection(map3D, sensorPose, directions, maxrange);
            
            if plotting==1
                % Define step to reduce the number of plotted rays 
                stepRow = 10;  % Step per Row
                stepCol = 10;  % Step per Col
    
                for i = 1:length(isOccupied)
                    row = mod(i-1, 50) + 1;  % Find row (1-50)
                    col = floor((i-1) / 50) + 1;  % Find column (1-50)
    
                    % Only show rays between steps
                    if mod(row, stepRow) == 0 && mod(col, stepCol) == 0
                        plot3([sensorPose(1), intersectionPts(i,1)], ...
                              [sensorPose(2), intersectionPts(i,2)], ...
                              [sensorPose(3), intersectionPts(i,3)], '-b'); % Plot ray
                        if isOccupied(i) == 1
                            % Draw intersection point
                            plot3(intersectionPts(i,1), intersectionPts(i,2), intersectionPts(i,3), '*r'); 
                        end
                    end
                end
            end
       
            % Mark occupied voxels (obstacles detected)
            validIntersections = isOccupied == 1;
            % Create matrix to store obstacles rounded coords
            obsCoords = nan(size(intersectionPts));

            % Aply rounding based on quadrant for each coord
            obsCoords(validIntersections, 1) = floor(intersectionPts(validIntersections, 1) + (intersectionPts(validIntersections, 1)...
                                        >= sensorPose(1)) * 1.0001 + (intersectionPts(validIntersections, 1) < sensorPose(1)) * 0.9999);
            obsCoords(validIntersections, 2) = floor(intersectionPts(validIntersections, 2) + (intersectionPts(validIntersections, 2)...
                                        >= sensorPose(2)) * 1.0001 + (intersectionPts(validIntersections, 2) < sensorPose(2)) * 0.9999);
            obsCoords(validIntersections, 3) = floor(intersectionPts(validIntersections, 3) + (intersectionPts(validIntersections, 3)...
                                        >= sensorPose(3)) * 1.0001 + (intersectionPts(validIntersections, 3) < sensorPose(3)) * 0.9999);
            
            % Eliminate NaN values and points out of limits
            obsCoords = obsCoords(validIntersections, :);
            inBounds = all(obsCoords >= 1 & obsCoords <= mapSize, 2);
            obsCoords = obsCoords(inBounds, :);
            
            % Set occpied voxels into the 3D map
            for i = 1:size(obsCoords, 1)
                setOccupancy(map3D, obsCoords(i, :), 1);
            end
            
            % Convert obscoords into linear indices
            linearIndices = sub2ind(mapSize, obsCoords(:,1), obsCoords(:,2), obsCoords(:,3));
            % set obstacles in voxelstates
            voxelStates(linearIndices) = 1;
            
            %%%%% Free space
            % Calculate all ray coordinates along each ray's range
            distances = sqrt((sensorPose(1) - intersectionPts(:,1)).^2 + ...
                             (sensorPose(2) - intersectionPts(:,2)).^2 + ...
                             (sensorPose(3) - intersectionPts(:,3)).^2);
            distances(~validIntersections) = maxrange;  % Rango máximo para rayos sin intersección
            
            % Generate vector of steps and points for each ray depending on the max disatnce
            maxSteps = ceil(max(distances) / 0.5); 
            t = (0:0.5:(maxSteps * 0.5))';
            
            % calculate directions from sensorPose and intersection points
            dirVectors = intersectionPts - sensorPose(1:3);
            
            % Calculate the size of each direction vector
            dirMagnitudes = sqrt(sum(dirVectors.^2, 2));
            
            % Normalize
            dirX = dirVectors(:, 1) ./ dirMagnitudes;
            dirY = dirVectors(:, 2) ./ dirMagnitudes;
            dirZ = dirVectors(:, 3) ./ dirMagnitudes;
            
            % Expand 't' to the total number of rays (columns) and compute coordinates
            x_rays = sensorPose(1) + t .* dirX';
            y_rays = sensorPose(2) + t .* dirY';
            z_rays = sensorPose(3) + t .* dirZ';
            
            % Limit points to the max range
            rayLimits = ceil(distances / 0.5);
            for r = 1:size(directions, 1)
                x_rays(rayLimits(r)+1:end, r) = NaN;
                y_rays(rayLimits(r)+1:end, r) = NaN;
                z_rays(rayLimits(r)+1:end, r) = NaN;
            end
            
            % Apply map limits
            validIdx = ~isnan(x_rays) & ~isnan(y_rays) & ~isnan(z_rays) & ...
                       x_rays <= mapSize(1) & x_rays > 0 & ...
                       y_rays <= mapSize(2) & y_rays > 0 & ...
                       z_rays <= mapSize(3) & z_rays > 0;
            
            % round depending con the quadrant
            x_rays(validIdx) = ceil(x_rays(validIdx));
            y_rays(validIdx) = ceil(y_rays(validIdx));
            z_rays(validIdx) = ceil(z_rays(validIdx));
            
            % Convert coordinates into indices in voxelStates
            linearIndices = sub2ind(mapSize, x_rays(validIdx), y_rays(validIdx), z_rays(validIdx));
            
            % Marck free cells
            isFreeCell = (voxelStates(linearIndices) ~= 1 & voxelStates(linearIndices) ~= 10 );
            voxelStates(linearIndices(isFreeCell)) = 0;
        
            %%%%%%%%%% %%%%%%%%%% ARROW P %%%%%%%%%% %%%%%%%%%%
            if ArrowPlotting==1
                    position = sensorPose(1:3);
                    quaternion = sensorPose(4:7);
                    range = 13;
                    rotm = quat2rotm(quaternion);
                    direction = rotm(:, 1);
                    direction = direction * range;
                    quiver3(position(1), position(2), position(3), ...
                    direction(1), direction(2), direction(3), ...
                    0, 'LineWidth', 2, 'MaxHeadSize', 2, 'Color', 'r'); % Flecha azul
            end

        end
        
        %% Plotting map in 3D
        function showVoxelMap(voxelStates)
            % Displays the voxel state map with colors:
            %   Occupied (1) - red || Free (0) - green transparent
            %   ROI (4) - yellow transparent || Goal (5) - violet
            hold on; axis equal;
            xlabel('X'); ylabel('Y'); zlabel('Z');
            title('3D Occupancy Map');
            
            % Find positions
            [ox, oy, oz] = ind2sub(size(voxelStates), find(voxelStates == 1)); % Obstacles
            [fx, fy, fz] = ind2sub(size(voxelStates), find(voxelStates == 0)); % Free
            [rx, ry, rz] = ind2sub(size(voxelStates), find(voxelStates == 4)); % ROI
            [gx, gy, gz] = ind2sub(size(voxelStates), find(voxelStates == 5)); % Goal (object)
            
            % Drawing
            scatter3(ox-0.5, oy-0.5, oz-0.5, 100, [0.7, 0.2, 0.2], 'filled', 'MarkerFaceAlpha', 1);
            scatter3(fx-0.5, fy-0.5, fz-0.5, 50, [0.2, 0.8, 0.2], 'filled', 'MarkerFaceAlpha', 0.3);
            scatter3(rx-0.5, ry-0.5, rz-0.5, 50, [1, 0.8, 0], 'filled', 'MarkerFaceAlpha', 0.05);
            scatter3(gx-0.5, gy-0.5, gz-0.5, 50, [0.6, 0.2, 0.8], 'filled', 'MarkerFaceAlpha', 1);

            view(135, 30);
        end
    end
end
