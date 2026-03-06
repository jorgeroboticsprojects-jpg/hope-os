% ExplorationManager.m
% Class for handling exploration decisions: NBM and NBO computation, gradient map generation, and orientation planning

classdef ExplorationManager
    % GradientMapManager - Handles generation and visualization of 3D gradient maps
    % Used for exploration decision making in voxel-based environments

    methods (Static)   
        %% gridSumAngles - Generates orientations from best orientations for resampling
        function new_orientations = gridSumAngles(bestAngle,div)
            % Increment the angle in each axis
            anglesVariation = deg2rad([-div, 0, div]);
        
            % Convert to Euler angles [yaw, pitch, roll]
            relative_angles = [anglesVariation(:), zeros(numel(anglesVariation),1), zeros(numel(anglesVariation), 1)];  % pitch=0, roll = 0, 2D
            relative_quats = eul2quat(relative_angles, 'ZYX'); % Conver to quaternions
            
            % Initialize new orientations matrix
            num_base_angles = size(bestAngle, 1);
            num_variations = size(relative_quats, 1);
            new_orientations = zeros(num_base_angles * num_variations, 4);
            
            % Compute new orientations using relative rotations to get
            % quaternion expression
            for i = 1:num_base_angles
                base_quat = repmat(bestAngle(i, :), num_variations, 1);
                new_orientations((i-1)*num_variations + (1:num_variations), :) = quatmultiply(base_quat, relative_quats);
            end
        end

        %% reorderAngles - This function reorders the angles to reduce the ang distance travelled
        function orderedOrientations = reorderAngles(bestAngle,LastPose)
            % Initialize variables
            startQuaternion = LastPose(4:7);  % Starting quaternion from `LastPose`
            numOrientations = size(bestAngle, 1);  % Number of orientations in `bestAngle`
            visited = false(numOrientations, 1);  % To mark visited orientations
            orderedOrientations = zeros(numOrientations, 4);  % To store the optimal order
            
            % Function to calculate angular distance between two quaternions
            quatDistance = @(q1, q2) 1 - abs(dot(q1, q2));  % Distance based on the dot product
            
            % Start from the initial quaternion
            currentQuaternion = startQuaternion;
            for i = 1:numOrientations
                % Find the next closest unvisited quaternion
                minDistance = inf;
                nextIndex = -1;
                for j = 1:numOrientations
                    if ~visited(j)
                        distance = quatDistance(currentQuaternion, bestAngle(j, :));
                        if distance < minDistance
                            minDistance = distance;
                            nextIndex = j;
                        end
                    end
                end
                
                % Add the next quaternion to the ordered list and mark it as visited
                orderedOrientations(i, :) = bestAngle(nextIndex, :);
                visited(nextIndex) = true;
                
                % Update the current quaternion to the selected quaternion
                currentQuaternion = bestAngle(nextIndex, :);
            end
        end
        
        %% NextBestOrientation - Compute best viewing orientations from NBM
        function NBO = NextBestOrientation(voxelStates, LastPose, NBM, roigradientMap,...
                        sampled_q_orientations, mapSize, voxelEdgeSize)

            x=NBM(1); y=NBM(2); z=NBM(3);
            sensorPose = [x-0.5, y-0.5, z-0.5, 1, 0, 0, 0]; % Shift X, Y, Z to voxel centers

            % Sensor FoV Simulation parameters
            maxrange =  1.9 / voxelEdgeSize;             % Set the sensor's fustrum range in meters
            numRaysXY = 60;                              % Set resolution of the sensor
            numRaysZ =  1;                               % Ajust depending on fustrum size and voxel size
            anglesXY = linspace(-pi/6, pi/6, numRaysXY); % Set fustrum yaw plane angle span (~60 deg)
            anglesZ = 0;                                 % Set fustrum pitch plane angle span (0 deg) 2D
            directions = zeros(numRaysXY * numRaysZ, 3); % Compute sensor rays directions
            
            % Unexplored Matrix except from obstacles, to clearly mark the visible voxels
            voxelSim = voxelStates;
            voxelSim(voxelSim ~= 1) = 2;
        
            bestAngle = [];
            % SET to only 1, simplest mode.
            for p = 1:1 % We don't want much reslotuion for now
                scores=zeros(1,length(sampled_q_orientations));
                scoresMean=scores;
                if p == 1
                    sampledAngles = sampled_q_orientations;
                elseif p == 2
                    % Resampling with a 15 degrees resolution
                    sampledAngles = ExplorationManager.gridSumAngles(bestAngle, 15);
                elseif p == 3
                    % Resampling with a 5 degrees resolution
                    sampledAngles = ExplorationManager.gridSumAngles(bestAngle, 5);
                end
            
                for i=1:size(sampledAngles,1)
                    sensorPose(4:7)=sampledAngles(i,:);
                    
                    plotting = 0; ArrowPlotting=0;
                    voxelStates_fake_FoV = VoxelMapManager.updateMap (voxelSim,sensorPose,maxrange,numRaysXY,...
                        numRaysZ, anglesXY, anglesZ, directions,voxelSim,mapSize,plotting,ArrowPlotting);
                    
                    % Eliminate borders to be pesimistic
                    % voxelStates_fake_FoV = VoxelMapManager.markFrontierVoxels(voxelStates,voxelStates_fake_FoV);

                    if plotting==1
                        VoxelMapManager.showVoxelMap(voxelStates_fake_FoV);
                    end
    
                    % Obtains the position of the visible cells
                    [x_FoV, y_FoV, z_FoV] = ind2sub(size(voxelStates_fake_FoV), find(voxelStates_fake_FoV == 0 & voxelStates ~= 0 ));  %does not count already explored cells

                    if ~isempty(x_FoV)
                        testedPosition=sensorPose(1:3)+0.5; % (+0.5) round to get coordinates
                        % Calculates distances from sensor to each voxel
                        distances_FoV = sqrt((x_FoV - testedPosition(1)).^2 + ...
                                             (y_FoV - testedPosition(2)).^2 + ...
                                             (z_FoV - testedPosition(3)).^2) * voxelEdgeSize;
            
                        gradients_FoV = roigradientMap(sub2ind(mapSize, x_FoV, y_FoV, z_FoV));

                        boostFactor = 100; % boost factor for roi voxels, will only affect when rois are visible
                        gradients_FoV(gradients_FoV == 1) = gradients_FoV(gradients_FoV == 1) * boostFactor;
                    
                        % Function that decreases as the distance increases
                        weights_FoV = (1 ./ (distances_FoV .^ 2 + 0.4))*0.4;  
                        % Aplies the weights of the gradient values and computes ponderated sum
                        sumScore = sum(weights_FoV .* gradients_FoV);
                        
                        % Average gradient value of visible voxels
                        meanScore = sum(gradients_FoV) / length(gradients_FoV);
                    else
                        sumScore=0; meanScore=0;
                    end
                    
                    % vector to store resulst of every orientation
                    scores(i)=sumScore;
                    scoresMean(i)=meanScore;
                end
                if ~max(scores)==0
                    scores=scores/max(scores); %Normalize the scores vector
                end
                if ~max(scoresMean)==0
                    scoresMean=scoresMean/max(scoresMean);
                end

                combinedGradientValue = 1*scores + 0.0*scoresMean;

                % Define threshold. Every orientation as good as the 80% of the
                % best orientation will be taken.
                threshold = 0.8 * (max(combinedGradientValue));
                % Obtain indices above threshold
                indices_above_threshold = find(combinedGradientValue > threshold);
                bestAngle = sampledAngles(indices_above_threshold,:);
            end
    
            % This function reorders the angles to reduce the ang distance travelled
            orderedOrientations = ExplorationManager.reorderAngles(bestAngle,LastPose);
    
            NBO = orderedOrientations;

            % Depuration
            if isempty(NBO) || size(NBO, 1) < 1
                fprintf('\n  [ERROR] No Next Best Orientation (NBO) was selected. In Funtcion ExplorationManager.NextBestOrientation \n');
                fprintf('  >>>> totalScore: min = %.3f, max = %.3f, mean = %.3f\n', ...
                    min(combinedGradientValue), max(combinedGradientValue), mean(combinedGradientValue));
                fprintf('\n   >>>> combined GradientValue')
                combinedGradientValue
                fprintf('\n   >>>> sampled Angles')
                sampledAngles
            
                fprintf('\n   >>>> ordered Orientations')
                orderedOrientations
            end
        end

        %% computeNextBestMove - Compute Next Best Move (NBM)
        function [NBM, roiGradient, fake_map] = ...
                 computeNextBestMove(voxelStates, LastPose, mapSize, map3D, voxelEdgeSize, WorkSpace, plotting,randNum)

            NBM = [];       % Placeholder for Next Best Move coordinates
            x0 = LastPose(1); y0 = LastPose(2); z0 = LastPose(3);
            
            % --- Gradient Maps Generation ---
            % 0 = ROI gradient | 1 = Obstacle repulsion gradient
            roiGradient = ExplorationManager.computeGradient(voxelStates, voxelEdgeSize, 0);
            obsGradient = ExplorationManager.computeGradient(voxelStates, voxelEdgeSize, 1);

            % Obtain the possible next possitions (Only known free Space)
            linearIdx = find(voxelStates == 0);
            % Filter only the ones that are inside the workspace
            validIdx = WorkSpace(linearIdx) == 1;
            % Obtain coordinates of possible NBP
            [x, y, z] = ind2sub(size(voxelStates), linearIdx(validIdx));
            free_voxels = [x, y, z];
                        
                        % [x, y, z] = ind2sub(size(voxelStates), find(voxelStates == 0));
                        % free_voxels = [x, y, z];

            % --- Spherical FoV Configuration ---
            maxrange = 1.9 / voxelEdgeSize;
            numRaysXY = 250;
            numRaysZ = 1;
            anglesXY = linspace(-pi, pi, numRaysXY);
            anglesZ = 0;
            directions = zeros(numRaysXY * numRaysZ, 3);

            % Create fake map with current known obstacles
            fake_map = occupancyMap3D(1);
            % Find 3D coords of obstacles detected
            [x, y, z] = ind2sub(size(voxelStates), find(voxelStates == 1));
            occupiedVoxels = [x, y, z] - 1;
            for i = 1:size(occupiedVoxels, 1)
                coord = occupiedVoxels(i, :);
                setOccupancy(fake_map, coord, 1);
            end

            % Unexplored Matrix except from obstacles, to clearly mark the visible voxels
            voxelStates_fake = voxelStates;
            voxelStates_fake(voxelStates_fake ~= 1) = 2;

            % --- K-Means Loop ---
            p = 0;
            while true
                p = p + 1;

                % Dynamically adjust number of clusters to max 26
                numCandidates = size(free_voxels, 1);
                clustersize = 1; target_clusters = 36;
                while true
                    % calculates en numb of clusters based on cluster size
                    num_clusters = ceil(numCandidates / clustersize^2);
                    % check is the number of clusters is the desired
                    if num_clusters <= target_clusters 
                        break; 
                    else
                        %Increase cluster size if not achieved
                        clustersize = clustersize + 1;
                    end
                end
                % --- We add roi value to k-means to increase sampling
                % resolution in forntier areas ---

                % Obtain freevoxels gradient values
                roiValues = roiGradient(sub2ind(size(voxelStates), ...
                                free_voxels(:,1), ...
                                free_voxels(:,2), ...
                                free_voxels(:,3)));

                roiWeight = 10;  % ← adjustable to give it more importance (since gradient is between 0-1)
                free_voxels = [free_voxels, roiWeight * roiValues];


                % --- Cluster free voxels using K-means ---
                %rng(randNum);  % Make it deterministic
                [idx, centroids] = kmeans(free_voxels, num_clusters,...
                        'Start', 'plus', ...
                        'Replicates', 5, ...
                        'MaxIter', 500,  ...
                        'Display', 'off');
                centroidVoxels = round(centroids(:,1:3)); % Round to get int voxel coord
                % Check validity
                validCentroids = [];
                for i = 1:size(centroidVoxels,1)
                    c = centroidVoxels(i,:);
                    % Check that the voxel index is within map bounds
                    if all(c > 0) && ...                    % All indices must be positive
                       c(1) <= size(voxelStates,1) && ...   % X index within bounds
                       c(2) <= size(voxelStates,2) && ...   % Y index within bounds
                       c(3) <= size(voxelStates,3)          % Z index within bounds

                       % Check if the voxel at this position is marked as free (value == 0)
                       if voxelStates(c(1), c(2), c(3)) == 0
                           % If it's valid and free, keep it
                            validCentroids(end+1,:) = centroidVoxels(i,:);
                       end
                    end
                end
                centroidVoxels = validCentroids;

                
                
                % Last pose is added for evaluation
                %if p == 1
                centroidVoxels(end+1, :) = [x0, y0, z0];  % Include current position
                %end

                %--- Optional Clustering Plot ---
                plotting=1;
                if plotting == 1
                    cla; hold on;
                    map3Dfake = occupancyMap3D(1);
                    [x, y, z] = ind2sub(size(voxelStates_fake), find(voxelStates_fake == 1));
                    occupiedVoxels = [x, y, z]-1;
                    % Stablish each occupied possition in the matrix
                    for i = 1:size(occupiedVoxels, 1)
                        setOccupancy(map3Dfake, occupiedVoxels(i, :), 1);
                    end

                    h = show(map3Dfake);
                    axis equal; grid on;
                    set(h, 'XLim', [0 mapSize(1)], 'YLim', [0 mapSize(2)], 'ZLim', [0 mapSize(3)]);
                    view(0, 90); title('Clusters');
                    xlabel('X'); ylabel('Y'); zlabel('Z');
                    title('Centroids and Clusters Division');

                    % Plot voxel cube
                    cubeV = [0 0 0; mapSize(1) 0 0; mapSize(1:2) 0; 0 mapSize(2) 0; ...
                             0 0 mapSize(3); mapSize(1) 0 mapSize(3); mapSize; 0 mapSize(2) mapSize(3)];
                    cubeF = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
                    patch('Vertices', cubeV, 'Faces', cubeF, 'FaceColor', 'none', 'EdgeColor', 'black', 'FaceAlpha', 0.5);

                    % Plot clusters
                    colors = lines(num_clusters); %numb of different colors
                    for k = 1:num_clusters
                        % Select points belonging to cluster 'k'
                        cluster_points = free_voxels(idx == k, :);
                        % Plot clusters
                        scatter3(cluster_points(:,1) - 0.5, cluster_points(:,2) - 0.5, cluster_points(:,3) - 0.5, ...
                                 15, 'filled', 'MarkerFaceColor', colors(k, :),'MarkerFaceAlpha', 0.7);
                    end
                    %Plot centroids
                    scatter3(centroidVoxels(:,1), centroidVoxels(:,2), centroidVoxels(:,3), 100, 'k', 'filled', 'square');
                     view(0, 90)
                     pause(0.5)
                end

                % --- Score Evaluation of the possible FoVs---
                allScoresGradient = zeros(size(centroidVoxels, 1), 1);
                allScoresGradientMean=allScoresGradient;
                allScoresDispCost = allScoresGradient;
                allScoresObsNear = allScoresGradient;

                for i = 1:size(centroidVoxels,1)
                    %% Evaluation of the possible FoV
                    %Set sensor pose to centroids cell pos
                    sensorPose = [centroidVoxels(i,:)-0.5, 1, 0, 0, 0]; % Shift X, Y, Z to voxel centers
                    
                    % -- IMPORTANT! -- occMatrix is sent as voxelStates_fake because 
                    % we want to simulate occlusions only with known obstacles
                    plotting=0; ArrowPlotting = 0;
                    voxelSim = VoxelMapManager.updateMap(voxelStates_fake,sensorPose,maxrange,numRaysXY, numRaysZ,...
                         anglesXY, anglesZ, directions,voxelStates_fake,mapSize,plotting,ArrowPlotting);

                    % Eliminate borders to be pesimistic
                    voxelSim = VoxelMapManager.markFrontierVoxels(voxelStates,voxelSim);
                    
                    if plotting==1
                        VoxelMapManager.showVoxelMap(voxelSim)
                    end
                    

                    % Obtains the position ob the visible cells
                    [xf, yf, zf] = ind2sub(size(voxelSim), find(voxelSim == 0 & voxelStates ~= 0)); %does not count already explored cells
                    
                    if ~isempty(xf)
    
                        gradients_FoV = roiGradient(sub2ind(mapSize, xf, yf, zf));

                        boostFactor = 100; % boos factor for roi voxels, will only affect when rois are visible
                        gradients_FoV(gradients_FoV == 1) = gradients_FoV(gradients_FoV == 1) * boostFactor;
    
                        sumScore = sum(gradients_FoV);;
                        
                        % Average gradient value of visible voxels
                        meanScore = sum(gradients_FoV) / length(gradients_FoV);
                    else
                        sumScore=0; meanScore=0;
                    end
                           
                    % --2-- Calculate displacement distance and penalize further distances
                    dist = norm(centroidVoxels(i,:) - [x0, y0, z0]) * voxelEdgeSize;
                    % Displacement cost function
                    ep=2;
                    disp_cost= -(1/(dist^2 + ep)*ep)+1;

                    % --3-- If the NBP is really close to a detected object is penalizesit
                    obstacleNear=obsGradient(round(centroidVoxels(i,1)), round(centroidVoxels(i,2)), round(centroidVoxels(i,3)));
                    
                    % --- Save scores in vectors ---
                    allScoresGradient(i,1)=sumScore;
                    allScoresGradientMean(i,1)=meanScore;
                    allScoresDispCost(i,1)=disp_cost;
                    allScoresObsNear(i,1)=obstacleNear;

                end

                % Normalize scores
                allScoresGradient       = ExplorationManager.safeNormalize(allScoresGradient);
                allScoresGradientMean   = ExplorationManager.safeNormalize(allScoresGradientMean);
                                
                % Combine quality and quantity of gradients
                combinedGradientValue = 1*allScoresGradient + 0.0*allScoresGradientMean;
                
                % ignore positions that don't give any information
                minGradientThreshold = 0.05;
                combinedGradientValue(combinedGradientValue < minGradientThreshold) = -Inf;

                % Combine scores with weighted parameters
                a = 0.4;  % Weight for gradient value
                b = 0.6;  % Penalty for travel cost, careful, can't be higher than gradient value
                c = 0.0;  % Penalty for obstacle proximity Not for the algorithm in 2D
                totalScore = (a * combinedGradientValue - b * allScoresDispCost - c * allScoresObsNear);
                
                % Sort the total scores in descending order to find the best candidates
                [~, sortedIdx] = sort(totalScore, 'descend');
                NBM = centroidVoxels(sortedIdx(1), :); % Select the best candidate as the Next Best Move (NBM)
                
                
                % Choose the top K clusters to focus on (up to 12)
                topK = min(12, length(sortedIdx)); % Ensure we don't exceed available entries
                topClusters = sortedIdx(1:topK); % Indices of the top-scoring centroids

                % Filter the free voxels that belong to the selected top clusters
                % fro reclusteringfrng
                free_voxels = free_voxels(ismember(idx, topClusters), :);
                
                % Stop the loop if the number of remaining voxels is small enough
                if size(free_voxels,1) <= 200, break; end
            end
            % Depuration
            if isempty(NBM) || size(NBM, 1) < 1
                fprintf('\n  [ERROR] No Next Best Move (NBM) was selected. In Funtcion ExplorationManager.computeNextBestMove\n');
                fprintf('  >>>> totalScore: min = %.3f, max = %.3f, mean = %.3f\n', ...
                    min(totalScore), max(totalScore), mean(totalScore));
            
                fprintf('  >>>> allScoresGradient: min = %.3f, max = %.3f\n', ...
                    min(allScoresGradient), max(allScoresGradient));
            
                fprintf('  >>>> allScoresDispCost: min = %.3f, max = %.3f\n', ...
                    min(allScoresDispCost), max(allScoresDispCost));
                fprintf('  >>>> free_voxels')
                free_voxels
            end
        end

        %% computeGradient - Computes a gradient map based on ROI or obstacles
        function gradientMap = computeGradient(voxelStates, voxelEdgeSize, selectGradient)
            % Inputs:
            %   voxelStates     - 3D matrix of voxel states
            %   voxelEdgeSize   - Real size of each voxel (used to scale distance)
            %   selectGradient  - 0: ROI gradient, 1: obstacle gradient
            %
            % Output:
            %   gradientMap     - Computed gradient field

            mapSize = size(voxelStates);
            gradientMap = zeros(mapSize);

            % Define binary mask based on selection
            if selectGradient == 0
                mask = (voxelStates == 4 | voxelStates == 5); % ROI
            elseif selectGradient == 1
                mask = (voxelStates == 1); % Obstacles
            else
                error('Invalid gradient type. Use 0 for ROI, 1 for obstacles.');
            end

            % Compute distance transform from masked voxels.
            distances = bwdist(mask);
            distances = distances * voxelEdgeSize; % Convert to metric system

            % Initialize gradient values
            gradientMap(mask) = 1;

            % Apply gradient decay function for NON ROI or Obstcale voxels.
            %  We use shortest distance to ROI or Obstacle voxel.
            non_mask = ~mask;
            min_distances = distances(non_mask);
            
            % Compute Gradient valu based on the following function.
            if selectGradient == 0 % ROI GRADIENT FUCNTION
                if ~any(mask(:))
                    warning('computeGradient: No ROI to base gradient on, returning ones matrix gradient.');
                    gradientMap = ones(mapSize); % Uniform gradient if no ROI
                else
                    sharpen = 4;
                    avgdiv = sharpen + 1;
                    gradientMap(non_mask) = ...
                         ( (1 ./ (min_distances.^2 + 0.002)) * 0.002 * sharpen + ...
                         (1 ./ (min_distances.^2 + 1)) * 1 ) / avgdiv;
                    %(1 ./ (min_distances.^2 + 0.5)) * 0.5;
                end
            elseif selectGradient == 1 % OBSTACLE GRADIENT FUCNTION
                if ~any(mask(:))
                    fprintf('    [Info] computeGradient: No obstacles detected, returning Obstacle gardient as zeros matrix. \n');
                    gradientMap = zeros(mapSize); % No gradient from obstacles
                else
                    gradientMap(non_mask) = 1 ./ (1 + exp(40 * (min_distances - 0.10)));
                end
            end

            %ExplorationManager.visualizeGradient(gradientMap);

        end
        
        %% visualizeGradient - Displays the 3D gradient map layer by layer
        function visualizeGradient(gradientMap)
            % Inputs:
            %   gradientMap   - Gradient values (same size as voxelStates)
            %   voxelStates   - Used only for axis reference
        
            hold on;
            colormap('parula');

            % Divide the map in slices for 3D optimized visualization
            % Z Slices
            [X, Y] = meshgrid(1:size(gradientMap, 1), 1:size(gradientMap, 2));
            for z = 1:size(gradientMap, 3)
                slice_data = gradientMap(:, :, z)';
                surf(X, Y, z * ones(size(slice_data)), slice_data, 'EdgeColor', 'k');
            end
        
            % % Y Slices
            % [X, Z] = meshgrid(1:size(gradientMap, 1), 1:size(gradientMap, 3));
            % for y = 1:size(gradientMap, 2)
            %     slice_data = squeeze(gradientMap(:, y, :))';
            %     surf(X, y * ones(size(slice_data)), Z, slice_data, 'EdgeColor', 'k');
            % end
            % 
            % % X Slices
            % [Y, Z] = meshgrid(1:size(gradientMap, 2), 1:size(gradientMap, 3));
            % for x = 1:size(gradientMap, 1)
            %     slice_data = squeeze(gradientMap(x, :, :))';
            %     surf(x * ones(size(slice_data)), Y, Z, slice_data, 'EdgeColor', 'k');
            % end
        
            % Display settings
            axis([0 size(gradientMap,2) 0 size(gradientMap,1) 0 size(gradientMap,3)]);
            xlabel('X'); ylabel('Y'); zlabel('Z');
            colorbar; clim([0 1]);
            title('3D Gradient Map Visualization');
            view(0, 90);
        end
        
        %% Function to normalize vectors avoiding division per 0
        function v_norm = safeNormalize(v)
            maxVal = max(v);
            if maxVal == 0
                v_norm = zeros(size(v));
            else
                v_norm = v / maxVal;
            end
        end


    end
end
