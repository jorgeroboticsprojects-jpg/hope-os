%profile on
clear; 
%% ENVIRONMENT SETTINGS
% Controls the visualization
plotting=0; % (There are also extra visualization controls inside some functions)
voxelEdgeSize=0.05; % %Set the real voxel edge size in meters (15cm)

%% SENSOR SETTINGS
maxrange =  2.5 / voxelEdgeSize;             % Set the sensor's fustrum range in meters
numRaysXY = 85;                              % Set resolution of the sensor
numRaysZ =  70;                              % Ajust depending on fustrum size and voxel size
anglesXY = linspace(-pi/4, pi/4, numRaysXY); % Set fustrum yaw plane angle span (~87 deg)
anglesZ = linspace(-pi/6, pi/6, numRaysZ);   % Set fustrum pitch plane angle span (~58 deg)
directions = zeros(numRaysXY * numRaysZ, 3); % Compute sensor rays directions

%% Select all testing Maps
mapsFolderPath = 'Maps_ROI_Object_Situations';
mapDirs = dir(fullfile(mapsFolderPath, '*_situations')); % Find every folder named '..._situations...'
mapDirs = mapDirs([mapDirs.isdir]);


for m=1:length(mapDirs)
    folderName = mapDirs(m).name;
    folderPath = fullfile(mapsFolderPath, folderName);

    % Extract map name (before "_situations")
    underscorePos = strfind(folderName, '_situations');
    mapName = folderName(1:underscorePos - 1);
    fprintf('Processing map: %s\n', mapName);


    %% Obtain different starting situations for each map
    situationFiles = dir(fullfile(folderPath, '*voxelStates_*.mat')); % Find every script named '...voxelStates_...'
    [~, idx] = sort( str2double( regexp({situationFiles.name}, '\d+', 'match', 'once') ) ); % Reorder them by name
    situationFiles = situationFiles(idx);

    % Common paths
    occMatrixPath = fullfile(folderPath, [mapName 'OccMatrix.mat']);
    workSpacePath = fullfile(folderPath, [mapName 'WorkSpace.mat']);
    sensorPosesPath = fullfile(folderPath, 'sensorPoses.mat');

    % Do different object locations and ROIs
    for i=1:length(situationFiles)
        % obtain situation path
        voxelStatesPath = fullfile(folderPath, situationFiles(i).name);
    
        % Initialize statistics tracking structure
        STATS.(mapName).(sprintf('situation%d', i)).posOrientations_done={}; 
        STATS.(mapName).(sprintf('situation%d', i)).exploredMap={};
        STATS.(mapName).(sprintf('situation%d', i)).exploreROI={};
        STATS.(mapName).(sprintf('situation%d', i)).objectFound={};
        STATS.(mapName).(sprintf('situation%d', i)).goalPosition={};

        % INITIALIZATION FUNCTION
        [~, ~, ~, ~,sampled_q_orientations, sensorPoses, GoalPosition, mapSize,WorkSpace] = ...
                    Initialization_ObjectSearch (occMatrixPath, voxelStatesPath, sensorPosesPath,workSpacePath);
        
        % Do different starting positions
        for numPose=1:size(sensorPoses,1)
                
            % INITIALIZATION FUNCTION
            [occMatrix, map3D, voxelStates, ROIsize,~, ~, ~, ~,~] = ...
                    Initialization_ObjectSearch (occMatrixPath, voxelStatesPath, sensorPosesPath,workSpacePath);
            
            fprintf('  [INFO] > Situation %d: %s\n', i, situationFiles(i).name);   % Show info
            fprintf('  [INFO] ---- > Starting Pose number %d: \n', numPose);     % Show info

            %% Show 3D map ===========================================================================
            % figure;
            % show(map3D);
            % axis equal; grid on; hold on;
            % view(135, 30);
            %% =======================================================================================

            % Initialize statistics for movement and exploration
            posOrientations_done=[]; % Saves the path chosen
            MapExp_Iterations=[]; % Map explored over total iterations (also orientations)
            ROIExp_Iterations=[]; % ROI explored over total iterations (also orientations)
            objectFound = 0; % 1 if object was found
   
            poses = sensorPoses(numPose,:);
            LastPose = poses;
            
            iteration=0;
            while ~objectFound
                % --- Calls method Movements (moves the agent to next poses and updates the voxelStates)
                [voxelStates, MapExp_Iterations, ROIExp_Iterations, posOrientations_done, objectFound] = VoxelMapManager.Movements(voxelStates,poses, maxrange, numRaysXY,...
                       numRaysZ, anglesXY, anglesZ, directions, GoalPosition, plotting, ROIsize,...
                       MapExp_Iterations, ROIExp_Iterations, posOrientations_done, objectFound,mapSize,occMatrix);
                
                %% % =============== PLOTTING FOR DEPURATION ======================================
                % for L=1:size(posOrientations_done,1)  
                %     pos = posOrientations_done(L,1:3);  quat = posOrientations_done(L,4:7);
                %     % draw trajectory
                %     hold on;
                %     % draw pos 
                %     scatter3(pos(1), pos(2), pos(3), 30, [1, 0.0, 0.1], 'filled', 'MarkerFaceAlpha', 1);
                %     if L >= 2
                %         prevPos = posOrientations_done(L-1,1:3);
                %         plot3([prevPos(1) pos(1)], [prevPos(2) pos(2)], [prevPos(3) pos(3)], 'k-', 'LineWidth', 1);
                %     end
                % 
                %     % draw orientations
                %     rotm = quat2rotm(quat);
                %     range=8;
                %     direction = rotm(:, 1) * range;
                %     quiver3(pos(1), pos(2), pos(3), ...
                %             direction(1), direction(2), direction(3), ...
                %             0, 'LineWidth', 2, 'MaxHeadSize', 2, 'Color', 'g');
                %     % Draw object
                %     [gx, gy, gz] = ind2sub(size(voxelStates), find(voxelStates == 5)); % Goal (object)
                %     scatter3(gx-0.5, gy-0.5, gz-0.5, 50, [0.6, 0.2, 0.8], 'filled', 'MarkerFaceAlpha', 1);
                % end
                %% % ==============================================================================
                
                % Exit if object was found
                if objectFound ==1
                    disp('  [INFO] Objective Found. Exiting loop...');
                    break;
                end

                % --- Next best position ------
                [NBM, roiGradient, fake_map] = ExplorationManager.computeNextBestMove(voxelStates, LastPose, mapSize, map3D, voxelEdgeSize, WorkSpace, plotting);
                %disp('NBM done');

                % --- Next Best orientations ------
                NBO = ExplorationManager.NextBestOrientation(voxelStates, LastPose, NBM, roiGradient,...
                              sampled_q_orientations, mapSize, voxelEdgeSize);
                %disp('NBO done');
                iteration=iteration+1;
                fprintf('  [INFO] -------- > Next best poses done, iteration num: %d \n', iteration);

                % --- Update new poses ------
                Newposes = zeros(size(NBO,1),7); % Initialize new poses
                for r=1:size(NBO,1)
                        Newposes(r,1)=NBM(1);   Newposes(r,2)=NBM(2);   Newposes(r,3)=NBM(3);
                        Newposes(r,4)=NBO(r,1); Newposes(r,5)=NBO(r,2); Newposes(r,6)=NBO(r,3); Newposes(r,7)=NBO(r,4);
                end
                poses=Newposes;

                % --- Update LastPose ------
                LastPose=Newposes(end,:);
            end
            
            %% Saving Stats
            STATS.(mapName).(sprintf('situation%d', i)).posOrientations_done{numPose,1} = posOrientations_done;
            STATS.(mapName).(sprintf('situation%d', i)).exploredMap{numPose,1} = MapExp_Iterations;
            STATS.(mapName).(sprintf('situation%d', i)).exploreROI{numPose,1} = ROIExp_Iterations;
            STATS.(mapName).(sprintf('situation%d', i)).objectFound{numPose,1} = objectFound;
            STATS.(mapName).(sprintf('situation%d', i)).goalPosition{numPose,1}=GoalPosition;
            % save('Test_results_data/All_7_Maps_booster_100_3asdfsd',"STATS");
            % disp('  [INFO] STATS pre-saved as: All_7_Maps_booster_100_3asdfsd');
            
            
            % folderName = 'Maps_ROI_Object_Situations/Office_1_Situations/Figs_situation_1';
            % figPath = fullfile(folderName, sprintf('figure_%d.fig', numPose));
            % savefig(gcf, figPath);
            % close all;

         end
        fprintf('  > Situation FINISHED %d: %s\n', i, situationFiles(i).name);
    end

end
disp('TESTING FINISHED')
%profile viewer




