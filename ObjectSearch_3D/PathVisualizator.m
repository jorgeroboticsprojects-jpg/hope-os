

load('Test_results_data/All_Maps_boosterChanges.mat', 'STATS');

folderPath=('maps_ROI_Object_Situations/RandomObstacles_2_situations');
mapName=('RandomObstacles_2');
occMatrixPath = fullfile(folderPath, [mapName 'OccMatrix.mat']);
sensorPosesPath = fullfile(folderPath, 'sensorPoses.mat');
voxelStatesPath = fullfile(folderPath, [mapName '_voxelStates_3.mat']);
workSpacePath = fullfile(folderPath, [mapName 'WorkSpace.mat']);

[~, map3D, voxelStates, ~,~, ~, ~, ~] = ...
                    Initialization_ObjectSearch (occMatrixPath, voxelStatesPath, sensorPosesPath, workSpacePath);

sit = STATS.RandomObstacles_2.situation3;
mapNum = length(sit.exploredMap);

for i =23:36%mapNum
    posOrientations_done = sit.posOrientations_done{i};

    % =============== Show 3D map ==================================================
    figure;
    show(map3D);
    axis equal; grid on; hold on;
    view(135, 30);
    % =============== PLOTTING FOR DEPURATION ======================================
    for L=1:size(posOrientations_done,1)  
        pos = posOrientations_done(L,1:3);  quat = posOrientations_done(L,4:7);
        % draw trajectory
        hold on;
        % draw pos 
        scatter3(pos(1), pos(2), pos(3), 30, [1, 0.0, 0.1], 'filled', 'MarkerFaceAlpha', 1);
        if L >= 2
            prevPos = posOrientations_done(L-1,1:3);
            plot3([prevPos(1) pos(1)], [prevPos(2) pos(2)], [prevPos(3) pos(3)], 'k-', 'LineWidth', 1);
        end

        % draw orientations
        rotm = quat2rotm(quat);
        range=8;
        direction = rotm(:, 1) * range;
        quiver3(pos(1), pos(2), pos(3), ...
                direction(1), direction(2), direction(3), ...
                0, 'LineWidth', 2, 'MaxHeadSize', 2, 'Color', 'g');
        % Draw object
        [gx, gy, gz] = ind2sub(size(voxelStates), find(voxelStates == 5)); % Goal (object)
        scatter3(gx-0.5, gy-0.5, gz-0.5, 50, [0.6, 0.2, 0.8], 'filled', 'MarkerFaceAlpha', 1);
    end
                
end

