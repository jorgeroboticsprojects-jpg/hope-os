

function [occMatrix, map3D, voxelStates, ROIsize, sampled_q_orientations, GoalPosition, mapSize,WorkSpace] = ...
         Initialization_ObjectSearch_2D(matPath,situationPath,workSpacePath)
    
    % Load file
    data = load(matPath);
    data2 = load(situationPath);
    %data3 = load(posesPath);
    data4 = load(workSpacePath);

    %% Extract voxelStates
    if isfield(data2, 'voxelStates')
        voxelStates = data2.voxelStates;
    else
        error('The .mat file must contain a variable named "voxelStates".');
    end
    mapSize = size(voxelStates);
    mapSize(3) = 1;
    ROIsize = length(find(voxelStates == 4 | voxelStates == 5));
    % Obtain object position
    idx = find(voxelStates == 5);  
    [x, y, z] = ind2sub(size(voxelStates), idx);  
    GoalPosition = [x, y, z]; 

    %% Extract sensor poses
    % if isfield(data3, 'sensorPose')
    %     sensorPose = data3.sensorPose;
    % else
    %     error('The .mat file must contain a variable named "sensorPoses".');
    % end

    %% Extract occMatrix
    if isfield(data, 'occMatrix')
        occMatrix = data.occMatrix;
    else
        error('The .mat file must contain a variable named "occMatrix".');
    end

    % Convert to occupied voxels
    [x, y, z] = ind2sub(size(occMatrix), find(occMatrix == 1));
    occupiedVoxels = [x y z] - 1;

    % Create occupancy map
    map3D = occupancyMap3D(1);  
    setOccupancy(map3D, occupiedVoxels, 1);

    %% Extract WorkSpace
    if isfield(data4, 'WorkSpace')
        WorkSpace = data4.WorkSpace;
    else
        error('The .mat file must contain a variable named "WorkSpace".');
    end

    %% Show 3D map
    % figure;
    % show(map3D);
    % title(['Occupancy Map: ' matPath], 'Interpreter', 'none');
    % axis equal; grid on;
    % view(135, 30);

    %% Sampled Angles for NBO
    % 1. Define angles (in degrees)

   %Create a vector of the sampled orientations in a 360x360 FoV
    yaw = deg2rad([0,30,60,90,120,150,180,210,240,270,300,330]); 
    pitch = deg2rad(0); % 2D
    
    % Initialize quaternions matrix
    sampled_q_orientations = zeros(length(yaw) * length(pitch)+2, 4);
    % Generate combinations using roll, pitch, yaw
    indx = 1;
    for i = 1:length(yaw)
        for j = 1:length(pitch)
            % roll is always 0
            q_total = eul2quat([yaw(i), pitch(j), 0], 'ZYX');
            sampled_q_orientations(indx, :) = q_total;
            indx = indx + 1;
        end
    end
    % Add manually looking up and down directions
    % q_total = eul2quat([0, pi/2, 0], 'ZYX');
    % sampled_q_orientations(end-1,:)=q_total;
    % q_total = eul2quat([0, 3*pi/2, 0], 'ZYX');
    % sampled_q_orientations(end,:)=q_total;
    
    % 4. Visualization of sampled orientations

    % figure; hold on; axis equal; grid on;
    % xlabel('X'); ylabel('Y'); zlabel('Z');
    % title('Sampled Orientations (Yaw & Pitch)');
    % view(135, 30);
    % % Reference sphere
    % [xs, ys, zs] = sphere(30);
    % surf(xs, ys, zs, 'FaceAlpha', 0.05, 'EdgeAlpha', 0.2, 'EdgeColor', [0.6 0.6 0.6]);
    % for i=1:length(sampled_q_orientations)
    %         hold on;
    %         position = [0,0,0];       % Initila position of the arrow
    %         quaternion = sampled_q_orientations(i,:);     % quaternion [qw, qx, qy, qz]
    %         range = 1;                  % Longitud de la flecha
    %         % Convert to rotation matirx
    %         rotm = quat2rotm(quaternion);   
    %         direction = rotm(:, 1);         
    %         % Scaling
    %         direction = direction * range;
    %         quiver3(position(1), position(2), position(3), ...
    %         direction(1), direction(2), direction(3), ...
    %         0, 'LineWidth', 1, 'MaxHeadSize', 1, 'Color', 'b'); 
    % end
end
