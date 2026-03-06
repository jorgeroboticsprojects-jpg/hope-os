%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{ In this script we generate the diferent starting situations for the 
% algorithm. 6 different orientations for every position.
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Define angles (in degrees)

   %Create a vector of the sampled orientations in a 360x360 FoV
    yaw = deg2rad([0,90,180,270]); 
    pitch = deg2rad(0);
    
    % Initialize quaternions matrix
    sampled_q_orientations = zeros(length(yaw) * length(pitch) + 2, 4);
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
    q_total = eul2quat([0, pi/2, 0], 'ZYX');
    sampled_q_orientations(end-1,:)=q_total;
    q_total = eul2quat([0, 3*pi/2, 0], 'ZYX');
    sampled_q_orientations(end,:)=q_total;

%% 4. Visualization of orientations

    figure; hold on; axis equal; grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Sampled Orientations (Yaw & Pitch)');
    view(135, 30);
    % Reference sphere
    [xs, ys, zs] = sphere(30);
    surf(xs, ys, zs, 'FaceAlpha', 0.05, 'EdgeAlpha', 0.2, 'EdgeColor', [0.6 0.6 0.6]);
    for i=1:length(sampled_q_orientations)
            hold on;
            position = [0,0,0];       % Initila position of the arrow
            quaternion = sampled_q_orientations(i,:);     % quaternion [qw, qx, qy, qz]
            range = 1;                  % Longitud de la flecha
            % Convert to rotation matirx
            rotm = quat2rotm(quaternion);   
            direction = rotm(:, 1);         
            % Scaling
            direction = direction * range;
            quiver3(position(1), position(2), position(3), ...
            direction(1), direction(2), direction(3), ...
            0, 'LineWidth', 1, 'MaxHeadSize', 1, 'Color', 'b'); 
    end


%% DEFINE STARTING POSITIONS
    % For every position 6 orientations will be applied
    
    positions = [
        55 15 50;
        55 45 50;
        55 75 50;
        62 15 25;
        62 45 25;
        62 75 25;
    ];
    
    n_positions = size(positions, 1);
    n_orientations = size(sampled_q_orientations, 1);
    
    % Initialize Sensor poses
    sensorPose = zeros(n_positions * n_orientations, 7);  % [x y z q1 q2 q3 q4]
    
    % Create all combinations
    row = 1;
    for i = 1:n_positions
        for j = 1:n_orientations
            sensorPose(row, 1:3) = positions(i, :);                     % x y z
            sensorPose(row, 4:7) = sampled_q_orientations(j, :);        % quaternion
            row = row + 1;
        end
    end

%% Save LHC map Initial poses
save('../../Maps_ROI_Object_Situations/LHC_situations/sensorPoses.mat', 'sensorPose');

