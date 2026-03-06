
load('Test_results_data/HOPE_OS_a06_b04_c0.mat', 'STATS');
load("Maps_struct.mat");
mapNames = fieldnames(STATS);

for r=1:5
    occupancyMap = Maps.occMaps{r,1}; 
    mapName = mapNames{r}; 

    trajectories = STATS.(mapName).situation1.posOrientations_done;

    figure;
    grid on;
    show(occupancyMap, "world");
    axis equal;
    axis tight;
    title('');
    xlabel('');
    ylabel('');
    hold on;
    xlim([0 90]);
    ylim([0 90]);

    % Plot 2 first trajectories
    for t=1:3
       pathDone = trajectories{t};

        plot(pathDone(:,1), pathDone(:,2), '-');
        hold on;
    
        for i = 1:size(pathDone, 1)
            x = pathDone(i, 1);
            y = pathDone(i, 2);
    
            % In your stored format, the planar yaw is encoded in (col4, col7)
            qz = pathDone(i, 4);
            qw = pathDone(i, 7);
    
            % Optional: normalize (safe against numerical drift)
            n = hypot(qz, qw);
            if n > 0
                qz = qz / n;
                qw = qw / n;
            end
    
            % Yaw angle in radians
            yaw = 2 * atan2(qz, qw);
    
            dx = arrowLength * cos(yaw);
            dy = arrowLength * sin(yaw);
    
            % All arrows in red
            quiver(x, y, dx, dy, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
        end
    end
end



