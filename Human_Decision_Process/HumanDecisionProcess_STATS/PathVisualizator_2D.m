

load('HDP_DATA_5_MAPS/HumanDecisionProcess_bocadillo.mat', 'STATS');
load("Maps_struct.mat");

for r=1:5
    occupancyMap = Maps.occMaps{r,1}; 
    
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
    
    pathDone=STATS.posOrientations_done{r,1};
    %hold on;
    plot(pathDone(:,1),pathDone(:,2), '-o');
    
    arrowLength = 5;
    % orientation arrows
    for i = 1:size(pathDone, 1)
        x = pathDone(i, 1);
        y = pathDone(i, 2);
    
        % Displacement between points
        angle = pathDone(i, 3);
        dx = arrowLength * cos(angle);
        dy = arrowLength * sin(angle);
    
        % plot arrow
        quiver(x, y, dx, dy, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    end
end