image = imread('865Office_ROI.jpg');

% Convert the image to double format to perform numerical computations
image = im2double(image);

% Define the brightness threshold. Colors close to white (RGB close to [1, 1, 1]) are considered free space
threshold = 0.95;  % Adjust this value if more precision is required

% Create a binary map where 1 represents free space (white or near white)
% and 0 represents obstacles (any other color)
% Here we compare the RGB channels with the threshold
binaryMap = all(image < threshold, 3);  % Check that R, G, and B are below the threshold

% Resize the image to a 90x90 grid
gridMap = imresize(binaryMap, [90, 90]);

% Create an occupancy map with the same grid dimensions (90x90)
occupancyMap = binaryOccupancyMap(90, 90, 1);  % Cells of 1 meter per side, for example

% Set the occupancy values of the map
setOccupancy(occupancyMap, gridMap);

% Display the occupancy map
figure;
%show(occupancyMap);
title('Occupancy map generated from image');

hold on;
for i = 0:90
    plot([i i], [0 90], 'k'); % Draw vertical grid line
    plot([0 90], [i i], 'k'); % Draw horizontal grid line
end

%save("LCH_map_cross_section","occupancyMap");
%save("Room","occupancyMap");

% Function to paint the map
cellStates = flipud(getOccupancy(occupancyMap));

% cellStates(cellStates == 1) = 5;
% cellStates(cellStates == false) = 2;

cellStates = cellStates * 2 + 2;
cellStates(75,75) = 0; 
cellStates(9,73) = 5;

paintMap(cellStates, occupancyMap)

%save("CCC_ROI_CellStates","cellStates");

function paintMap(cellStates, map)
    cla;  % Clear previous plot
    
    % Redraw the map without removing colored cells
    %show(map);
    hold on;
    
    % Manually redraw the grid and color the cells
    for x = 1:90
        for y = 1:90
            if cellStates(y, x) == 1
                % Occupied cell (obstacle)
                fill([x-1, x, x, x-1], [y-1, y-1, y, y], 'r', 'FaceAlpha', 0.3);
            elseif cellStates(y, x) == 0
                % Free cell
                fill([x-1, x, x, x-1], [y-1, y-1, y, y], 'g', 'FaceAlpha', 0.3);
            elseif cellStates(y, x) == 3
                % Frontier cell (purple)
                fill([x-1, x, x, x-1], [y-1, y-1, y, y], 'm', 'FaceAlpha', 0.3);
            elseif cellStates(y, x) == 5
                % Target cell
                fill([x-1, x, x, x-1], [y-1, y-1, y, y], 'b', 'FaceAlpha', 0.7);
            elseif cellStates(y, x) == 4
                % Probable area
                fill([x-1, x, x, x-1], [y-1, y-1, y, y], 'y', 'FaceAlpha', 0.3);
            end
        end
    end
    
    for i = 0:90
        plot([i i], [0 90], 'k'); % Draw vertical grid line
        plot([0 90], [i i], 'k'); % Draw horizontal grid line
    end
end