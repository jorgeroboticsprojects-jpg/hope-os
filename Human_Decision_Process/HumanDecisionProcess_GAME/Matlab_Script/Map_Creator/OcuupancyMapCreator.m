image = imread('LHC_cross_section.jpg');

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
show(occupancyMap);
title('Occupancy map generated from image');

hold on;
for i = 0:90
    plot([i i], [0 90], 'k'); % Draw vertical grid line
    plot([0 90], [i i], 'k'); % Draw horizontal grid line
end

% Save the generated occupancy map
save("LCH_map_cross_section","occupancyMap");