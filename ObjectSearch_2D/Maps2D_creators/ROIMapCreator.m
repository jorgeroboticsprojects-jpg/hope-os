imageFolder = 'Map_ROIs_Images';
imageFiles = dir(fullfile(imageFolder, '*.jpg'));

% Track seen map names and assign numbers
mapNames = {};
mapIndex = containers.Map;

startingPositions = [10 80;
                     70 25;
                     20 10;
                     20 10;
                     10 80];

objectPositions = [78 35;
                   40 51;
                   84 44;
                   19 65;
                   86  9];

for k = 1:length(imageFiles)
    filename = imageFiles(k).name;
    filepath = fullfile(imageFolder, filename);

    % Read and convert image
    image = im2double(imread(filepath));

    % Threshold for obstacle detection
    threshold = 0.95;
    binaryMap = all(image < threshold, 3);

    % Resize to 90x90
    gridMap = imresize(binaryMap, [90, 90]);

    % Create occupancy map
    occupancyMap = binaryOccupancyMap(90, 90, 1);
    setOccupancy(occupancyMap, gridMap);

    % Get map name without suffix
    [~, baseName, ~] = fileparts(filename);
    if endsWith(baseName, '_ROI')
        mapName = extractBefore(baseName, '_ROI');
        % Clean map name for variable: remove leading digits
        mapName = regexprep(mapName, '^\d+', '');
    elseif endsWith(baseName, '_Occ')
        mapName = extractBefore(baseName, '_Occ');
        % Clean map name for variable: remove leading digits
        mapName = regexprep(mapName, '^\d+', '');
    else
        continue;
    end

    % Assign number if this is the first time we see this map
    if ~isKey(mapIndex, mapName)
        mapNumber = numel(mapNames) + 1;
        mapIndex(mapName) = mapNumber;
        mapNames{end+1} = mapName;
    else
        mapNumber = mapIndex(mapName);
    end

    % Define new folder name with map number
    numberedFolderName = sprintf('map_%d_%s', mapNumber, mapName);
    saveFolder = fullfile('..', 'Maps_ROI_Object_Situations', numberedFolderName);

    % Create folder if it doesn't exist
    if ~exist(saveFolder, 'dir')
        mkdir(saveFolder);
    end

    % Process and save accordingly
    if endsWith(baseName, '_Occ')
        occMatrix = flipud(getOccupancy(occupancyMap));
        occMatrix=occMatrix';
        occMatrix = cat(3, occMatrix);  % Change to 3D

        saveName = fullfile(saveFolder, [mapName, 'OccMatrix.mat']);
        save(saveName, 'occMatrix');
        fprintf('Saved occupancy matrix: %s\n', saveName);

    elseif endsWith(baseName, '_ROI')
        cellStates = flipud(getOccupancy(occupancyMap));
        cellStates = cellStates * 2 + 2;

        % Set start and object location for this map
        startPos = startingPositions(mapNumber, :);
        objPos   = objectPositions(mapNumber, :);
        cellStates(startPos(2), startPos(1)) = 0;  % Staring position
        cellStates(objPos(2), objPos(1))     = 5;  % Object Location
        cellStates=cellStates';

        voxelStates = cat(3, cellStates); %Convert to 3D but still plane

        varName = [mapName, '_voxelStates_1'];
        s = struct();
        s.('voxelStates') = voxelStates;
        saveName = fullfile(saveFolder, [varName, '.mat']);
        save(saveName, '-struct', 's');
        fprintf('Saved voxel states: %s\n', saveName);
    end
    if ~mod(k,2) %only once per map
        % Define WorkSpace
        WorkSpace = ones([90 90 1]); % 1=> reachable workspace | 0=> not reachable
        varName = [mapName, 'WorkSpace'];
        s = struct(); 
        s.('WorkSpace') = WorkSpace;
        saveName = fullfile(saveFolder, [varName, '.mat']);
        save(saveName, '-struct', 's');
        fprintf('Saved WorkSpace: %s\n', saveName);
    end

end

