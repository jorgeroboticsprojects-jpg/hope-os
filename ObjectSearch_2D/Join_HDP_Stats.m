
folderPath = './HDP_DATA_5_MAPS/';

% Search all files .mat
fileList = dir(fullfile(folderPath, '**', '*.mat'));
% Obtain paths
fileNames = fullfile({fileList.folder}, {fileList.name});

%Initialize the combined statistics structure
STATSjoined.HDP.situation1.posOrientations_done = {};
STATSjoined.HDP.situation1.exploredMap = {};
STATSjoined.HDP.situation1.exploreROI = {};
STATSjoined.HDP.situation1.angularDist = {};
STATSjoined.HDP.situation1.objectFound = {};

% Process each file in the provided list
for idx = 1:length(fileNames)
    % Load the file
    load(fileNames{idx}, 'STATS');

    % Append the data from the current struct to STATSjoined
    STATSjoined.HDP.situation1.posOrientations_done = [STATSjoined.HDP.situation1.posOrientations_done; STATS.posOrientations_done];
    STATSjoined.HDP.situation1.exploredMap = [STATSjoined.HDP.situation1.exploredMap; STATS.exploredMap];
    STATSjoined.HDP.situation1.exploreROI = [STATSjoined.HDP.situation1.exploreROI; STATS.exploreROI];
    STATSjoined.HDP.situation1.angularDist = [STATSjoined.HDP.situation1.angularDist; STATS.angularDist];
    STATSjoined.HDP.situation1.objectFound = [STATSjoined.HDP.situation1.objectFound; STATS.objectFound];
end

% Rename STATSjoined to STATS before saving
STATS = STATSjoined; 
save('Test_results_data/statsJoinedHDP_NEW.mat', 'STATS');
disp('Saved joined stats od human decision process as statsJoinedHDP_NEW.mat ')