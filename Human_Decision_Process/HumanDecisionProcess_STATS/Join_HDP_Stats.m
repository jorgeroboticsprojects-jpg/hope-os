% This script loads all Human Decision Process (HDP) result .mat files from a folder (and subfolders),
% concatenates/merges the per-player STATS into a single combined structure (STATSjoined),
% and saves it as a single .mat file. The goal is to aggregate all competitors' statistics so they
% can be directly compared against the HOPE-OS algorithm results.

% Specify the folder where the .mat files are located
folderPath = './HDP_DATA_5_MAPS/'; % Change './' to your folder path if it is not in the same directory

% Search for all .mat files in the folder and subfolders
fileList = dir(fullfile(folderPath, '**', '*.mat'));

% Extract full file paths
fileNames = fullfile({fileList.folder}, {fileList.name});

% Initialize the combined statistics structure
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
disp('Saved joined human decision process stats as statsJoinedHDP_NEW.mat');