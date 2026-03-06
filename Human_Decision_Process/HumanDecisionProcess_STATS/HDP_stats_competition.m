% Specify the folder where the .mat files are located
folderPath = './HDP_DATA_5_MAPS/'; % Change './' to your folder path if it is not in the same directory
% Search for all .mat files in the folder and subfolders
fileList = dir(fullfile(folderPath, '**', '*.mat'));
% Extract full file paths
fileNames = fullfile({fileList.folder}, {fileList.name});
disp('Archivos encontrados:');
% Initialize an array to store player names
playerNames = cell(1, length(fileNames));
% Iterate through each file and extract the player name
for idx = 1:length(fileNames)
    % Get the filename without path or extension
    [~, name, ~] = fileparts(fileNames{idx});
    
    % Extract the part after "HumanDecisionProcess_"
    splitName = split(name, '_');
    if length(splitName) > 1
        playerNames{idx} = splitName{2}; % Player name
    else
        playerNames{idx} = 'Unknown'; % If it does not match the expected format
    end
end

analyzePlayerPerformance(fileNames, playerNames)
visualizeAverageScores(fileNames);
%optimizedDataProcessing(fileNames,playerNames);

function visualizeAverageScores(fileNames)
    % Initialize containers for metrics
    allDistancesPerMap = {};
    allAngularDistancesPerMap = {};
    allIterationsPerMap = {};

    % Process each file
    for idx = 1:length(fileNames)
        % Load data
        load(fileNames{idx}, 'STATS');

        % Call DataOrganization to extract metrics per map
        [~, ~, ~, ~, ~, ~, ~, FinalDistTravelledVec, FinalAngDistVec, ~, ~, ~, TotalIterationsVec] = DataOrganization(STATS);

        % Store metrics per map
        allDistancesPerMap{idx} = FinalDistTravelledVec;
        allAngularDistancesPerMap{idx} = FinalAngDistVec;
        allIterationsPerMap{idx} = TotalIterationsVec;
    end

    % Combine data from all players by map
    numMaps = 5; % Assume there are 5 maps
    combinedDistances = nan(length(fileNames), numMaps);
    combinedAngularDistances = nan(length(fileNames), numMaps);
    combinedIterations = nan(length(fileNames), numMaps);

    for idx = 1:length(fileNames)
        % Extract data for each player and populate the combined matrix
        combinedDistances(idx, 1:length(allDistancesPerMap{idx})) = allDistancesPerMap{idx};
        combinedAngularDistances(idx, 1:length(allAngularDistancesPerMap{idx})) = allAngularDistancesPerMap{idx};
        combinedIterations(idx, 1:length(allIterationsPerMap{idx})) = allIterationsPerMap{idx};
    end

    % Calculate the global average per map (ignoring NaN)
    avgDistances = nanmean(combinedDistances, 1);
    avgAngularDistances = nanmean(combinedAngularDistances, 1);
    avgIterations = nanmean(combinedIterations, 1);

    % Visualization
    figure;
    subplot(3, 1, 1);
    plot(1:numMaps, avgDistances, '-o');
    xlabel('Map Index');
    ylabel('Traveled Distance');
    title('Average Traveled Distance per Map');

    subplot(3, 1, 2);
    plot(1:numMaps, avgAngularDistances, '-o');
    xlabel('Map Index');
    ylabel('Angular Distance');
    title('Average Angular Distance per Map');

    subplot(3, 1, 3);
    plot(1:numMaps, avgIterations, '-o');
    xlabel('Map Index');
    ylabel('Total Iterations');
    title('Average Total Iterations per Map');
end

function paddedData = padDataForAveraging(dataCellArray)
    % Find the maximum length across players
    maxMaps = max(cellfun(@length, dataCellArray));

    % Pad with NaN to match lengths
    paddedData = cellfun(@(x) [x(:)', nan(1, maxMaps - length(x))], dataCellArray, 'UniformOutput', false);

    % Convert from cell array to matrix
    paddedData = cell2mat(paddedData);
end

function analyzePlayerPerformance(fileNames, playerNames)
    % Initialize containers for metrics
    FinalDistTravelledVecs = {};
    FinalAngDistVecs = {};
    TotalIterations = {};
    
    % Process each file to extract metrics
    for idx = 1:length(fileNames)
        % Load data
        load(fileNames{idx}, 'STATS');
        
        % Call DataOrganization to extract data
        [~, ~, ~, ~, ~, ~, ~, FinalDistTravelledVec, FinalAngDistVec, ~, ~, ~, TotalIterationsVec] = DataOrganization(STATS);
        
        % Store results
        FinalDistTravelledVecs{idx} = FinalDistTravelledVec;
        FinalAngDistVecs{idx} = FinalAngDistVec;
        TotalIterations{idx} = TotalIterationsVec;
    end

    % Calculate the average of each metric for each player
    avgDistances = cellfun(@mean, FinalDistTravelledVecs);
    avgAngularDistances = cellfun(@mean, FinalAngDistVecs);
    avgIterations = cellfun(@mean, TotalIterations);

    % Identify the best player in each metric
    [~, bestDistanceIdx] = min(avgDistances);
    [~, bestAngularDistanceIdx] = min(avgAngularDistances);
    [~, bestIterationIdx] = min(avgIterations);

    % Create a combined ranking based on the sum of normalized metrics
    normalizedDistances = normalize(avgDistances, 'range');
    normalizedAngularDistances = normalize(avgAngularDistances, 'range');
    normalizedIterations = normalize(avgIterations, 'range');
    combinedScores = (normalizedDistances + normalizedAngularDistances + normalizedIterations) / 3;

    % Sort players by combined ranking
    [~, sortedIndices] = sort(combinedScores);
    sortedPlayerNames = playerNames(sortedIndices);

    % Display results
    fprintf('=== Average Metrics ===\n');
    for idx = 1:length(playerNames)
        fprintf('Player: %s\n', playerNames{idx});
        fprintf('  Average Distance: %.2f\n', avgDistances(idx));
        fprintf('  Average Angular Distance: %.2f\n', avgAngularDistances(idx));
        fprintf('  Average Iterations: %.2f\n\n', avgIterations(idx));
    end

    fprintf('=== Best Players ===\n');
    fprintf('Best in Distance Traveled: %s (%.2f)\n', playerNames{bestDistanceIdx}, avgDistances(bestDistanceIdx));
    fprintf('Best in Angular Distance: %s (%.2f)\n', playerNames{bestAngularDistanceIdx}, avgAngularDistances(bestAngularDistanceIdx));
    fprintf('Best in Iterations: %s (%.2f)\n', playerNames{bestIterationIdx}, avgIterations(bestIterationIdx));

    fprintf('\n=== Overall Ranking ===\n');
    for idx = 1:length(sortedPlayerNames)
        fprintf('%d. %s (Score: %.2f)\n', idx, sortedPlayerNames{idx}, 10 * (1 - combinedScores(sortedIndices(idx))));
    end
end

function optimizedDataProcessing(fileNames,playerNames)
    % fileNames: Cell array containing the .mat filenames to be loaded and processed
    
    % Pre-allocate containers to store results for each file
    countVectors = {};
    countVectorsTotal = {};
    avgDistTravelledIters = {};
    avgDistToObjects = {};
    avgAngDistIters = {};
    avgMapExps = {};
    avgROIExps = {};
    FinalDistTravelledVecs = {};
    FinalAngDistVecs = {};
    FinalMapExpVecs = {};
    FinalROIExpVecs = {};
    IterationsPos = {};
    TotalIterations = {};
    
    % Process each file in the provided list
    for idx = 1:length(fileNames)
        % Load the file
        load(fileNames{idx}, 'STATS');
        
        % Call DataOrganization to extract data
        [countVector, countVectorTotal, avgDistTravelledIter, avgDistToObject, avgAngDistIter, avgMapExp, avgROIExp, FinalDistTravelledVec, FinalAngDistVec, FinalMapExpVec, FinalROIExpVec, IterationsPosVec, TotalIterationsVec] = DataOrganization(STATS);
        
        % Store the results for this file
        countVectors{idx} = countVector;
        countVectorsTotal{idx} = countVectorTotal;
        avgDistTravelledIters{idx} = avgDistTravelledIter;
        avgDistToObjects{idx} = avgDistToObject;
        avgAngDistIters{idx} = avgAngDistIter;
        avgMapExps{idx} = avgMapExp;
        avgROIExps{idx} = avgROIExp;
        FinalDistTravelledVecs{idx} = FinalDistTravelledVec;
        FinalAngDistVecs{idx} = FinalAngDistVec;
        FinalMapExpVecs{idx} = FinalMapExpVec;
        FinalROIExpVecs{idx} = FinalROIExpVec;
        IterationsPos{idx} = IterationsPosVec;
        TotalIterations{idx} = TotalIterationsVec;
    end

    % Visualization
    visualizeResults(countVectors, countVectorsTotal, avgDistTravelledIters, avgDistToObjects, avgAngDistIters, avgMapExps, avgROIExps, FinalDistTravelledVecs, FinalAngDistVecs, FinalMapExpVecs, FinalROIExpVecs, IterationsPos, TotalIterations, playerNames);
end

function visualizeResults(countVectors, countVectorsTotal, avgDistTravelledIters, avgDistToObjects, avgAngDistIters, avgMapExps, avgROIExps, FinalDistTravelledVecs, FinalAngDistVecs, FinalMapExpVecs, FinalROIExpVecs, IterationsPos, TotalIterations, fileNames)
    % Predefined colors for plots
    baseColors = {'b', 'r', 'g', 'm', 'c', 'y', 'k'};
    numFiles = length(fileNames);
    
    % Assign colors dynamically if there are more files than predefined colors
    if numFiles > length(baseColors)
        additionalColors = lines(numFiles - length(baseColors)); % Generate additional colors
        colors = [baseColors, mat2cell(additionalColors, ones(size(additionalColors, 1), 1), 3)];
    else
        colors = baseColors;
    end

    % Number of data points per iterations (positions only)
    figure;
    subplot(2, 1, 1);
    hold on;
    for idx = 1:numFiles
        numIter = 1:length(countVectors{idx});
        plot(numIter, countVectors{idx}, 'LineWidth', 2, 'Color', colors{idx});
    end
    legend(fileNames, 'Interpreter', 'none');
    xlabel('Iterations (only positions)');
    ylabel('Number of maps');
    title('Number of iterations (only positions) needed to find the object per map');
    grid on;

    % Number of data points per total iterations
    subplot(2, 1, 2);
    hold on;
    for idx = 1:numFiles
        numIter = 1:length(countVectorsTotal{idx});
        plot(numIter, countVectorsTotal{idx}, 'LineWidth', 2, 'Color', colors{idx});
    end
    legend(fileNames, 'Interpreter', 'none');
    xlabel('Total Iterations');
    ylabel('Number of maps');
    title('Number of iterations needed to find the object per map');
    grid on;

    % Accumulated plots
    figure;
    subplot(2, 3, 1);
    hold on;
    for idx = 1:numFiles
        numIter = 1:length(avgDistTravelledIters{idx});
        plot(numIter, avgDistTravelledIters{idx}, 'LineWidth', 2, 'Color', colors{idx});
    end
    legend(fileNames, 'Interpreter', 'none');
    xlabel('Iterations (only positions)');
    ylabel('Distance');
    title('Accumulated Traveled Distance per Iteration');
    grid on;

    subplot(2, 3, 2);
    hold on;
    for idx = 1:numFiles
        numIter = 1:length(avgDistToObjects{idx});
        plot(numIter, avgDistToObjects{idx}, 'LineWidth', 2, 'Color', colors{idx});
    end
    xlabel('Iterations (only positions)');
    ylabel('Distance');
    title('Distance from the Agent to the Object per Iteration');
    grid on;

    subplot(2, 3, 3);
    hold on;
    for idx = 1:numFiles
        numIter = 1:length(avgMapExps{idx});
        plot(numIter, avgMapExps{idx}, 'LineWidth', 2, 'Color', colors{idx});
    end
    xlabel('Total Iterations');
    ylabel('Proportion');
    title('Proportion of Explored Map Area per Iteration');
    grid on;

    subplot(2, 3, 4);
    hold on;
    for idx = 1:numFiles
        numIter = 1:length(avgROIExps{idx});
        plot(numIter, avgROIExps{idx}, 'LineWidth', 2, 'Color', colors{idx});
    end
    xlabel('Total Iterations');
    ylabel('Proportion');
    title('Proportion of Explored ROI Area per Iteration');
    grid on;

    subplot(2, 3, 5);
    hold on;
    for idx = 1:numFiles
        numIter = 1:length(avgAngDistIters{idx});
        plot(numIter, avgAngDistIters{idx}, 'LineWidth', 2, 'Color', colors{idx});
    end
    xlabel('Total Iterations');
    ylabel('Angular distance (radians)');
    title('Accumulated Traveled Angular Distance per Iteration');
    grid on;

    % Boxplots for final metrics
    figure;
    subplot(2, 3, 1);
    boxplot(padDataForBoxplot(FinalDistTravelledVecs), 'Labels', arrayfun(@num2str, 1:numFiles, 'UniformOutput', false));
    ylabel('Distance');
    title('Total Traveled Distance per Map');

    subplot(2, 3, 2);
    boxplot(padDataForBoxplot(FinalAngDistVecs), 'Labels', arrayfun(@num2str, 1:numFiles, 'UniformOutput', false));
    ylabel('Angular Distance (radians)');
    title('Total Traveled Angular Distance per Map');

    subplot(2, 3, 3);
    boxplot(padDataForBoxplot(FinalMapExpVecs), 'Labels', arrayfun(@num2str, 1:numFiles, 'UniformOutput', false));
    ylabel('Proportion of Explored Map Area');
    title('Total Explored Area per Map');

    subplot(2, 3, 6);
    boxplot(padDataForBoxplot(FinalROIExpVecs), 'Labels', arrayfun(@num2str, 1:numFiles, 'UniformOutput', false));
    ylabel('Proportion of Explored ROI Area');
    title('Total Explored ROI Area per Map');

    subplot(2, 3, 4);
    boxplot(padDataForBoxplot(IterationsPos), 'Labels', arrayfun(@num2str, 1:numFiles, 'UniformOutput', false));
    ylabel('Iterations');
    title('Iterations per Map (positions only)');

    subplot(2, 3, 5);
    boxplot(padDataForBoxplot(TotalIterations), 'Labels', arrayfun(@num2str, 1:numFiles, 'UniformOutput', false));
    ylabel('Iterations');
    title('Total Iterations per Map');
end

function paddedData = padDataForBoxplot(dataCellArray)
    % Find the maximum length of the data
    maxLength = max(cellfun(@length, dataCellArray));
    
    % Pad each group with NaN to match lengths
    paddedData = cellfun(@(x) [x(:); nan(maxLength - length(x), 1)], dataCellArray, 'UniformOutput', false);
    
    % Convert from cell array to matrix
    paddedData = cell2mat(paddedData);
end

% Function that allows summing vectors of different sizes
function result = sumVectors(v1, v2)
    % Find the maximum length between the two vectors
    max_len = max(length(v1), length(v2));
    
    % Extend vectors with zeros to match lengths
    v1_ext = [v1, zeros(1, max_len - length(v1))];
    v2_ext = [v2, zeros(1, max_len - length(v2))];
    
    % Sum vectors element-wise
    result = v1_ext + v2_ext;
end

function [countVector, countVectorTotal, avgDistTravelledIter, avgDistToObject, avgAngDistIter, avgMapExp, avgROIExp, FinalDistTravelledVec, FinalAngDistVec, FinalMapExpVec, FinalROIExpVec, IterationsPos, TotalIterations] = DataOrganization(STATS)


    mapNUm = cell2mat(STATS.mapNum);    
    % Default values of variables
    SumPosIter = 0;
    SumTotalIter = 0;
    distTraVector1 = 0; distTraVector2 = 0;
    distToObjVector1 = 0; distToObjVector2 = 0; 
    distAngVector1 = 0; distAngVector2 = 0; distAngVectorDeg = 0;
    exploredMapVec1 = 0; exploredMapVec2 = 0;
    exploredROIVec1 = 0; exploredROIVec2 = 0;
    maxIter=0; maxIterOrient=0;
    SumDistTravelled=0;
    SumAngDist=0;
    SumMapExp=0;
    SumROIExp=0;
    vectorDistTrav=[];
    
    FinalDistTravelledVec=[]; FinalAngDistVec=[]; FinalMapExpVec=[]; FinalROIExpVec=[];
    
    IterationsPos = cell2mat(STATS.positionIterations);
    TotalIterations = cell2mat(STATS.totalIterations);
    
    %%% Success Rate %%%
    successRateVec = cell2mat(STATS.objectFound);
    [indx,~]=find(successRateVec==0);
    successRate = sum(successRateVec)/mapNUm;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % In this section we obtain the average position iterations, average
    % total iterations, average total distance travelled and average angular 
    % total distance travelled.
    
    for i=1:mapNUm
        %Sums the number of iterations (only considers positions)
        SumPosIter = SumPosIter + STATS.positionIterations{i,1};
        %Gets de mas number of iterations (only considers positions)
        if maxIter < STATS.positionIterations{i,1}
            maxIter = STATS.positionIterations{i,1};
        end
    
        %Sums the number of iterations (considers positions and orientations)
        SumTotalIter = SumTotalIter + STATS.totalIterations{i,1};
        %Gets de mas number of iterations (considers positions and orientations)
        if maxIterOrient < STATS.totalIterations{i,1}
            maxIterOrient = STATS.totalIterations{i,1};
        end
        
        % sizeVectorDistTrav ES IGUAL EN TODAS, HAY Q OPTIMIZaerlo
        %Obtain the total distance travelled in each map
        vectorDistTrav = STATS.distanceTravelledAccum{i};
        sizeVectorDistTrav = STATS.positionIterations{i,1};
        SumDistTravelled = SumDistTravelled + vectorDistTrav(sizeVectorDistTrav);
        FinalDistTravelledVec = [FinalDistTravelledVec,  vectorDistTrav(sizeVectorDistTrav)];
    
        %Obtain the total angular distance travelled in each map
        vectorAngDist = STATS.angularDist{i};
        sizeVectorAngDist = STATS.totalIterations{i,1};
        SumAngDist = SumAngDist + vectorAngDist(sizeVectorAngDist);
        FinalAngDistVec = [FinalAngDistVec, vectorAngDist(sizeVectorAngDist)];
    
        %Obtain the total map exploration in each map
        vectorMapExp = STATS.exploredMap{i};
        sizeVectorMapExp = STATS.totalIterations{i,1};
        SumMapExp = SumMapExp + vectorMapExp(sizeVectorMapExp);
        FinalMapExpVec = [FinalMapExpVec, vectorMapExp(sizeVectorMapExp)];
    
        %Obtain the total ROI ecploration in each map
        vectorROIExp = STATS.exploreROI{i};
        sizeVectorROIExp = STATS.totalIterations{i,1};
        SumROIExp = SumROIExp + vectorROIExp(sizeVectorROIExp);
        FinalROIExpVec = [FinalROIExpVec, vectorROIExp(sizeVectorROIExp)];
    
    
        %%% Obtain data per iteratio and save it in vectors to calculate STD
        % per iteration and boc graph.
    end
    
    averagePosIter = SumPosIter/mapNUm;
    averageTotalIter = SumTotalIter/mapNUm;
    averageTotalDistTravelled = SumDistTravelled/mapNUm;
    averageTotalAngDist = SumAngDist/mapNUm;
    averageTotalAngDistDeg = rad2deg(averageTotalAngDist);
    averageTotalMapExp = SumMapExp/mapNUm;
    averageTotalROIExp = SumROIExp/mapNUm;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % In this section we obtain the average distance travelled in each
    % iteration.
    
    % Count number of data in each iteration to calculate de average
    countVector=zeros(maxIter, 1); 
    countVectorTotal=zeros(maxIterOrient, 1); 
    %countVectorCheck=zeros(maxIter, 1); 
    %countVectorCheckTotal=zeros(maxIterOrient, 1);
    
    avgDistTravelledIter =zeros(maxIter, 1); 
    avgDistToObject =zeros(maxIter, 1);
    avgAngDistIter=zeros(maxIterOrient, 1);
    avgAngDistIterDeg=zeros(maxIterOrient, 1);
    avgMapExp=zeros(maxIterOrient, 1);
    avgROIExp=zeros(maxIterOrient, 1);
    
    %We need to calculate this numbers "countVector" in order to then divide
    %each position of the summed distance travelled vector stored in distTraVector1
    %at the end of the this "for" fucntion, and obtain the correct average
    %for each iteration
    for i=1:mapNUm
        for j=1:maxIter
            if (STATS.positionIterations{i,1})== j
                %countVectorCheck(j)= countVectorCheck(j) + 1; %gets de number vector of size 1, size 2,... size(maxIter)
                for k=1:j
                    countVector(k)= countVector(k) + 1; %gets the number of 1 iterations done, 2 iterations done, maxIter iterations done
                end
            end
        end
    
        %Sums accumulated distance vectors, final result stored in "distTraVector1" 
        distTraVector2 = STATS.distanceTravelledAccum{i,1};
        sumTravDist = sumVectors (distTraVector1,distTraVector2);
        distTraVector1 = sumTravDist;
    
        %Sums distance to object vectors, final result stored in "distToObjVector1" 
        distToObjVector2 = STATS.distanceToObjective{i,1};
        sumDistToObj = sumVectors (distToObjVector1,distToObjVector2);
        distToObjVector1 = sumDistToObj;
    
        %we obtain how many 1 iterations are, 3iterations... maxiterorient
        %iterations so we can calculate average later
        for j=1:maxIterOrient
            if (STATS.totalIterations{i,1})== j
                %countVectorCheckTotal(j)= countVectorCheckTotal(j) + 1; %gets de number vector of size 1, size 2,... size(maxIter)
                for k=1:j
                    countVectorTotal(k)= countVectorTotal(k) + 1; %gets the number of 1 iterations done, 2 iterations done, maxIter iterations done
                end
            end
        end
    
        %Sums accumulated angular distance vectors, final result stored in "distAngVector1" 
        distAngVector2 = STATS.angularDist{i,1};
        sumAngDist = sumVectors (distAngVector1,distAngVector2);
        distAngVector1 = sumAngDist;
        distAngVectorDeg = rad2deg(distAngVector1);
    
        %Sums accumulated explored map, final result stored in "exploredMap" 
        exploredMapVec2 = STATS.exploredMap{i,1};
        sumExpMap = sumVectors (exploredMapVec1,exploredMapVec2);
        exploredMapVec1 = sumExpMap;
    
        %Sums accumulated explored ROI, final result stored in "exploredMap" 
        exploredROIVec2 = STATS.exploreROI{i,1};
        sumExpROI = sumVectors (exploredROIVec1,exploredROIVec2);
        exploredROIVec1 = sumExpROI;
    end
    
    % we calculate average accum. distance travelled per ietration and dist to object
    % per iteration
    for i=1:length(countVector)
        avgDistTravelledIter(i) = distTraVector1(i) / countVector(i);
        avgDistToObject(i) =  distToObjVector1(i) / countVector(i);
    end
    
    % we calculate average accum. angular distance travelled per ietration
    for i=1:length(distAngVector1)
        avgAngDistIter(i) = distAngVector1(i) / countVectorTotal(i);
        avgAngDistIterDeg(i) = distAngVectorDeg(i) / countVectorTotal(i); 
    
        % we calculate average accum. explored map per ietration
        avgMapExp(i) = exploredMapVec1(i) / countVectorTotal(i);
    
        % we calculate average accum. explored ROI per ietration
        avgROIExp(i) = exploredROIVec1(i) / countVectorTotal(i);
    end
end

