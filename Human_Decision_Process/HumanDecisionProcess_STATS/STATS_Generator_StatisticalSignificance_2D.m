% Plot the statistics results and the statistical significance 

% Analyze and visualize the data
fileNames = {'Test_results_data/statsJoinedHDP_NEW.mat', 'Test_results_data/HDP_2D_cleanedcode_b06_t08.mat'}; % Updated with joined file
legendNames = {'HDP','Algorithm'};
analyze3DExploration(fileNames, legendNames);


%% Main Analysis and Visualization Functions

function analyze3DExploration(fileNames, playerNames)
    % Pre-allocate containers to store results for each file
    avgMapExps = {};
    avgROIExps = {};
    avgDistances = {};
    avgAngularDistances = {};
    finalDistances = {};
    finalAngDistances = {};
    posIterations = {};
    allIterations = {};
    FinalMapExpVecs = {};
    FinalROIExpVecs = {};

    % Process each file in the provided list
    for idx = 1:length(fileNames)
        % Load the file
        load(fileNames{idx}, 'STATS');
        
        % Call DataOrganization to extract data
        [avgMapExp, avgROIExp, avgDist, avgAngDist, FinalMapExpVec, FinalROIExpVec,FinalDist,FinalAngDist,positionIterations,totalIterations] = DataOrganization3D(STATS);
        
        % Store the results for this file
        avgMapExps{idx} = avgMapExp;
        avgROIExps{idx} = avgROIExp;
        avgDistances{idx} = avgDist;
        avgAngularDistances{idx} = avgAngDist;
        finalDistances{idx} = FinalDist;
        finalAngDistances{idx} = FinalAngDist;
        posIterations{idx} = positionIterations;
        allIterations{idx} = totalIterations;
        FinalMapExpVecs{idx} = FinalMapExpVec;
        FinalROIExpVecs{idx} = FinalROIExpVec;
    end

    % Visualization
    visualize3DResults(avgMapExps, avgROIExps, avgDistances, avgAngularDistances,finalDistances, finalAngDistances, posIterations,allIterations, FinalMapExpVecs, FinalROIExpVecs, playerNames);
    
    % Convergence diagnostics for final metrics
    % plotConvergenceAnalysis(FinalMapExpVecs, 'Total Explored Area per Map');
    % plotConvergenceAnalysis(FinalROIExpVecs, 'Total Explored ROI Area per Map');
    % plotConvergenceAnalysis(finalDistances, 'Total Travelled Distance per Map');
    % plotConvergenceAnalysis(finalAngDistances, 'Total Angular Distance per Map');
    % plotConvergenceAnalysis(posIterations, 'Total Position Iterations per Map');
    % plotConvergenceAnalysis(allIterations, 'Total Iterations per Map');
end

%% Visualization Function

function visualize3DResults(avgMapExps, avgROIExps, avgDistances, avgAngularDistances, finalDistances, finalAngDistances, positionIterations,allIterations, FinalMapExpVecs, FinalROIExpVecs, playerNames)
    numFiles = length(playerNames);
    baseColors = lines(numFiles); % Dynamic color generation
    
    % Plot for exploration statistics
    figure;
    subplot(2, 2, 1);
    hold on;
    for idx = 1:numFiles
        plot(avgMapExps{idx}, 'LineWidth', 2, 'Color', baseColors(idx, :));
    end
    title('Average Map Exploration Per Iteration');
    xlabel('Iterations'); ylabel('Proportion Explored');
    legend(playerNames, 'Location', 'Best');

    subplot(2, 2, 2);
    hold on;
    for idx = 1:numFiles
        plot(avgROIExps{idx}, 'LineWidth', 2, 'Color', baseColors(idx, :));
    end
    title('Average ROI Exploration Per Iteration');
    xlabel('Iterations'); ylabel('Proportion ROI Explored');
    legend(playerNames, 'Location', 'Best');

    subplot(2, 2, 3);
    hold on;
    for idx = 1:numFiles
        plot(avgDistances{idx}, 'LineWidth', 2, 'Color', baseColors(idx, :));
    end
    title('Cumulative Distance Traveled Per Iteration');
    xlabel('Iterations'); ylabel('Distance Traveled [meters]');
    legend(playerNames, 'Location', 'Best');

    subplot(2, 2, 4);
    hold on;
    for idx = 1:numFiles
        plot(avgAngularDistances{idx}, 'LineWidth', 2, 'Color', baseColors(idx, :));
    end
    title('Cumulative Angular Distance Traveled Per Iteration');
    xlabel('Iterations'); ylabel('Angular Distance [radians]');
    legend(playerNames, 'Location', 'Best');

    % % Boxplots for final metrics
    % figure;
    % subplot(1, 6, 1);
    % boxplot(padDataForBoxplot(FinalMapExpVecs));
    % title('Total Explored Area per Map');
    % ylabel('Proportion');
    % 
    % subplot(1, 6, 2);
    % boxplot(padDataForBoxplot(FinalROIExpVecs));
    % title('Total Explored ROI Area per Map');
    % ylabel('Proportion');
    % 
    % subplot(1, 6, 3);
    % boxplot(padDataForBoxplot(finalDistances));
    % title('Total Travelled Distance per Map');
    % ylabel('Distance [meters]');
    % 
    % subplot(1, 6, 4);
    % boxplot(padDataForBoxplot(finalAngDistances));
    % title('ang Distances');
    % ylabel('Distance [radians]');
    % 
    % subplot(1, 6, 5);
    % boxplot(padDataForBoxplot(positionIterations));
    % title('Total Position Iterations per Map');
    % ylabel('iterations');
    % 
    % subplot(1, 6, 6);
    % boxplot(padDataForBoxplot(allIterations));
    % title('Total Iterations per Map');
    % ylabel('iterations');
    % %Crear un layout de 1 fila y 6 columnas
figure;   
t = tiledlayout(1, 6, 'Padding', 'compact', 'TileSpacing', 'compact');

% 1. Total Explored Area per Map
nexttile;
boxplot(padDataForBoxplot(FinalMapExpVecs));
title('Map Coverage');
ylabel('Proportion');
ylim([0 1]);

% 2. Total Explored ROI Area per Map
nexttile;
boxplot(padDataForBoxplot(FinalROIExpVecs));
title('ROI Coverage');
ylabel('Proportion');
ylim([0 1]);

% 3. Total Travelled Distance per Map
nexttile;
boxplot(padDataForBoxplot(finalDistances));
title('Total Distance Travelled');
ylabel('Distance [meters]');

% 4. Angular Distances
nexttile;
boxplot(padDataForBoxplot(finalAngDistances));
title('Total Angular Distance');
ylabel('Distance [radians]');

% 5. Total Position Iterations per Map
nexttile;
boxplot(padDataForBoxplot(positionIterations));
title('Total Position Iterations');
ylabel('iterations');

% 6. Total Iterations per Map
nexttile;
boxplot(padDataForBoxplot(allIterations));
title('Total Iterations');
ylabel('iterations');

end

%% Data Organization Function for 3D Exploration

function [avgMapExp, avgROIExp, avgDist, avgAngDist, FinalMapExpVec, FinalROIExpVec,FinalDist,FinalAngDist, positionIterations,totalesIterations] = DataOrganization3D(STATS)
    
    mapNames = fieldnames(STATS); % Extract map names

    % situations = fieldnames(STATS.LHC); % {'situation1', 'situation2', ..., 'situationN'}
    % numSituations = numel(situations);

    % Initialize general counters
    FinalMapExpVec = [];
    FinalROIExpVec = [];
    FinalDist = [];
    FinalAngDist = [];
    positionIterations = [];
    totalesIterations = [];

    % determine max number of iterations for prealloc
    maxIter = 0;
    for m = 1:numel(mapNames)
        map = STATS.(mapNames{m});
        situations = fieldnames(map);
        numSituations = numel(situations);

        for s = 1:numSituations
            sit = map.(situations{s});
            for j = 1:length(sit.exploredMap)
                maxIter = max(maxIter, length(sit.exploredMap{j}));
            end
        end
    end
    
    % Initialize accumulative counters
    avgMapExp = zeros(1, maxIter);
    avgROIExp = zeros(1, maxIter);
    avgDist = zeros(1, maxIter);
    avgAngDist = zeros(1, maxIter);
    countIterations = zeros(1, maxIter);
    
    for m = 1:numel(mapNames)
        map = STATS.(mapNames{m});
        situations = fieldnames(map);
        numSituations = numel(situations);

        for s = 1:numSituations
            sit = map.(situations{s});
            numRuns = length(sit.exploredMap);
        
            for i = 1:numRuns
                mapExploration = sit.exploredMap{i};
                ROIExploration = sit.exploreROI{i};
                positions = sit.posOrientations_done{i};
                positions = positions * 0.05;
        
                % Distancias
                cumulativeDist = computeCumulativeDistance(positions);
                if size(positions,2)==3
                    % Angular distance (from precomputed data)
                    cumulativeAngDist = sit.angularDist{i};
                    %cumulativeAngDist = cumsum([0, angularDist]);
                elseif size(positions,2)==7
                    cumulativeAngDist = computeCumulativeAngularDistance(positions);
                else
                    error('Unexpected format in positions: expected 3 or 7 columns.');
                end
        
                FinalDist(end+1) = cumulativeDist(end);
                FinalAngDist(end+1) = cumulativeAngDist(end);
        
                % Iteraciones únicas
                uniquePositions = unique(positions(:, 1:3), 'rows', 'stable');
                positionIterations(end+1) = size(uniquePositions, 1);
                totalesIterations(end+1) = size(positions, 1);
        
                FinalMapExpVec(end+1) = mapExploration(end);
                FinalROIExpVec(end+1) = ROIExploration(end);
        
                for iter = 1:length(mapExploration)
                    avgMapExp(iter) = avgMapExp(iter) + mapExploration(iter);
                    avgROIExp(iter) = avgROIExp(iter) + ROIExploration(iter);
                    avgDist(iter) = avgDist(iter) + cumulativeDist(iter);
                    avgAngDist(iter) = avgAngDist(iter) + cumulativeAngDist(iter);
                    countIterations(iter) = countIterations(iter) + 1;
                end
            end
        end
    end

    avgMapExp = avgMapExp ./ countIterations;
    avgROIExp = avgROIExp ./ countIterations;
    avgDist = avgDist ./ countIterations;
    avgAngDist = avgAngDist ./ countIterations;

end

%% Helper Function to Compute Cumulative Distance
function cumulativeDist = computeCumulativeDistance(positions)
    cumulativeDist = zeros(1, size(positions, 1));
    for i = 2:size(positions, 1)
        pos1 = positions(i-1, 1:3);
        pos2 = positions(i, 1:3);
        dist = norm(pos2 - pos1);
        cumulativeDist(i) = cumulativeDist(i-1) + dist;
    end
end

%% Helper Function to Compute Cumulative Angular Distance
function cumulativeAngDist = computeCumulativeAngularDistance(positions)
    numSteps = size(positions, 1);
    cumulativeAngDist = zeros(1, numSteps);

    for i = 2:numSteps
        q1 = positions(i-1, 4:7);
        q2 = positions(i, 4:7);
        deltaQ = quatmultiply(q2, quatconj(q1));
        deltaQ = deltaQ / norm(deltaQ);
        angle = 2 * acos(min(1, max(-1, deltaQ(1))));

        cumulativeAngDist(i) = cumulativeAngDist(i-1) + angle;
    end
end


%%
function plotConvergenceAnalysis(metricCellArray, metricName)
    % plotConvergenceAnalysis - Plots statistical convergence of a metric
    %
    % Inputs:
    %   metricCellArray - cell array of vectors (e.g. FinalMapExpVecs, etc.)
    %   metricName      - string with the name of the metric to display on plots

    % Combine data into a single vector
    data = padDataForBoxplot(metricCellArray);  
    n = size(data, 1);

    meanVec = zeros(n,1);
    medianVec = zeros(n,1);
    stdVec = zeros(n,1);
    q1Vec = zeros(n,1);
    q3Vec = zeros(n,1);
    
    % Compute statistics incrementally as more simulations are included
    for i = 2:n
        subset = data(1:i, :);
        subset = subset(~isnan(subset)); % Eliminate NaNs

        meanVec(i) = mean(subset);
        medianVec(i) = median(subset);
        stdVec(i) = std(subset);
        q1Vec(i) = prctile(subset, 25);
        q3Vec(i) = prctile(subset, 75);
    end

    % Plot the convergence of statistics
    figure; hold on; grid on;
    x = 1:n;
    plot(x, meanVec, 'b-', 'LineWidth', 2);
    plot(x, medianVec, 'r--', 'LineWidth', 2);
    plot(x, q1Vec, 'k:', 'LineWidth', 1.5);
    plot(x, q3Vec, 'k:', 'LineWidth', 1.5);
    plot(x, stdVec, 'g-.', 'LineWidth', 1.5);
    legend('Mean', 'Median', 'Q1 (25%)', 'Q3 (75%)', 'Standard Deviation');
    xlabel('Number of simulations');
    ylabel(metricName);
    title(['Convergence Analysis: ' metricName]);
end


%% Helper Function to Pad Data for Boxplot
function paddedData = padDataForBoxplot(dataCellArray)
    maxLength = max(cellfun(@length, dataCellArray));
    paddedData = cellfun(@(x) [x(:); nan(maxLength - length(x), 1)], dataCellArray, 'UniformOutput', false);
    paddedData = cell2mat(paddedData);
end