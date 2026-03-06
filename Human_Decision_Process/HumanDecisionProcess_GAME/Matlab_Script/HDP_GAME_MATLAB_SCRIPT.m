close all;
load('Maps_struct.mat');

% Ask for user name
username=createUserInputGUI();

% Tutorial Images setting
numImages1=1; numImages2=16;
createImageTutorialGUI(numImages1,numImages2)

tutorial(username, Maps);

numImages1=17; numImages2=18;
createImageTutorialGUI(numImages1,numImages2)


%Fake empty map for visualization
gridMap = zeros(90, 90);
% Create a occupancy map with same dimensions
fakemap = binaryOccupancyMap(90, 90, 1);
setOccupancy(fakemap, gridMap);

% Initialize statistics tracking structure
STATS.positionIterations={}; 
STATS.totalIterations={};
STATS.posOrientations_done={}; 
STATS.distanceTravelledAccum={};
STATS.distanceToObjective={};
STATS.angularDist={};
STATS.angularDistDeg={};
STATS.exploredMap={};
STATS.exploreROI={};
STATS.objectFound={};
STATS.mapNum={};
STATS.time2decide={};


% Initialize laser sensor parameters
maxRange = 40;                           % Set maximum range (in voxels)
numRays = 50;                            % adjust depending on maxRange and FoV span
angles = linspace(-pi/6, pi/6, numRays); % Range of sensor scan angles

imageNumber=19; % variable to control the image tha its going to pop up after download the map

% Loop to run the 5 maps
for r=1:5
    map = Maps.occMaps{r,1}; 
    % download ROI cells
    cellStates=Maps.ROIcells{r,1}; 
    [rw,cl]=find(cellStates == 4 | cellStates == 5);
    sizeROI=length(rw);

    % Set up map and display 
    screenSize = get(0, 'ScreenSize');
    figure('Position', [1, 1, screenSize(3), screenSize(4)]);

    grid on;
    show(fakemap,"world");
    axis equal; axis tight;
    title('');
    xlabel('X');
    ylabel('Y');
    hold on;
    xlim([0 90]);
    ylim([0 90]);

    % Manually plot grid for better visualization
    for i = 0:90
        plot([i i], [0 90], 'k'); % Vertical grid line
        plot([0 90], [i i], 'k'); % Horizontal grid line
    end

    % Get occupancy status and flip for alignment
    occupied = getOccupancy(map);
    

    %%%%%%%%%----------     AGENT STARTING POSITION    ----------%%%%%%%%%%   
    [Yinitial, Xinitial] = find(cellStates ==0);
    intialOrient = deg2rad(270); % Set starting orientation

    poses = [Xinitial-0.5, Yinitial-0.5, intialOrient]; % we pack the position info         
    LastPose=poses;% set last position of the agent

    
    %%%%%%%%%%----------    OBJECT STARTING POSITION   ----------%%%%%%%%%%
    [XiniOb, YiniOb] = find(cellStates ==5);
    
    %%%%%%%%%%----------        STATS SETTINGS         ----------%%%%%%%%%%
   
    % Initialize statistics for movement and exploration
    posOrientations_done=[]; % Saves the path chosen
    posOrientations_done(1,:)=poses;
    distanceTravelled = 0;
    distanceToObject = [];
    distanceToObject = [distanceToObject, sqrt((Xinitial- (YiniOb))^2+(Yinitial-(XiniOb))^2)];
    
    MapExp_Iterations=[]; % Map explored over total iterations (also orientations)
    ROIExp_Iterations=[]; % ROI explored over total iterations (also orientations)
    angles_Iterations=[]; % angles achieved over iterations

    time2decide=[]; %stores the time take to choose nex best movement

    ObjectFound = 0; % Track if target object is found


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%         STARTS THE PROCESS          %%%%%%%%%%%%%%%%%
    
    exitValue=0; % set exit value to 0, if it changes to 1, code ends
    contadorTotal=0; contadorPos=1; % Preset stats values
    % Calls movement function with initial values to start exploration
    disp('calls Movements');
    plotting=1;
    [cellStates,MapExp_Iterations,ROIExp_Iterations, contadorTotal, angles_Iterations] = Movements (poses, map, cellStates, maxRange, numRays, angles, contadorTotal, XiniOb, YiniOb, MapExp_Iterations, ROIExp_Iterations, angles_Iterations, sizeROI,contadorPos,distanceTravelled);

    MapExp_Iterations_Before = MapExp_Iterations;
    
    % Checks if the obsject was found
    if cellStates(XiniOb, YiniOb) ~= 5
            disp('Object found');
            hold on;
            show(map);
            disp('show map)  ')
            ObjectFound = 1;
            exitValue=1;
    end
    
   iteration=1;
   while true
        iteration=iteration+1;
        disp('calls paintMap');
        paintfinalmap=0;
        paintMap(cellStates, map,paintfinalmap,contadorPos,contadorTotal,distanceTravelled,angles_Iterations); % Render the updated map for visualization
        disp('calls NextBestMove');
        % NextBestMove decides best cell to move 
            tic;
       [NBM, exitValue, angulosExp]= NextBestMove (cellStates, LastPose, map);
            responseTime = toc;
            time2decide = [time2decide,responseTime];
            fprintf('You took %.2f seconds to respond.\n', responseTime);

        % Calculate distance metrics
        Newposes=[]; % Initialize new poses
        distanceTravelled = [distanceTravelled, sqrt((LastPose(1)-(NBM(1)-0.5))^2+(LastPose(2)-(NBM(2)-0.5))^2) + distanceTravelled(end)];
        distanceToObject = [distanceToObject, sqrt( (NBM(1)- YiniOb)^2+(NBM(2)-XiniOb)^2)];

        Newposes(1,1)=NBM(1)-0.5;
        Newposes(1,2)=NBM(2)-0.5;
        Newposes(1,3)=angulosExp(1);

        if ceil(LastPose(1))~=NBM(1) || ceil(LastPose(2))~= NBM(2)
            contadorPos=contadorPos+1; % position iterations counter for STATS
        end

        disp('new poses selected are: ');
        posOrientations_done(iteration,:)=Newposes;   
        LastPose=[NBM(1)-0.5, NBM(2)-0.5, Newposes(end,3)]; % save new last pose
        [rows, cols]=size(Newposes);
        
        
        poses=Newposes;
    
        % Calls movement function with new poses to start exploration
        [cellStates,MapExp_Iterations,ROIExp_Iterations, contadorTotal, angles_Iterations] = ... 
            Movements (poses, map, cellStates, maxRange, numRays, angles, contadorTotal, ...
            XiniOb, YiniOb, MapExp_Iterations, ROIExp_Iterations, angles_Iterations, sizeROI, contadorPos,distanceTravelled);

        % Checks if the object was found
        if cellStates(XiniOb, YiniOb) ~= 5
            disp('Objet found');
            ObjectFound = 1;
            break;
        end        

        MapExp_Iterations_Before = max(MapExp_Iterations);
    end
    
    totalAngularDist=[0];
    totalAngularDistDegrees=[];
    for i=1:length(angles_Iterations)-1
        delta_theta = atan2(sin(angles_Iterations(i+1) - angles_Iterations(i)), cos(angles_Iterations(i+1) - angles_Iterations(i)));
        totalAngularDist(i+1)= abs(delta_theta) + totalAngularDist(i);
    end
    totalAngularDistDegrees = rad2deg(totalAngularDist);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%                  SAVING STATS                  %%%%%%%%%%%
    STATS.positionIterations{r,1} = contadorPos;
    STATS.totalIterations{r,1} = contadorTotal;
    STATS.posOrientations_done{r,1} = posOrientations_done;
    STATS.distanceTravelledAccum{r,1} = distanceTravelled;
    STATS.distanceToObjective{r,1} = distanceToObject;
    STATS.angularDist{r,1} = totalAngularDist;
    STATS.angularDistDeg{r,1} = totalAngularDistDegrees;
    STATS.exploredMap{r,1} = MapExp_Iterations;
    STATS.exploreROI{r,1} = ROIExp_Iterations;
    STATS.objectFound{r,1} = ObjectFound;
    STATS.time2decide{r,1} = time2decide;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pause(2.5);
    close all;
    numImages1=imageNumber; 

    if imageNumber <=22
        numImages2=imageNumber;
    else
        numImages2=imageNumber+1;
    end
    createImageTutorialGUI(numImages1,numImages2);
    imageNumber=imageNumber+1;
end

STATS.mapNum{1,1}=r; %stores the amount of maps simulated  

filename = ['./HDP_DATA_5_MAPS/HumanDecisionProcess_' username];
save(filename,'STATS');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%             FUNCTIONS              %%%%%%%%%%%%%%%%%%%

% Main function to move the sensor to designated positions
function [cellStates,MapExp_Iterations,ROIExp_Iterations, contadorTotal, angles_Iterations] = ...
    Movements (poses, map, cellStates, maxRange, numRays, angles, contadorTotal, ...
    XiniOb, YiniOb, MapExp_Iterations, ROIExp_Iterations, angles_Iterations, sizeROI,contadorPos,distanceTravelled)
    
    % Loop over each position (pose) the sensor needs to move to
    for k = 1:size(poses, 1)
        contadorTotal = contadorTotal + 1;  % Track total movements performed (STATS)     
        angles_Iterations= [angles_Iterations, poses(k,3)];  % Stores orientation angle for each movement
        sensorPose = poses(k, :); % Set current position and orientation
        
        % Display map updates after the first movement to reflect changes
        if k > 1
            paintfinalmap=0;
            paintMap(cellStates, map, paintfinalmap,contadorPos,contadorTotal,distanceTravelled,angles_Iterations);
        end
        % Update the map based on sensor's new pose, marking detected obstacles and free spaces
        disp('calls updateMap')
        plotting=1; 
        cellStates = updateMap(sensorPose, map, cellStates, maxRange, numRays, angles,plotting);
        disp('updateMap done'); 
        
        % Function that calculates proportion of explored map and ROI (STATS)
        [mapExp, ROIExp] = mapExpPercentage(cellStates,sizeROI);
  
        MapExp_Iterations = [MapExp_Iterations, mapExp];
        ROIExp_Iterations = [ROIExp_Iterations, ROIExp];

        % Checks is the object was found
        if cellStates(XiniOb, YiniOb) ~= 5
            disp('Objeto encontrado');
            disp('muestas el mapita real');
            hold on;
            paintfinalmap=1;        
            paintMap(cellStates, map, paintfinalmap,contadorPos,contadorTotal,distanceTravelled,angles_Iterations);
            plot(YiniOb-0.5, XiniOb-0.5, 'bo', 'MarkerSize', 8, 'LineWidth', 4);  % Sampled positions in blue
            plot(YiniOb-0.5, XiniOb-0.5, 'ko', 'MarkerSize', 60, 'LineWidth', 7); % circle arround
            plot(YiniOb-0.5, XiniOb-0.5, 'ro', 'MarkerSize', 60, 'LineWidth', 5); % circle arround
            break;  
        end
        pause(0.1);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Function to calculate exploration proportion of the map and ROI (Region of Interest)
function [mapExp, ROIExp]= mapExpPercentage (cellStates,sizeROI)
    [r,~]=find(cellStates == 4 | cellStates == 5);
    ROIExp = 1-(length(r)/sizeROI);
    [rws,~]=find(cellStates == 4 | cellStates == 5 | cellStates == 2);
    mapExp = 1 - (length(rws)/(90*90));

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [NBM, exitValue,angulosExp] = NextBestMove (cellStates, LastPose, map)
    exitValue = 0;
    x0 = ceil(LastPose(1));
    y0 = ceil(LastPose(2));

    hFig = gcf;  
    hold on;

    % Iterate over the range of cells around agents position
    radius=15; % movement range
    cellsInRadius = zeros(90,90);
    for i = max(1, x0 - radius) : min(90, x0 + radius)
        for j = max(1, y0 - radius) : min(90, y0 + radius)
            distance = sqrt((i - x0)^2 + (j - y0)^2);
            
            % include cells inside radius
            if distance <= radius && (cellStates(j, i) == 0 || cellStates(j, i) == 3)
                cellsInRadius(i,j)=1;
                patch([i-1, i-1, i, i], [j-1, j, j, j-1],'k','FaceAlpha',0.3);
            end
        end
    end
    patch([x0-1, x0-1, x0, x0], [y0-1, y0, y0, y0-1],'m','FaceAlpha',0.5  );

    htext = text(45, 94, 'Select Next Position', 'FontSize', 35, 'Color', 'black', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

    % Loop for choosing next position
    while true
        disp('Select next position');
        % update message
        set(htext, 'String', 'Select Next Position', 'Color', 'black');

        % Capture mouse click
        [x, y] = ginput(1);
        NBM = floor([x, y]) + [1, 1];

        if NBM(1) >= 1 && NBM(1) <= 90 && NBM(2) >= 1 && NBM(2) <= 90 && cellsInRadius(NBM(1), NBM(2))==1
            % Valid position
            set(htext, 'String', 'Valid Position', 'Color', 'green');
            pause(0.1);
            break;
        else
            % Invalid position
            disp('Invalid');
            set(htext, 'String', 'Invalid Position', 'Color', 'red');
            pause(0.5);
        end
    end

    % Orientation selection
    arrowLength = 10;
    arrowAngle = 0;  % Initial arrow angle
    x_end = NBM(1)  + arrowLength * cos(arrowAngle); % Coordinate X of the arrow tip
    y_end = NBM(2)  + arrowLength * sin(arrowAngle); % Coordinate Y of the arrow tip

    % Show arrow
    hArrow = quiver(NBM(1)-0.5, NBM(2)-0.5, x_end - NBM(1), y_end - NBM(2), 0, 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Color', 'b');
    
    %offset angles
    x0=NBM(1)-0.5;
    y0=NBM(2)-0.5;
    x2= x0-0.5 + 40 * cos(arrowAngle+pi/6);
    y2= y0-0.5 + 40 * sin(arrowAngle+pi/6);
    % Show FoV arrows
    hLine1 = plot([NBM(1)-0.5,x2], [NBM(2)-0.5, y2], 'b-', 'LineWidth', 1.5);
    x2= x0-0.5 + 40 * cos(arrowAngle-pi/6);
    y2= y0-0.5 + 40 * sin(arrowAngle-pi/6);
    hLine2 = plot([NBM(1)-0.5, x2], [NBM(2)-0.5, y2], 'b-', 'LineWidth', 1.5);

    theta1=(arrowAngle-pi/6);
    theta2=(arrowAngle+pi/6);
    % Ensure shortest arc
    if theta2 < theta1
        theta2 = theta2 + 2*pi;
    end
    if theta2 - theta1 > pi
        theta1 = theta1 + 2*pi;
    end
    theta = linspace(theta1, theta2, 100);
    % Coordinates of the arc
    x_arc = x - 0.5 + 40 * cos(theta);
    y_arc = y - 0.5 + 40 * sin(theta);
    hArc = plot(x_arc, y_arc, 'b-', 'LineWidth', 2);


    % Setuo figure and  activate double click detection
    exitLoop = false;
    set(hFig, 'WindowButtonDownFcn', @detectClick);
    
    x0 = NBM(1) - 0.5;
    y0 = NBM(2) - 0.5;
    while ~exitLoop
        % to void saturation
        pause(0.01);
    end

    % Function to detect double clicks
    function detectClick(~, ~)
        % Check if double click
        if strcmp(get(hFig, 'SelectionType'), 'open')
            exitLoop = true;
        else
            % Obtain click position
            clickPoint = get(gca, 'CurrentPoint');
            x_click = clickPoint(1,1);
            y_click = clickPoint(1,2);
    
            % Calculate angle between click position and agents actual position
            arrowAngle = calculateAngle(x_click, y_click, x0, y0);
            arrowAngle = rad2deg(arrowAngle);
            % Round up angle to closest 5 degrees mutiple
            arrowAngle = round(arrowAngle / 5) * 5;
            arrowAngle = deg2rad(arrowAngle);
            
            updateArrowAndLines(arrowAngle, NBM, arrowLength, hArrow, hLine1, hLine2);
            updateArc(arrowAngle, NBM, hArc);
        end
    end

    % Function to calculate angle between click position and agents actual position
    function angle = calculateAngle(x_click, y_click, x0, y0)
        deltaX = x_click - x0;
        deltaY = y_click - y0;
        angle = atan2(deltaY, deltaX);
    end

    % Function to update arrow and lines in the figure
    function updateArrowAndLines(arrowAngle, NBM, arrowLength, hArrow, hLine1, hLine2)
        % Compute arrow coords
        x_end = NBM(1) + arrowLength * sin(arrowAngle);
        y_end = NBM(2) + arrowLength * cos(arrowAngle);
        
        % update arrow
        set(hArrow, 'UData', y_end - NBM(2), 'VData', x_end - NBM(1));
        
        % Compute FoV lines coords
        x0 = NBM(1) - 0.5;
        y0 = NBM(2) - 0.5;
        x2 = x0 + 40 * cos(arrowAngle + pi/6);
        y2 = y0 + 40 * sin(arrowAngle + pi/6);
        set(hLine1, 'XData', [x0, x2], 'YData', [y0, y2]);
        
        x2 = x0 + 40 * cos(arrowAngle - pi/6);
        y2 = y0 + 40 * sin(arrowAngle - pi/6);
        set(hLine2, 'XData', [x0, x2], 'YData', [y0, y2]);
    end

    % Function to update de arc in the figure
    function updateArc(arrowAngle, NBM, hArc)
        x0 = NBM(1) - 0.5;
        y0 = NBM(2) - 0.5;
        theta1 = arrowAngle - pi/6;
        theta2 = arrowAngle + pi/6;
    
        % Adjust to the shortest angle
        if theta2 < theta1
            theta2 = theta2 + 2*pi;
        end
        if theta2 - theta1 > pi
            theta1 = theta1 + 2*pi;
        end
    
        % Generate arc
        theta = linspace(theta1, theta2, 100);
        x_arc = x0 + 40 * cos(theta);
        y_arc = y0 + 40 * sin(theta);
        set(hArc, 'XData', x_arc, 'YData', y_arc);
    end
    angulosExp = arrowAngle;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Funtion to paint the map
function paintMap(cellStates, map, paintfinalmap,contadorPos,contadorTotal,distanceTravelled,angles_Iterations)
    
    %cla;  % Limpiar el grÃ¡fico anterior
    % Volver a dibujar el mapa sin borrar las celdas coloreadas
    if paintfinalmap==1
        show(map);
    else
        cla;
    end

    % verify if the text exisit
    persistent posText iterText distText angText;
    
    % if any of the text is not exisiting, regenerate it
    if isempty(posText) || ~isvalid(posText) || ...
       isempty(iterText) || ~isvalid(iterText) || ...
       isempty(distText) || ~isvalid(distText) || ...
       isempty(angText) || ~isvalid(angText)
       
        % Positions for the text
        xTextPosition = 92;
        yStartPosition = 88;
        lineSpacing = 2;
        
        posText = text(xTextPosition, yStartPosition, '', 'FontSize', 15, 'HorizontalAlignment', 'left');
        iterText = text(xTextPosition, yStartPosition - lineSpacing, '', 'FontSize', 15, 'HorizontalAlignment', 'left');
        distText = text(xTextPosition, yStartPosition - 2 * lineSpacing, '', 'FontSize', 15, 'HorizontalAlignment', 'left');
        angText = text(xTextPosition, yStartPosition - 3 * lineSpacing, '', 'FontSize', 15, 'HorizontalAlignment', 'left');
    end
    
  
    hold on;
    % Draw the grid manually and paint the cells
    
    [x,y]=find(cellStates == 1);
    if ~isempty(x)
        % Obtain coordinates for actual state
        xCoords = [x - 1, x, x, x - 1]';
        yCoords = [y - 1, y - 1, y, y]';
        patch(yCoords, xCoords,'r','FaceAlpha',0.7);
    end

    [x,y]=find(cellStates == 0);
    if ~isempty(x)
        % Obtain coordinates for actual state
        xCoords = [x - 1, x, x, x - 1]';
        yCoords = [y - 1, y - 1, y, y]';
        patch(yCoords, xCoords,'g','FaceAlpha',0.3);
    end

    [x,y]=find(cellStates == 4 | cellStates == 5);
    if ~isempty(x)
        % Obtain coordinates for actual state
        xCoords = [x - 1, x, x, x - 1]';
        yCoords = [y - 1, y - 1, y, y]';
        patch(yCoords, xCoords,'y','FaceAlpha',0.5  );
    end

    for i = 0:90
        plot([i i], [0 90], 'k');
        plot([0 90], [i i], 'k');
    end
    
    totalAngularDist=[0];
    totalAngularDistDegrees=[];
    for i=1:length(angles_Iterations)-1
        delta_theta = atan2(sin(angles_Iterations(i+1) - angles_Iterations(i)), cos(angles_Iterations(i+1) - angles_Iterations(i)));
        totalAngularDist(i+1)= abs(delta_theta) + totalAngularDist(i);
    end
    totalAngularDistDegrees = rad2deg(totalAngularDist);

    set(posText, 'String', sprintf('Positions done: %d', contadorPos(end)));
    set(iterText, 'String', sprintf('Total iterations: %d', contadorTotal(end)));
    set(distText, 'String', sprintf('Distance travelled: %.2f', distanceTravelled(end)));
    set(angText, 'String', sprintf('Angular distance travelled (deg): %.2f', totalAngularDistDegrees(end)));
    % text(xTextPosition, yStartPosition, sprintf('Positions done: %d', contadorPos(end)), 'FontSize', 15, 'HorizontalAlignment', 'left');
    % text(xTextPosition, yStartPosition - 2*lineSpacing, sprintf('Total iterations: %d', contadorTotal(end)), 'FontSize', 15, 'HorizontalAlignment', 'left');
    % text(xTextPosition, yStartPosition - 4 * lineSpacing, sprintf('Distance travelled: %.2f', distanceTravelled(end)), 'FontSize', 15, 'HorizontalAlignment', 'left');
    % text(xTextPosition, yStartPosition - 6 * lineSpacing, sprintf('Angular distance travelled (deg): %.2f', totalAngularDistDegrees(end)), 'FontSize', 15, 'HorizontalAlignment', 'left');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Function to do raycasting
function cellStates = updateMap(sensorPose, map, cellStates, maxRange, numRays, angles, plotting)

    % Calculate intersections with obstacles for each ray
    intersectionPts = rayIntersection(map, sensorPose, angles, maxRange);

    % Filter valid intersection points and round based on quadrant
    validIntersections = ~isnan(intersectionPts(:,1)) & ~isnan(intersectionPts(:,2));
    
    % Adjust rounding based on quadrant of each ray for x and y coordinates
    dirX = cos(angles + sensorPose(3));
    dirY = sin(angles + sensorPose(3));
    
    % Preallocate arrays for rounded obstacle coordinates
        obsX = nan(size(intersectionPts, 1), 1);
        obsY = nan(size(intersectionPts, 1), 1);
        
        % Apply quadrant-based rounding logic for x coordinates
        obsX(validIntersections & dirX' >= 0) = floor(intersectionPts(validIntersections & dirX' >= 0, 1) + 1.0001);
        obsX(validIntersections & dirX' < 0) = floor(intersectionPts(validIntersections & dirX' < 0, 1) + 0.9999);
        
        % Apply quadrant-based rounding logic for y coordinates
        obsY(validIntersections & dirY' >= 0) = floor(intersectionPts(validIntersections & dirY' >= 0, 2) + 1.0001);
        obsY(validIntersections & dirY' < 0) = floor(intersectionPts(validIntersections & dirY' < 0, 2) + 0.9999);

        % Filter out NaN values for valid obstacle coordinates
        obsX = obsX(~isnan(obsX)) ;
        obsY = obsY(~isnan(obsY)) ;
    
    % Filter obstacle points within map boundaries
    validObsIdx = obsX >= 1 & obsX <= 90 & obsY >= 1 & obsY <= 90;
    obsX = obsX(validObsIdx);
    obsY = obsY(validObsIdx);
    
    % Mark obstacle cells in cellStates
    cellIdxObs = sub2ind(size(cellStates), obsY, obsX);
    cellStates(cellIdxObs) = 1;

    % Calculate all ray coordinates along each ray's range
    distances = sqrt((sensorPose(1) - intersectionPts(:,1)).^2 + (sensorPose(2) - intersectionPts(:,2)).^2);
    distances(~validIntersections) = maxRange;  % Set max range for rays without intersections

    % Generate step vector and broadcast coordinates for all rays
    maxSteps = ceil(max(distances) / 0.5);
    t = (0:0.5:(maxSteps * 0.5))'; % Step vector for each point along the ray

    % Convert 't' into a matix with same number of o=columns as dirX
    x_rays = sensorPose(1) + t .* dirX;
    y_rays = sensorPose(2) + t .* dirY;

    % Remove points beyond valid distances for each ray
    rayLimits = ceil(distances / 0.5);
    for r = 1:numRays
        x_rays(rayLimits(r)+1:end, r) = NaN;
        y_rays(rayLimits(r)+1:end, r) = NaN;
    end

    % Restrict coordinates to map boundaries and remove NaN indices
    validIdx = ~isnan(x_rays) & ~isnan(y_rays) & x_rays <= 90 & x_rays > 0  & y_rays <= 90 & y_rays > 0; % Valid indices across rays; % Valid indices across rays
        % Rounding for x_rays
        x_rays(validIdx) = ceil(x_rays(validIdx));
        % Rounding for y_rays
        y_rays(validIdx) = ceil(y_rays(validIdx));

    idx_rays = sub2ind(size(cellStates), y_rays(validIdx), x_rays(validIdx));

    % Mark cells as free when not occupied by obstacles
        isFreeCell = (cellStates(idx_rays) ~= 1);
        cellStates(idx_rays(isFreeCell)) = 0;
    
    % Plotting rays
    if plotting == 1
         hold on
        for r = 1:2:numRays
            x2=sensorPose(1) + distances(r)*cos(angles(r)+sensorPose(3));
            y2=sensorPose(2) + distances(r)*sin(angles(r)+sensorPose(3));
            % Plot ray from sensor origin to valid range limit or obstacle
            plot([sensorPose(1), x2], [sensorPose(2),y2], 'b ');
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% GUI for username
function username = createUserInputGUI()
    screenSize = get(0, 'ScreenSize');
    fig = uifigure('Name', 'User Input', 'Position', [1, 1, screenSize(3), screenSize(4)]);  % Posición para pantalla completa

    % Add background image
    bgImage = uiimage(fig, 'ImageSource', 'Tutorial_IMG/logo.jpg');
    bgImage.Position = [1, 1, screenSize(3), screenSize(4)];

    % White square
    panelWidth = 650;
    panelHeight = 200;
    panel = uipanel(fig, 'BackgroundColor', 'white', 'Position', ...
        [screenSize(3)/2 - panelWidth/2, screenSize(4)/2 - panelHeight/2, panelWidth, panelHeight]);

    % Text box dimensions
    width = 600;
    height = 40;
    % Label dimensions
    labelWidth = 430;
    labelHeight = 30;
    
    % Craete label centered
    uilabel(panel, 'Position', [panelWidth/2 - labelWidth/2, panelHeight - labelHeight - 30, labelWidth, labelHeight], ...
        'Text', 'Enter your username: (NO spaces or symbols)', 'FontSize', 20);
    
    % Create text box centered
    usernameBox = uieditfield(panel, 'text', 'Position', [panelWidth/2 - width/2, panelHeight/2, width, height], 'FontSize', 20);

    % Create button to accept
    acceptButton = uibutton(panel, 'push', 'Text', 'Accept', ...
        'Position', [panelWidth/2 - width/2, panelHeight/2 - height - 50, width, height], 'FontSize', 20);

    username = '';  % Initialize

    % Asign a function to the button to store the username
    acceptButton.ButtonPushedFcn = @(src, event) onAcceptButtonPushed(fig, usernameBox);
    waitfor(fig);
    
    function onAcceptButtonPushed(fig, usernameBox)
        % Obtain the username
        username = usernameBox.Value;
        close(fig);
    end


end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                TUTORIAL                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function tutorial(username, Maps) 
    gridMap = zeros(90, 90);
    % Create a occupancy map with same dimensions
    fakemap = binaryOccupancyMap(90, 90, 1);
    setOccupancy(fakemap, gridMap);
    
    % Initialize statistics tracking structure
    STATS.positionIterations={}; 
    STATS.totalIterations={};
    STATS.posOrientations_done={}; 
    STATS.distanceTravelledAccum={};
    STATS.distanceToObjective={};
    STATS.angularDist={};
    STATS.angularDistDeg={};
    STATS.exploredMap={};
    STATS.exploreROI={};
    STATS.objectFound={};
    STATS.mapNum={};
    STATS.time2decide={};
    
    
    % Initialize laser sensor parameters
    maxRange = 40;                           % Set maximum range (in voxels)
    numRays = 50;                            % adjust depending on maxRange and FoV span
    angles = linspace(-pi/6, pi/6, numRays); % Range of sensor scan angles

    % This loop runs map 6, tutorial map
    for r=6:6
        map = Maps.occMaps{r,1}; 
        % download ROI cells
        cellStates=Maps.ROIcells{r,1}; 
        r=1;
        [rw,~]=find(cellStates == 4 | cellStates == 5);
        sizeROI=length(rw);
    
        % Set up map and display 
        screenSize = get(0, 'ScreenSize');
        figure('Position', [1, 1, screenSize(3), screenSize(4)]);
    
        grid on;
        show(fakemap,"world");
        axis equal; axis tight;
        title('');
        xlabel('X');
        ylabel('Y');
        hold on;
        xlim([0 90]);
        ylim([0 90]);
    
        % Manually plot grid for better visualization
        for i = 0:90
            plot([i i], [0 90], 'k'); % Vertical grid line
            plot([0 90], [i i], 'k'); % Horizontal grid line
        end
    
        % Get occupancy status and flip for alignment
        occupied = getOccupancy(map);
        
        %%%%%%%%%----------     AGENT STARTING POSITION    ----------%%%%%%%%%%   
        [Yinitial, Xinitial] = find(cellStates ==0);
        intialOrient = deg2rad(270); % Set starting orientation
    
        poses = [Xinitial-0.5, Yinitial-0.5, intialOrient]; % we pack the position info         
        LastPose=poses; % set last position of the agent
    
        %%%%%%%%%%----------    OBJECT STARTING POSITION   ----------%%%%%%%%%%
        [XiniOb, YiniOb] = find(cellStates ==5);
        
        %%%%%%%%%%----------        STATS SETTINGS         ----------%%%%%%%%%%

        % Initialize statistics for movement and exploration
        posOrientations_done=[]; % Saves the path chosen
        posOrientations_done(1,:)=poses;
        distanceTravelled = 0;
        distanceToObject = [];
        distanceToObject = [distanceToObject, sqrt((Xinitial- (YiniOb))^2+(Yinitial-(XiniOb))^2)];
        
        MapExp_Iterations=[]; % Map explored over total iterations (also orientations)
        ROIExp_Iterations=[]; % ROI explored over total iterations (also orientations)
        angles_Iterations=[]; % angles achieved over iterations

        time2decide=[]; %stores the time take to choose nex best movement
    
        ObjectFound = 0; % Track if target object is found
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%         STARTS THE PROCESS          %%%%%%%%%%%%%%%%%
        
        exitValue=0; % set exit value to 0, if it changes to 1, code ends
        contadorTotal=0; contadorPos=1;% Preset stats values
        % Calls movement function with initial values to start exploration
        disp('calls Movements');
        plotting=1;
        [cellStates,MapExp_Iterations,ROIExp_Iterations, contadorTotal, angles_Iterations] = Movements (poses, map, cellStates, maxRange, numRays, angles, contadorTotal, XiniOb, YiniOb, MapExp_Iterations, ROIExp_Iterations, angles_Iterations, sizeROI,contadorPos,distanceTravelled);
    
        MapExp_Iterations_Before = MapExp_Iterations;
        
        % Checks if the obsject was found
        if cellStates(XiniOb, YiniOb) ~= 5
                disp('Object found');
                hold on;
                show(map);
                disp('show map)  ')
                ObjectFound = 1;
                exitValue=1;
        end
        
       iteration=1;
       while true
            iteration=iteration+1;
            disp('calls paintMap');
            paintfinalmap=0;
            paintMap(cellStates, map,paintfinalmap,contadorPos,contadorTotal,distanceTravelled,angles_Iterations); % Render the updated map for visualization
            disp('calls NextBestMove');
            % NextBestMove decides best cell to move 
            tic;
            [NBM, exitValue, angulosExp]= NextBestMove (cellStates, LastPose, map);
                            responseTime = toc;
                            time2decide = [time2decide,responseTime];
                            fprintf('You took %.2f seconds to respond.\n', responseTime);
    
            % Calculate distance metrics
            Newposes=[]; % Initialize new poses
            distanceTravelled = [distanceTravelled, sqrt((LastPose(1)-(NBM(1)-0.5))^2+(LastPose(2)-(NBM(2)-0.5))^2) + distanceTravelled(end)];
            distanceToObject = [distanceToObject, sqrt( (NBM(1)- YiniOb)^2+(NBM(2)-XiniOb)^2)];
    
            Newposes(1,1)=NBM(1)-0.5;
            Newposes(1,2)=NBM(2)-0.5;
            Newposes(1,3)=angulosExp(1);
    
            if ceil(LastPose(1))~=NBM(1) || ceil(LastPose(2))~= NBM(2)
                contadorPos=contadorPos+1; % position iterations counter for STATS
            end
    
            disp('new poses selected are: ');
            posOrientations_done(iteration,:)=Newposes;   
            LastPose=[NBM(1)-0.5, NBM(2)-0.5, Newposes(end,3)]; % save new last pose
            [rows, cols]=size(Newposes);
            
            poses=Newposes;
        
            % Calls movement function with new poses to start exploration
             [cellStates,MapExp_Iterations,ROIExp_Iterations, contadorTotal, angles_Iterations] = ...
                 Movements (poses, map, cellStates, maxRange, numRays, angles, contadorTotal, ...
                 XiniOb, YiniOb, MapExp_Iterations, ROIExp_Iterations, angles_Iterations, sizeROI,contadorPos,distanceTravelled);
    
            % Checks if the object was found
            if cellStates(XiniOb, YiniOb) ~= 5
                disp('Objet found');
                ObjectFound = 1;
                break;
            end        
    
            MapExp_Iterations_Before = max(MapExp_Iterations);
        end
        
        totalAngularDist=[0];
        totalAngularDistDegrees=[];
        for i=1:length(angles_Iterations)-1
            delta_theta = atan2(sin(angles_Iterations(i+1) - angles_Iterations(i)), cos(angles_Iterations(i+1) - angles_Iterations(i)));
            totalAngularDist(i+1)= abs(delta_theta) + totalAngularDist(i);
        end
        totalAngularDistDegrees = rad2deg(totalAngularDist);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%                  SAVING STATS                  %%%%%%%%%%%
        STATS.positionIterations{r,1} = contadorPos;
        STATS.totalIterations{r,1} = contadorTotal;
        STATS.posOrientations_done{r,1} = posOrientations_done;
        STATS.distanceTravelledAccum{r,1} = distanceTravelled;
        STATS.distanceToObjective{r,1} = distanceToObject;
        STATS.angularDist{r,1} = totalAngularDist;
        STATS.angularDistDeg{r,1} = totalAngularDistDegrees;
        STATS.exploredMap{r,1} = MapExp_Iterations;
        STATS.exploreROI{r,1} = ROIExp_Iterations;
        STATS.objectFound{r,1} = ObjectFound;
        STATS.time2decide{r,1} = time2decide;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        pause(5);
        close all;
    end
    STATS.mapNum{1,1}=r; % stores the amount of maps simulated  

    filename = ['TUTORIAL_HumanDecisionProcess_' username];
    save(filename,'STATS');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function createImageTutorialGUI(numImages1,numImages2)
    screenSize = get(0, 'ScreenSize');
    fig = uifigure('Name', 'Image Tutorial', 'Position', [1, 1, screenSize(3), screenSize(4)]);
    
    Tutorial_IMG = struct();
    for i = numImages1:numImages2
        Tutorial_IMG(i - numImages1 + 1).Image = imread(['Tutorial_IMG/Image_' num2str(i) '.jpg']);
    end
    
    % Variable to control images
    currentIndex = 1;
    
    imgDisplay = uiimage(fig, 'Position', [screenSize(3)/8, screenSize(4)/7, screenSize(3)/1.3, screenSize(4)/1.3]);
    imgDisplay.ImageSource = Tutorial_IMG(currentIndex).Image;

    % create buttons to go forward and backward
    btnPrev = uibutton(fig, 'push', 'Text', 'Previous', ...
                       'Position', [screenSize(3)/4 - 120, screenSize(4)/6 - 40, 100, 40], ...
                       'BackgroundColor', [1, 0.6, 0.6],...
                       'FontSize', 20, ...
                       'ButtonPushedFcn', @(btn,event) showPreviousImage());
    btnNext = uibutton(fig, 'push', 'Text', 'Next', ...
                       'Position', [screenSize(3)/4 + screenSize(3)/2 + 20, screenSize(4)/6 - 40, 100, 40], ...
                       'BackgroundColor', [0.6, 0.8, 1], ...
                       'FontSize', 20, ...
                       'ButtonPushedFcn', @(btn,event) showNextImage());

    % Function to show previous image
    function showPreviousImage()
        if currentIndex > 1
            currentIndex = currentIndex - 1;
            imgDisplay.ImageSource = Tutorial_IMG(currentIndex).Image;
        end
    end

    % Function to show next image
    function showNextImage()
        if currentIndex < numImages2 - numImages1 + 1
            currentIndex = currentIndex + 1;
            imgDisplay.ImageSource = Tutorial_IMG(currentIndex).Image;
        elseif currentIndex==16 && numImages2==16
            % Show confirmation message at last image
            answer = uiconfirm(fig, 'Are you sure you want to proceed to the tutorial?', ...
                               'Confirm Proceed', 'Options', {'Yes', 'No'}, ...
                               'DefaultOption', 2, 'CancelOption', 2);
            if strcmp(answer, 'Yes')
                close(fig);
            end
        elseif currentIndex==2 && numImages2==24
             % Show confirmation message at last image
            answer = uiconfirm(fig, 'Are you sure you want to exit the game?', ...
                               'Confirm Proceed', 'Options', {'Yes', 'No'}, ...
                               'DefaultOption', 2, 'CancelOption', 2);
            if strcmp(answer, 'Yes')
                close(fig);
            end
        elseif currentIndex==2 && numImages2==18
            % Show confirmation message at last image
            answer = uiconfirm(fig, 'Are you sure you want to start the game?', ...
                               'Confirm Proceed', 'Options', {'Yes', 'No'}, ...
                               'DefaultOption', 2, 'CancelOption', 2);
            if strcmp(answer, 'Yes')
                close(fig);
            end

        else 
            % Show confirmation message at last image
            answer = uiconfirm(fig, 'Are you sure you want to play next level?', ...
                               'Confirm Proceed', 'Options', {'Yes', 'No'}, ...
                               'DefaultOption', 2, 'CancelOption', 2);
            if strcmp(answer, 'Yes')
                close(fig);
            end
        end
    end

    waitfor(fig);
end