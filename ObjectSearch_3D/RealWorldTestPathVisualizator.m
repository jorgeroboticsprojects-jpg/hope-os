% ================== CONFIG ==================
csvPath    = 'realTrajectories/position_orientation_data1.csv';
folderPath = 'maps_ROI_Object_Situations/RandomObstacles_2_situations';
mapName    = 'RandomObstacles_2';
eulerSeq   = 'XYZ';

% ================== MAP ====================
occMatrixPath   = fullfile(folderPath, [mapName 'OccMatrix.mat']);
sensorPosesPath = fullfile(folderPath, 'sensorPoses.mat');
voxelStatesPath = fullfile(folderPath, [mapName '_voxelStates_3.mat']);
workSpacePath   = fullfile(folderPath, [mapName 'WorkSpace.mat']);

[~, map3D, voxelStates, ~,~, ~, ~, ~] = Initialization_ObjectSearch( ...
    occMatrixPath, voxelStatesPath, sensorPosesPath, workSpacePath);

% Reads CSV and returns [x y z qw qx qy qz]
posOrientations_done = readTrajectoryCSV_toQuat(csvPath, eulerSeq);

% ============ PLOTTING  ============
rangeStart = 23;  
rangeEnd   = 36;

N = size(posOrientations_done,1);
if ~isempty(rangeStart) && ~isempty(rangeEnd)
    i1 = max(1, min(N, rangeStart));
    i2 = max(1, min(N, rangeEnd));
    if i1 > i2
        warning('Rango [%d,%d] fuera de N=%d. Dibujo todas las muestras.', rangeStart, rangeEnd, N);
        Lidx = 1:N;
    else
        Lidx = i1:i2;
    end
else
    Lidx = 1:N;
end

figure;
show(map3D);
axis equal; grid on; hold on; view(135, 30);

for L = Lidx
    pos  = posOrientations_done(L,1:3);
    quat = posOrientations_done(L,4:7); % [w x y z]

    % point
    scatter3(pos(1), pos(2), pos(3), 30, [1, 0.0, 0.1], 'filled', 'MarkerFaceAlpha', 1);

    % segment
    if L > Lidx(1)
        prevPos = posOrientations_done(L-1,1:3);
        plot3([prevPos(1) pos(1)], [prevPos(2) pos(2)], [prevPos(3) pos(3)], 'k-', 'LineWidth', 1);
    end

    % orientatino
    rotm = quat2rotm(quat);
    range = 8;
    direction = rotm(:, 1) * range; % eje X del cuerpo
    quiver3(pos(1), pos(2), pos(3), direction(1), direction(2), direction(3), ...
            0, 'LineWidth', 2, 'MaxHeadSize', 2, 'Color', 'g');

    % objet (voxel == 5)
    [gx, gy, gz] = ind2sub(size(voxelStates), find(voxelStates == 5));
    if ~isempty(gx)
        scatter3(gx-0.5, gy-0.5, gz-0.5, 50, [0.6, 0.2, 0.8], 'filled', 'MarkerFaceAlpha', 1);
    end
end

% ================= HELPER =================
function PO = readTrajectoryCSV_toQuat(csvPath, eulerSeq)
% returns matrix Nx7 con [x y z qw qx qy qz] from CSV.
% Accepts:
%  - Position: nextBestPosition_X/Y/Z  o  x/y/z  o  posX/posY/posZ
%  - Orientation:
%       * Quaternion: qw qx qy qz  (o qx qy qz qw)
%       * Euler rad:  Orientation_X/Y/Z  (secuence indicated in eulerSeq variable)

% ---------- Read CSV ----------
assert(exist(csvPath,'file')==2, 'CSV not found: %s', csvPath);
opts = detectImportOptions(csvPath, 'NumHeaderLines', 0, ...
    'Delimiter', {',',';','\t'});
if isprop(opts, 'VariableNamingRule')
    opts.VariableNamingRule = 'preserve';
end
T = readtable(csvPath, opts);


% ---- positions ----
posNames = chooseCols(T, ["nextBestPosition_X","nextBestPosition_Y","nextBestPosition_Z"], ...
                         {"x","y","z"; "posX","posY","posZ"});
assert(~isempty(posNames), 'No encuentro columnas de posición en el CSV.');
pos = T{:, posNames}; % Nx3

% ---- orientations ----
quatNamesW = chooseCols(T, ["qw","qx","qy","qz"]);
quatNamesX = chooseCols(T, ["qx","qy","qz","qw"]);
if ~isempty(quatNamesW) || ~isempty(quatNamesX)
    if ~isempty(quatNamesW)
        q = T{:, quatNamesW};                % [qw qx qy qz]
    else
        qtemp = T{:, quatNamesX};            % [qx qy qz qw]
        q     = [qtemp(:,4) qtemp(:,1:3)];   % → [qw qx qy qz]
    end
else
    eulNames = chooseCols(T, ["Orientation_X","Orientation_Y","Orientation_Z"]);
    assert(~isempty(eulNames), 'No encuentro cuaternión ni Euler en el CSV.');
    E = T{:, eulNames};
    % Converts Euler → Rotm → Quat [w x y z]
    q = zeros(size(E,1),4);
    for i=1:size(E,1)
        R = eul2rotm(E(i,:), upper(eulerSeq));
        q(i,:) = rotm2quat(R);              % [w x y z]
    end
end

% normalization
n = sqrt(sum(q.^2,2)); n(n==0)=1; q = q./n;
flip = q(:,1) < 0; q(flip,:) = -q(flip,:);

PO = [pos q];  % [x y z qw qx qy qz]
end

function cols = chooseCols(T, primary, alts)
% returns cellstrwith real names of the table
names = T.Properties.VariableNames;  % cellstr

% normalice 'primary' a cellstr
if isstring(primary), primary = cellstr(primary); end
if ischar(primary),   primary = {primary};        end

cols = findNames(names, primary);
if ~isempty(cols) || nargin < 3 || isempty(alts)
    return;
end

% try alternative by rows
if iscell(alts) && ~isempty(alts) && size(alts,1) > 1
    for r = 1:size(alts,1)
        alt = alts(r,:);
        cols = findNames(names, alt);
        if ~isempty(cols), return; end
    end
else
    if isstring(alts), alts = cellstr(alts); end
    if ischar(alts),   alts = {alts};        end
    cols = findNames(names, alts);
end
end

function cols = findNames(names, req)

cols = cell(1, numel(req));
for i = 1:numel(req)
    idx = find(strcmpi(names, req{i}), 1);
    if isempty(idx)
        cols = {};  % if one is missing, not valid
        return;
    end
    cols{i} = names{idx};  % return exact name
end
end

