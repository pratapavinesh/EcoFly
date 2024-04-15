clc, clearvars, close all;
% Network Model Parameters
numBlindNodes = 60;       % Number of blind nodes
numAnchorNodes = 30;       % Number of anchor nodes
communicationRange = 50;   % Communication range of UAVs
bsPosition = [0, 0, 0];    % Base Station position

% 3D Space Boundaries
xBounds = [-100, 100];
yBounds = [-100, 100];
zBounds = [-100, 100];

% Energy Model Parameters
energyThreshold = 30;     % Energy threshold for CH selection
initialEnergyRange = [50, 100];  % Initial energy range for blind nodes

% Velocity Model Parameters
velocityMean = 5;    % Mean of the velocity distribution
velocityStdDev = 2;  % Standard deviation of the velocity distribution

% Initialize Anchor Nodes
anchorNodes = repmat(struct('id', [], 'position', []), numAnchorNodes, 1);

for i = 1:numAnchorNodes
    anchorNodes(i).id = i + numBlindNodes;
    anchorNodes(i).position = [randi(xBounds), randi(yBounds), randi(zBounds)];
end

% Initialize Blind Nodes as an array of structures
blindNodes = repmat(struct('id', [], 'position', [], 'energy', [], 'isCH', [], ...
    'velocity',[]), numAnchorNodes, 1);

% Populate Blind Nodes
for i = 1:numBlindNodes
    blindNodes(i).id = i;
    blindNodes(i).position = [randi(xBounds), randi(yBounds), randi(zBounds)];
    blindNodes(i).energy = randi(initialEnergyRange);
    blindNodes(i).isCH = false; % Initializing blind nodes as not CH
    blindNodes(i).velocity = normrnd(velocityMean, velocityStdDev, 1, 3);
end

% Plot Network
figure(1);
hold on;
view(3); % Sets the view to 3D
grid on; % Add grid
% Extract and plot Blind Nodes positions
blindNodesPositions = vertcat(blindNodes.position);
scatter3(blindNodesPositions(:,1), blindNodesPositions(:,2), blindNodesPositions(:,3), 'o', 'filled', 'DisplayName', 'Blind Nodes');

% Extract and plot Anchor Nodes positions
anchorNodesPositions = vertcat(anchorNodes.position);
scatter3(anchorNodesPositions(:,1), anchorNodesPositions(:,2), anchorNodesPositions(:,3), '^', 'filled', 'DisplayName', 'Anchor Nodes', 'MarkerEdgeColor', 'b');

scatter3(bsPosition(1), bsPosition(2), bsPosition(3), 'd', 'filled','b', 'DisplayName', 'Base Station', 'MarkerEdgeColor', 'k');
legend('Location', 'Best');
xlabel('X-coordinate');
ylabel('Y-coordinate');
zlabel('Z-coordinate');
title('Network Model');

hold off;
% Display Node Information
disp('Blind Node Information:');
disp(blindNodes);

disp('Anchor Node Information:');
disp(anchorNodes);






