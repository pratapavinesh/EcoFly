% Extract actual positions of blind nodes
blindNodesPositions = vertcat(blindNodes.position);

% Perform K-means clustering
numClusters = 25; % Adjust as needed
[idx, centroids] = kmeans(blindNodesPositions, numClusters);



% Initialize cell array to store blind nodes for each cluster
clusterNodes = cell(numClusters, 1);

% Assign blind nodes to corresponding clusters
for j = 1:length(blindNodes)
    clusterNodes{idx(j)} = [clusterNodes{idx(j)}; blindNodes(j)];
end


% Plot Network
figure(3);
hold on;
view(3); % Sets the view to 3D
grid on; % Add grid

% Define colors for each cluster
colors = ["red", "green", "blue", "cyan", "magenta", "yellow", "black", "white", ...
          "#FFA07A", "#00FF7F", "#000080", "#00BFFF", "#800080", "#FFD700", ...
          "#FFA500", "#008080", "#A52A2A", "#FF69B4", "#FA8072", "#808000", ...
          "#4B0082", "#808080", "#F08080", "#BDB76B", "#A0522D", "#BC8F8F", ...
          "#2F4F4F", "#BA55D3", "#FF8C00"]; % Add more colors if needed

% Plot Blind Nodes positions with different symbols for each cluster
for k = 1:numClusters
    clusterPoints = blindNodesPositions(idx == k, :);
    scatter3(clusterPoints(:, 1), clusterPoints(:, 2), clusterPoints(:, 3), 'filled', 'DisplayName', ['Cluster ' num2str(k)], 'MarkerFaceColor', colors(k));
end

% Plot Base Station position
scatter3(bsPosition(1), bsPosition(2), bsPosition(3), 'd', 'filled', 'DisplayName', 'Base Station', 'MarkerEdgeColor', 'k');

% Add legends for scatter plots only
legend('Location', 'Best');
xlabel('X-coordinate');
ylabel('Y-coordinate');
zlabel('Z-coordinate');
title('Network Model with K-means Clustering');

hold off;

