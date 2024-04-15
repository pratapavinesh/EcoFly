clc, clearvars, close all;
% Network Model Parameters
numBlindNodes = 100;       % Number of blind nodes
numAnchorNodes = 40;       % Number of anchor nodes
communicationRange = 70;   % Communication range of UAVs
bsPosition = [0, 0, 0];    % Base Station position

% 3D Space Boundaries
xBounds = [-100, 100];
yBounds = [-100, 100];zBounds = [-100, 100];

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

%%
% UAV Positioning Algorithm with IPSO
estimatedPositions = repmat(struct('position', []), numBlindNodes, 1);
for i = 1:numBlindNodes
    tempVariable = findUavPosition(blindNodes(i),anchorNodes,communicationRange, ...
        xBounds,yBounds,zBounds);
    estimatedPositions(i).position = tempVariable.position;
end
% Plot Network
figure(2);
hold on;
view(3); % Sets the view to 3D
grid on;% Add grid

% Extract and plot Blind Nodes positions
estimatedblindNodesPositions = vertcat(estimatedPositions.position);
scatter3(estimatedblindNodesPositions(:,1), estimatedblindNodesPositions(:,2), estimatedblindNodesPositions(:,3), 's', 'filled', 'DisplayName', 'Estimated Position of Blind Nodes');

% Extract and plot Blind Nodes positions
blindNodesPositions = vertcat(blindNodes.position);
scatter3(blindNodesPositions(:,1), blindNodesPositions(:,2), blindNodesPositions(:,3), 'o', 'filled', 'DisplayName', 'Actual Position of Blind Nodes');

% Extract and plot Anchor Nodes positions
%anchorNodesPositions = vertcat(anchorNodes.position);
%scatter3(anchorNodesPositions(:,1), anchorNodesPositions(:,2), anchorNodesPositions(:,3), '^', 'filled', 'DisplayName', 'Anchor Nodes', 'MarkerEdgeColor', 'b');

% Connect the estimated positions with actual blind node positions
for i = 1:numBlindNodes
    line([estimatedblindNodesPositions(i, 1), blindNodesPositions(i, 1)], ...
         [estimatedblindNodesPositions(i, 2), blindNodesPositions(i, 2)], ...
         [estimatedblindNodesPositions(i, 3), blindNodesPositions(i, 3)], ...
         'Color', 'k', 'LineStyle', '--', 'HandleVisibility', 'off');
end

scatter3(bsPosition(1), bsPosition(2), bsPosition(3), 'd', 'filled','b', 'DisplayName', 'Base Station', 'MarkerEdgeColor', 'k');
legend('Location', 'Best');
xlabel('X-coordinate');
ylabel('Y-coordinate');
zlabel('Z-coordinate');
title('Network Model');

hold off;

function Gbest = findUavPosition(blindNode,anchorNodes,communicationRange,xBounds,yBounds,zBounds)
    % IPSO Parameters
    NSC = 20;       % Number of particles in the swarm
    MaxIter = 50;   % Maximum number of iterations

    % IPSO Initialization
    nighbourAncherNodes = findNighboureNodes(blindNode, anchorNodes, communicationRange);

    %intilised the particles 
    particles = initializeParticles(NSC, xBounds, yBounds, zBounds);
    Pbest = particles;
    Gbest = getGlobalBest(particles, nighbourAncherNodes, blindNode);
    % IPSO Loop
    for iter = 1:MaxIter
        for i = 1:NSC
            % Update particle position and velocity using IPSO equations
            particles(i) = updateParticles(particles(i), Pbest(i), Gbest);

            % insure the boundary condition
            particles(i) = insureWithinBoundary(particles(i),xBounds, yBounds, zBounds);

            % Evaluate fitness function
            fitness = evaluateFitness(particles(i), nighbourAncherNodes,blindNode);
            
            % Update Pbest and Gbest
            if fitness < evaluateFitness(Pbest(i), nighbourAncherNodes,blindNode)
                Pbest(i) = particles(i);
            end
            if fitness < evaluateFitness(Gbest, nighbourAncherNodes,blindNode)
                Gbest = particles(i);
            end
        end
        
        % Apply VNS on Gbest
        %Gbest = applyVNS(Gbest);
    end    
end

% Helper function to initialize particle positions, velocities, and energies
function particles = initializeParticles(numParticles, xBounds, yBounds, zBounds)
    particles(numParticles) = struct('position', [], 'velocity', [], 'energy', [], 'energyVelocity', []);

    for i = 1:numParticles
        particles(i).position = [xBounds(1) + rand() * (xBounds(2) - xBounds(1)), ...
                                  yBounds(1) + rand() * (yBounds(2) - yBounds(1)), ...
                                  zBounds(1) + rand() * (zBounds(2) - zBounds(1))];
                              
        % Initialize velocity within bounds
        particles(i).velocity = (rand(1, 3) - 0.5) * 2; % Random values between -1 and 1
    end
end

%Function to get Distance b/w two nodes 
function distance = calculateDistance(node, anchorNode)
         distance = sqrt(sum((node.position-anchorNode.position).^2));
end

% Function to get global best
function Gbest = getGlobalBest(Pbest,nighbourAncherNodes,blindNode)
        Gbest = Pbest(1);
        for i = 1:length(Pbest)
            if evaluateFitness(Pbest(i),nighbourAncherNodes,blindNode) < evaluateFitness(Gbest,nighbourAncherNodes,blindNode)
                    Gbest = Pbest(i);
            end
        end
end 

% Function to update particles using IPSO equations
function updatedParticle = updateParticles(particle, Pbest, Gbest)
        w = 0.5;  % Inertia weight
        c1 = 3.5;  % Cognitive parameter
        c2 = 3.5;  % Social parameter
        % Update velocity
        particle.velocity = w * particle.velocity + ...
         c1 * rand() * (Pbest.position - particle.position) + ...
         c2 * rand() * (Gbest.position - particle.position);
            
         % Update position
         particle.position = particle.position + particle.velocity;
         updatedParticle = particle;
end

% Function to evaluate fitness function
function fitness = evaluateFitness(particle,nighbourAncherNodes,blindNode)
         dist = 0;
         for i = 1: length(nighbourAncherNodes)
            dist = dist + (calculateDistance(particle,nighbourAncherNodes(i))- ...
                calculateDistance(blindNode,nighbourAncherNodes(i)))^2;
         end
         fitness = dist/ length(nighbourAncherNodes);
end

%Function to insure that particle don't go outside of boundary position
function updatedParticle = insureWithinBoundary(particle,xBounds, yBounds, zBounds)
          bounds = [xBounds; yBounds; zBounds];  % Assuming bounds is a 3x2 matrix where each row i contains the lower and upper bounds for the i-th dimension
          for i = 1:3
            if particle.position(i) < bounds(i,1) || particle.position(i) > bounds(i,2)
                particle.position(i) = randi([bounds(i,1), bounds(i,2)]);
            end
          end
         updatedParticle = particle;
end

% helper function to find the nighbour ancher nodes that are with the communication range
function nighbourAncherNodes = findNighboureNodes(blindNode, anchorNodes, communicationRange)
    nighbourAncherNodes = [];
    for j = 1:length(anchorNodes)
      if  calculateDistance(blindNode, anchorNodes(j)) <= communicationRange
            nighbourAncherNodes = [nighbourAncherNodes, anchorNodes(j)];
      end
    end
end

% Function to Calculate the average position for intilisation purpose
function avergaePosition = findAveragePosition(nighbourAncherNodes)
     avergaePosition = [0,0,0];
     for i = 1:length(nighbourAncherNodes)
         avergaePosition = avergaePosition + nighbourAncherNodes(i).position;
     end
     if ~isempty(nighbourAncherNodes)
         avergaePosition = avergaePosition / length(nighbourAncherNodes);
     end
end

%%

% Extract actual positions of blind nodes
blindNodesPositions = vertcat(blindNodes.position);

% Perform K-means clustering
numClusters = 5; % Adjust as needed
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
colors = ['r', 'g', 'b', 'c', 'm']; % Add more colors if needed

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

%%

selectedCH = repmat(struct('position', [], 'energy', []), numClusters, 1);
for i = 1:numClusters
    tempResult = selectClusterHead(clusterNodes{i}, bsPosition, ...
        xBounds, yBounds, zBounds, initialEnergyRange);
    selectedCH(i).position = tempResult.position;
    selectedCH(i).energy = tempResult.energy;
end
clusterHeads = repmat(struct('id', [], 'position', [], 'energy', [], 'isCH', [], ...
    'velocity',[]), numClusters, 1);
for i = 1:numClusters
    nodes = clusterNodes{i};
    bestNodeIndex = 1; % Assume the first node is the best initially

    for j = 1:length(nodes)
        distance = calculate4DimesnsionalDistance(nodes(j), selectedCH(i));

        if distance < calculate4DimesnsionalDistance(nodes(bestNodeIndex), selectedCH(i))
            bestNodeIndex = j;
        end
    end

    % Mark the best node as CH
    clusterNodes{i}(bestNodeIndex).isCH = true;
    clusterHeds(i) = clusterNodes{i}(bestNodeIndex);
end

% Plot Network
figure(4);
hold on;
view(3); % Sets the view to 3D
grid on; % Add grid

% Define colors for each cluster
colors = ['r', 'g', 'b', 'c', 'm']; % Add more colors if needed

% Plot Cluster Head nodes
for i = 1:numClusters
    nodes = clusterNodes{i};
    
    % Plot all nodes in the cluster
    clusterPoints = vertcat(nodes.position);
    scatter3(clusterPoints(:, 1), clusterPoints(:, 2), clusterPoints(:, 3), 'filled', ...
        'DisplayName', ['Cluster ' num2str(i)], 'MarkerFaceColor', colors(i), 'MarkerFaceAlpha', 0.3);

    % Find Cluster Head node in the cluster
    clusterHeadIndex = find([nodes.isCH]);
    % Plot Cluster Head node if found
    if ~isempty(clusterHeadIndex)
        clusterHead = nodes(clusterHeadIndex);
        % Label the Cluster Head node as 'CH'
        text(clusterHead.position(1), clusterHead.position(2), clusterHead.position(3), 'CH', ...
            'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k');
        % Plot Cluster Head node with larger size and different color
        scatter3(clusterHead.position(1), clusterHead.position(2), clusterHead.position(3), ...
            'filled', 'MarkerFaceColor', colors(i),'DisplayName', ['CH ' num2str(i)]);
    end
end

% Plot Base Station position
scatter3(bsPosition(1), bsPosition(2), bsPosition(3), 'd', 'filled', 'DisplayName', 'Base Station', 'MarkerEdgeColor', 'k');

% Add legends for scatter plots only
legend('Location', 'Best');
xlabel('X-coordinate');
ylabel('Y-coordinate');
zlabel('Z-coordinate');
title('Network Model with Cluster Heads');

hold off;



function distance = calculate4DimesnsionalDistance(node1, node2 )
      % weight parameter
      a = 0.8;
      b = 0.2;
      distance = a* calculateDistance(node1.position, node2.position)+... 
      b* abs(node1.energy - node2.energy);
end
% Function to select CH nodes using IPSO algorithm (Algorithm 3)
function selectedCH = selectClusterHead(clusterNodes, bsPosition,xBounds, ...
    yBounds,zBounds,energyRange)
    % Initialization
    numParticles = 20; % Number of particles 
    maxIterations = 100; % Adjust as needed
    % PSO Initialization
    particles = initializeParticles(numParticles,xBounds,yBounds, ...
        zBounds,energyRange); % Initialize particles
    Pbest = particles; % Personal best positions
    Gbest = getGlobalBest(Pbest, clusterNodes, bsPosition); % Global best position
    
    % PSO Iterations
    for itr = 1:maxIterations
        for i = 1:numParticles
            % Update velocities and positions using PSO equations (with energy movement)
            particles(i) = updateParticle(particles(i), Pbest(i), Gbest);
            particles(i) = insureWithinBoundary(particles(i), xBounds, yBounds, zBounds, energyRange);
            
            % Evaluate fitness function using Equation (31) with combined metric
            fitnessValue = evaluateFitness(particles(i), clusterNodes, bsPosition);
            
            % Update pBest and gBest based on fitness values
            if fitnessValue < evaluateFitness(Pbest(i),clusterNodes, bsPosition)
                Pbest(i)= particles(i);
            end
            if fitnessValue < evaluateFitness(Gbest,clusterNodes, bsPosition)
                Gbest = particles(i);
            end

       end
       % Apply VNS on update gBest (placeholder)
       %gBest = applyVNS(gBest);
    end
    % Output the selected cluster head based on the final gBest position
    selectedCH = Gbest;
end

% Helper function to initialize particle positions, velocities, and energies
function particles = initializeParticles(numParticles, xBounds, yBounds, ...
    zBounds, energyRange)
    particles(numParticles) = struct('position', [], 'velocity', [], 'energy', [], 'energyVelocity', []);

    for i = 1:numParticles
        particles(i).position = [xBounds(1) + rand() * (xBounds(2) - xBounds(1)), ...
                                  yBounds(1) + rand() * (yBounds(2) - yBounds(1)), ...
                                  zBounds(1) + rand() * (zBounds(2) - zBounds(1))];
        % Ensure energy is within the specified range
        particles(i).energy = energyRange(1) + rand() * (energyRange(2) - energyRange(1));
                              
        % Initialize velocity within bounds
        particles(i).velocity = (rand(1, 3) - 0.5) * 2; % Random values between -1 and 1
        % Initialize energy velocity within bounds
        particles(i).energyVelocity = (rand() - 0.5) * 2 * 5; % Random value between -5 and 5
    end
end
% Helper function to get Gbest 
function Gbest = getGlobalBest(Pbest, clusterNodes, bsPosition)
       Gbest = Pbest(1);
       for i = 1: length(Pbest)
           if evaluateFitness(Pbest(i) ,clusterNodes, bsPosition) < evaluateFitness(Gbest,clusterNodes, bsPosition)
               Gbest = Pbest(i);
           end
       end
end
% Helper function to update velocities and positions of particles
function particle = updateParticle(particle, Pbest, Gbest)
     % parameters for updation
     w = 0.9;  % Inertia weight
     c1 = 2.5;  % Cognitive parameter
     c2 = 2.5;  % Social parameter  
     % Update velocity
     particle.velocity = w * particle.velocity + ...
         c1 * rand() * (Pbest.position - particle.position) + ...
         c2 * rand() * (Gbest.position - particle.position);
     particle.energyVelocity =  w * particle.energyVelocity + ...
         c1 * rand() * (Pbest.energy - particle.energy) + ...
         c2 * rand() * (Gbest.energy - particle.energy);
     % Update position
     particle.position = particle.position + particle.velocity;
     % Update energy
     particle.energy = particle.energy + particle.energyVelocity;
end
% Helper function to calculate distance between two positions
function distance = calculateDistance(position1,position2)
          distance = sqrt(sum((position1-position2).^2));
end
% Helper function to evaluate fitness function with combined metric
function fitnessValue = evaluateFitness(particle, clusterNodes, bsPosition)
     % weight parameter
     alpha = 0.7;
     beta = 0.1;
    fitnessValue = alpha * f1(particle,clusterNodes) + ...
        beta * f2(particle,bsPosition) + ...
        (1-alpha-beta)*f3(particle);
end

% Helper function to evaluate fitness function 1
function fitnessValue = f1(particle, clusterNodes)
    distance=0;
    for i = 1: length(clusterNodes)
        distance = distance + calculateDistance(particle.position,clusterNodes(i).position );
    end
    fitnessValue = distance / length(clusterNodes);
end
% Helper function to evaluate fitness function 1
function fitnessValue = f2(particle, bsPosition)
    fitnessValue = calculateDistance(particle.position, bsPosition);
end
% Helper function to evaluate fitness function 1
function fitnessValue = f3(particle)
    fitnessValue=1/particle.energy;
end

%Function to insure that particle don't go outside of boundary position
function updatedparticle = insureWithinBoundary(particle,xBounds, yBounds, zBounds,energyRange)
          bounds = [xBounds; yBounds; zBounds];  % Assuming bounds is a 3x2 matrix where each row i contains the lower and upper bounds for the i-th dimension
          for i = 1:3
            if particle.position(i) < bounds(i,1) || particle.position(i) > bounds(i,2)
                particle.position(i) = randi([bounds(i,1), bounds(i,2)]);
            end
          end
         if particle.energy < energyRange(1)|| particle.energy > energyRange(2)
             particle.energy = randi([energyRange(1), energyRange(2)]);
         end
        updatedparticle = particle;
 end
%%

%parameter for cluster based routing
communicationRangeForHead = 100;
%define the nighbourhood for clusterheads ( in real this can be done easily
%by sending hellow message )

% Initialize cell array to store blind nodes for each cluster
nighboureCHs = cell(numClusters, 1);
for i = 1:numClusters
    for j = 1:numClusters
        if calculateDistance(clusterHeds(i).position,clusterHeds(j).position ...
                )<= communicationRangeForHead && i~=j
          nighboureCHs{i} = [nighboureCHs{i}, clusterHeds(j)];
        end
    end
end

%calculate next hop for each ch
selectedNextHops = repmat(struct('id', [], 'position', [], 'energy', [], 'isCH', [], ...
    'velocity',[]), numAnchorNodes, 1);
for i = 1: numClusters
  temp = selectedNextHop(i, nighboureCHs, clusterHeads, bsPosition, xBounds, yBounds, zBounds, velocityMean , velocityStdDev);
  selectedNextHops.position = temp.position(1:3); %first 3 dimesions of position repersent postion
  selectedNextHops.velocity = temp.position(4:6); %next 3 dimesions of position repersent velocity
end

%find the actual next hop node
for i = 1:numClusters
    nodes = nighboureCHs{i};
    bestNodeIndex = 1; % Assume the first node is the best initially

    for j = 1:length(nodes)
        distance = calculate6DimesnsionalDistance(nodes(j), selectedNextHop(i));

        if distance < calculate6DimesnsionalDistance(nodes(bestNodeIndex), selectedNextHop(i))
            bestNodeIndex = j;
        end
    end
    selectedNextHops(i) = nodes(bestNodeIndex);
end

% Plotting
figure(6);
hold on;
view(3); % Sets the view to 3D
grid on; % Add grid

% Plot Cluster Head nodes and arrows to next hops
for i = 1:numClusters
    clusterHead = clusterHeds(i);
    
    % Plot Cluster Head node
    scatter3(clusterHead.position(1), clusterHead.position(2), clusterHead.position(3), ...
        'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', colors(i), 'DisplayName', ['CH ' num2str(i)]);
    
    % Plot arrow from CH to selected next hop
    quiver3(clusterHead.position(1), clusterHead.position(2), clusterHead.position(3), ...
        selectedNextHops(i).position(1) - clusterHead.position(1), ...
        selectedNextHops(i).position(2) - clusterHead.position(2), ...
        selectedNextHops(i).position(3) - clusterHead.position(3), ...
        'Color', 'k', 'LineStyle', '--', 'MaxHeadSize', 0.5);
end

% Plot Base Station position
scatter3(bsPosition(1), bsPosition(2), bsPosition(3), 'd', 'filled', 'DisplayName', 'Base Station', 'MarkerEdgeColor', 'k');

% Add legends for scatter plots only
legend('Location', 'Best');
xlabel('X-coordinate');
ylabel('Y-coordinate');
zlabel('Z-coordinate');
title('Network Model with Cluster Heads and Next Hops');

hold off;




function distance = calculate6DimesnsionalDistance(node1, node2 )
      % weight parameter
      a = 0.7;
      b = 0.3;
      distance = a* calculateDistance(node1.position, node2.position)+... 
      b* calculateDistance(node1.velocity - node2.velocity);
end

% Function to select CH nodes using IPSO algorithm (Algorithm 3)
function selectedNextHop = selectedNextHop(clusterIndex, nighboureCHs, clusterHeds, bsPosition,xBounds, yBounds,zBounds,velocityMean , velocityStdDev)
    % Initialization
    numParticles = 20;   % Number of particles 
    maxIterations = 100; % Adjust as needed

    % PSO Initialization
    particles = initializeParticles(numParticles, xBounds, yBounds, zBounds,velocityMean , velocityStdDev);  % Initialize particles
    Pbest = particles;                                                         % Personal best positions
    Gbest = getGlobalBest(Pbest, clusterIndex, nighboureCHs, clusterHeds, bsPosition);       % Global best position
    
    % PSO Iterations
    for itr = 1:maxIterations
        for i = 1:numParticles
            % Update velocities and positions using PSO equations (with energy movement)
            particles(i) = updateParticle(particles(i), Pbest(i), Gbest);
            particles(i) = insureWithinBoundary(particles(i), xBounds, yBounds, zBounds);
            
            % Evaluate fitness function using Equation (31) with combined metric
            fitnessValue = evaluateFitness(particles(i), clusterIndex, nighboureCHs, clusterHeds, bsPosition);
            
            % Update pBest and gBest based on fitness values
            if fitnessValue < evaluateFitness(Pbest(i), clusterIndex, nighboureCHs, clusterHeds, bsPosition)
                Pbest(i)= particles(i);
            end
            if fitnessValue < evaluateFitness(Gbest, clusterIndex, nighboureCHs, clusterHeds, bsPosition)
                Gbest = particles(i);
            end

       end
       % Apply VNS on update gBest (placeholder)
       %gBest = applyVNS(gBest);
    end
    % Output the selected cluster head based on the final gBest position
    selectedNextHop = Gbest;
end

% Helper function to initialize particle positions, velocities, and energies
function particles = initializeParticles(numParticles, xBounds, yBounds, zBounds, velocityMean, velocityStdDev)
    particles(numParticles) = struct('position', [], 'velocity', []);

    for i = 1:numParticles
        % Initialize position and velocity components within bounds
        positionXYZ = [xBounds(1) + rand() * (xBounds(2) - xBounds(1)), ...
                       yBounds(1) + rand() * (yBounds(2) - yBounds(1)), ...
                       zBounds(1) + rand() * (zBounds(2) - zBounds(1))];
                   
        velocityXYZ = normrnd(velocityMean, velocityStdDev, [1, 3]);

        % Combine position and velocity components
        particles(i).position = [positionXYZ, velocityXYZ];
        particles(i).velocity = (rand(1, 6) - 0.5) * 2; % Random values between -1 and 1;
    end
end


% Helper function to get Gbest 
function Gbest = getGlobalBest(Pbest, clusterIndex, nighboureCHs, clusterHeds, bsPosition)
       Gbest = Pbest(1);
       for i = 1: length(Pbest)
           if evaluateFitness(Pbest(i), clusterIndex, nighboureCHs, clusterHeds, bsPosition)...
               < evaluateFitness(Gbest, clusterIndex, nighboureCHs, clusterHeds, bsPosition)
               Gbest = Pbest(i);
           end
       end
end
% Helper function to update velocities and positions of particles
function particle = updateParticle(particle, Pbest, Gbest)
     % parameters for updation
     w = 0.9;  % Inertia weight
     c1 = 2.5;  % Cognitive parameter
     c2 = 2.5;  % Social parameter  
     % Update velocity
     particle.velocity = w * particle.velocity + ...
         c1 * rand() * (Pbest.position - particle.position) + ...
         c2 * rand() * (Gbest.position - particle.position);
     particle.energyVelocity =  w * particle.energyVelocity + ...
         c1 * rand() * (Pbest.energy - particle.energy) + ...
         c2 * rand() * (Gbest.energy - particle.energy);
     % Update position
     particle.position = particle.position + particle.velocity;
end

% Helper function to evaluate fitness function with combined metric
function fitnessValue = evaluateFitness(particle, clusterIndex, nighboureCHs, clusterHeds, bsPosition)
    modifiedParticle.position = particle.position(1:3);
    modifiedParticle.velocity = particle.position(4:6);
    nighboureNodes = nighboureCHs{clusterIndex};
    bestNode = nighboureNodes(1);
    for i = 1 : length(nighboureNodes)
        if calculate6DimesnsionalDistance(modifiedParticle, nighboureNodes(i))< ...
                calculate6DimesnsionalDistance(modifiedParticle, bestNode)
            bestNode = nighboureNodes(i);
        end
    end

    bestNodeIndex = 1;
    for i = 1:length(clusterHeds)
        if bestNode == clusterHeds(i)
            bestNodeIndex = i;
        end
    end
     % weight parameter
     alpha = 0.7;
     beta = 0.1;
    fitnessValue = alpha * f1(nighboureCHs{bestNodeIndex}, bestNode) + beta * f2(nighboureCHs{bestNodeIndex}, bsPosition);
end

% Helper function to evaluate fitness function 1
function fitnessValue = f1(nighboures, bestNode)
    averageLR=0;
    for i = 1: length(nighboures)
        averageLR = averageLR + calculateLR(nighboures(i), bestNode);
    end

    if ~ isempty(nighboures)
        averageLR = averageLR / length(nighboures);
    end
    fitnessValue = averageLR / length(nighboures);
end

% Helper function to evaluate fitness function 1
function fitnessValue = f2(nighboures, bsPosition)
    distance = 0;

    for i =1 : length(nighboures)
        distance = distance + calculateDistance(nighboures(i).position, bsPosition);
    end

    if ~ isempty(nighboures)
        distance = distance / length(nighboures);
    end
    fitnessValue = distance;
end

%Function to insure that particle don't go outside of boundary position
function updatedparticle = insureWithinBoundary(particle,xBounds, yBounds, zBounds,energyRange)
          bounds = [xBounds; yBounds; zBounds];  % Assuming bounds is a 3x2 matrix where each row i contains the lower and upper bounds for the i-th dimension
          for i = 1:3
            if particle.position(i) < bounds(i,1) || particle.position(i) > bounds(i,2)
                particle.position(i) = randi([bounds(i,1), bounds(i,2)]);
            end
          end
         if particle.energy < energyRange(1)|| particle.energy > energyRange(2)
             particle.energy = randi([energyRange(1), energyRange(2)]);
         end
        updatedparticle = particle;
end


% Helper function to calculate mean relative velocity
function mean_delta_v = meanDeltaV(node1, node2)
    % Implement Equation (36)
    delta_v = abs(calculateRelativeVelocity(node1, node2));
    mean_delta_v = mean(delta_v);
end

% Helper function to calculate standard deviation of relative velocity
function sigma_delta_v = calculateSigmaDeltaV(node1, node2)
    % You may adjust the range based on your system characteristics
    sigma_delta_v = rand() * 2; % Example range, adjust as needed
end


% Helper function to calculate relative transmission range
function r_ab = calculateRab(node1, node2)
    % Implement Equation (37)
    d = calculateDistance(node1.position, node2.position);
    relativeVelocity = calculateRelativeVelocity(node1, node2);
    
    if relativeVelocity > 0
        r_ab = d + communicationRangeForHead;
    else
        r_ab = d - communicationRangeForHead;
    end
end

% Helper function to calculate communication time T_C
function T_C = calculateCommunicationTime(node1, node2, r_ab)
    % Implement Equation (34)
    A = calculateRelativeVelocity(node1, node2)^2;
    B = 2 * dot(calculateRelativePosition(node1, node2), calculateRelativeVelocity(node1, node2));
    C = calculateDistance(node1.position, node2.position)^2 - r_ab^2;

    discriminant = B^2 - 4 * A * C;
    
    % Only consider real roots
    if discriminant < 0
        T_C = 0;
    else
        T_C = max((-B + sqrt(discriminant)) / (2 * A), (-B - sqrt(discriminant)) / (2 * A));
    end
end
% Function to calculate link reliability between two nodes
function linkreliability = calculateLR(node1, node2)
    % Constants and parameters
    r_ab = calculateRab(node1, node2); % Calculate relative transmission range
    T_C = calculateCommunicationTime(node1, node2, r_ab);

    % Define the PDF function f(T)
    f_T = @(T) r_ab / (sqrt(2 * pi) * calculateSigmaDeltaV(node1, node2)) * (1 / T^2) * ...
        exp(-((r_ab / T - meanDeltaV(node1, node2))^2) / (2 * calculateSigmaDeltaV(node1, node2)^2));

    % Integrate the PDF over the communication time period T_C
    linkreliability = integral(f_T, 0, T_C);
end


% Helper function to calculate distance between two positions
function distance = calculateDistance(position1, position2)
    distance = norm(position1 - position2);
end

% Helper function to calculate relative position
function relative_position = calculateRelativePosition(node1, node2)
    relative_position = node2.position - node1.position;
end

% Helper function to calculate relative velocity
function relative_velocity = calculateRelativeVelocity(node1, node2)
    relative_velocity = norm(node2.velocity - node1.velocity);
end








