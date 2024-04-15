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
    clusterHeads(i) = clusterNodes{i}(bestNodeIndex);
end

% Plot Network
figure(4);
hold on;
view(3); % Sets the view to 3D
grid on; % Add grid


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
    maxIterations = 50; % Adjust as needed
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
