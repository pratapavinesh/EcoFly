%parameter for cluster based routing
communicationRangeForHead = 100;
%define the nighbourhood for clusterheads ( in real this can be done easily
%by sending hellow message )

% Initialize cell array to store blind nodes for each cluster
nighboureCHs = cell(numClusters, 1);
for i = 1:numClusters
    for j = 1:numClusters
        if calculateDistance(clusterHeads(i).position,clusterHeads(j).position ...
                )<= communicationRangeForHead && i~=j
          nighboureCHs{i} = [nighboureCHs{i}, clusterHeads(j)];
        end
    end
end

%calculate next hop for each ch
selectedNextHops = repmat(struct('id', [], 'position', [], 'energy', [], 'isCH', [], ...
    'velocity',[]), numAnchorNodes, 1);
for i = 1: numClusters
  temp = findselectedNextHop(i, nighboureCHs, clusterHeads, bsPosition, xBounds, ...
      yBounds, zBounds, velocityMean , velocityStdDev,communicationRangeForHead);
  selectedNextHops(i).position = temp.position(1:3); %first 3 dimesions of position repersent postion
  selectedNextHops(i).velocity = temp.position(4:6); %next 3 dimesions of position repersent velocity
end

%find the actual next hop node
for i = 1:numClusters
    nodes = nighboureCHs{i};
    bestNode = nodes(1); % Assume the first node is the best initially

    for j = 1:length(nodes)
        distance = calculate6DimesnsionalDistance(nodes(j), selectedNextHops(i));
        if distance < calculate6DimesnsionalDistance(bestNode, selectedNextHops(i))
            bestNode = nodes(j);
        end    
    end
    selectedNextHops(i) = bestNode;
end

% Plotting
figure(6);
hold on;
view(3); % Sets the view to 3D
grid on; % Add grid

% Plot Cluster Head nodes and arrows to next hops
for i = 1:numClusters
    clusterHead = clusterHeads(i);
    
    % Plot Cluster Head node
    scatter3(clusterHead.position(1), clusterHead.position(2), clusterHead.position(3), ...
        'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', colors(i), 'DisplayName', ['CH ' num2str(i)]);
    
    quiver3(clusterHead.position(1), clusterHead.position(2), clusterHead.position(3), ...
        selectedNextHops(i).position(1) - clusterHead.position(1), ...
        selectedNextHops(i).position(2) - clusterHead.position(2), ...
        selectedNextHops(i).position(3) - clusterHead.position(3), ...
        'Color', colors(i), 'LineStyle', '-', 'LineWidth', 2, 'MaxHeadSize', 0.5);
end

% Plot Base Station position
scatter3(bsPosition(1), bsPosition(2), bsPosition(3), 'd', 'filled', 'DisplayName', 'Base Station', 'MarkerEdgeColor', 'k');

% Add legends for scatter plots only
legend(findobj(gca,'Type','scatter'), 'Location', 'Best');
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
      b* calculateDistance(node1.velocity , node2.velocity);
end

% Function to select CH nodes using IPSO algorithm (Algorithm 3)
function selectedNextHop = findselectedNextHop(clusterIndex, nighboureCHs, ...
    clusterHeads, bsPosition,xBounds, yBounds,zBounds,velocityMean , ...
    velocityStdDev,communicationRangeForHead)
    % Initialization
    numParticles = 10;   % Number of particles 
    maxIterations = 50;  % Adjust as needed

    % PSO Initialization
    particles = initializeParticles(numParticles, xBounds, yBounds, zBounds, ...
        velocityMean , velocityStdDev);  % Initialize particles
    Pbest = particles;                                                         % Personal best positions
    Gbest = getGlobalBest(Pbest, clusterIndex, nighboureCHs, clusterHeads, ...
        bsPosition,communicationRangeForHead, velocityMean , velocityStdDev);       % Global best position
    
    % PSO Iterations
    for itr = 1:maxIterations
        for i = 1:numParticles
            % Update velocities and positions using PSO equations (with energy movement)
            particles(i) = updateParticle(particles(i), Pbest(i), Gbest);
            particles(i) = insureWithinBoundary(particles(i), xBounds, yBounds, zBounds);
            
            % Evaluate fitness function using Equation (31) with combined metric
            fitnessValue = evaluateFitness(particles(i), clusterIndex, ...
                nighboureCHs, clusterHeads, bsPosition,communicationRangeForHead, ...
                velocityMean , velocityStdDev);
            
            % Update pBest and gBest based on fitness values
            if fitnessValue < evaluateFitness(Pbest(i), clusterIndex, ...
                    nighboureCHs, clusterHeads, bsPosition,communicationRangeForHead, ...
                    velocityMean , velocityStdDev)
                Pbest(i)= particles(i);
            end
            if fitnessValue < evaluateFitness(Gbest, clusterIndex, ...
                    nighboureCHs, clusterHeads, bsPosition, communicationRangeForHead, ...
                    velocityMean , velocityStdDev)
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
function Gbest = getGlobalBest(Pbest, clusterIndex, nighboureCHs, clusterHeads, ...
    bsPosition, communicationRangeForHead, velocityMean , velocityStdDev)
       Gbest = Pbest(1);
       for i = 1: length(Pbest)
           if evaluateFitness(Pbest(i), clusterIndex, nighboureCHs, ...
                   clusterHeads, bsPosition, communicationRangeForHead, ...
                   velocityMean , velocityStdDev)...
               < evaluateFitness(Gbest, clusterIndex, nighboureCHs, ...
               clusterHeads, bsPosition, communicationRangeForHead, ...
               velocityMean , velocityStdDev)
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
     % Update position
     particle.position = particle.position + particle.velocity;
end

%Function to insure that particle don't go outside of boundary position
function updatedparticle = insureWithinBoundary(particle,xBounds, yBounds, zBounds)
          bounds = [xBounds; yBounds; zBounds];  % Assuming bounds is a 3x2 matrix where each row i contains the lower and upper bounds for the i-th dimension
          for i = 1:3
            if particle.position(i) < bounds(i,1) || particle.position(i) > bounds(i,2)
                particle.position(i) = randi([bounds(i,1), bounds(i,2)]);
            end
          end
        updatedparticle = particle;
end

% Helper function to evaluate fitness function with combined metric
function fitnessValue = evaluateFitness(particle, clusterIndex, nighboureCHs, ...
    clusterHeads, bsPosition,communicationRangeForHead,velocityMean , velocityStdDev)
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
    for i = 1:length(clusterHeads)
        if bestNode.position == clusterHeads(i).position
            bestNodeIndex = i;
        end
    end
     % weight parameter
     alpha = 0.7;
     beta = 0.3;
    fitnessValue =  + beta * f2(nighboureCHs{bestNodeIndex}, bsPosition)+ ...
    alpha * f1(nighboureCHs{bestNodeIndex}, bestNode,communicationRangeForHead, ...
    velocityMean , velocityStdDev);
end

% Helper function to evaluate fitness function 1
function fitnessValue = f1(nighboures, bestNode, communicationRangeForHead, ...
    velocityMean , velocityStdDev)
    averageLR=0;
    for i = 1: length(nighboures)
        averageLR = averageLR + calculateLR(nighboures(i), bestNode, ...
            communicationRangeForHead, velocityMean , velocityStdDev);
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


% Helper function to calculate mean relative velocity
function mean_delta_v = meanDeltaV(node1, node2, velocityMean , velocityStdDev)
    % Implement Equation (36)
    mean_delta_v = velocityMean;
end

% Helper function to calculate standard deviation of relative velocity
function sigma_delta_v = calculateSigmaDeltaV(node1, node2,velocityMean , velocityStdDev)
    % You may adjust the range based on your system characteristics
    sigma_delta_v = velocityStdDev;
end


% Helper function to calculate relative transmission range
function r_ab = calculateRab(node1, node2, communicationRangeForHead)
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
    A = norm(node1.velocity-node2.velocity)^2;
    B = 2 * dot(calculateRelativePosition(node1, node2), ...
        calculateRelativeVelocity(node1, node2));
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
function linkreliability = calculateLR(node1, node2, communicationRangeForHead,velocityMean , velocityStdDev)
    % Constants and parameters
    r_ab = calculateRab(node1, node2, communicationRangeForHead); % Calculate relative transmission range
    T_C = calculateCommunicationTime(node1, node2, r_ab);

    % Define the PDF function f(T)
    sigma = calculateSigmaDeltaV(node1, node2,velocityMean , velocityStdDev);
    mu = meanDeltaV(node1, node2,velocityMean ,velocityStdDev);
    coefficient = r_ab / (sqrt(2 * pi)* sigma);
    f_T = @(T) coefficient * (1 / T^2) *...
        exp(-(r_ab / T - mu )^2 / (2 * sigma^2));

    % Integrate the PDF over the communication time period T_C
    linkreliability = integral(f_T, 1, T_C+1,'ArrayValued', true);
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
    relative_velocity = node2.velocity - node1.velocity;
end

