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