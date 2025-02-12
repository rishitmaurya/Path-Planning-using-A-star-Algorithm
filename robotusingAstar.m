clc; clear; close all;

% Step 1: Create a 30x30 grid map
gridSize = 30;
map = zeros(gridSize);  % Empty map

% Step 2: Add Obstacles
map(15, 10:20) = 1;  % First horizontal wall
map(10, 15:25) = 1;  % Second horizontal wall
map(5:10, 20) = 1;    % Vertical wall
% map(20, 20:29) = 1;

% Step 3: Define Start and Goal Locations
startLocation = [2, 2];      % Start Location (row, col)
goalLocation = [28, 28];     % Goal Location (row, col)

% Step 4: Find Path using A* Algorithm
path = AStar(map, startLocation, goalLocation);

% Step 5: Plot the Grid, Obstacles, and Path
figure;
hold on;
imagesc(~map);
colormap(gray);
grid on;
axis equal;
title('A* Path and Differential Drive Robot Simulation');

% Mark Start and Goal
plot(startLocation(2), startLocation(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalLocation(2), goalLocation(1), 'bo', 'MarkerSize', 10, 'LineWidth', 2);

% Plot the Path
if ~isempty(path)
    plot(path(:,2), path(:,1), 'r', 'LineWidth', 2);
else
    disp('No Path Found!');
    return;
end

% Step 6: Simulate Differential Drive Robot on Same Figure
robotPlot = plot(startLocation(2), startLocation(1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

DifferentialDriveSimulation(path);

%% A* Algorithm Function
function path = AStar(grid, startNode, goalNode)
    % Define movement directions (8 possible moves)
    directions = [ 0  1;  1  0;  0 -1; -1  0;  1  1;  1 -1; -1  1; -1 -1];
    
    openList = startNode;
    cameFrom = zeros(size(grid,1), size(grid,2), 2);
    gScore = inf(size(grid));  
    fScore = inf(size(grid));  

    gScore(startNode(1), startNode(2)) = 0;
    fScore(startNode(1), startNode(2)) = heuristic(startNode, goalNode);

    while ~isempty(openList)
        [~, idx] = min(fScore(sub2ind(size(grid), openList(:,1), openList(:,2))));
        current = openList(idx, :);

        if isequal(current, goalNode)
            path = reconstructPath(cameFrom, current);
            return;
        end

        openList(idx, :) = [];
        fScore(current(1), current(2)) = inf;

        for i = 1:size(directions,1)
            neighbor = current + directions(i, :);

            if neighbor(1) > 0 && neighbor(1) <= size(grid,1) && ...
               neighbor(2) > 0 && neighbor(2) <= size(grid,2) && ...
               grid(neighbor(1), neighbor(2)) == 0
                
                tentative_gScore = gScore(current(1), current(2)) + 1;

                if tentative_gScore < gScore(neighbor(1), neighbor(2))
                    cameFrom(neighbor(1), neighbor(2), :) = current;
                    gScore(neighbor(1), neighbor(2)) = tentative_gScore;
                    fScore(neighbor(1), neighbor(2)) = tentative_gScore + heuristic(neighbor, goalNode);

                    if ~ismember(neighbor, openList, 'rows')
                        openList = [openList; neighbor];
                    end
                end
            end
        end
    end
    path = [];
end

%% Heuristic Function (Euclidean Distance)
function h = heuristic(node, goal)
    h = sqrt((goal(1) - node(1))^2 + (goal(2) - node(2))^2);
end

%% Path Reconstruction Function
function path = reconstructPath(cameFrom, current)
    path = current;
    while any(cameFrom(current(1), current(2), :))
        current = squeeze(cameFrom(current(1), current(2), :))';
        path = [current; path];
    end
end

%% Differential Drive Simulation Function
function DifferentialDriveSimulation(path)
    % Robot Parameters
    wheelBase = 2;   % Distance between wheels
    robotLength = 3; % Robot body length
    robotWidth = 2;  % Robot body width
    dt = 0.1;        % Time step
    maxSpeed = 1.0;  % Max linear speed

    % Initial Robot Pose
    x = path(1,2); % X-coordinate
    y = path(1,1); % Y-coordinate
    theta = 0;     % Orientation

    % Create Figure for Robot
    hold on;
    
    % Draw Initial Robot as a Rectangle
    robotBody = drawRobot(x, y, theta, robotLength, robotWidth);

    % Threshold for reaching goal
    goalThreshold = 0.5; 

    % Move along path
    i = 2;
    while i <= size(path,1)
        % Extract next waypoint
        xTarget = path(i,2);
        yTarget = path(i,1);

        % Compute desired heading
        thetaDesired = atan2(yTarget - y, xTarget - x);

        % Compute error in heading
        thetaError = thetaDesired - theta;

        % Normalize angle to [-pi, pi]
        thetaError = mod(thetaError + pi, 2*pi) - pi;

        % Set linear velocity
        v = min(maxSpeed, sqrt((xTarget - x)^2 + (yTarget - y)^2) / dt);

        % Set angular velocity using proportional control
        Kp = 2.0; % Proportional gain for angular control
        omega = Kp * thetaError;

        % Compute wheel velocities
        vL = v - omega * wheelBase / 2;
        vR = v + omega * wheelBase / 2;

        % Update robot pose using differential drive kinematics
        theta = theta + omega * dt;
        x = x + v * cos(theta) * dt;
        y = y + v * sin(theta) * dt;

        % Update Robot's Visualization
        updateRobot(robotBody, x, y, theta, robotLength, robotWidth);
        pause(0.1);

        % Move to the next waypoint when close to current one
        if sqrt((xTarget - x)^2 + (yTarget - y)^2) < goalThreshold
            i = i + 1;
        end
    end

    % Mark final position
    plot(x, y, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
end

%% Function to Draw the Differential Drive Robot
function robotBody = drawRobot(x, y, theta, length, width)
    % Define robot body as a rectangle centered at (0,0)
    bodyX = [-length/2, length/2, length/2, -length/2];
    bodyY = [-width/2, -width/2, width/2, width/2];

    % Rotate and translate the body
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    transformedBody = R * [bodyX; bodyY];

    % Draw robot body
    robotBody = patch(transformedBody(1,:) + x, transformedBody(2,:) + y, 'b');
end

%% Function to Update Robot's Position
function updateRobot(robotBody, x, y, theta, length, width)
    % Define robot body as a rectangle centered at (0,0)
    bodyX = [-length/2, length/2, length/2, -length/2];
    bodyY = [-width/2, -width/2, width/2, width/2];

    % Rotate and translate the body
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    transformedBody = R * [bodyX; bodyY];

    % Update robot body position
    set(robotBody, 'XData', transformedBody(1,:) + x, 'YData', transformedBody(2,:) + y);
end
