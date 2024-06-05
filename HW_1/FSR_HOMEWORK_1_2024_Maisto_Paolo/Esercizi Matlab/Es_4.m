%% ESERCIZIO 4 HOMEWORK 1 FSR MAISTO PAOLO
%% MATRICOLA DISPARI: P38000191

clc;
clear all;
close all;

%% Map and Points Visualization
% Load the map
load('image_map.mat');

% Definition of the start and goal points
q_start = [30, 125]; % Start point coordinates
q_goal = [135, 400]; % Goal point coordinates

% Plot initialization
figure;
imshow(image_map);
hold on;

% Plotting the start and goal points on the map
plot(q_start(2), q_start(1), 'ro', 'MarkerSize', 10);                       % Start point (red)
plot(q_goal(2), q_goal(1), 'bo', 'MarkerSize', 10);                         % Goal point (blue)

%% RRT Algorithm Implementation

% Algorithm parameters
max_iterations_per_attempt = 150;                                           % Maximum iterations per attempt
max_attempts = 5;                                                           % Maximum number of attempts
delta = 10;                                                                 % Connection threshold

% Solution variable initialization
solution_found = false;

% Loop over attempts
for attempt = 1:max_attempts
    disp(['Attempt ', num2str(attempt), ':']);
    
    % Resetting the plot and tree for each attempt
    clf;
    imshow(image_map);
    hold on;
    plot(q_start(2), q_start(1), 'ro', 'MarkerSize', 10);                   % Start point (red)
    plot(q_goal(2), q_goal(1), 'bo', 'MarkerSize', 10);                     % Goal point (blue)
    
    % Initialization of the initial node and tree
    nodes = q_start;
    parent = zeros(1, max_iterations_per_attempt);                          % Parent vector contains all random generated points, even those discarded 
    
    % Loop over iterations
    for iter = 1:max_iterations_per_attempt
        
        % Generating a random point inside the map
        q_rand = [randi(size(image_map,1)), randi(size(image_map,2))];

        % Finding the nearest node to the random point (q_rand)
        min_distance = norm(q_rand - q_start);
        nearest_idx = 1;
        for i = 1:size(nodes, 1)
            distance = norm(q_rand - nodes(i,:));
            if distance < min_distance
                nearest_idx = i;
                min_distance = distance;
            end
        end
        q_near = nodes(nearest_idx,:);

        % Calculating the direction from the nearest node to the random point (q_rand)
        direction = q_rand - q_near;
        direction = direction / norm(direction);

        % Extending the branch from q_near to q_rand with a length of delta
        q_new = round(q_near + delta * direction);

        % Checking if the new node (q_new) is inside the map and not in an obstacle
        if q_new(1) >= 1 && q_new(1) <= size(image_map,1) && q_new(2) >= 1 && q_new(2) <= size(image_map,2) && image_map(q_new(1), q_new(2)) == 1
            % Checking if the line between q_near and q_new intersects an obstacle
            intersecting_blu = false;
            line_points = round([linspace(q_near(1), q_new(1), 300); linspace(q_near(2), q_new(2), 300)]');
            for i = 1:size(line_points, 1)
                if image_map(line_points(i, 1), line_points(i, 2)) == 0
                    intersecting_blu = true;
                    break;
                end
            end

            % If there is no intersection with obstacles, add the new node (q_new)
            if ~intersecting_blu
                % Adding the new node (q_new) to the set of nodes
                nodes = [nodes; q_new];
                % Setting the parent of the new node
                parent(size(nodes, 1)) = nearest_idx;
                % Drawing a line from the nearest node (q_near) to the new node (q_new)
                line([q_near(2), q_new(2)], [q_near(1), q_new(1)], 'Color', 'b');
            end
        end

        % When reaching the last iteration
        if iter == max_iterations_per_attempt
            % Finding the node nearest to the goal point among the explored nodes
            min_distance_to_goal = inf;
            nearest_to_goal_idx = 1;
            for i = 1:size(nodes, 1)
                distance_to_goal = norm(nodes(i, :) - q_goal);
                if distance_to_goal < min_distance_to_goal
                    min_distance_to_goal = distance_to_goal;
                    nearest_to_goal_idx = i;
                end
            end

            % Calculating the line between the node nearest to the goal and the goal itself
            x_values = linspace(nodes(nearest_to_goal_idx, 2), q_goal(2), 300);
            y_values = linspace(nodes(nearest_to_goal_idx, 1), q_goal(1), 300);
            line_points = round([y_values; x_values]');
        end
    end

    %% Checking the Solution at this Attempt

    % Drawing a magenta line
    plot(x_values, y_values, 'm-');

    % Checking if the line intersects an obstacle
    intersecting_mag = false;
    for i = 1:size(line_points, 1)
        if image_map(line_points(i, 1), line_points(i, 2)) == 0
            intersecting_mag = true;
            break;
        end
    end

    % If the magenta line intersects an obstacle, regenerate the points
    if intersecting_mag
        disp('Intersection detected in magenta line. NEXT ATTEMPT!');
        continue; % Move to the next iteration cycle
    else
        disp('SOLUTION FOUND!');
        solution_found = true;
        break;
    end

end

%% If Solution is Found (i.e. Magenta Line Doesn't Intersect Obstacles) Draw Optimal Path in Red

% Check if the solution is found
if solution_found
    % Drawing the optimal path in red
    current_node = nearest_to_goal_idx;
    while current_node ~= 1
        line([nodes(current_node, 2), nodes(parent(current_node), 2)], [nodes(current_node, 1), nodes(parent(current_node), 1)], 'Color', 'r', 'LineWidth', 2);
        current_node = parent(current_node);
    end

    disp(['Optimal path found at the Attempt number: ', num2str(attempt) '.']);

end

% If no solutions are found with a fixed number of attempts
if attempt == max_attempts && ~solution_found
    disp(['FAILURE! Optimal path is not found with ', num2str(attempt), ' Attempts.'])
end

%% Adding Explored Points to the Plot
hold on
plot(nodes(:, 2), nodes(:, 1), 'k.', 'MarkerSize', 4);
