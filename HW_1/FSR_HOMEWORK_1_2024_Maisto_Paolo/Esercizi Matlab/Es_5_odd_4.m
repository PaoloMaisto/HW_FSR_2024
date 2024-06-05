%% ESERCIZIO 5 HOMEWORK 1 FSR MAISTO PAOLO
%% MATRICOLA DISPARI: P38000191
%% 4 ADJACENT CELLS

%% MAP
clc; clear;

% Matrix
rows = 9;
columns = 12;

% Matrix initialization
G = zeros(rows, columns);

% Obstacles
G(1,:) = 99; G(9,:) = 99; G(:,1) = 99; G(:,12) = 99;
G(2,2) = 99; G(2,5) = 99; G(2,6) = 99; G(2,11) = 99;
G(3,6) = 99; G(3,11) = 99; 
G(4,3) = 99; G(4,4) = 99; G(4,11) = 99;
G(5,4) = 99; 
G(6,4) = 99; G(6,7) = 99; G(6,8) = 99; G(6,9) = 99; 
G(7,5) = 99; G(7,6) = 99; G(7,7) = 99;
q_G =  0;

% Coordinates of the starting cell, q_g
row_affected = 5;
column_affected = 11;

% Counter for the value to be assigned to adjacent cells
increment_value = 1;

% Initialization of cells to be examined
cell_to_examine = [row_affected, column_affected];

% Examine all cells
while ~isempty(cell_to_examine)
   
    % New cells to examine
    new_cell_to_examine = [];
    
    % Iteration through the cells to be examined
    for i = 1:size(cell_to_examine, 1)
        row_affected = cell_to_examine(i, 1);
        column_affected = cell_to_examine(i, 2);
        
        % Definition of adjacent cells
        for direction = 1:4
            if direction == 1
                row_adjacent = row_affected - 1;
                column_adjacent = column_affected;
            elseif direction == 2
                row_adjacent = row_affected + 1;
                column_adjacent = column_affected;
            elseif direction == 3
                row_adjacent = row_affected;
                column_adjacent = column_affected + 1;
            else
                row_adjacent = row_affected;
                column_adjacent = column_affected - 1;
            end
            
            % Check if adjacent cell is within matrix limits
            if (G(row_adjacent, column_adjacent) == 0 && G(row_affected,column_affected)~=99 )  % check collision

                % Assign incremented value to adjacent cell
                G(row_adjacent, column_adjacent) = increment_value;

                % Add the cell adjacent to the list of new cells to examine
                new_cell_to_examine = [new_cell_to_examine; row_adjacent, column_adjacent];

            end
        end
    end
    
    % Increments the value to be assigned to adjacent cells
    increment_value = increment_value +1 ;
    
    % Update the list of cells to examine for next increment
    cell_to_examine = new_cell_to_examine;

    % Definition of q_G
    G(5,11)=q_G;
    
end

% View the obtained matrix
disp('Matrix G:');
disp(G);

%% PATH
% Define the starting and arrival cells
starting = [5, 11];         %coordinates q_g
arrival = [5, 2];           %coordinates q_s

% Initialization of variables
visited = zeros(size(G));                      % Define the matrix where the cell takes value 1 if the cell has been visited
queue = {starting};
predecessors = containers.Map('KeyType', 'double', 'ValueType', 'any');

flag_queue = true;

% BFS
while flag_queue

    current_cell = queue{1};
    queue = queue(2:end);
    row = current_cell(1);
    column = current_cell(2);

    % If the cell is visited it takes value 1
    visited(row, column) = 1;

    % Defines the vector of adjacent cells to check
    adjacent = [row-1, column; row+1, column; row, column-1; row, column+1];

    % Check cells
    for i = 1:size(adjacent, 1)
        row_adjacent = adjacent(i, 1);
        column_adjacent = adjacent(i, 2);

        if ((G(row_adjacent, column_adjacent) ~= 99) && (visited(row_adjacent, column_adjacent) ~= 1))
            queue{end+1} = [row_adjacent, column_adjacent];
            predecessors(row_adjacent + column_adjacent * size(G, 1)) = [row, column];
        end

    end

    if isequal([row, column], arrival)
        flag_queue = false;
    end
end

% Build the path
path = [arrival];
visited(arrival(1),arrival(2)) = 10;
while ~isequal(path(1,:), starting)
    previous_cell = predecessors(path(1,1) + path(1,2) * size(G, 1));
    path = [previous_cell; path];
    visited(previous_cell(1),previous_cell(2)) = 10;
end

% Matrix of visited cells
disp('Matrix of visited cells:');
disp(visited);

% View the path found
disp('Path from q_g to q_s:');
disp(path);

%% Graphic representation of the map
figure;
hold on;

% Examine each cell of the matrix
for row = 1:rows
    for col = 1:columns

        % Draw borders to cells
        rectangle('Position',[col-1, rows-row, 1, 1],'EdgeColor','k','FaceColor','w');

        % If cell value is 99, draw a black cell
        if G(row, col) == 99
            rectangle('Position',[col-1, rows-row, 1, 1],'FaceColor','k');

        % If the cell value is 10, write the number in red
        elseif visited(row, col) == 10
            text(col-0.5, rows-row+0.5, num2str(G(row, col)), 'HorizontalAlignment', 'center', 'Color', 'r');

        % If the cell value is different from zero and not 99, draw a white cell with numeric value
        elseif G(row, col) ~= 0
            text(col-0.5, rows-row+0.5, num2str(G(row, col)), 'HorizontalAlignment', 'center');
            
        end
    end
end

% Set the axes
axis equal;
axis([0 columns 0 rows]);