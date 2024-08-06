function [route,numExpanded] = AStarGrid (input_map, start_coords, dest_coords,drawMapEveryTime)
% Run A* algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%   the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%   respectively, the first entry is the row and the second the column.
% Output :
%    route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route. This is a single dimensional vector
%    numExpanded: Remember to also return the total number of nodes
%    expanded during your search. Do not count the goal node as an expanded node. 

% Set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination
% 7 - gray - route

cmap = [1 1 1; ...
    0 0 0; ...
    1 0 0; ...
    0 0 1; ...
    0 1 0; ...
    1 1 0; ...
    0.5 0.5 0.5];

colormap(cmap);

[nrows, ncols] = size(input_map);

% Initialize map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;   % Mark free cells
map(input_map)  = 2;   % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% Initialize parent array
parent = zeros(nrows,ncols);

% Create grid of coordinates
[X, Y] = meshgrid (1:ncols, 1:nrows);

xd = dest_coords(1);
yd = dest_coords(2);

% Evaluate Heuristic function, H, for each grid cell
% Using Manhattan distance
H = abs(X - xd) + abs(Y - yd);
H = H';

% Initialize cost arrays
f = Inf(nrows,ncols);
g = Inf(nrows,ncols);

g(start_node) = 0;
f(start_node) = H(start_node);

% Initialize counter for expanded nodes
numExpanded = 0;

% Main Loop
while true
    
    % Update map display
    map(start_node) = 5;
    map(dest_node) = 6;
    
    % Draw current map if drawMapEveryTime is true
    if (drawMapEveryTime)
        image(1.5, 1.5, map);
        grid on;
        axis image;
        drawnow;
    end
    
    % Find the node with the minimum f value
    [min_f, current] = min(f(:));
    
    % Check if destination is reached or if there's no path
    if ((current == dest_node) || isinf(min_f))
        break;
    end;
    
    % Mark current node as visited
    map(current) = 3;
    f(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(f), current);
    
    % Visit neighbors of the current node
    neighbors = [i-1, j; i+1, j; i, j-1; i, j+1]; % north, south, west, east
    for k = 1:size(neighbors, 1)
        ni = neighbors(k, 1);
        nj = neighbors(k, 2);
        
        % Check if neighbor is valid and not an obstacle or already visited
        if ni > 0 && ni <= nrows && nj > 0 && nj <= ncols && map(ni, nj) ~= 2 && map(ni, nj) ~= 3
            neighbor_index = sub2ind(size(map), ni, nj);
            tentative_g = g(current) + 1;
            
            % If this path to neighbor is better, update neighbor
            if tentative_g < g(neighbor_index)
                g(neighbor_index) = tentative_g;
                f(neighbor_index) = g(neighbor_index) + H(neighbor_index);
                parent(neighbor_index) = current;
                map(ni, nj) = 4; % Mark as on list
            end
        end
    end
    
    % Increment number of expanded nodes
    numExpanded = numExpanded + 1;
end

% Construct route from start to dest by following the parent links
if (isinf(f(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end

    % Visualize the map and the path
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.1);
        image(1.5, 1.5, map);
        grid on;
        axis image;
    end
end

end