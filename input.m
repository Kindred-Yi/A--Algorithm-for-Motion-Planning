map = false(10); %Input map parameters
map (1:5, 6) = true; %Obstacle Declaration
start_coords = [6, 2]; %Starting coordinates
dest_coords  = [8, 9]; %Destination Coordinates
drawMapEveryTime = false; %Display Outputs
[route, numExpanded] = AStarGrid(map, start_coords, dest_coords,drawMapEveryTime) %Implementation