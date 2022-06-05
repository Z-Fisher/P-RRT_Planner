function [path, collision] = discretizePath(q, boundary, obstacles, fixObst, path, closestNode, robot)
%
% INPUTS:
%   q   - 1x2 position to be added to tree path if possible
%   boundary - dimensions of field
%   obstacles - dynamic obstacle positions
%   fixObst - fixed obstacle positions
%   path - Nx2 matrix consisting of cumulative positions so far
%   closestNode - 1x2 array of closest position within tree
%   robot - radius of robot
%
% OUTPUTS:
%   path  - Mx2 matrix with potential position addition
%   collision - an integer. 1 indicates collision. 0 indicates NO collision


%% Discretize Path Between Random Node and Closest Node

collision = 0;  % No collisions detected as of start of each loop
numSteps = 100; % discretized steps between random node and closest

% Determine Current Number of Configurations Up to qB
[configs, ~] = size(path);

% Check For Collisions with Obstacles
if (isRobotCollided(q, obstacles, fixObst, robot) == 0)  % NO Collision at New Node

    % Discretize Path Between Existing Node and q to Check Intermediate Collisions
    incr = (q - closestNode)/numSteps;
    for j=1:1:numSteps-1
        discreteQ = closestNode + (incr*j);
        if (isRobotCollided(discreteQ, obstacles, fixObst, robot) == 1)
            collision = 1;
           break;
        end
    end
    
    % If No Collisions, Add the Configuration to the Path
    if (collision == 0)
        path(configs+1,:) = q;
    else  % If Collision, Return the Original Path
        path = path;
    end
    
else  % New node is in collision
    path = path;
    collision = 1;
end
        
end