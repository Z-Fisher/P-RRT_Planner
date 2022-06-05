function [isCollided] = isRobotCollided(q, obstacles, fixObst, robot)
%
% INPUTS:
%   q   - a 1x2 configuration of the robot
%   obstacles - Nx2 dynamic obstacle positions
%   fixObst - fixed obstacle positions
%   robot - radius of robot
%
% OUTPUTS:
%   isCollided - a boolean flag: 1 if the robot is in collision, 0 if not.


%% Robot Collision Detection

% Parameters
isCollided = 0;
numDynObst = size(obstacles,1);
numFixObst = size(fixObst,1);

% Loop for Each Fixed Obstacle to Check for Collision
for i=1:1:numFixObst
    obstacle = fixObst(i,:);
    [dist, unit] = distPointToBox(q, obstacle);
    unit = -unit;
    if (dist <= 2*robot)
        isCollided = 1;
        break;
    end
end

% This section of code was not used in Experiment #5 so that RRT would
% not be hindered by dynamic obstacles in narrow passages. {
if (~isCollided)
    % Loop for Each Dynamic Obstacle to Check for Collision
    for i=1:1:numDynObst
        obstacle = obstacles(i,:);
        dist = norm(q - obstacle);
        if (dist <= 2*robot)
            isCollided = 1;
            break;
        end
    end
end
%}

end