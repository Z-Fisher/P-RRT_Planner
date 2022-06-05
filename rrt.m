function [pathRaw, pathOpt, plotPairs, allNodes] = rrt(boundary, obstacles, fixObst, start, goal, robot)
%
% INPUTS:
%   boundary - 1x4 array of field dimensions
%   obstacles - Nx2 array of dynamic obstacles
%   fixObst - Mx4 array of fixed obstacles
%   start   - 1x2 vector of the starting position
%   goal   - 1x2 vector of the goal position
%   robot  - radius of robot
% 
% OUTPUTS:
%   pathRaw   - before post-processing
%   pathOpt   - after post-processing
%   plotPairs - individual lines created through branching strategy
%   allNodes  - random nodes generated


%% RRT Code

% Input Parameter Descriptions
startPath = start;  % Initialize Path from Start
goalPath = goal;  % Initialize Path from Goal
N = 1000; % Number of Allowed Node Creations
threshold = 500;

xMin = boundary(1,1) + 2*robot;
xMax = boundary(1,2) - 2*robot;
yMin = boundary(1,3) + 2*robot;
yMax = boundary(1,4) - 2*robot;

plotPairs = [];
allNodes = [];

% Check if immediate connection from start to goal is possible
[immediatePath,~] = discretizePath(start, boundary, obstacles, fixObst, startPath, goal, robot);
if (size(immediatePath,1) > 1)
    n = 0;
    pathRaw = [];
    plotPairs = [start; goal];
    pathOpt = [start; goal];
else
    
    % Node Creation Loop
    for n=1:1:N
        % Make Random Node Configurations Within Environment
        x = (xMax - xMin)*rand(1,1) + xMin;
        y = (yMax - yMin)*rand(1,1) + yMin;
        q = [x, y];
        allNodes = [allNodes; q];

        % Find Closest Node in Start Path
        qA = closestNode(startPath, q);

        % Find Closest Node in Goal Path
        qB = closestNode(goalPath, q);

        % Record Size of Current Goal and Start Paths
        prevGoalSize = size(goalPath,1);
        prevStartSize = size(startPath,1);

        % Check for Collisions at/between Closest Existing Nodes and New Node
        [goalPath, ~] = discretizePath(q, boundary, obstacles, fixObst, goalPath, qB, robot);
        [startPath, ~] = discretizePath(q, boundary, obstacles, fixObst, startPath, qA, robot);

        % If the New Node Can be Added to BOTH Paths
        if ((size(goalPath,1) > prevGoalSize) && (size(startPath,1) > prevStartSize))
            goalPath = flipud(goalPath);  % Flip Order of goalPath
            path = [startPath; goalPath];  % Merge Paths from Start to Goal
            plotPairs = [plotPairs; q; qB; q; qA];
            break;  % Exit the Node Creation Loop
        else
            if (size(goalPath,1) > prevGoalSize)
                plotPairs = [plotPairs; q; qB];
            elseif (size(startPath,1) > prevStartSize)
                plotPairs = [plotPairs; q; qA];
            end
        end
        
    end
    
    % If the Paths Cannot be Merged at Max Iterations, Return Empty Path
    if (n == N)
        pathRaw = [];
        pathOpt = [];
    else 
        pathRaw = path;
        pathOpt = path;
        
        % Optimize Path ("Greedy Post-Processing") by Connecting Nodes where Able
        node = 1;  % First Configuration to Examine (start)
        while(1)  % Loop Until Broken
            
            % Check if Examining Last node in path
            if (size(pathOpt,1) == node)
                break;  % Exit Searching Loop
            end

            q = pathOpt(node,:);  % Declare Configuration being Examined

            % Search the Path After the Examined Node
            for next=node+1:1:size(pathOpt,1)
                qNext = pathOpt(next,:);  % Next Node in path
                [~, collisionFlag] = discretizePath(q, boundary, obstacles, fixObst, pathOpt, qNext, robot); % returns 1 if collision

                % If a Collision is Detected between Configurations
                if (collisionFlag == 1)
                    % If Sufficient Configurations can be Bypassed
                    if (next - node >= 3)
                        % Delete these Bypassed Configurations from Path
                        pathOpt(node+1:next-2,:) = [];
                        break;  % Exit Loop, to restart on updated path
                end
            end
            node = node + 1;  % Increment to Next Configuration in Path
        end      
    end
    end

% Print Number of Iterations Taken
fprintf('Number of Iterations: %.0f \n',n)
fprintf('Number of Configurations in Path: %.0f',size(pathOpt,1));
end

