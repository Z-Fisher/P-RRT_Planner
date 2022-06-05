function [qNext,isDone,stuck,movingObstacles,history] = potentialField(boundary,start,goal,globalGoal,obstacles,fixObst,robot,stepSize,epsilon,numRobots)
%
% INPUTS:
%   boundary - field dimensions
%   start - 1x2 array of first position of robot
%   goal - 1x2 array of desired final position of robot within this planner
%   globalGoal - 1x2 array of desired final position (after all RRT and P)
%   obstacles - Nx2 array of dynamic obstacles
%   fixObst - Mx4 array of fixed obstacles
%   robot - radius of robot
%   stepSize - distance increment for each iteration
%   epsilon - acceptable distance from goal when planner finishes
%   numRobots - number of dynamic obstacle robots
%
% OUTPUTS:
%   qNext  - 1x2 array of position at end of planner
%   isDone - a boolean flag signifying planner completion
%   stuck  - a boolean flag signifying if the planner is stuck
%   movingObstacles - Nx2 array of perturbed new dynamic obstacle positions
%   history - Array of robot's positions

%% Setup

% EPSILON: Satisfactory normalized distance to goal configuration
parabolic_conic_threshold = 150;%500;  % Distance from goal where attraction equation changes
stuck_threshold = 50;  % Norm diff between current and a past configuration to trigger stuck exit
alpha = stepSize;  % (mm) step size to next configuration
rho_0_dyn = 4.5*robot;  % (mm) Distance from Obstacle where repulsion starts
rho_0_stat = 2.5*robot;

zeta = 1;  % Attractive Field Strength (Force Per Distance)
eta = 900000000;   % (N*mm^3) Repulsive Force coefficient

goingUp = ones(numRobots);

% Initially not done and at start location
isDone = 0;
stuck = 0;
qCurr = start;
fillerPath = start;
history(1,:) = start;
iter = 2;
movingObstacles = [];

%% While not finished, take a step
while ~isDone
    %% Optimize Path if Possible (Exit RRT Intermediate Vertices)
    
    % Check for collisions between current position and global goal (ball) positions
    [~, collision] = discretizePath(qCurr, boundary, obstacles, fixObst, fillerPath, globalGoal, robot);
    
    % If Shortcut Is Possible
    if ~collision
        goal = globalGoal;
        epsilon = 30;
    end
    
    
    %% Attractive Forces

    % Attractive Forces on Each Origin
    totalDist = norm(qCurr - goal);
    if (totalDist < parabolic_conic_threshold)
        % Parabolic Well
        alpha = stepSize / 2;  % Decrease the step size when close to the goal
        F_A = -zeta * (qCurr - goal);
    else
        % Conic Well
        F_A = -(qCurr - goal) / norm(qCurr - goal);
    end


    %% Repulsive Forces

    % Repulsive Forces on Robot from Each Dynamic Obstacles
    [numDynObstacles, ~] = size(obstacles);
    F_R = 0;
    for i=1:1:numDynObstacles    

        % Distance and Unit Vector to Obstacle
        xDist = qCurr(1,1) - obstacles(i,1);
        yDist = qCurr(1,2) - obstacles(i,2);
        dist = norm(qCurr - obstacles(i,:));
        unit = [xDist, yDist] / dist;

        % Repulsive Forces
        if (dist > rho_0_dyn)             % If the joint is not repellable or is outside of repulsion zone
            F_R_obst = zeros(1,2);    % zero force vector
        elseif (dist <= 2*robot)      % Robot is in or on an obstacle
            F_R_obst = zeros(1,2);    % zero force vector
        else                          % If the robot is affected by a repulsive force
            F_R_obst = eta * ((1/dist)-(1/rho_0_dyn)) * (1/(dist*dist)) * unit;
        end

        % Sum Repulsive Forces Across All Obstacles
        F_R = F_R + F_R_obst;
    end
    
    
    % Repulsive Forces on Robot from Fixed Obstacles
    [numFixObstacles, ~] = size(fixObst);
    for i=1:1:numFixObstacles
        
        % Distance and Unit Vector to Obstacle
        [dist, unit] = distPointToBox(qCurr, fixObst(i,:));
        unit = -unit;
        
        % Repulsive Forces
        if (dist > rho_0_stat)             % If the joint is not repellable or is outside of repulsion zone
            F_R_obst = zeros(1,2);    % zero force vector
        elseif (dist <= robot)        % Robot is in or on an obstacle
            F_R_obst = zeros(1,2);    % zero force vector
        else                          % If the robot is affected by a repulsive force
            F_R_obst = eta * ((1/dist)-(1/rho_0_stat)) * (1/(dist*dist)) * unit;
        end
        
        % Sum Repulsive Forces Across All Obstacles
        F_R = F_R + F_R_obst;
    end
    
    
    % Repulsive Forces on Robot from Field Boundaries
    % Distance and Unit Vector to Obstacle
    distAll = [];
    unitAll = [];
    distAll(1) = qCurr(1,1) - boundary(1,1); % Xmin
    unitAll(1,:) = [1,0];
    distAll(2) = boundary(1,2) - qCurr(1,1); % Xmax
    unitAll(2,:) = [-1,0];
    distAll(3) = qCurr(1,2) - boundary(1,3); % Ymin
    unitAll(3,:) = [0,1];
    distAll(4) = boundary(1,4) - qCurr(1,2); % Ymax
    unitAll(4,:) = [0,-1];
    dist = min(distAll,[],'all');
    I = find(distAll == dist);
    unit = unitAll(I,:);
    
    % Repulsive Forces
    if (dist > rho_0_stat)             % If the joint is not repellable or is outside of repulsion zone
        F_R_obst = zeros(1,2);    % zero force vector
    elseif (dist <= robot)        % Robot is in or on an obstacle
        F_R_obst = zeros(1,2);    % zero force vector
    else                          % If the robot is affected by a repulsive force
        F_R_obst = eta * ((1/dist)-(1/rho_0_stat)) * (1/(dist*dist)) * unit;
    end

    % Sum Repulsive Forces from Boundary with Obstacles
    F_R = F_R + F_R_obst;


    %% Next Step
    % Sum All Forces
    F = F_A + F_R;

    % Calculate Next Position
    qNext = qCurr + alpha * F/norm(F);


    %% Termination Condition

    history(iter,:) = qNext;
    
    % If the history is of sufficient size
    if (size(history,1) > 6)
        % Compare the current with an old configuration (10 previous)
        if (norm(history(iter,:) - history(iter-6,:)) < stuck_threshold)
            disp('Robot is stuck!')
            stuck = 1;
            isDone = 1;
        end
    end

    % If the robot is not already stuck
    if (isDone ~= 1)
        % If next configuration is sufficiently close to goal configuration
        if (norm(qNext - goal) <= epsilon)
            isDone = 1;  % Terminate potentialFieldStep
        end
    end
    
    %% Overall
    
    % Code for dynamic obstacles in Experiments #3-5 {    
    % Obstacles Take a Random Step
    for j=1:1:numRobots
       step = [2*stepSize*rand(1,1) - stepSize, 2*stepSize*rand(1,1) - stepSize];
       obstacles(j,:) = obstacles(j,:) + step;
    end
    movingObstacles = [movingObstacles; obstacles];
    % }
    
    % Code for Experiments #1-2 in which robot obstacles are entirely
    % static {
    % movingObstacles = obstacles;
    %}
    
    % Take the step
    qCurr = qNext;
    iter = iter + 1;

end
end

