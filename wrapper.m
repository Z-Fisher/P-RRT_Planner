clc; clear all;

%% Input Parameters
numRobots = 9;  % Other robots on the field
robot = 200;  % Radius of Robots (mm)
stepSize = 50; % Max Speed (mm/s) (based on simulation iterations)
goal = [7500,3000];  % Target Position (Ball)
globalGoal = goal;
start = [1000,3500];  % Start Position (Our Robot)


%% Fixed Obstacle Creation
fixObst = [];

% Code for Experiment #5 {
% Format:(Xmin Ymin Xmax Ymax)
%fixObst(1,:) = [2500 2500 3000 6000];
%rect(1,:) = [fixObst(1,1:2) fixObst(1,3)-fixObst(1,1) fixObst(1,4)-fixObst(1,2)];
%}


%% Environment Constraints (RoboCup Soccer Field Dimensions) 
xMin = 0;
xMax = 9000;
yMin = 0;
yMax = 6000;
boundary = [xMin xMax yMin yMax]; % (mm) Origin at bottom left corner


%% Dynamic Obstacle Creation
% Spawning Field Dimensions (Boundary shrunk by robot radius)
xMaxS = xMax - robot;
yMaxS = yMax - robot;
xMinS = xMin + robot;
yMinS = yMin + robot;

% Initialize
obstacles = zeros(numRobots,2);

% List of points where not to spawn
spawnList = [start;goal;obstacles];

% Initial Obstacle Creation
i = 1;
while(i <= numRobots)
    % Clear Collision Flag
    collision = 0;
    
    % Generate a Random Point within Spawning Field
    x = (xMaxS - xMinS)*rand(1,1) + xMinS;
    y = (yMaxS - yMinS)*rand(1,1) + yMinS;
    
    % Check Proximity to Existing Dynamic Obstacles, Start, and Goal
    for j=1:1:2+numRobots
        dist = abs(sqrt((x - spawnList(j,1))^2 + (y - spawnList(j,2))^2));
        if (dist <= 2.5*robot)
            collision = 1;
            break;
        end
    end
    
    % Check Proximity to Fixed Obstacles
    for j=1:1:size(fixObst,1)
        [distToBox, unit] = distPointToBox([x,y], fixObst(j,:));
        if (distToBox <= 2.5*robot)
            collision = 1;
            break;
        end
    end
    
    % If No Collisions, Add the Point to the Obstacle List
    if (collision == 0)
        obstacles(i,:) = [x,y]; 
        i = i + 1;
        % Updates spawnList with new obstacles
        spawnList = [start;goal;obstacles]; 
    end
end


%% Manual Dynamic Obstacle Creation (Optional)
% Use for Special Case Scenarios / Examination. Removes all randomly
% created obstacles from previous code section.
%obstacles = [];
%obstacles = [1000, 1000];


%% RRT
disp("RRT: ");
tic;
[pathRaw1,pathOpt1,plotPairs1,allNodes1] = rrt(boundary, obstacles, fixObst, start, goal, robot);
fprintf('\n');
toc;

% Length of Path
numLines = size(pathOpt1,1)
length_RRT = 0;
for i=2:1:numLines
    length_RRT = length_RRT + abs(norm(pathOpt1(i,:) - pathOpt1(i-1,:)));
end
length_RRT


%% Potential Field 
disp("APF: ");
tic;
epsilon = 30;
[qNext1,isDone1,stuck1,movingObstacles1,history1] = potentialField(boundary,start,goal,globalGoal,obstacles,fixObst,robot,stepSize,epsilon,numRobots);
fprintf('\n');
toc;

% Length of Path
numSteps = size(history1,1)
length_APF = 0;
for i=2:1:numSteps
    length_APF = length_APF + abs(norm(history1(i,:) - history1(i-1,:)));
end
length_APF


%% P-RRT
disp("P-RRT:");
tic;
% First Case Parameters: when robot has never been stuck
interimStart = start;
interimGoal = goal;
globalGoal = goal;
pathOptAll = [];
plotPairsAll = [];
stuckCount = 0;
framesWhenStuck = [];

% Potential Field Used as Planner First
epsilon = 30;  % Satisfactory Distance from Position to Goal Where Planner Stops
[qNext,isDone,stuck,movingObstacles,history] = potentialField(boundary,interimStart,interimGoal,globalGoal,obstacles,fixObst,robot,stepSize,epsilon,numRobots);

% Loop When the First Potential Field is Stuck
while stuck
    
    % Record Each Time the Planner is Stuck
    stuckCount = stuckCount + 1;
    framesWhenStuck(stuckCount) = size(history,1);
    
    % When Stuck State Occurs: Compute RRT to Escape Local Minima
    disp("RRT: Calculating a Global Path")
    if (stuckCount == 1)
        [pathRaw,pathOpt,plotPairs,allNodes] = rrt(boundary, obstacles, fixObst, qNext, goal, robot);
        plotPairs = [];
    else
        [pathRaw,pathOpt,plotPairs,allNodes] = rrt(boundary, obstacles, fixObst, qNext, goal, robot);
    end
    
    % Append Plotting Datasets for Each RRT Path
    plotPairsAll = [plotPairsAll; plotPairs];
    pathOptAll = [pathOptAll; pathOpt];
    plotPairsLengths(stuckCount) = size(plotPairsAll,1);
    RRTpathLengths(stuckCount) = size(pathOptAll,1);
    pathReal = pathOpt;
    numSegs = size(pathOpt,1);

    % Loop Through RRT Vertices
    for seg=2:1:numSegs
        
        % Individual Potential Field Setup Parameters
        interimStart = pathReal(seg-1,:);
        interimGoal = pathReal(seg,:);
        
        % If Running Pot Field on to Last RRT Vertex
        if (seg == numSegs)
            epsilon = 30;  % Require Higher Accuracy for Completion 
        % If Running Pot Field on Intermediate RRT Vertex
        else 
            epsilon = 600; % Require Lower Accuracy for Completion
        end
        
        % Run Pot Field
        [qNext,isDone,stuck,newObstacles,newHistory] = potentialField(boundary,interimStart,interimGoal,globalGoal,obstacles,fixObst,robot,stepSize,epsilon,numRobots);
        
        % Update Actual Motion Path
        if (isDone)
            pathReal(seg,:) = qNext;
        end
        
        % Append Datasets for New Motions (Obstacles and Our Bot)
        movingObstacles = [movingObstacles; newObstacles];
        history = [history; newHistory];
        
        % If Stuck AGAIN, Exit Loop and Recompute RRT from this Position
        if stuck
            break;
        end
    end
    
end

fprintf('\n');
toc;

% Length of Path
numSteps = size(history,1)
length_PRRT = 0;
for i=2:1:numSteps
    length_PRRT = length_PRRT + abs(norm(history(i,:) - history(i-1,:)));
end
length_PRRT


%% Simulation: RRT

writerRRT = VideoWriter('RRT_Video.avi');
writerRRT.FrameRate = 3;
open(writerRRT);

% Creates Field
figure(1);
title('RRT Planner');
xlabel('Field Length (mm)');
ylabel('Field Width (mm)');
daspect([1 1 1]);  % Ensures aspect ratio
axis(boundary);  % Adds axis dimensions
rectangle('Position',[0 0 9000 6000],'FaceColor',[1 1 1],'EdgeColor','k','LineWidth',3);  % field

% Creates Notable Points
viscircles(start,50,'Color','g');  % Starting reference point
viscircles(goal,50,'Color','r');  % Desired final point
viscircles(start,robot,'Color','b');  % Our robot

% Fixed Obstacles
for i=1:1:size(fixObst,1)
    fixObst(i,3:4) = fixObst(i,3:4) - fixObst(i,1:2);
    rectangle('Position',fixObst(i,:),'FaceColor',[0,0,0],'EdgeColor',[0,0,0]);
end

% Draws Dynamic Obstacle Robots at Time=0 (Treated as Stagnant)
for i=1:1:numRobots
    viscircles(obstacles(i,:),robot,'Color','k');
end

% Draw Nodes
[numNodes,~] = size(allNodes1);
for i=1:1:numNodes
    viscircles(allNodes1(i,:),10,'Color','k');  
end

% Draw Branches
[pairs,~] = size(plotPairs1);
for i=1:2:pairs-1 
    line([plotPairs1(i,1), plotPairs1(i+1,1)], [plotPairs1(i,2), plotPairs1(i+1,2)],'Color','r','LineWidth',1,'LineStyle','--');
    frame = getframe(gcf);
    writeVideo(writerRRT,frame);
    pause(.2);
end

% Draws our robot's path after post-processing
[checkpoints,~] = size(pathOpt1);
for i=1:1:checkpoints-1 
    line([pathOpt1(i,1), pathOpt1(i+1,1)], [pathOpt1(i,2), pathOpt1(i+1,2)],'Color','b','LineWidth',1,'LineStyle','--');
end

close(writerRRT);


%% Simulation: Potential Fields

% Create Field
figure(2);
title('Potential Field Planner');
xlabel('Field Length (mm)');
ylabel('Field Width (mm)');
daspect([1 1 1]);  % Ensures aspect ratio
axis(boundary);  % Adds axis dimensions
rectangle('Position',[0 0 9000 6000],'FaceColor',[1 1 1],'EdgeColor','k','LineWidth',3); 

% Creates Notable Points
hold on
viscircles(start,50,'Color','g');  % Starting reference point
viscircles(goal,50,'Color','r');  % Desired final point

% Fixed Obstacles
for i=1:1:size(fixObst,1)
    rectangle('Position',rect(i,:),'FaceColor',[0,0,0],'EdgeColor',[0,0,0]);
end

% Initialize Obstacle Drawings
circs = [];

% ALL STATIC OBSTACLES (Experiments 1-2)
%{
% Obstacle Robots
size(movingObstacles1,1);
for i=1:1:numRobots 
    circs(i) = viscircles(movingObstacles1(i,:),robot,'Color','k');
end
[steps,~] = size(history1);
for n=1:1:steps-4
    
    % Our Robot
    body = viscircles(history1(n,:),robot,'Color','b');
    
    % Path
    line([history1(n,1), history1(n+1,1)], [history1(n,2), history1(n+1,2)],'Color','b','LineWidth',1,'LineStyle','--');

    % Time Interval Between Frames
    pause(0.05);
    
    % Remove Obstacle and Robot Drawings from Previous Step
    if (n ~= steps-4)
        delete(body);
    end
end
%}

% ALL DYNAMIC OBSTACLES (Experiments 3-5) {
% Loop through Potential Field Steps
[steps,~] = size(history1);
for n=1:1:steps-4
    
    % Our Robot
    body = viscircles(history1(n,:),robot,'Color','b');
    
    % Obstacle Robots
    size(movingObstacles1,1);
    for i=1:1:numRobots 
        circs(i) = viscircles(movingObstacles1(i+(n-1)*numRobots,:),robot,'Color','k');
    end
    
    % Path
    line([history1(n,1), history1(n+1,1)], [history1(n,2), history1(n+1,2)],'Color','b','LineWidth',1,'LineStyle','--');

    % Time Interval Between Frames
    pause(0.05);
    
    % Remove Obstacle and Robot Drawings from Previous Step
    if (n ~= steps-4)
        delete(circs);
        delete(body);
    end
    
end
%}


%% Simulation: P-RRT

% Create Video File
writerObj = VideoWriter('potFieldVideo.avi');
writerObj.FrameRate = 10;

% Create Field
figure(3);
title('P-RRT');
xlabel('Field Length (mm)');
ylabel('Field Width (mm)');
daspect([1 1 1]);  % Ensures aspect ratio
axis(boundary);  % Adds axis dimensions
rectangle('Position',[0 0 9000 6000],'FaceColor',[1 1 1],'EdgeColor','k','LineWidth',3); 

% Creates Notable Points
hold on
viscircles(start,50,'Color','g');  % Starting reference point
viscircles(goal,50,'Color','r');  % Desired final point

% Open Video
open(writerObj);

% Fixed Obstacles
for i=1:1:size(fixObst,1)
    rectangle('Position',rect(i,:),'FaceColor',[0,0,0],'EdgeColor',[0,0,0]);
end

% Initialize variables
circsPRRT = [];
RRTlines = [];
RRTversion = 1;
branches = [];
[steps,~] = size(history);

% FOR STATIC CASES (Experiments #1-2) {
%{
% Obstacle Robots
for i=1:1:numRobots 
    circsPRRT(i) = viscircles(movingObstacles(i,:),robot,'Color','k');
end
%}
%}

% Loop Through Potential Field Steps
for n=1:1:steps-8
    
    % Our Robot
    body = viscircles(history(n,:),robot,'Color','b');
    
    % FOR DYNAMIC (Experiments #3-4){
    % Obstacle Robots
    for i=1:1:numRobots 
        circsPRRT(i) = viscircles(movingObstacles(i+(n-1)*numRobots,:),robot,'Color','k');
    end
    %}
    
    % Path
    line([history(n,1), history(n+1,1)], [history(n,2), history(n+1,2)],'Color','b','LineWidth',1,'LineStyle','--');
    
    % Draws our robot's path after RRT post-processing when stuck
    if ismember(n,framesWhenStuck) 
        
        % Remove Old RRT Lines from Preceive Global Plan
        delete(RRTlines);
        
        % Check for which RRT planner is being drawn
        if (RRTversion == 1)
            RRTi = pathOptAll(1:RRTpathLengths(RRTversion),:);
        else 
            RRTi = pathOptAll(RRTpathLengths(RRTversion-1):RRTpathLengths(RRTversion),:);
        end
        
        % Loop Through RRT segments to draw line path
        checkpoints = size(RRTi,1);
        for i=1:1:checkpoints-1 
            RRTlines(i) = line([RRTi(i,1), RRTi(i+1,1)], [RRTi(i,2), RRTi(i+1,2)],'Color','g','LineWidth',1,'LineStyle','--');
        end
        
        % Iterate to Next RRT Planner Used
        RRTversion = RRTversion + 1;
    end
    
    % Extract frame and save 
    frame = getframe(gcf);
    writeVideo(writerObj,frame);
    
    % Time Increment Between Frames
    pause(0.05);
    
    % Delete Dynamic Obstacles and Robot from previous frame
    if (n ~= steps-8)
        % FOR DYNAMIC (Experiments 3-5){
        delete(circsPRRT);
        %}
        delete(body);
    end
    
end
close(writerObj);



