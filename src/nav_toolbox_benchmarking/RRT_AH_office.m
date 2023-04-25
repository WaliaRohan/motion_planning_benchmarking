close all; clear; clc;

occGrid = generateMap("map.png")
show(occGrid);

% Set start and goal poses.
start = [32.35,132.45,pi];
goal = [291.55,15.45,-pi];

% Show start and goal positions of robot.
lw = 2.5;

hold on
plot(start(1),start(2),'go', MarkerSize=10, LineWidth=lw)
plot(goal(1),goal(2),'ro', MarkerSize=10, LineWidth=lw)

% Show start and goal headings.
r = 1.5;
plot([start(1),start(1) + r*cos(start(3))],[start(2),start(2) + r*sin(start(3))],'g-', LineWidth=lw)
plot([goal(1),goal(1) + r*cos(goal(3))],[goal(2),goal(2) + r*sin(goal(3))],'r-', LineWidth=lw)
hold off

%% Define state space
bounds = [occGrid.XWorldLimits; occGrid.YWorldLimits; [-pi pi]];

ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.4;

%% Plan the path

stateValidator = validatorOccupancyMap(ss);                                 
stateValidator.Map = occGrid;
stateValidator.ValidationDistance = 0.05;                                   

planner = plannerRRT(ss,stateValidator);
planner.MaxConnectionDistance = 2.0;                                        % Check what this is used for
planner.MaxIterations = 30000;
planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

rng default

[pthObj, solnInfo] = plan(planner,start,goal);

%% Plot the path
show(occGrid)
hold on

% Plot entire search tree.
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');

% Interpolate and plot path.
interpolate(pthObj,300)
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)

% Show start and goal in grid map.
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')
hold off


%% Helper function(s)

function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
    isReached = false;
    threshold = 0.1;
    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end
end