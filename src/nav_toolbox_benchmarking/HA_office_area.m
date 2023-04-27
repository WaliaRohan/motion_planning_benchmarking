clear; clc; close all;
load("office_area_gridmap.mat","occGrid")

% Set start and goal poses.
start = [-1.00 0 0];
goal = [14 -2.25 0];

% Show start and goal positions of robot.
hold on
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')

% Plan path and plot results
sv = validatorOccupancyMap(stateSpaceSE2,Map=occGrid);

pb = plannerBenchmark(sv,start,goal);
plnFcn = @(planner,s,g)plan(planner,s,g);
plannerHAFcn = @(sv)plannerHybridAStar(sv, ...
    MinTurningRadius=0.64, ...
    NumMotionPrimitives=3);

pb.addPlanner(plnFcn, plannerHAFcn);

runs = 10;

for i = 1:runs

    runPlanner(pb,1)
    metrics = metric(pb);
    output = pb.PlannerOutput;
    
    % verify path lenth
    path = output.plannerHAFcn_plnFcn.PlanOutput.Run1{:}.States;
    
    path_length = 0;
    
    for j = 2:length(path)
        point1 = path(j-1, 1:2);
        point2 = path(j, 1:2);
        path_length = path_length + pdist2(point1, point2);
    end

    if i == 1
        mean_metrics_execution_time = metrics.executionTimeSummary.Mean;
        mean_metrics_path_length = metrics.pathLengthSummary.Mean;
    else 
        mean_metrics_execution_time = mean([mean_metrics_execution_time metrics.executionTimeSummary.Mean]);
        mean_metrics_path_length = mean([mean_metrics_path_length metrics.pathLengthSummary.Mean]);
    end

end

% plot path
show(output.plannerHAFcn_plnFcn.InitializationOutput)
set(gca,'YTickLabel',[]);
set(gca,'XTickLabel',[]);
set(gca,'Title',[]);
set(gca,'XLabel',[]);
set(gca,'YLabel',[]);
set(gca,'TickLength',[0 0])

mean_metrics_execution_time
mean_metrics_path_length