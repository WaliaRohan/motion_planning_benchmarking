clear; clc;

load parkingLotCostVal.mat % costVal

resolution = 3;
map = binaryOccupancyMap(costVal,resolution);

start = [4 9 pi/2]; % 4 9 pi/2
goal = [30 19 -pi/2]; % 30 19 -pi/2

sv = validatorOccupancyMap(stateSpaceSE2,Map=map);

sv.isStateValid(start);

pb = plannerBenchmark(sv,start,goal);
plnFcn = @(planner,s,g)plan(planner,s,g);
plannerHAFcn = @(sv)plannerHybridAStar(sv, ...
    MinTurningRadius=0.64, ...
    NumMotionPrimitives=3);

pb.addPlanner(plnFcn, plannerHAFcn);

runs = 10;

% Metrics calculated using plannerBenchmark
mean_metrics_path_length = 0;
mean_metrics_execution_time = 0; 
mean_manual_path_length = 0;

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