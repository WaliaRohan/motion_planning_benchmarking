clear; clc;

rng('default');
map = mapClutter;
% inflate(map, 0.55)

start = [2 247];
goal = [248 2];

pb = plannerBenchmark(map, map.grid2world(start), map.grid2world(goal));
plnFcn = @(planner,s,g)plan(planner,s,g, 'world');
plannerAStarFcn = @(map)plannerAStarGrid(map, 'DiagonalSearch', 'on');
pb.addPlanner(plnFcn, plannerAStarFcn);

runs = 10;

% Metrics calculated using plannerBenchmark
mean_metrics_path_length = 0;
mean_metrics_execution_time = 0; 
mean_manual_path_length = 0;

for i = 1:runs

    runPlanner(pb,1)
    metrics = metric(pb);
    output = pb.PlannerOutput;

    % show execution time and path length metrics
    metrics.executionTimeSummary
    metrics.pathLengthSummary
    
    % verify path lenth
    path = output.plannerAStarFcn_plnFcn.PlanOutput.Run1{:};
    
    path_length = 0;
    
    for j = 2:length(path)
        point1 = path(j-1, :);
        point2 = path(j, :);
        path_length = path_length + pdist2(point1, point2);
    end

    if i == 1
        mean_metrics_execution_time = metrics.executionTimeSummary.Mean;
        mean_metrics_path_length = metrics.pathLengthSummary.Mean;
        mean_manual_path_length = path_length;
    else 
        mean_metrics_execution_time = mean([mean_metrics_execution_time metrics.executionTimeSummary.Mean]);
        mean_metrics_path_length = mean([mean_metrics_path_length metrics.pathLengthSummary.Mean]);
        mean_manual_path_length = mean([mean_manual_path_length path_length]);
    end
    
    %     tic
    %     plan(plannerAStarGrid(map), start, goal);
    %     toc
end

% plot path
show(output.plannerAStarFcn_plnFcn.InitializationOutput)
set(gca,'YTickLabel',[]);
set(gca,'XTickLabel',[]);
set(gca,'Title',[]);
set(gca,'XLabel',[]);
set(gca,'YLabel',[]);
set(gca,'TickLength',[0 0])

mean_metrics_execution_time
mean_metrics_path_length
mean_manual_path_length