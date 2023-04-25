clear; clc;

rng('default');
AH_office_size = [132.5 53]; % Estimated from google maps
map = generateMap('map.png', AH_office_size, 1);

x_factor = map.XWorldLimits(2)/396;
y_factor = map.YWorldLimits(2)/209;

start = [12.759696006774902*x_factor 191.51528930664062*y_factor];
goal = [295.7162780761719*x_factor 85.76693725585938*y_factor];
% goal = [148.85 65.65];

pb = plannerBenchmark(map, map.world2grid(start), map.world2grid(goal));
plnFcn = @(planner,s,g)plan(planner,s,g);
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
    
    % calculate path lenth
    path = output.plannerAStarFcn_plnFcn.PlanOutput.Run1{:};
    
    path_length = 0;
    
    for j = 2:length(path)
        point1 = map.grid2world(path(j-1, :));
%         point1(1) = point1(1)/x_factor;
%         point1(2) = point1(2)/y_factor;
        point2 = map.grid2world(path(j, :));
%         point2(1) = point2(1)/x_factor;
%         point2(2) = point2(2)/y_factor;
        path_length = path_length + pdist2(point1, point2);
    end
    
    if i == 1
        mean_metrics_execution_time = metrics.executionTimeSummary.Mean;
        mean_manual_path_length = path_length;
    else 
        mean_metrics_execution_time = mean([mean_metrics_execution_time metrics.executionTimeSummary.Mean]);
        mean_manual_path_length = mean([mean_manual_path_length path_length]);
    end
    
    %     planner = plannerAStarGrid(map);
    %     tic
    %     plan(plannerAStarGrid(map), map.world2grid(start), map.world2grid(goal));
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
mean_manual_path_length  
