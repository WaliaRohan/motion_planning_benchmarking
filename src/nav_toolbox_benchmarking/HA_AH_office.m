clear; clc;

rng('default');

map_size = [132.5 53];
scale = 0.1;

% Read image, convert to grayscale, and normalize
RGB = imread("map.png");
grayscale = rgb2gray(RGB);
resized_grayscale = imresize(grayscale, scale);
imageNorm = double(resized_grayscale)/255;

% Convert normalized grayscale image to occupancy map
imageOccupancy = 1 - imageNorm;
imageOccupancy(imageOccupancy > 0) = 1; % Turn all unkown areas to absolute obstacles
resolution = mean([size(imageOccupancy, 2)/map_size(1) ...
    size(imageOccupancy, 1)/map_size(2)]);
map = occupancyMap(imageOccupancy, resolution);

start = [4.26 49 0];
goal = [97.129 20.19 0];

stateBounds = [map.XWorldLimits; map.YWorldLimits; -pi pi];

ss = stateSpaceSE2(stateBounds);
sv = validatorOccupancyMap(ss,Map=map);

% Plan path and plot results
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