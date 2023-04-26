clear; clc;

rng('default');
% AH_office_size = [132.5 53]; % Estimated from google maps
% map = generateMap('map.png', AH_office_size, 1);

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
map.FreeThreshold = 0.01;
map.OccupiedThreshold = 0.99;

start = [4.26 49];
goal = [97.129 20.19];

pb = plannerBenchmark(map, map.world2grid(start), map.world2grid(goal));
plnFcn = @(planner,s,g)plan(planner,s,g);
plannerAStarFcn = @(map)plannerAStarGrid(map);%, 'DiagonalSearch', 'on');
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
