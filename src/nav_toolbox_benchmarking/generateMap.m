function occMap = generateMap(file_name, map_size, scale)

    % Read image, convert to grayscale, and normalize
    RGB = imread(file_name);
    grayscale = rgb2gray(RGB);
    resized_grayscale = imresize(grayscale, scale);
    imageNorm = double(resized_grayscale)/255;

    % Convert normalized grayscale image to occupancy map
    imageOccupancy = 1 - imageNorm;
    resolution = mean([size(imageOccupancy, 2)/map_size(1) ...
        size(imageOccupancy, 1)/map_size(2)]);
    occMap = occupancyMap(imageOccupancy, resolution);
end