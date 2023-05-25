% Open the text file for reading the traceback data
tracebackFileID = fopen('data.txt', 'r');

% Read the traceback data from the text file
tracebackData = textscan(tracebackFileID, 'traceback - x: %f, y: %f, t: %f');
fclose(tracebackFileID);

% Extract the x, y, and t coordinates from the traceback data
tracebackX = tracebackData{1};
tracebackY = tracebackData{2};
tracebackT = tracebackData{3};

% Open the text file for reading the map points
mapFileID = fopen('map3.txt', 'r');

% Read the map points from the text file until the letter 'M' is reached
mapPoints = [];
t = 1;
line = fgetl(mapFileID);
while ischar(line)
    if strcmp(line, 'M')
        break;
    else
        coords = sscanf(line, '%f,%f');
        if numel(coords) == 2 % Check if both coordinates are present
            mapPoints = [mapPoints; coords(1), coords(2), t];
            t = t + 1;
        end
    end
    line = fgetl(mapFileID);
end
fclose(mapFileID);

% Extract the x, y, and t coordinates from the map points
mapX = mapPoints(:, 1);
mapY = mapPoints(:, 2);
mapT = mapPoints(:, 3);

% Create a combined array of x, y, and t coordinates
combinedX = [tracebackX; mapX];
combinedY = [tracebackY; mapY];
combinedT = [tracebackT; mapT];

% Create a 3D scatter plot with different colors for each series
figure;
scatter3(tracebackX, tracebackY, tracebackT, 'filled', 'b');
hold on;
scatter3(mapX, mapY, mapT, 'filled', 'r');
xlabel('x');
ylabel('y');
zlabel('t');
title('3D Scatter Plot');
legend('Traceback Data', 'Map Points');
hold off;
