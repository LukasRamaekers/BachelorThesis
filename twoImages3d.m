%{
first video 
camera: 1,15m height
         4m depth
         1,5m along axis

 ball: 50 cm; 1,62m; 2m
========================================================
second video:
cmaera: 1,40m along axis
        90cm height
        4m depth

%}

function twoImages3d

fname1 = openFile();
fname2 = openFile();
[filepath1, name1, ext] = fileparts(fname1);
[filepath2, name2, ext] = fileparts(fname2);

v1 = VideoReader([filepath1 filesep name1 '.avi']);
v2 = VideoReader([filepath2 filesep name2 '.avi']);

try
   data1 = importBlazeDepthAIfile([filepath1 filesep name1 '.csv']);
   data2 = importBlazeDepthAIfile([filepath2 filesep name2 '.csv']);
catch
   M = zeros(1, 100);
end

frameV1 = getFrame(v1);
frameV2 = getFrame(v2);

coords1 = customOrigin(frameV1);
coords2 = customOrigin(frameV2);

[time1, startTime1] = getTime(data1);
[time2, startTime2] = getTime(data2);

if time1 < time2
    runtime = time1;
    startTime = startTime1;
else
    runtime = time2;
    startTime = startTime2;
end

if runtime ~= 0
    M = zeros(runtime, 100); 

    M = xAndYAxis(data1, coords1, M, runtime, startTime);
    M = zAxis(data2, coords2, M, runtime, startTime);
    
    plotSkeleton3d(M);
else
    [x, y] = singlePointCoordinateXY(frameV1, coords1);
    z = singlePointCoordinateZ(frameV2, coords2);
    
    text = [x , y , z];
    
    disp(text);
end
end

function [fname] = openFile()
[file, path] = uigetfile({'*.avi';'*.csv'});
fname = [path filesep file];
end

function [frame] = getFrame(vidObj)
% Read the desired frame number
frameNumber = 20;
vidObj.CurrentTime = (frameNumber - 1) / vidObj.FrameRate;
frame = readFrame(vidObj);

%imshow(frame);
end

function origin = customOrigin(grayImage)
[rows, columns, numberOfColorChannels] = size(grayImage);

imshow(grayImage);

[xi, yi] = ginput(1);
origin = [xi, (yi)];

xdata = -origin(1) : columns - origin(1);
ydata = -origin(2) : columns - origin(2);

%imshow(grayImage, 'XData', xdata, 'YData', ydata);
%imshow(grayImage);
%axis on;
%hp = impixelinfo
%grid on;
end

function [time, startTime] = getTime(data)
time = 0;
startTime = 0;
started = false;

for i = 1:size(data,1)
    if data(i, 24) ~= -1
       time = time + 1;
       started = true;
    else
        if started == false
            startTime = startTime + 1;
        end
    end
end
end

function M = xAndYAxis(data, coords, M, time, startTime)
for i = startTime+1:time
    xColumn = 3;
    yColumn = 2;
    for j = 1:width(data)
        if mod(j,2) == 0 %Even columns have the X coordinates
            M(i-startTime+1, xColumn) = (data(i, j)) - coords(1,1);
            xColumn = xColumn + 3;
        elseif j == 1 %First column has the time
            M(i-startTime+1, j) = data(i, j) - data(startTime+1, j);
        else %Odd columns have the Y coordinates
            M(i-startTime+1, yColumn) = data(i, j) - coords(1,2);
            yColumn = yColumn + 3;
        end
    end
end
end

function M = zAxis(data, coords, M, time, startTime)
for i = startTime+1:time
    zColumn = 4;
    for j = 1:width(data)
        if mod(j,2) == 0 %Even columns have the Z coordinates
            M(i-startTime+1, zColumn) = (data(i, j)) - coords(1,1);
            zColumn = zColumn + 3;
        end
    end
end
end

function [x, y] = singlePointCoordinateXY(image, coord)
imshow(image);
[xi, yi] = ginput(1);
x = xi - coord(1,1); 
y = yi - coord(1,2);
end

function z = singlePointCoordinateZ(image, coord)
imshow(image);
[xi, yi] = ginput(1);
z = xi - coord(1,1); 
end

function plot(M)
%for j = 2:length(M);
for i = 1:((width(M)-1)/3)-1;
    y = M(80,2 + 3*i);
    x = M(80,3 + 3*i);
    z = M(80,4 + 3*i); 
    
    plot3(z, x, y, '.', 'Color', 'red', 'MarkerSize', 10);
    
    axis([-600 0 -100 500 -200 400]);    
    xlabel('Z');
    ylabel('X');
    zlabel('Y');
    grid on;
    hold on;
end
hold off;
%end
end

function plotSkeleton3d(data)

close all;

resolution = [0 500 400];

links(1,:) = [12 11 0 1 0 3];
links(2,:) = [11 23 0 1 0 3];
links(3,:) = [23 24 0 1 0 3];
links(4,:) = [24 12 0 1 0 3];
links(5,:) = [12 14 1 0 0 3];
links(6,:) = [14 16 1 0 0 3];
links(7,:) = [11 13 1 0 0 3];
links(8,:) = [13 15 1 0 0 3];
links(9,:) = [24 26 0 0 1 4];
links(10,:) = [26 28 0 0 1 4];
links(11,:) = [23 25 0 0 1 4];
links(12,:) = [25 27 0 0 1 4];
links(13,:) = [8 0 1 0 0 1];
links(14,:) = [0 7 1 0 0 1];
links(15,:) = [9 10 1 0 0 1];

figure
%set(gcf,'position',[1966  137  1173 665])
set(gca,'DataAspectRatio',[1,1,1])
hold on;

lines = cell(height(links),1);
landMarks = cell((width(data)-1)/3,1);

xlim([-400 200])
ylim([-100 500])
zlim([-600 0])
xlabel('Y');
ylabel('X');
zlabel('Z');

for link = 1:height(links)
   [x1,x2,y1,y2,z1,z2] = getLineCoords(1, data, links(link,:), resolution);
   lines{link} = line( [x1 x2],[y1 y2], [z1 z2]);
   set(lines{link},'color',links(link,3:5));
   set(lines{link},'linewidth',links(link,6));
end

for landMarkNr = 0:numel(landMarks)-1
    [x,y,z] = getLandMarkCoords(1, data, landMarkNr, resolution);
    landMarks{landMarkNr+1} = plot3(x,y,z,'O','MarkerSize',5,'MarkerEdgeColor','b','MarkerFaceColor',[0.5,0.5,0.5]);
end

%readFrame(v);

for row = 2:height(data)

      axesHandlesToChildObjects = findobj(gca, 'Type', 'image');
      if ~isempty(axesHandlesToChildObjects)
        delete(axesHandlesToChildObjects);
      end
    
     %frm = readFrame(v);
     %frm = flip(frm,1);
     %imgHandle = image(frm);
    
    for link = 1:height(links)
        [x1,x2,y1,y2,z1,z2] = getLineCoords(row, data, links(link,:), resolution);
        set(lines{link},'XData',[x1 x2],'YData',[y1 y2], 'ZData',[z1 z2])
    end
    
    for landMarkNr = 0:numel(landMarks)-1
        [x,y,z] = getLandMarkCoords(row, data, landMarkNr, resolution);
        set(landMarks{landMarkNr+1},'XData',x,'YData',y,'ZData',z)
    end
    
    %uistack(imgHandle,'bottom');
    pause(0.066)
end



end

function [x1,x2,y1,y2,z1,z2] = getLineCoords(row, data, link, resolution)
    if link(1) == 8
        a=1;
    end
    x1 = data(row,2 + link(1) * 3);
    y1 = resolution(2) - data(row,3+ link(1) * 3);
    x2 = data(row,2+ link(2) * 3);
    y2 = resolution(2) - data(row,3+ link(2) * 3);
    z1 = data(row,4 + link(1) * 3);
    z2 = data(row,4 + link(2) * 3);
end

function [x,y,z] = getLandMarkCoords(row, data, landMarkNr, resolution)

    x = data(row,2+ landMarkNr * 3);
    y = resolution(2) - data(row,3 + landMarkNr * 3);
    z = data(row,4+ landMarkNr * 3);
end