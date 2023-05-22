%{
 1) Open video file and coresponding CSV file
 2) Get aprilTag info from a frame from the video file
 3) Compare the apriltags in both videos
 4) Make new matrix, add data from first video/CSV up until skeleton
    equals aprilTag center position
 5) Add data from second CSV file starting at aprilTag center, adding the
    width and time to the second file so it continues from the first
%}

function combineCSVs(fname1, fname2)

if nargin < 2
    fname1 = openFile();
    fname2 = openFile();
end

[filepath1, name1, ext] = fileparts(fname1);
[filepath2, name2, ext] = fileparts(fname2);

v1 = VideoReader([filepath1 filesep name1 '.avi']);
data1 = importBlazeDepthAIfile([filepath1 filesep name1 '.csv']);

v2 = VideoReader([filepath2 filesep name2 '.avi']);
data2 = importBlazeDepthAIfile([filepath2 filesep name2 '.csv']);

frameV1 = getFrame(v1);
frameV2 = getFrame(v2);

[tags1id, tags1loc] = aprilTagDetection(frameV1);
[tags2id, tags2loc] = aprilTagDetection(frameV2);

try
    [vertical1, horizontal1] = detectCross(tags1loc);
    [vertical2, horizontal2] = detectCross(tags2loc);

    [sizeV, sizeH] = imagesSize(vertical1, vertical2, horizontal1, horizontal2);

catch
    sizeV = 1;
    sizeH = 1;
end 

compareTags(tags1id, tags2id);

[centerX1, centerY1] = tagCenter(tags1loc)
[centerX2, centerY2] = tagCenter(tags2loc);

heightDifference = centerY1 - centerY2;

[time1, extraTime1] = timeToApriltag(data1, centerX1);

[time2, extraTime2] = timeToApriltag(data2, centerX2);

fullTime2 = height(data2);

newTime = (time1 - extraTime1) + (fullTime2-time2);

M = zeros(newTime, 67);

M = copyData1(M, data1, centerX2, centerX1, time1, extraTime1);

M = copyData2(M, data2, time2, time1, extraTime1, heightDifference);

csvwrite('combinedFile.csv', M);

plotSkeleton(M);
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

function [id,loc,detectedFamily] = aprilTagDetection(I)
tagFamily = ("tag36h11");

[id,loc,detectedFamily] = readAprilTag(I,tagFamily);

    for idx = 1:length(id)
        % Display the ID and tag family
        disp("Detected Tag ID, Family: " + id(idx) + ", " ...
            + detectedFamily(idx));
 
        % Insert markers to indicate the locations
        markerRadius = 3;
        numCorners = size(loc,1);
        markerPosition = [loc(:,:,idx),repmat(markerRadius,numCorners,1)];
        I = insertShape(I,"FilledCircle",markerPosition,Color="red",Opacity=1);
        
    end
    %imshow(I);
    
end

function [vertical, horizontal] = detectCross(tagsLoc)
verticalPoints = [tagsLoc(1,1,3), tagsLoc(1,2,3); tagsLoc(1,1,4), tagsLoc(1,2,4)];
vertical = pdist(verticalPoints, 'euclidean');

horizontalPoints = [tagsLoc(1,1,5), tagsLoc(1,2,5); tagsLoc(1,1,6), tagsLoc(1,2,6)];
horizontal = pdist(horizontalPoints, 'euclidean');
end

function [sizeV, sizeH] = imagesSize(vert1, vert2, horiz1, horiz2)
sizeV = vert1/vert2;
sizeH = horiz1/horiz2;
end

function compareTags(tags1, tags2)
if ~isempty(tags1) && ~isempty(tags2)
    % Find the common TagIndexes
    commonTagIndexes = intersect(tags1, tags2);

    if ~isempty(commonTagIndexes)
        fprintf('AprilTag(s) with id(s) %s are shared between the two images.\n', num2str(commonTagIndexes));
    else
        fprintf('No AprilTags with common IDs detected in the two images.\n');
    end
else
    fprintf('No AprilTags detected in one or both images.\n');
end
end

function [centerX, centerY] = tagCenter(tagcorners)
%Two points from line 1
ULx = tagcorners(4, 1);
ULy = tagcorners(4, 2); 

BRx = tagcorners(2, 1);
BRy = tagcorners(2, 2);

%Two points from line 2
URx = tagcorners(3, 1);
URy = tagcorners(3, 2);
    
BLx = tagcorners(1, 1);
BLy = tagcorners(1, 2);
    
    
% Calculate the slopes of the two lines
ma = (ULy - BRy) / (ULx - BRx);
mb = (BLy - URy) / (BLx - URy);

% Calculate the y-intercepts of the two lines
ba = BRy - ma * BRx;
bb = BLy - mb * BLx;

% Calculate the x-coordinate of the intersection point
centerX = (bb - ba) / (ma - mb);

% Calculate the y-coordinate of the intersection point
centerY = ma * centerX + ba;

fprintf('The intersection point is at x = %.2f, y = %.2f\n', centerX, centerY);
end

function [time, extraTime] = timeToApriltag(data, aprilTagX)
time = 0;
extraTime = 0;
for i = 1:size(data,1)
    if data(i, 24) < (aprilTagX + 10) & data(i,24) > (aprilTagX - 10)
        return;
    elseif data(i, 24) == -1
       extraTime = extraTime + 1;
    end
    time = time + 1;
end
end

function M = copyData1(M, data, centerX2, centerX1, time, start)
for i = start+1:time
    for j = 1:width(data)
        if mod(j,2) == 0
            M(i-start, j) = (data(i, j) + centerX2 - centerX1);
        elseif j == 1
            M(i-start, j) = data(i, j) - data(start, j);
        else
            M(i-start, j) = data(i, j);
        end
    end
end
end

function M = copyData2(M, data, time2, time1, extraTime1, height)
start = time1 - extraTime1 + 1;
for i = time2:length(data)
    for j = 1:width(data)
        if mod(j,2) == 0
            M(start+(i-time2), j) = data(i, j);
        elseif j == 1
            M(start+(i-time2), j) = data(i, j) - data(time2, j) + M(start-1, j);
        else
            M(start+(i-time2), j) = data(i, j) + (height/2);
        end
    end
end
end

function plotSkeleton(data)

%resolution = [1152 648];
resolution = [2304 648];

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
landMarks = cell((width(data)-1)/2,1);

xlim([0 resolution(1)])
ylim([0 resolution(2)])

for link = 1:height(links)
   [x1,x2,y1,y2] = getLineCoords(1, data, links(link,:), resolution);
   lines{link} = line( [x1 x2],[y1 y2]);
   set(lines{link},'color',links(link,3:5));
   set(lines{link},'linewidth',links(link,6));
end

for landMarkNr = 0:numel(landMarks)-1
    [x,y] = getLandMarkCoords(1, data, landMarkNr, resolution);
    landMarks{landMarkNr+1} = plot(x,y,'O','MarkerSize',5,'MarkerEdgeColor','b','MarkerFaceColor',[0.5,0.5,0.5]);
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
        [x1,x2,y1,y2] = getLineCoords(row, data, links(link,:), resolution);
        set(lines{link},'XData',[x1 x2],'YData',[y1 y2])
    end
    
    for landMarkNr = 0:numel(landMarks)-1
        [x,y] = getLandMarkCoords(row, data, landMarkNr, resolution);
        set(landMarks{landMarkNr+1},'XData',x,'YData',y)
    end
    
    %uistack(imgHandle,'bottom');
    pause(0.066)
end



end

function [x1,x2,y1,y2] = getLineCoords(row, data, link, resolution)
    if link(1) == 8
        a=1;
    end
    x1 = data(row,2 + link(1) * 2);
    y1 = resolution(2) - data(row,3+ link(1) * 2);
    x2 = data(row,2+ link(2) * 2);
    y2 = resolution(2) - data(row,3+ link(2) * 2);
end

function [x,y] = getLandMarkCoords(row, data, landMarkNr, resolution)

    x = data(row,2+ landMarkNr * 2);
    y = resolution(2) - data(row,3 + landMarkNr * 2);
end