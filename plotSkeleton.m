function plotSkeleton(fname)

close all

if nargin == 0
    [file,path] = uigetfile({'*.avi';'*.csv'});
    fname = [path filesep file];
end

[filepath,name,ext] = fileparts(fname);

%v = VideoReader([filepath filesep name '.avi']);
data = importBlazeDepthAIfile([filepath filesep name '.csv']);

close all

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

