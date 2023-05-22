I = imread("test_image.jpg");
v = VideoReader("20230331-105803_camera_0.avi");

tagFamily = ("tag36h11");

frameNumber = 20;
v.CurrentTime = (frameNumber - 1) / v.FrameRate;
%I = readFrame(v);

[id,loc,detectedFamily] = readAprilTag(I,tagFamily);

for idx = 1:length(id)
        % Display the ID and tag family
        disp("Detected Tag ID, Family: " + id(idx) + ", " ...
            + detectedFamily(idx));
 
        % Insert markers to indicate the locations
        markerRadius = 8;
        numCorners = size(loc,1);
        markerPosition = [loc(:,:,idx),repmat(markerRadius,numCorners,1)];
        I = insertShape(I,"FilledCircle",markerPosition,Color="red",Opacity=1);
        
end

imshow(I)