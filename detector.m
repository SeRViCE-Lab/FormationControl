% Version 1.4:
%               - Replaces centroids by median of upper edge of the bbox.
%                       this provides a more stable representation for the
%                       location of the spheros
%
% Version 1.3:
%               - Sends back the run time as a parameter
%
% Version 1.2:
%               - Refined search method:
%                       blob detection, then circle detection around the
%                       blobs, but only when the correct number of robots
%                       is not detected
%
% Version 1.1:
%               - It doesn't work, its just for trial purposes
%
% Version 1.0: 
%               - Initial version
%               - Detects blobs of resonable size using binarization
%               according to a threshold
%               - Can distinguish between robots that have collided (using erosion) 
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [locs, bboxes,t] = detect_SpheroV1_4 (frame, numRob)
tic
% initialize variables
locs = zeros(2, numRob);
centersTemp(:,1) = [0;0]; 
bboxes = zeros(4, numRob);
bboxesTemp(:,1) = [0;0;0;0];
thresh = 0.9;
newDetectedTotal = 0;
numBlobsChecked = 0;

% resizing the image
frame = imresize(frame, [480,640]);

% changing frame to grayscale
frameGray = rgb2gray(frame); 
% figure;
% imshow(frameGray);

% binarize blurred image usign a threshold
frameBin = imbinarize (frameGray,thresh); % generate binary image using thresh
% figure;
% imshow(frameBin);

% erode the image to remove noise
erodeElt = strel('disk',5);
frameEr = imerode(frameBin,erodeElt);
% figure;
% imshow(frameEr);

% dilate eroded image
dilateElt = strel('disk',5);
frameDil = imdilate(frameEr, dilateElt);
% figure;
% imshow(frameDil);

% detect large enough blobs
whiteBlobs = bwpropfilt(frameDil, 'Area', [20, 100000]); % find white blobs
% figure;
% imshow(whiteBlobs);

% get statistics from whiteBolobs image
stats1 = regionprops ( logical(whiteBlobs), ...
    'BoundingBox', 'Centroid', 'Area',...
    'MajorAxisLength', 'MinorAxisLength');

% organize data
center1 = reshape([stats1.Centroid]', 2, numel(stats1));
bboxes1 = reshape([stats1.BoundingBox]', 4, numel(stats1)); % format: ULcorner(x,y), x-width, y-width
area1 = reshape([stats1.Area]', 1, numel(stats1));
majAxLeng1 = reshape([stats1.MajorAxisLength]', 1, numel(stats1));
minAxLeng1 = reshape([stats1.MinorAxisLength]', 1, numel(stats1));

numRobDetect = numel(stats1); % number of robots detected

% check to see if all robots were detected
if (numRobDetect == numRob) % if robots detected
    bboxes = bboxes1;
    locs(:,:) = bboxes(1:2, :) + [bboxes(3,:)/2;zeros(1,numel(bboxes)/4)];
elseif (numRobDetect > numRob)
    disp('Error: More objects detected than spheros');
    disp('Centers will be set to zero');
else % objects detected < num Spheros
    % calculate ratios of maj/min axis length
    for i = 1 : numel(stats1)
        maj_minAxRatio(i) = majAxLeng1(i)/ minAxLeng1(i);
    end
    
    % sort the detected blobs based on maj/min axis ratio
    [sortedAxRatio,sortAxRatioIndex] = sort(maj_minAxRatio); %finding sorting index using ratio
    stats2 = stats1(sortAxRatioIndex); % sort stats based on index obtained
    
    % organize data
    centers2 = reshape([stats2.Centroid]', 2, numel(stats2));
    bboxes2 = reshape([stats2.BoundingBox]', 4, numel(stats2)); % format: ULcorner(x,y), x-width, y-width
    area2 = reshape([stats2.Area]', 1, numel(stats2));
    majAxLeng2 = reshape([stats2.MajorAxisLength]', 1, numel(stats2));
    minAxLeng2 = reshape([stats2.MinorAxisLength]', 1, numel(stats2));
    
    % go through list to detect circles in blobs
    for i = numel(stats2) : -1 : 1
        if (numRobDetect ~= numRob)
            box = bboxes2(:,i); % get bbox
            center = centers2(:,i); % store center
            cornerUL = [box(1); box(2)];% store UL corner
            xCorner = box(1);
            yCorner = box(2);
            xWidth = box(3);
            yWidth = box(4);
            
            % zoom in on object
            xWidthN = xWidth * 1.5; % new x-width
            yWidthN = yWidth * 1.5; % new y-width
            dxWidth = xWidthN -xWidth; % variation in xWidth
            dyWidth = yWidthN -yWidth; % variation in yWidth
            xCornerN = xCorner - dxWidth/2; % new x for UL corner
            yCornerN = yCorner - dyWidth/2; % new y for UL corner
            
            boxN = [xCornerN, yCornerN, xWidthN, yWidthN]; % new bbox
            % take only image in new bbox
            frameCrop = frameGray( ...
                max(0,round(yCornerN)) : min(round(yCornerN + yWidthN), 480) ,...
                max(0,round(xCornerN)) : min(round(xCornerN + xWidthN), 680));
%             frame;
%             imshow(frameCrop);
%             hold on;
%             scatter(xWidthN/2 ,yWidthN/2, 'filled', 'LineWidth', 2); % display center on image
%             hold off;
            
            % use circle detection on zoomed image
            d = min(xWidthN, yWidthN); % minimum of new bbox sides
            tic
            [c, r] = imfindcircles(frameCrop,[round(d*0.1667), 3*round(0.1667*d)-1],'Sensitivity',0.9); % find circles
            toc
%             frame;
%             imshow(frameCrop);
%             hold on;
%             viscircles(c, r,'Color','b');
%             hold off
%             
            % moving back centers to the initial frame
            cFrame = c + [xCornerN, yCornerN]; % cFrame = [x1, y1 ; x2, y2; x3,y3 ; ...]
%             frame;
%             imshow(frameGray);
%             hold on;
%             viscircles(cFrame, r,'Color','b');
%             hold off;
            
            % saving the new centers and bboxes
            newDetected = numel(cFrame)/2; 
            for j = 1 : newDetected
                centersTemp(:,numel(centersTemp)/2+1) = cFrame(j,:); % sotre new center
                cornerUL = cFrame(j,:) - r(j); % calculate new corener for bbox
                bboxesTemp(:,numel(bboxesTemp)/4+1) = [cornerUL' ; 2*r(j); 2*r(j)]; % store new bboc
            end
            
            % deleting old centers2 and bboxes2
            centers2(:,i) = [];
            bboxes2(:,i) = [];
            
            % update numBlobsChecked, newDetectedTotal, and numRobDetect
            numBlobsChecked = numBlobsChecked + 1; % keep track of number of blobs checked for robots
            newDetectedTotal = newDetectedTotal + newDetected; % keep track of number of newly discovered robots
            numRobDetect = numRobDetect + newDetected - 1; % update number of robots detected
           
        end
    end

    bboxes(:, 1 : (numel(bboxesTemp)/4-1)) = bboxesTemp(:, 2 : end);
    bboxes(:, (numel(bboxesTemp)/4) : numRob) = bboxes2(:,:);
    
    locs(:,:) = bboxes(1:2, :) + [bboxes(3,:)/2;zeros(1,numel(bboxes)/4)];
    
end

t = toc;
        
        
 
    
    