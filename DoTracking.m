function DoTracking(datapath, startFrame)

prefix = 'USB-VID_045E&PID_02AE-A00365A09389104A_';

load(fullfile(datapath, [prefix, 'DepthFrameToColorFrameMapping.mat']));

Im = imread(fullfile(datapath, num2str(startFrame, '%d.png')));
load(fullfile(datapath, [prefix, num2str(startFrame, '%d.mat')]));
Id = DepthFrame.DepthData.Depth;
Id = double(Id);
Id = smooth_depth(Id .* (Id >0));
Id = map_depthframe_to_colorframe(Id, DepthFrameToColorFrameMapping);

Im_e = GetBlurredEdge(Im, 2);
Id_e = GetBlurredEdge(Id, 1);

[Im_t, Id_t, bbox] = GetHeadTemplate(Im_e, Id_e, Im);

% save('tracking_temp.mat');
% load('tracking_temp.mat');
bbox(1:2)
Im = Im(bbox(2)-bbox(4):bbox(2)+2*bbox(4), bbox(1)-bbox(3):bbox(1)+2*bbox(3), :);
bboxROI = bbox;
bboxROI(1) = bboxROI(1) - (bbox(1)-bbox(3)) + 1;
bboxROI(2) = bboxROI(2) - (bbox(2)-bbox(4)) + 1;
points = detectMinEigenFeatures(rgb2gray(Im), 'ROI', bboxROI);
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
points = points.Location;
initialize(pointTracker, points, Im);
for i=startFrame+1:startFrame+1000
    tic;
    i
%     size(points)
    Im = imread(fullfile(datapath, num2str(i, '%d.png')));
    ImOrig = Im;
    load(fullfile(datapath, [prefix, num2str(i, '%d.mat')]));
    Id = DepthFrame.DepthData.Depth;
    
    Id = map_depthframe_to_colorframe(Id, DepthFrameToColorFrameMapping);
    
    
    Im = Im(bbox(2)-bbox(4):bbox(2)+2*bbox(4), bbox(1)-bbox(3):bbox(1)+2*bbox(3), :);
    Id = Id(bbox(2)-bbox(4):bbox(2)+2*bbox(4), bbox(1)-bbox(3):bbox(1)+2*bbox(3));
    Id = double(Id);
    Id = smooth_depth(Id .* (Id >0));
    
    % estimate displacement
    oldPoints = points;
    [points, isFound] = step(pointTracker, Im);
    visiblePoints = points(isFound, :);
    oldInliers = oldPoints(isFound, :);
    MotionProb = GetMotionLikelihood(Id, visiblePoints, oldInliers, bboxROI);
    MotionProbPad = zeros(480, 640);
    MotionProbPad(bbox(2)-bbox(4):bbox(2)+2*bbox(4), bbox(1)-bbox(3):bbox(1)+2*bbox(3)) = MotionProb;
    
    Im_e = GetBlurredEdge(Im, 2);
    Id_e = GetBlurredEdge(Id, 1);
    
    ImCorr = GetCorrMap(Im_e, Im_t);
    IdCorr = GetCorrMap(Id_e, Id_t);
    toc
    
    ImCorrPad = zeros(480, 640);
    ImCorrPad(bbox(2)-bbox(4):bbox(2)+2*bbox(4), bbox(1)-bbox(3):bbox(1)+2*bbox(3)) = ImCorr;
    IdCorrPad = zeros(480, 640);
    IdCorrPad(bbox(2)-bbox(4):bbox(2)+2*bbox(4), bbox(1)-bbox(3):bbox(1)+2*bbox(3)) = IdCorr;
    
    FinalProbMap = 0 * ImCorrPad + IdCorrPad + (MotionProbPad ./ max(MotionProbPad(:)));
    
%     [maxVal, ind] = max(MotionProbPad(:));
%     maxVal
%     [i, j] = ind2sub([480 640], ind)
%     [maxVal, ind] = max(ImCorrPad(:));
%     maxVal
%     [i, j] = ind2sub([480 640], ind)
%     [maxVal, ind] = max(IdCorrPad(:));
%     maxVal
%     [i, j] = ind2sub([480 640], ind)

    bbox = GetNewBbox(FinalProbMap, bbox);
    bbox(1:2)
    
    bboxROI = bbox;
    bboxROI(1) = bboxROI(1) - (bbox(1)-bbox(3)) + 1;
    bboxROI(2) = bboxROI(2) - (bbox(2)-bbox(4)) + 1;
    
%     Im_t = UpdateTemplate(Im_e, bboxROI);
%     Id_t = UpdateTemplate(Id_e, bboxROI);
    
    points = detectMinEigenFeatures(rgb2gray(Im), 'ROI', bboxROI);
    points = points.Location;
    release(pointTracker);
    initialize(pointTracker, points, Im);

    bboxPolygon = [bbox(1), bbox(2), bbox(1)+bbox(3), bbox(2), bbox(1)+bbox(3), bbox(2)+bbox(4), bbox(1), bbox(2)+bbox(4)];
    ImOrig = insertShape(ImOrig, 'Polygon', bboxPolygon);
    
    DrawHeatMap(ImOrig, FinalProbMap);
    p = points;
    p(:,1) = p(:,1) + (bbox(1)-bbox(3)) - 1;
    p(:,2) = p(:,2) + (bbox(2)-bbox(4)) - 1;
    hold on; plot(p(:,1), p(:,2), 'g+'); hold off;
    drawnow;
end

end

function bbox = GetNewBbox(CorrMap, bbox)

[maxVal, ind] = max(CorrMap(:));

[i, j] = ind2sub([480 640], ind);

bbox(1) = j - bbox(3)/2 + 1;
bbox(2) = i - bbox(4)/2 + 1;

end

function prob = GetMotionLikelihood(I, visiblePoints, oldPoints, bbox)
try
    displacements = visiblePoints - oldPoints;
    clustersEM = gmdistribution.fit(displacements, 1);
    clustersEM = gmdistribution(clustersEM.mu, clustersEM.Sigma * (max(bbox(3:4))) ^ 2, clustersEM.PComponents);
catch
    prob = ones(size(I));
    disp('Point tracking error');
    return;
end
prevPos = [bbox(2)+bbox(4)/2, bbox(1)+bbox(3)/2];
x =repmat( 1:size(I,2),size(I,1),1);
y = repmat([1:size(I,1)]',1,size(I,2));

prob = pdf(clustersEM, [x(:)-prevPos(2) y(:)-prevPos(1)]);
prob = reshape(prob, size(I));
end

function template = UpdateTemplate(I, bbox)
template = I(bbox(2):bbox(2)+bbox(4), bbox(1):bbox(1)+bbox(3));
end

