function [template, ImOrig] = GetBbox(Im, prefix, frameNumber, template, DepthFrameToColorFrameMapping, useDepth, useColor)

% useDepth = 1;
% useColor = 0;
% load([prefix, 'DepthFrameToColorFrameMapping.mat']);

% disp('1');
tic;

ImOrig = Im;
if useDepth
    load([prefix, num2str(frameNumber, '%d.mat')], 'DepthFrame');
    Id = DepthFrame.DepthData.Depth;
    Id = map_depthframe_to_colorframe(Id, DepthFrameToColorFrameMapping);
end

% toc
% tic;
minY = max(template.bbox(2)-template.bbox(4)/2, 1);
maxY = min(template.bbox(2)+ 3/2*template.bbox(4), size(Im, 1));
minX = max(template.bbox(1)-template.bbox(3)/2, 1);
maxX = min(template.bbox(1)+ 3/2*template.bbox(3), size(Im, 2));

Im = Im(minY:maxY, minX:maxX, :);
if useDepth
    Id = Id(minY:maxY, minX:maxX);
    Id = double(Id);
%     Id = smooth_depth(Id .* (Id >0));
end

% estimate displacement
oldPoints = template.points;
[template.points, isFound] = step(template.pointTracker, Im);
visiblePoints = template.points(isFound, :);
oldInliers = oldPoints(isFound, :);
MotionProb = GetMotionLikelihood(Im(:,:,1), visiblePoints, oldInliers, template.bboxROI);
MotionProbPad = zeros(480, 640);
MotionProbPad(minY:maxY, minX:maxX) = MotionProb;
if useColor
    Im_e = GetBlurredEdge(Im, 2);
end
if useDepth
    Id_e = GetBlurredEdge(Id, 1);
end

if useColor
    ImCorr = GetCorrMap(Im_e, template.Im_t);
end
if useDepth
    IdCorr = GetCorrMap(Id_e, template.Id_t);
end

if useColor
    ImCorrPad = zeros(480, 640);
    ImCorrPad(minY:maxY, minX:maxX) = ImCorr;
end
if useDepth
    IdCorrPad = zeros(480, 640);
    IdCorrPad(minY:maxY, minX:maxX) = IdCorr;
end

% FinalProbMap = 0 * ImCorrPad + IdCorrPad + (MotionProbPad ./ max(MotionProbPad(:)));
if useDepth
    FinalProbMap = IdCorrPad .* (MotionProbPad ./ max(MotionProbPad(:)));
end
if useColor
    FinalProbMap = ImCorrPad .* (MotionProbPad ./ max(MotionProbPad(:)));
end

%     [maxVal, ind] = max(MotionProbPad(:));
%     maxVal
%     [i, j] = ind2sub([480 640], ind)
%     [maxVal, ind] = max(ImCorrPad(:));
%     maxVal
%     [i, j] = ind2sub([480 640], ind)
%     [maxVal, ind] = max(IdCorrPad(:));
%     maxVal
%     [i, j] = ind2sub([480 640], ind)

template.bbox = GetNewBbox(FinalProbMap, template.bbox);
minY = max(template.bbox(2)-template.bbox(4)/2, 1);
maxY = min(template.bbox(2)+3/2*template.bbox(4), size(ImOrig, 1));
minX = max(template.bbox(1)-template.bbox(3)/2, 1);
maxX = min(template.bbox(1)+3/2*template.bbox(3), size(ImOrig, 2));
Im = ImOrig(minY:maxY, minX:maxX, :);

template.bboxROI = template.bbox;
template.bboxROI(1) = template.bboxROI(1) - minX + 1;
template.bboxROI(2) = template.bboxROI(2) - minY + 1;
% toc
% disp('5');
% tic;
%     Im_t = UpdateTemplate(Im_e, bboxROI);
%     Id_t = UpdateTemplate(Id_e, bboxROI);

try
    points = detectMinEigenFeatures(rgb2gray(Im), 'ROI', template.bboxROI);
    template.points = points;
    template.points = template.points.Location;
    
    release(template.pointTracker);
    initialize(template.pointTracker, template.points, Im);
catch
end

% disp('after');
% size(Im)

bboxPolygon = [template.bbox(1), template.bbox(2), template.bbox(1)+template.bbox(3), template.bbox(2), template.bbox(1)+template.bbox(3), template.bbox(2)+template.bbox(4), template.bbox(1), template.bbox(2)+template.bbox(4)];
ImOrig = insertShape(ImOrig, 'Polygon', bboxPolygon);
toc
% DrawHeatMap(ImOrig, FinalProbMap);
% p = template.points;
% p(:,1) = p(:,1) + (template.bbox(1)-template.bbox(3)) - 1;
% p(:,2) = p(:,2) + (template.bbox(2)-template.bbox(4)) - 1;
% hold on; plot(p(:,1), p(:,2), 'g+'); hold off;
% drawnow;
end

function bbox = GetNewBbox(CorrMap, bbox)

[maxVal, ind] = max(CorrMap(:));

[i, j] = ind2sub([480 640], ind);

bbox(1) = j - bbox(3)/2 + 1;
bbox(2) = i - bbox(4)/2 + 1;

bbox(1) = max(1, bbox(1));
bbox(1) = min(640, bbox(1));
bbox(2) = max(1, bbox(2));
bbox(2) = min(480, bbox(2));
end

function prob = GetMotionLikelihood(I, visiblePoints, oldPoints, bbox)
try
    displacements = visiblePoints - oldPoints;
    clustersEM = gmdistribution.fit(displacements, 1);
    clustersEM = gmdistribution(clustersEM.mu, clustersEM.Sigma * (max(bbox(3:4)/2) ^ 2) + ([1 0; 0 1] * (max(bbox(3:4)/2) ^ 2)), clustersEM.PComponents);
catch
%     prob = ones(size(I));
    clustersEM = gmdistribution([0 0], [1 0; 0 1] * (max(bbox(3:4)/2) ^ 2), 1);
    disp('Point tracking error');
%     return;
end
prevPos = [bbox(2)+bbox(4)/2, bbox(1)+bbox(3)/2];
x =repmat( 1:size(I,2),size(I,1),1);
y = repmat([1:size(I,1)]',1,size(I,2));

prob = pdf(clustersEM, [x(:)-prevPos(2) y(:)-prevPos(1)]);
prob = reshape(prob, size(I));
end

