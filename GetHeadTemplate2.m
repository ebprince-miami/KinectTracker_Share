function [template] = GetHeadTemplate2(Im, prefix, frameNumber, DepthFrameToColorFrameMapping, imhandle)

% load([prefix, 'DepthFrameToColorFrameMapping.mat']);

load([prefix, num2str(frameNumber, '%d.mat')], 'DepthFrame');
Id = DepthFrame.DepthData.Depth;
Id = double(Id);
% Id = smooth_depth(Id .* (Id >0));
Id = map_depthframe_to_colorframe(Id, DepthFrameToColorFrameMapping);

Im_e = GetBlurredEdge(Im, 2);
Id_e = GetBlurredEdge(Id, 1);

[x1, y1] = ginput(1);
ImDraw = insertShape(Im, 'FilledCircle', [x1 y1 3], 'Color', 'green', 'LineWidth', 3);
set(imhandle, 'CData', ImDraw);
drawnow update;
[x2, y2] = ginput(1);
ImDraw = insertShape(ImDraw, 'FilledCircle', [x2 y2 3], 'Color', 'green', 'LineWidth', 3);
set(imhandle, 'CData', ImDraw);
drawnow update;
x = [x1;x2];
y = [y1;y2];
x = floor(x / 2) * 2;
y = floor(y / 2) * 2;

template.Im_t = Im_e(y(1):y(2),x(1):x(2));
template.Id_t = Id_e(y(1):y(2),x(1):x(2));

template.bbox = [x(1) y(1) x(2)-x(1) y(2)-y(1)];

template.frameNumber = frameNumber;
template.prefix = prefix;

minY = max(template.bbox(2)-template.bbox(4)/2, 1);
maxY = min(template.bbox(2)+3/2*template.bbox(4), size(Im, 1));
minX = max(template.bbox(1)-template.bbox(3)/2, 1);
maxX = min(template.bbox(1)+3/2*template.bbox(3), size(Im, 2));
Im = Im(minY:maxY, minX:maxX, :);
template.bboxROI = template.bbox;
template.bboxROI(1) = template.bboxROI(1) - minX + 1;
template.bboxROI(2) = template.bboxROI(2) - minY + 1;
points = detectMinEigenFeatures(rgb2gray(Im), 'ROI', template.bboxROI);
template.pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
template.points = points.Location;
initialize(template.pointTracker, template.points, Im);
size(Im)
end