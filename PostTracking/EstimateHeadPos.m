function translation = EstimateHeadPos(headPCD, centroid, radius, kalmanFilter)
% This function requires that the object has already been tracked in 2d
% (so it requires a 2d bounding box of the object)

if size(headPCD, 1) < 100
    translation = zeros(1, 3);
    return;
end

options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', 'Display', 'off');

if sum(centroid) == 0;
    depth = sort(headPCD(:,3),'ascend');
    th1 = depth(ceil(0.25 * length(depth)));
    th2 = depth(ceil(0.15 * length(depth)));
    selected = (headPCD(:,3) <= th1) & (headPCD(:,3) >= th2);
    T = mean(headPCD(selected, :), 1) + [0 0 50];
else
    T = zeros(1, 3);
end

if ~isempty(kalmanFilter)
    variance = diag(kalmanFilter.StateCovariance);
    covariance = diag(variance(1:2:end));
end

for i=1:20
    inliers = [];
    threshold = 50;
    while (sum(inliers) < 0.1 * size(headPCD, 1)) %&& (threshold < 200)
        distFromBoundary = sphere_fit(headPCD, centroid + T(end, :), radius, [0 0 0]);
        inliers = abs(distFromBoundary) < threshold;
        threshold = threshold + 20;
    end
    
    if sum(inliers) < 0.1 * size(headPCD, 1)
        translation = [0 0 0];
        return;
    end
    if isempty(kalmanFilter)
        fitfunc = @(x)sphere_fit(headPCD(inliers, :), centroid + T(end, :), radius, x); % distance of points to the boundary of a sphere
    else
        fitfunc = @(x)sphere_fit_constrained(headPCD(inliers, :), centroid + T(end, :), radius, x, centroid, covariance); % distance of points to the boundary of a sphere
    end
    t = lsqnonlin(fitfunc, zeros(1,3), [], [], options);
    T(i+1, :) = T(i, :) + t;
    if sum(t.^2) < 1
        break;
    end
%     imshow(Im);
%     hold on;
%     newpos = headPCD(inliers,:);
%     xpos = newpos(:,1)./newpos(:,3) * fx + cx;
%     ypos = newpos(:,2)./newpos(:,3) * fy + cy;
%     plot(xpos,ypos,'r+');
%     
%     newpos = bsxfun(@plus, sphere, centroid+T(end, :));
%     xpos = newpos(:,1)./newpos(:,3) * fx + cx;
%     ypos = newpos(:,2)./newpos(:,3) * fy + cy;
%     plot(xpos,ypos,'g+');
%     drawnow;
%     hold off;
%     pause;
end

translation = T(end, :);
end

function distFromBoundary = sphere_fit(pcd, centroid, radius, t)

centroid = centroid + t;
distFromBoundary = bsxfun(@minus, pcd, centroid);
distFromBoundary = sqrt(sum(distFromBoundary.^2, 2)) - radius;
end

function distFromBoundary = sphere_fit_constrained(pcd, centroid, radius, t, prevPos, covariance)

centroid = centroid + t;
prob = exp(((centroid-prevPos) / (covariance)) * (centroid-prevPos)');
prob = max(prob, 0.02);
prob = 1;
distFromBoundary = bsxfun(@minus, pcd, centroid);
distFromBoundary = (sqrt(sum(distFromBoundary.^2, 2)) - radius) / prob;
end