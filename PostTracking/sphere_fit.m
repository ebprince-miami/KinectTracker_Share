function distFromBoundary = sphere_fit(pcd, centroid, radius, t)

centroid = centroid + t;
distFromBoundary = bsxfun(@minus, pcd, centroid);
distFromBoundary = sqrt(sum(distFromBoundary.^2, 2)) - radius;
end