function [fx, fy, cx, cy] = compute_K(pcd)

pcdX = pcd(:,:,1);
pcdY = pcd(:,:,2);
pcdZ = pcd(:,:,3);
valid = find(pcdZ>0);

[I,J] = ind2sub(size(pcdX), valid);

X = pcdX(valid);
Y = pcdY(valid);
Z = pcdZ(valid);

temp = [X./Z, ones(size(X))] \ J;
fx = temp(1);
cx = temp(2);
temp = [Y./Z, ones(size(Y))] \ I;
fy = temp(1);
cy = temp(2);