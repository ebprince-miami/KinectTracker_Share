function writePly(pcd_color,fileName)
% pcd_color is (x,y,z,r,g,b)

noOfpoints = size(pcd_color,1);
fid = fopen(fileName, 'wt');
fprintf(fid,'ply\nformat ascii 1.0\ncomment zipper output\nelement vertex %d\n', noOfpoints);
fprintf(fid,'property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nelement face 0\n');
fprintf(fid,'property list uchar int vertex_indices\nend_header\n');

for ii = 1 : noOfpoints
    fprintf(fid,'%f %f %f %d %d %d\n', pcd_color(ii,1), pcd_color(ii,2), pcd_color(ii,3), pcd_color(ii,4), pcd_color(ii,5), pcd_color(ii,6));
end

fclose(fid);