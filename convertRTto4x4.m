clear;clc;
load('C:\Users\eprince\Desktop\Images\Stereo\Calib_Results_stereo.mat');

x_d = repmat([1:nx], ny, 1);
y_d = repmat([1:ny]', 1, nx);

fx_d = fc_right(1);
fy_d = fc_right(2);
cx_d = cc_right(1);
cy_d = cc_right(2);

fx_c = fc_left(1);
fy_c = fc_left(2);
cx_c = cc_left(1);
cy_c = cc_left(2);

A = [eye(3), T; 0 0 0 1];
B = [R, [0;0;0]; 0 0 0 1];
extrinsics = A * B;

x_cen = (x_d - cx_d) / fx_d;
y_cen = (y_d - cy_d) / fy_d;

XYZ = extrinsics * [x_cen(:)'; y_cen(:)'; ones(1,numel(x_cen)); ones(1, numel(x_cen))];

x_c = (XYZ(1,:) - cx_c) * fx_c;

DtC.X = (x_d - cx_d) / fx_d;
DtC.Y = (y_d - cy_d) / fy_d;

save('FDtC.mat','DtC')
% DtC.X = ()
% depthData = 
% 
% rgbData = 
% 
% A = [eye(3), T; 0 0 0 1];
% B = [R, [0;0;0]; 0 0 0 1];
% extrinsics = A * B;
% 
% 
% fx_rgb = fc_left(1);
% fy_rgb = fc_left(2);
% cx_rgb = cc_left(1);
% cy_rgb = cc_left(2);
% 
% 
% [aligned] = depth_rgb_registration(depthData, rgbData,...
%                     fx_d, fy_d, cx_d, cy_d,...
%                     fx_rgb, fy_rgb, cx_rgb, cy_rgb,...
%                     extrinsics);