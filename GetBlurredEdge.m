function edge_blur = GetBlurredEdge(I, mode)
% mode = 1 for depth image, mode = 2 for color image

if mode == 1
    I = double(I);
%     I = smooth_depth(I .* (I >0));
    [grad] = imgradient(I);
else
%     I = rgb2gray(rgb2hsv(I));
    I = double(I);
    [grad1] = imgradient(I(:,:,1));
    [grad2] = imgradient(I(:,:,2));
    [grad3] = imgradient(I(:,:,3));
    grad = max(max(grad1, grad2), grad3);

%     I = rgb2gray(I);
%     I = double(I);
%     h = fspecial('gaussian',[15 15],7);
%     edge_blur = imfilter(I, h, 'replicate');
%     return;
end
% I = gpuArray(I);


h = fspecial('gaussian',[15 15],7);

% th for depth image is 500
th = 250; %500
if mode == 2
    th = 100;
end
edge_blur = imfilter(double(grad > th), h, 'replicate'); 

% edge_blur = gather(edge_blur);
end

