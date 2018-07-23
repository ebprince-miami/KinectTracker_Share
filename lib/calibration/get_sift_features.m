function count = get_sift_features()

datapath = 'C:\Users\Explorer\Documents\Dataset\sfm';
idx = [7 4 3 2 5 6 8 9 10 11 12];
count = [];
for i=1:length(idx)
    load(fullfile(datapath,['livingroom',num2str(idx(i))],'USB-VID_045E&PID_02AE-A00365904718133A_1.mat'),'ColorFrame');
    Ia = ColorFrame.ColorData;
    [fa,da] = vl_sift(im2single(rgb2gray(Ia))) ;
    count(i,:) = [i size(fa,2)];
    figure;imshow(Ia);
    hold on;
    vl_plotframe(fa);
    hold off;
end
avg = sum(count(:,2))/length(idx)
