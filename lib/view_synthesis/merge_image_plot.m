function merge_image_plot(impath2, plot3d, savepath2)
% impath = '/home/arri/Dataset/FN038_processed/reunion2/new_view1_track';
% impath2 = '/home/arri/Dataset/FN038_processed/reunion2/video';
% plot3d = '/home/arri/Dataset/FN038_processed/reunion2/plot3d';
% plot2d = '/home/arri/Dataset/FN038_processed/reunion2/plot2d';
% savepath = '/home/arri/Dataset/FN038_processed/reunion2/trackplot1';
% savepath2 = '/home/arri/Dataset/FN038_processed/reunion2/trackplot2';

mkdir(savepath2)

% for i=1:5486
% %     i
%     im = imread(fullfile(impath,num2str(i,'%d.png')));
%     pl2d = imread(fullfile(plot2d,num2str(i,'%d.jpg')));
%     pl3d = imread(fullfile(plot3d,num2str(i,'%d.jpg')));
%     
%     pl2d = imresize(pl2d,[300 size(im,2)]);
%     pl3d = imresize(pl3d,[300 size(im,2)]);
%     
%     imwrite([im;pl2d;pl3d],fullfile(savepath,num2str(i,'%d.jpg')),'Quality',90);
% end

filelist = dir(impath2);
filelist = filelist(3:end);
for i=1:length(filelist)
    if mod(i,100) == 1
        i
    end
    im = imread(fullfile(impath2,filelist(i).name));
%     pl2d = imread(fullfile(plot2d,num2str(i,'%d.jpg')));
    pl3d = imread(fullfile(plot3d,[filelist(i).name(1:end-3), 'jpg']));
    
%     pl2d = imresize(pl2d,[300 size(im,2)]);
    pl3d = imresize(pl3d,[400 size(im,2)]);
    
%     imwrite([im;pl2d;pl3d],fullfile(savepath2,num2str(i,'%d.jpg')),'Quality',90);
    imwrite([im;pl3d],fullfile(savepath2,[filelist(i).name(1:end-3), 'jpg']),'Quality',90);
end