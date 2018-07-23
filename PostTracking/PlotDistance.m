function PlotDistance(datapath)

events = dir(fullfile(datapath,'*_range.txt'));

baby = dir(fullfile(datapath,'baby*.txt'));
mother = dir(fullfile(datapath,'mother*.txt'));
if isempty(mother)
    mother = dir(fullfile(datapath,'mom*.txt'));
end
toy = dir(fullfile(datapath,'toy*.txt'));

svpath = datapath;
svpath(1) = 'C';
for i=1:length(events)
        events(i).name
        rangestr = textread(fullfile(datapath,events(i).name),'%s');
        sep = regexp(rangestr{1},':');
        range = [str2double(rangestr{1}(1:sep-1)):str2double(rangestr{1}(sep+1:end))];

        tracking_viz_path = fullfile(datapath, num2str(str2num(events(i).name(end-10)),'Pos3D_R%d_filtered'));

        savepath = fullfile(svpath, num2str(str2num(events(i).name(end-10)),'Pos3D_R%d_distanceplot'));
%         savepath_combined = fullfile(svpath, num2str(str2num(events(i).name(end-10)),'Pos3D_R%d_combinedplot'));
%         mkdir(savepath);
%         mkdir(savepath_combined);
%         
%         load([tracking_viz_path, '.mat'], 'mPos3DInterpolated', 'bPos3DInterpolated');
%         dist3d = sqrt(sum((mPos3DInterpolated - bPos3DInterpolated).^2, 2));
% 
%         figure;
%         
%         for j=1:length(dist3d)
%             hold on;
%             axis([0 length(dist3d) 0 3500])
%             xlabel('Time','fontsize',24,'fontweight','b');
%             ylabel('Infant-Mother Distance (mm)','fontsize',24,'fontweight','b');
% 
%             if mod(j,100)==1
%                 disp(['Writing ', savepath, ': ', num2str(j/length(dist3d) * 100)]);
%             end
%             plot(dist3d(1:j), 'k', 'LineWidth',3)
%             saveas(gcf,fullfile(savepath,[num2str(j),'.png']));
%             cla reset;
%         end
%         hold off;
%         close;

        system(['ffmpeg -r 30 -i ', fullfile(savepath, '%d.png'), ' -b 3500k -r 30 ', savepath, '.avi']);
        
%         if exist(tracking_viz_path)
%             filelist = dir(fullfile(tracking_viz_path,'*.jpg'));
%             for j=1:length(filelist)
%                 if mod(j,100)==1
%                     disp(['Writing ', savepath_combined, ': ', num2str(j/length(dist3d) * 100)]);
%                 end
%                 im = imread(fullfile(tracking_viz_path,filelist(j).name));
%                 pl3d = imread(fullfile(savepath,[filelist(j).name(1:end-3), 'png']));
%                 pl3d = imresize(pl3d,[600 size(im,2)]);
% 
%                 imwrite([im;pl3d],fullfile(savepath_combined,[filelist(j).name(1:end-3), 'jpg']),'Quality',90);
%             end
%             system(['ffmpeg -r 30 -i ', fullfile(savepath_combined, '%d.jpg'), ' -b 3500k -r 30 ', savepath_combined, '.avi']);
%         end
end