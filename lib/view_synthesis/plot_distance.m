
function plot_distance(pos3d,pos2d, savepath)
savepath1 = [savepath, '_dist1'];
savepath2 = [savepath, '_dist2'];
mkdir(savepath1);
mkdir(savepath2);
filt = CreateGaussFilter(3, 8);

% for i=1:2:size(pos3d,2)
%     dist(ceil(i/2)) = norm(pos2d(1:2,i)-pos2d(1:2,i+1));
% end
for i=1:2:size(pos3d,2)
    idx = ceil(i/2);
    dist3d(idx) = norm(pos3d(1:3,i)-pos3d(1:3,i+1));
    if idx == 1
        babySpeed(idx) = 0;
        momSpeed(idx) = 0;
    else
        babySpeed(idx) = dist3d(idx - 1) - norm(pos3d(1:3,i)-pos3d(1:3,i+1-2));
        momSpeed(idx) = dist3d(idx - 1) - norm(pos3d(1:3,i-2)-pos3d(1:3,i+1));
    end
end
% babySpeed = medfilt1(babySpeed, 5);
% momSpeed = medfilt1(momSpeed, 5);

% dist3d = conv(dist3d, filt, 'same');
babySpeed = conv(babySpeed, filt, 'same');
momSpeed = conv(momSpeed, filt, 'same');

for i=1:2:size(pos3d,2)
    hdiff(ceil(i/2)) = norm(pos3d(3,i)-pos3d(3,i+1));
end

% figure;
% hold on;
% axis([0 7000 0 400])
% % plot(1:1146,dist(1:1146),'b+');
% for i=1:length(dist)
%     if i>1
%         plot(i-1,dist(i-1),'b+');
%     end
%     plot(i,dist(i),'r+');
%     drawnow;
%     saveas(gcf,fullfile('/Users/karanjitcheema/Desktop/reunion_dist',[num2str(i),'.png']));
% end


% figure;
% hold on;
% axis([0 7000 0 400])

% for i=1:length(dist)
%     axis([0 7000 0 400]);
%     hold on;
%     
%     if i>1
%         plot(1:i,dist(1:i),'b+');
%     end
% %     tic;    
% %     plot(i,dist3d(i),'b+');
%      
%     saveas(gcf,fullfile('/home/arri/Dataset/FN038_processed/reunion2/plot2d',[num2str(i),'.jpg']));
%     cla reset;
%     
% end

for i=1:length(dist3d)
    if (sum(pos3d(1:3,i*2)==[0;0;0]) == 3) || (sum(pos3d(1:3,i*2-1)==[0;0;0]) == 3)
        dist3d(i) = 1e9;
    end
end

figure;
hold on;
axis([0 6000 0 3500])
xlabel('time','fontsize',48,'fontweight','b');
ylabel('distance (mm)','fontsize',48,'fontweight','b');
%title('Baby-Mother Distance','fontsize',48,'fontweight','b');
title('Baby-Toy Distance','fontsize',48,'fontweight','b');
sel = [];

% temporary
% interpolate dist3d on missing data
missing_data_idx = find(dist3d > 1e6);
valid_data_idx = setdiff(1:length(dist3d), missing_data_idx);
dist3d_orig = dist3d;
for i=1:length(missing_data_idx)
    dif = valid_data_idx - missing_data_idx(i);
    dif(dif>0) = -1e9;
    [a, prev] = max(dif);
    if a== -1e9
        prev = [];
    end
    dif = valid_data_idx - missing_data_idx(i);
    dif(dif<0) = 1e9;
    [a, next] = min(dif(dif >= 0));
    if a== 1e9
        next = [];
    end
    prev_closest = [];
    next_closest = [];
    if isempty(prev)
        prev_closest = valid_data_idx(next);
    else
        prev_closest = valid_data_idx(prev);
    end
    
    if isempty(next)
        next_closest = valid_data_idx(prev);
    else
        next_closest = valid_data_idx(next);
    end
    
    range = next_closest - prev_closest;
    
    if range > 0
        dist3d(missing_data_idx(i)) = (range - (missing_data_idx(i) - prev_closest)) / range * dist3d_orig(prev_closest) + ...
                            (range - (next_closest - missing_data_idx(i))) / range * dist3d_orig(next_closest);
    end
end

plot(1:length(dist3d),dist3d,'b+')
return;
% end temporary
for i=1:length(dist3d)
    axis([0 6000 0 3500])
    xlabel('time','fontsize',48,'fontweight','b');
    ylabel('distance (mm)','fontsize',48,'fontweight','b');
    title('Baby-Mother Distance','fontsize',48,'fontweight','b');
    hold on;
    if mod(i,100) == 1
        i
    end
    
    if i>1
        plot(1:i,dist3d(1:i),'b+');
        
        if babySpeed(i) > 5
            sel = [sel i];
        end
%         plot(sel,dist3d(sel),'g+');
%         plot(1:i,babySpeed(1:i)*50,'r+');
%         plot(1:i,momSpeed(1:i)*50,'g+');
    end
%     drawnow;
%     tic;    
%     plot(i,dist3d(i),'b+');
     
    saveas(gcf,fullfile(savepath1,[num2str(i),'.png']));
    if i>1
        plot(sel,dist3d(sel),'g+');
    end
    
    saveas(gcf,fullfile(savepath2,[num2str(i),'.png']));
    cla reset;
    
end

% figure;hold on;
% axis([0 1650 0 300])
% plot(1:length(dist),dist,'b+');
% plot(1:length(dist),hdiff,'r+');
% plot(1:length(dist),dist3d,'g+');
% xlabel('t','fontsize',32,'fontweight','b');
% ylabel('distance','fontsize',32,'fontweight','b');
% legend('2d distance','height difference','3d distance','NorthEast')

end

function gaussFilter = CreateGaussFilter(sigma, size)

sigma = 5;
size = 30;
x = linspace(-size / 2, size / 2, size);
gaussFilter = exp(-x .^ 2 / (2 * sigma ^ 2));
gaussFilter = gaussFilter / sum (gaussFilter); % normalize

end