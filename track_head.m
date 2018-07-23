function track_head()

% savepath = 'C:\Users\Explorer\Documents\Dropbox\Dropbox\Dataset\SS\images_9520-9674';
dirpath = '/Users/karanjitcheema/Desktop/RA_Work/Strange Situation/Data';

% start = 450;
start = 520;
range = 1000;
% offset = 3666;
% old name

fname = 'USB-VID_045E&PID_02AE-A00366809492052A';
load(fullfile(dirpath,[fname,'_',num2str(start),'.mat']),'ColorFrame','DepthFrame');
load(fullfile(dirpath,[fname,'_DepthFrameToColorFrameMapping.mat']));
% new name
% fname_color = 'color_USB-VID_0409&PID_005A-5&152EDADC&0&5_';
% fname_depth = 'depth_USB-VID_0409&PID_005A-5&152EDADC&0&5_';

c_img = ColorFrame.ColorData;
d_img = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DepthFrameToColorFrameMapping);

figure;imshow(c_img);
% figure;imshow(d_img);
% upper left first, then lower right of the head
[x y] = ginput(2);
% save('xy_3.mat','x','y');
% load('xy.mat');

head_length = sqrt((x(2)-x(1))^2 + (y(2)-y(1))^2);
x(3) = (x(1)+x(2))/2;
y(3) = (y(1)+y(2))/2;
x = round(x);
y = round(y);
center_history = [x(3) y(3)];

length_th = 1;
limit_length = head_length/2 + length_th;

imsize = size(d_img);
img_x = repmat(1:imsize(2),imsize(1),1);
img_y = repmat([1:imsize(1)]',1,imsize(2));

head_depth(1) = d_img(y(1),x(1));
head_depth(2) = d_img(y(2),x(2));
head_depth(3) = d_img(y(3),x(3));

min_depth = min(head_depth);
max_depth = max(head_depth); 
min_depth = (max_depth+min_depth)/2 - 50;
max_depth = (max_depth+min_depth)/2 + 50;
head_depth
% depth_th = (max_depth - min_depth) * 2;

%%% get color model for head
%%% Set offx and offy to 0 if the depth and color are aligned
offx = 0; offy = 0;
% offset for 1 old
% offx = 16; offy = 6;
% offset for 3 old
% offx = 6; offy = 14;
% offset for 1 new
% offx = 0; offy = 6;
adjx = offx/2; adjy = offy/2;
x_color = x; %x.*2 + 1 + offx;
y_color = y; %y.*2 + 1 + offy;
ch = c_img(y_color(1):y_color(2),x_color(1):x_color(2),:);
ch_r = single(ch(:,:,1));
ch_g = single(ch(:,:,2));
ch_b = single(ch(:,:,3));
gm_child = gmmb_em([ch_r(:) ch_g(:) ch_b(:)],'components',1);

depth_th = 100;
draw_offset = 2;
old_head = [];
for i=1:range%100
    i
    load(fullfile(dirpath,[fname,'_',num2str(start+i),'.mat']),'ColorFrame','DepthFrame');
    c_img = ColorFrame.ColorData;
    d_img = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DepthFrameToColorFrameMapping);
    x_diff = abs(img_x - x(3));
    y_diff_top = y(3) - img_y;
    y_diff_top(y_diff_top<0) = 10^5;
    y_diff_bot = img_y - y(3);
    y_diff_bot(y_diff_bot<0) = 10^5;
    
    % Get prior from color
    Data = [reshape(c_img(:,:,1),size(c_img,1)*size(c_img,2),1) reshape(c_img(:,:,2),size(c_img,1)*size(c_img,2),1) reshape(c_img(:,:,3),size(c_img,1)*size(c_img,2),1)];
    Data = double(Data);
    prob = gmmb_pdf(Data,gm_child);
    prob = prob./max(prob);
    prob = reshape(prob,[480 640]);
    %prob = imresize(prob,0.5);
    prob = prob>0.4;
    mask = zeros(480,640);
    mask(1:end-adjy,1:end-adjx) = prob(adjy+1:end,adjx+1:end);
    
    try
        new_head = mask & (d_img <= max_depth+depth_th) & (d_img >= min_depth-depth_th) & (x_diff <= limit_length) & ((y_diff_top <= limit_length+2) | (y_diff_bot <= limit_length+2));
    catch
        new_head = old_head;
    end
    if sum(new_head(:))==0
        new_head = old_head;
        disp('Using previous result');
    else
        old_head = new_head;
    end
    
%     x(1) = max(img_x(:) .* new_head(:));
%     min_x_img = img_x(:) .* new_head(:);
%     min_x_img(min_x_img==0) = 10^4;
%     x(2) = min(min_x_img);
%     y(1) = max(img_y(:) .* new_head(:));
%     min_y_img = img_y(:) .* new_head(:);
%     min_y_img(min_y_img==0) = 10^4;
%     y(2) = min(min_y_img);
%     x(3) = (x(1)+x(2))/2;
%     y(3) = (y(1)+y(2))/2;
    
    % using EM to find the center 
    xpos = img_x(:) .* new_head(:);
    xpos = xpos(xpos>0);
    ypos = img_y(:) .* new_head(:);
    ypos = ypos(ypos>0);
    try
        if length(xpos)>1
            gm = gmmb_em([xpos ypos],'components', 1);
        else
            gm.mu(1) = xpos;
            gm.mu(2) = ypos;
        end
    catch
        disp('');
    end
    x(3) = gm.mu(1);
    y(3) = gm.mu(2);
    % using means
%     x(3) = sum(img_x(:) .* new_head(:)) ./ sum(new_head(:));
%     y(3) = sum(img_y(:) .* new_head(:)) ./ sum(new_head(:));
    
    x(1) = x(3) - limit_length+length_th;
    x(2) = x(3) + limit_length-length_th;
    y(1) = y(3) - limit_length+length_th;
    y(2) = y(3) + limit_length-length_th;
    x = round(x);
    y = round(y);
    center_history = [center_history;[x(3) y(3)]];

%     head_depth(1) = d_img(y(1),x(1));
%     head_depth(2) = d_img(y(2),x(2));
%     head_depth(3) = d_img(y(3),x(3));
    hd = d_img(:) .* new_head(:);
    hd = hd(hd>0);
    hd_range = unique(d_img .* new_head);
    if hd_range(1) == 0
        hd_range = hd_range(2:end);
    end
    
    head_depth(1) = min(hd_range);
    head_depth(2) = max(hd_range);
    head_depth(3) = (head_depth(1)+head_depth(2))/2;
    
%     min_depth = min(head_depth);
%     max_depth = max(head_depth);
%     new_min = (max_depth+min_depth)/2 - 1;
%     new_max = (max_depth+min_depth)/2 + 1;
    if length(hd)>1
        mean_depth = gmmb_em(single(hd),'components', 1);
    else
        mean_depth.mu = hd;
    end
    mean_depth = mean_depth.mu;
    min_depth = mean_depth-50;
    max_depth = mean_depth+50;
    [min_depth max_depth]
%     imshow(new_head);
%     head_depth
%     imshow(d_img);
%     hold on;
%     plot(x(1),y(1),'r+');
%     plot(x(2),y(2),'g+');
%     plot(x(3),y(3),'c+');
%     hold off;

    % draw color image
    close;
    imshow(c_img);
    x_plot = [x(1)-draw_offset x(1)-draw_offset x(2)+draw_offset x(2)+draw_offset x(1)-draw_offset];
    y_plot = [y(1)-draw_offset y(2)+draw_offset y(2)+draw_offset y(1)-draw_offset y(1)-draw_offset];
    % x_plot = x_plot.*2+1+offx;
    % y_plot = y_plot.*2+1+offy;
    hold on;
    plot(x_plot,y_plot,'r','LineWidth',2);
    drawnow;
    % save image
%     hdl = get(0,'CurrentFigure');
%     saveas(hdl,fullfile(savepath,num2str(i,'%05d.jpg')));

    % drawh history of the track
    if i>1
        ch = center_history;
        ch(:,1) = ch(:,1) + 1 + offx;
        ch(:,2) = ch(:,2) + 1 + offy;
        plot(ch(1,1),ch(1,2),'g+')
        for j=1:i-1
            plot([ch(j,1) ch(j+1,1)],[ch(j,2) ch(j+1,2)],'b','LineWidth',2);
            plot(ch(j+1,1),ch(j+1,2),'g+');
        end
        hdl = get(0,'CurrentFigure');
% % % % %         saveas(hdl,fullfile(savepath,['hist_',num2str(i,'%05d.jpg')]));
    end
end

% draw history of the track
ch = center_history;
ch(:,1) = ch(:,1) + 1 + offx;
ch(:,2) = ch(:,2) + 1 + offy;
plot(ch(1,1),ch(1,2),'g+')
for i=1:range-1
    plot([ch(i,1) ch(i+1,1)],[ch(i,2) ch(i+1,2)],'b','LineWidth',2);
    plot(ch(i+1,1),ch(i+1,2),'g+');
end
disp('end');
save('center_history.mat','center_history');
