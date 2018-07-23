dirpath = '/Users/karanjitcheema/Desktop/RA_Work/StrangeSituation/Data/127A';
fname = 'USB-VID_045E&PID_02AE-A00364A04906127A';
fid = fopen('/Users/karanjitcheema/Desktop/RA_Work/StrangeSituation/reunion_1/results_127A_mother/centres_127A_mother.txt', 'r');
fid2 = fopen('/Users/karanjitcheema/Desktop/RA_Work/StrangeSituation/reunion_1/results_127A_baby/centres_127A_baby.txt', 'r');
load(fullfile(dirpath,[fname,'_DepthFrameToColorFrameMapping.mat']));

i=1
for start = 21650:21950

    load(fullfile(dirpath,[fname,'_',num2str(start),'.mat']),'ColorFrame','DepthFrame');
    d_img = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DepthFrameToColorFrameMapping);
    A = fscanf(fid,'%d %f %f %f %f', 5);
    depth_values = zeros(uint8(A(5)),uint8(A(4)));
    for y = uint16(A(3) - (A(5)/2)) : uint16(A(3) + (A(5)/2))
        for x = uint16(A(2) - (A(4)/2)) : uint16(A(2) + (A(4)/2))
             if(y<481 && y>0 && x < 641 && x>0)
                depth_values(y - uint16(A(3) - (A(5)/2)) +1, x - uint16(A(2) - (A(4)/2)) +1  ) = d_img(y,x);
             else
                depth_values(y - uint16(A(3) - (A(5)/2)) +1, x - uint16(A(2) - (A(4)/2)) +1  ) = 0;
             end
        end
    end
    median_depth = median(depth_values(:));
    [row,col] = find(depth_values == median_depth);
    x_value = col(1) + uint16(A(2) - (A(4)/2)) ;
    y_value = row(1) + uint16(A(3) - (A(5)/2)) ;
% %     now find the X and Y values in the depth mapping...
    [found_row,found_col] = find(DepthFrameToColorFrameMapping.X(1,:) == x_value);
    if(numel(found_col) ==0 ||numel(found_col) ==0)
        continue;
    end
    new_x_value = found_col(1);
    [found_row,found_col] = find(DepthFrameToColorFrameMapping.Y(:,1) == y_value);
    if(numel(found_col) ==0)
        continue;
    end
    new_y_value = found_row(1);
% %     
    xlim([-1500 0])
    ylim([500 3000])
    zlim([-500 1500])
    three_d_x = DepthFrame.DepthData.X(new_y_value ,new_x_value );
    three_d_y = DepthFrame.DepthData.Y(new_y_value ,new_x_value );
    three_d_z = DepthFrame.DepthData.Depth(new_y_value ,new_x_value );
    plot3(three_d_x,three_d_z,three_d_y,'LineWidth',50);
    file_name = strcat('/results/',int2str(i));
    i = i +1;
% %     Fig1Ax1 = get(1, '')
% % %     set(Fig1Ax1 , 'LineWidth', 5);
% %     set(gcf,'LineWidth',5);
% % h = gcf;
% % axesObjs = get(h, 'Children');
% % dataObjs = get(axesObjs, 'Children'); 
% % % % % set(dataObjs , 'LineWidth', 5);
% % % set(findall(gcf,'type','line'),'LineWidth',50)
saveas(gcf,fullfile(dirpath,[file_name,'.png']));
        hold on;
end


for start = 21755:25000
    load(fullfile(dirpath,[fname,'_',num2str(start),'.mat']),'ColorFrame','DepthFrame');
    d_img = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DepthFrameToColorFrameMapping);
    
    A = fscanf(fid2,'%d %f %f %f %f', 5);
    depth_values = zeros(uint8(A(5)),uint8(A(4)));
    for y = uint16(A(3) - (A(5)/2)) : uint16(A(3) + (A(5)/2))
        for x = uint16(A(2) - (A(4)/2)) : uint16(A(2) + (A(4)/2))
            if(y<481 && x < 641)
                depth_values(y - uint16(A(3) - (A(5)/2)) +1, x - uint16(A(2) - (A(4)/2)) +1  ) = d_img(y,x);
            else
                depth_values(y - uint16(A(3) - (A(5)/2)) +1, x - uint16(A(2) - (A(4)/2)) +1  ) = 0;
            end
        end
    end
    median_depth = uint16(median(depth_values(:)));
    [row,col] = find(depth_values == median_depth);
    if(numel(col)==0)
        i  = i+1;
        continue;
    end
    x_value = col(1) + uint16(A(2) - (A(4)/2)) ;
    y_value = row(1) + uint16(A(3) - (A(5)/2)) ;
% %     now find the X and Y values in the depth mapping...
    [found_row,found_col] = find(DepthFrameToColorFrameMapping.X(1,:) == x_value);
    new_x_value = found_col(1);
    [found_row,found_col] = find(DepthFrameToColorFrameMapping.Y(:,1) == y_value);
    new_y_value = found_row(1);
%   
    xlim([-1500 0])
    ylim([500 3000])
    zlim([-500 1500])
    three_d_x = DepthFrame.DepthData.X(new_y_value ,new_x_value );
    three_d_y = DepthFrame.DepthData.Y(new_y_value ,new_x_value );
    three_d_z = DepthFrame.DepthData.Depth(new_y_value ,new_x_value );
    plot3(three_d_x,three_d_z,three_d_y,'LineWidth',5);
    file_name = strcat('/results/',int2str(i));
    i = i +1;
    saveas(gcf,fullfile(dirpath,[file_name,'.png']));
    hold on;
end

xlabel('x')
ylabel('depth')
zlabel('y')
fclose(fid);
fclose(fid2);