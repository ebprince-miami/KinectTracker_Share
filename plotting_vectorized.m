mom_file = fopen('/Users/karanjitcheema/Desktop/FN026/centres_mother_reunion1_cam2,3.txt', 'r');
baby_file = fopen('/Users/karanjitcheema/Desktop/FN026/centres_baby_reunion1_cam2,3.txt', 'r');
mother = struct([]);
baby = struct([]);

i=1;
while(~feof(mom_file))
    A = fscanf(mom_file,'%f %f %f %f %f %f %f %f', 8);
    mother(i).cam  = A(1);
    mother(i).frame  = A(2);
    mother(i).valid  = A(3);
    mother(i).offset  = A(4);
    mother(i).x  = A(5);
    mother(i).y  = A(6);
    mother(i).height  = A(7);
    mother(i).width  = A(8);
    i = i+1;
end
i=1;
while(~feof(baby_file))
    A = fscanf(baby_file,'%f %f %f %f %f %f %f %f', 8);
    baby(i).cam  = A(1);
    baby(i).frame  = A(2);
    baby(i).valid  = A(3);
    baby(i).offset  = A(4);
    baby(i).x  = A(5);
    baby(i).y  = A(6);
    baby(i).height  = A(7);
    baby(i).width  = A(8);
    i = i+1;
end

directory = '/Users/karanjitcheema/Desktop/FN026/Data/';
% filename_1_prefix = strcat(directory,'USB-VID_045E&PID_02AE-A00364A04906127A_');
filename_2_prefix = strcat(directory,'USB-VID_045E&PID_02AE-A00367A14179123A_');
filename_3_prefix = strcat(directory,'USB-VID_045E&PID_02AE-A00365A09389104A_');
% filename_4_prefix = strcat(directory,'USB-VID_045E&PID_02AE-B00362214481051B_');

% cam1_depth_mapping = load(fullfile([filename_1_prefix,'DepthFrameToColorFrameMapping.mat']));
cam2_depth_mapping = load(fullfile([filename_2_prefix,'DepthFrameToColorFrameMapping.mat']));
cam3_depth_mapping = load(fullfile([filename_3_prefix,'DepthFrameToColorFrameMapping.mat']));
% cam4_depth_mapping = load(fullfile([filename_4_prefix,'DepthFrameToColorFrameMapping.mat']));

three_d_coordinates  = struct([]);
j=1;
valid = 0;
prob =0;
for i = 1:length(mother)
    if(mother(i).valid ==0)
        continue;
    end
    valid = valid +1;
    three_d_coordinates(j).line_num = i;
    
    if(mother(i).cam == 1)
        load(fullfile([filename_1_prefix,num2str(mother(i).frame),'.mat']),'ColorFrame','DepthFrame');
        d_img = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,cam1_depth_mapping.DepthFrameToColorFrameMapping);
    elseif(mother(i).cam ==2)
        load(fullfile([filename_2_prefix,num2str(mother(i).frame),'.mat']),'ColorFrame','DepthFrame');
        d_img = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,cam2_depth_mapping.DepthFrameToColorFrameMapping);
    elseif(mother(i).cam ==3)
        load(fullfile([filename_3_prefix,num2str(mother(i).frame),'.mat']),'ColorFrame','DepthFrame');
        d_img = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,cam3_depth_mapping.DepthFrameToColorFrameMapping);
    elseif(mother(i).cam ==4)
        load(fullfile([filename_4_prefix,num2str(mother(i).frame),'.mat']),'ColorFrame','DepthFrame');
        d_img = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,cam4_depth_mapping.DepthFrameToColorFrameMapping);
    end
    
    
    
%     depth_values = zeros(mother(i).height,mother(i).width);
    center_x = mother(i).x;
    center_y = mother(i).y;
    width = mother(i).width;
    height = mother(i).height;
    min_x = floor(center_x - width/2);
    max_x = floor(center_x + width/2);
    min_y = floor(center_y - height/2);
    max_y = floor(center_y + height/2);
    if(i==15)
    end
    if(min_x<=0)
        min_x = 1;
    end
    if(max_x > 640)
        max_x = 640;
    end
    if(min_y<=0)
        min_y=1;
    end
    if(max_y >480)
        max_y =480;
    end
    
    depth_values = d_img(min_y:max_y,min_x:max_x);
    
% %     for y = uint16(A(3) - (A(5)/2)) : uint16(A(3) + (A(5)/2))
% %         for x = uint16(A(2) - (A(4)/2)) : uint16(A(2) + (A(4)/2))
% %              if(y<481 && y>0 && x < 641 && x>0)
% %                 depth_values(y - uint16(A(3) - (A(5)/2)) +1, x - uint16(A(2) - (A(4)/2)) +1  ) = d_img(y,x);
% %              else
% %                 depth_values(y - uint16(A(3) - (A(5)/2)) +1, x - uint16(A(2) - (A(4)/2)) +1  ) = 0;
% %              end
% %         end
% %     end
    median_depth = median(depth_values(:));
    B = unique(depth_values(:));
    C = (B - median_depth);
    D = min(abs(C));
    closest_value = min(abs(unique(depth_values(:))- median_depth));
    [row,col] = find(depth_values == (median_depth+ closest_value));
    if(isempty(row))
        [row,col] = find(depth_values == (median_depth -  closest_value));
    end
    x_value = floor(col(floor(length(row)/2)) + min_x);
    y_value = floor(row(floor(length(row)/2)) + min_y);
    
% %     now find the X and Y values in the depth mapping...
    if(mother(i).cam == 1)
        [found_row,found_col] = find(cam1_depth_mapping.DepthFrameToColorFrameMapping.X(1,:) == x_value);
    elseif(mother(i).cam == 2)
        [found_row,found_col] = find(cam2_depth_mapping.DepthFrameToColorFrameMapping.X(1,:) == x_value);
    elseif(mother(i).cam == 3)
        [found_row,found_col] = find(cam3_depth_mapping.DepthFrameToColorFrameMapping.X(1,:) == x_value);
    elseif(mother(i).cam == 4)
        [found_row,found_col] = find(cam4_depth_mapping.DepthFrameToColorFrameMapping.X(1,:) == x_value);
    end
    if(numel(found_col) ==0 ||numel(found_col) ==0)
        x_value
        prob = prob +1;
        continue;
    end
    new_x_value = found_col(1);
    
    if(mother(i).cam == 1)
        [found_row,found_col] = find(cam1_depth_mapping.DepthFrameToColorFrameMapping.Y(:,1) == y_value);
    elseif(mother(i).cam == 2)
        [found_row,found_col] = find(cam2_depth_mapping.DepthFrameToColorFrameMapping.Y(:,1) == y_value);
    elseif(mother(i).cam == 3)
        [found_row,found_col] = find(cam3_depth_mapping.DepthFrameToColorFrameMapping.Y(:,1) == y_value);
    elseif(mother(i).cam == 4)
        [found_row,found_col] = find(cam4_depth_mapping.DepthFrameToColorFrameMapping.Y(:,1) == y_value);
    end
    
    if(numel(found_col) ==0)
        y_value
        prob = prob +1;
        continue;
    end
    new_y_value = found_row(1);
    
    three_d_coordinates(j).x = DepthFrame.DepthData.X(new_y_value ,new_x_value );
    three_d_coordinates(j).y = DepthFrame.DepthData.Y(new_y_value ,new_x_value );
    three_d_coordinates(j).z = DepthFrame.DepthData.Depth(new_y_value ,new_x_value );
    three_d_coordinates(j).cam = mother(i).cam;
    three_d_coordinates(j).frame = mother(i).frame;
    three_d_coordinates(j).valid = mother(i).valid;
    three_d_coordinates(j).offset = mother(i).offset;
    three_d_coordinates(j).bounding_box_x = mother(i).x;
    three_d_coordinates(j).bounding_box_y = mother(i).y;
    three_d_coordinates(j).bounding_box_width = mother(i).width;
    three_d_coordinates(j).bounding_box_height = mother(i).height;
%     plot3(three_d_coordinates(j).x,three_d_coordinates(j).y,three_d_coordinates(j).z,'LineWidth',5);
% %     pause(0.2);
%     hold on;
    j=j+1;
end



three_d_coordinates_baby  = struct([]);
j=1;
valid_baby = 0;
prob_baby =0;
cases =0;
for i = 1:length(baby)
    if(baby(i).valid ==0)
        continue;
    end
    valid_baby = valid_baby +1;
    three_d_coordinates_baby(j).line_num = i;
    
    if(baby(i).cam == 1)
        load(fullfile([filename_1_prefix,num2str(baby(i).frame),'.mat']),'ColorFrame','DepthFrame');
        d_img = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,cam1_depth_mapping.DepthFrameToColorFrameMapping);
    elseif(baby(i).cam ==2)
        load(fullfile([filename_2_prefix,num2str(baby(i).frame),'.mat']),'ColorFrame','DepthFrame');
        d_img = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,cam2_depth_mapping.DepthFrameToColorFrameMapping);
    elseif(baby(i).cam ==3)
        load(fullfile([filename_3_prefix,num2str(baby(i).frame),'.mat']),'ColorFrame','DepthFrame');
        d_img = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,cam3_depth_mapping.DepthFrameToColorFrameMapping);
    elseif(baby(i).cam ==4)
        load(fullfile([filename_4_prefix,num2str(baby(i).frame),'.mat']),'ColorFrame','DepthFrame');
        d_img = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,cam4_depth_mapping.DepthFrameToColorFrameMapping);
    end
    
    
    
%     depth_values = zeros(baby(i).height,baby(i).width);
    center_x = baby(i).x;
    center_y = baby(i).y;
    width = baby(i).width;
    height = baby(i).height;
    min_x = floor(center_x - width/2);
    max_x = floor(center_x + width/2);
    min_y = floor(center_y - height/2);
    max_y = floor(center_y + height/2);
    
    if(max_y>480)
        max_y =480;
        cases =cases+1;
    end
    if(min_x<=0)
        min_x = 1;
    end
    if(max_x > 640)
        max_x = 640;
    end
    if(min_y<=0)
        min_y=1;
    end
    if(max_y >480)
        max_y =480;
    end
    
    depth_values = d_img(min_y:max_y,min_x:max_x);
    
% %     for y = uint16(A(3) - (A(5)/2)) : uint16(A(3) + (A(5)/2))
% %         for x = uint16(A(2) - (A(4)/2)) : uint16(A(2) + (A(4)/2))
% %              if(y<481 && y>0 && x < 641 && x>0)
% %                 depth_values(y - uint16(A(3) - (A(5)/2)) +1, x - uint16(A(2) - (A(4)/2)) +1  ) = d_img(y,x);
% %              else
% %                 depth_values(y - uint16(A(3) - (A(5)/2)) +1, x - uint16(A(2) - (A(4)/2)) +1  ) = 0;
% %              end
% %         end
% %     end
    median_depth = median(depth_values(:));
    B = unique(depth_values(:));
    C = (B - median_depth);
    D = min(abs(C));
    closest_value = min(abs(unique(depth_values(:))- median_depth));
    [row,col] = find(depth_values == (median_depth+ closest_value));
    if(isempty(row))
        [row,col] = find(depth_values == (median_depth -  closest_value));
    end
    x_value = floor(col(ceil(length(row)/2)) + min_x);
    y_value = floor(row(ceil(length(row)/2)) + min_y);
    
% %     now find the X and Y values in the depth mapping...
    if(baby(i).cam == 1)
        [found_row,found_col] = find(cam1_depth_mapping.DepthFrameToColorFrameMapping.X(1,:) == x_value);
    elseif(baby(i).cam == 2)
        [found_row,found_col] = find(cam2_depth_mapping.DepthFrameToColorFrameMapping.X(1,:) == x_value);
    elseif(baby(i).cam == 3)
        [found_row,found_col] = find(cam3_depth_mapping.DepthFrameToColorFrameMapping.X(1,:) == x_value);
    elseif(baby(i).cam == 4)
        [found_row,found_col] = find(cam4_depth_mapping.DepthFrameToColorFrameMapping.X(1,:) == x_value);
    end
    if(numel(found_col) ==0 ||numel(found_col) ==0)
        x_value
        prob_baby = prob_baby +1;
        continue;
    end
    new_x_value = found_col(1);
    
    if(baby(i).cam == 1)
        [found_row,found_col] = find(cam1_depth_mapping.DepthFrameToColorFrameMapping.Y(:,1) == y_value);
    elseif(baby(i).cam == 2)
        [found_row,found_col] = find(cam2_depth_mapping.DepthFrameToColorFrameMapping.Y(:,1) == y_value);
    elseif(baby(i).cam == 3)
        [found_row,found_col] = find(cam3_depth_mapping.DepthFrameToColorFrameMapping.Y(:,1) == y_value);
    elseif(baby(i).cam == 4)
        [found_row,found_col] = find(cam4_depth_mapping.DepthFrameToColorFrameMapping.Y(:,1) == y_value);
    end
    
    if(numel(found_col) ==0)
        y_value
        prob_baby = prob_baby +1;
        continue;
    end
    new_y_value = found_row(1);
    
    three_d_coordinates_baby(j).x = DepthFrame.DepthData.X(new_y_value ,new_x_value );
    three_d_coordinates_baby(j).y = DepthFrame.DepthData.Y(new_y_value ,new_x_value );
    three_d_coordinates_baby(j).z = DepthFrame.DepthData.Depth(new_y_value ,new_x_value );
    
    three_d_coordinates_baby(j).cam = baby(i).cam;
    three_d_coordinates_baby(j).frame = baby(i).frame;
    three_d_coordinates_baby(j).valid = baby(i).valid;
    three_d_coordinates_baby(j).offset = baby(i).offset;
    three_d_coordinates_baby(j).bounding_box_x = baby(i).x;
    three_d_coordinates_baby(j).bounding_box_y = baby(i).y;
    three_d_coordinates_baby(j).bounding_box_width = baby(i).width;
    three_d_coordinates_baby(j).bounding_box_height = baby(i).height;
% % %     plot3(three_d_coordinates_baby(j).x,three_d_coordinates_baby(j).y,three_d_coordinates_baby(j).z,'LineWidth',5);
% % %     pause(0.01);
% % %     hold on;
% %     three_d_coordinates(j).x = new_x_value;
% %     three_d_coordinates(j).y = new_y_value;

    j=j+1;
end

fclose(mom_file);
fclose(baby_file);