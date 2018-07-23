dirpath = '/Users/karanjitcheema/Desktop/FN026/';
directory = '/Users/karanjitcheema/Desktop/video';
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


mother_frame_numbers = zeros(1,length(mother));
for i = 1: length(mother)
    mother_frame_numbers(i) = mother(i).frame;
end

baby_frame_numbers = zeros(1,length(baby));
for i = 1: length(baby)
    baby_frame_numbers(i) = baby(i).frame;
end

for i = 1:length(mother)
    if(mother(i).cam == 1)
        dirpath2 = strcat(dirpath,'cam1_timestamp/cam1/');
    elseif(mother(i).cam == 2)
        dirpath2 = strcat(dirpath,'cam2_timestamp/cam2/');
    elseif(mother(i).cam == 3)
        dirpath2 = strcat(dirpath,'cam3_timestamp/cam3/');
    else
        dirpath2 = strcat(dirpath,'cam4_timestamp/cam4/');
    end
    file = strcat(dirpath2,num2str(mother(i).frame));
    file = strcat(file,'.png');
    imshow(file);
    hold on;
    if(mother(i).valid == 1)
        center_x = mother(i).x;
        center_y = mother(i).y;
        width = mother(i).width;
        height = mother(i).height;
        rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','r') ;
    end
    row_number = find(baby_frame_numbers == mother(i).frame );
    if(~isempty(row_number))
        for j = 1:length(row_number)
            if(baby(row_number(j)).cam == mother(i).cam && baby(row_number(j)).valid ==1)
                center_x = baby(row_number(j)).x;
                center_y = baby(row_number(j)).y;
                width = baby(row_number(j)).width;
                height = baby(row_number(j)).height;
                rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','b') ;
            end
        end
    end
    dest = strcat(num2str(i),'.jpg');
    saveas(gcf, fullfile(directory,dest));
    cla reset;
end


directory_baby = '/Users/karanjitcheema/Desktop/video_baby';
for i = 1:length(baby)
    if(baby(i).cam == 1)
        dirpath2 = strcat(dirpath,'cam1_timestamp/cam1/');
    elseif(baby(i).cam == 2)
        dirpath2 = strcat(dirpath,'cam2_timestamp/cam2/');
	elseif(baby(i).cam == 3)
        dirpath2 = strcat(dirpath,'cam3_timestamp/cam3/');
    elseif(baby(i).cam == 4)
        dirpath2 = strcat(dirpath,'cam4_timestamp/cam4/');
    end
    
    file = strcat(dirpath2,num2str(baby(i).frame));
    file = strcat(file,'.png');
    imshow(file);
    hold on;
    if(baby(i).valid == 1)
        center_x = baby(i).x;
        center_y = baby(i).y;
        width = baby(i).width;
        height = baby(i).height;
        rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','r') ;
    end
    row_number = find(mother_frame_numbers == baby(i).frame );
    if(~isempty(row_number))
        for j = 1:length(row_number)
            if(mother(row_number(j)).cam == mother(i).cam && mother(row_number(j)).valid ==1)
                center_x = mother(row_number(j)).x;
                center_y = mother(row_number(j)).y;
                width = mother(row_number(j)).width;
                height = mother(row_number(j)).height;
                rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','b') ;
            end
        end
    end
    dest_baby = strcat(num2str(i),'.jpg');
    saveas(gcf, fullfile(directory_baby,dest_baby));
    cla reset;
end