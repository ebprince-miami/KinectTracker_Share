function video_generation()

% load('/Users/karanjitcheema/Desktop/RA_Work/new_stuff/code/viewpoint_az_el.mat')
% load('/Users/karanjitcheema/Desktop/RA_Work/new_stuff/code/calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
p1name = 'cam3/USB-VID_045E&PID_02AE-A00365A09389104A';
p2name = 'cam4/USB-VID_045E&PID_02AE-A00366809492052A';
p3name = 'cam1/USB-VID_045E&PID_02AE-A00364A04906127A';
p4name = 'cam2/USB-VID_045E&PID_02AE-A00367A14179123A';

fid = fopen('saved_data.txt', 'a');

filepath = '/home/arri/Dataset/FN038/';
imagepath = '/home/arri/Dataset/FN038/';
% 
% filepath = '/Users/karanjitcheema/Desktop/RA_Work/Data/';


startidx = 35254;
endidx = 40739;

% offset_1_2 = 8505;
% offset_1_3 = -22821;
% offset_1_4 = -9218;
% 
%                 
% offset_2_1 = -offset_1_2;  %p3
% offset_2_3 = (offset_1_3) - (offset_1_2); %p1
% offset_2_4 = (offset_1_4) - (offset_1_2); %p2
%     
% offset_to_p4 = [-offset_2_3 -offset_2_4 -offset_2_1];
% time3 = load('/Users/karanjitcheema/Desktop/RA_Work/new_stuff/code/cam1_timestamp.mat','time');
% time3 =  time3.time;
% time2 = load('/Users/karanjitcheema/Desktop/RA_Work/new_stuff/code/cam4_timestamp.mat','time');
% time2 =  time2.time;
% time1 = load('/Users/karanjitcheema/Desktop/RA_Work/new_stuff/code/cam3_timestamp.mat','time');
% time1 =  time1.time;

offset_2_1 = 0;
offset_2_3 = -48582;
offset_2_4 = -34200; % This will change once we get cam4 and cam1. Right now acc to cam3.

offset_to_p4 = [-offset_2_3 -offset_2_4 -offset_2_1];

time3 = load('/home/arri/Dataset/FN038/cam1_timestamp.mat','time');
time3 =  time3.time;
time2 = load('/home/arri/Dataset/FN038/cam4_timestamp.mat','time');
time2 =  time2.time;
time1 = load('/home/arri/Dataset/FN038/cam3_timestamp.mat','time');
time1 =  time1.time;

track = load('/home/arri/Dataset/FN038_processed/reunion2/three_d_coordinates_mother.mat');
mom_track = interpolate_tracks(track.three_d_coordinates);

track  = load('/home/arri/Dataset/FN038_processed/reunion2/three_d_coordinates_baby.mat');
baby_track  = interpolate_tracks(track.three_d_coordinates_baby);

last_view =2;
seq_index =1;
for i=startidx:endidx
    
    load(fullfile(filepath,[p4name,'_',num2str(i),'.mat']),'Timestamp');
    Timestamp = double(Timestamp);
    load(fullfile(filepath,[p4name,'_',num2str(i),'.mat']),'DepthFrame','Timestamp');
    cam4_c = imread(strcat(imagepath,'cam2/',num2str(i),'.png'));
    
%     target = double(Timestamp - offset_to_p4(3));
%     [t idx3] = min(abs(double(time3) - target));
%     load(fullfile(filepath,[p3name,'_',num2str(idx3),'.mat']),'DepthFrame');
%     cam3_c = imread(strcat(imagepath,'cam1_timestamp/cam1/',num2str(idx3),'.png'));
    
    target = double(Timestamp - offset_to_p4(2));
    [t idx2] = min(abs(double(time2) - target));
    load(fullfile(filepath,[p2name,'_',num2str(idx2),'.mat']),'DepthFrame');
    cam2_c = imread(strcat(imagepath,'cam4/',num2str(idx2),'.png'));
   
    target = double(Timestamp - offset_to_p4(1));
    [t idx1] = min(abs(double(time1) - target));
    load(fullfile(filepath,[p1name,'_',num2str(idx1),'.mat']),'DepthFrame');
    cam1_c = imread(strcat(imagepath,'cam3/',num2str(idx1),'.png'));
    
 

    cam1_row_mom = find( mom_track(:,2) == idx1  );
    if(~isempty( cam1_row_mom))
        if( mom_track(cam1_row_mom,1) ~= 3)
            cam1_row_mom =[];
        elseif(length(cam1_row_mom) >1)
            for k  =1:length((cam1_row_mom))
                if(mom_track(cam1_row_mom(k), 1 ) ==3 )
                    cam1_row_mom = cam1_row_mom(k);
                    break;
                end
            end
        end
    end
    cam2_row_mom = find( mom_track(:,2)== idx2);
    if(~isempty( cam2_row_mom))
        if( mom_track(cam2_row_mom,1) ~= 4)
            cam2_row_mom =[];
        elseif(length(cam2_row_mom) >1)
            for k  =1:length((cam2_row_mom))
                if(mom_track(cam2_row_mom(k), 1 ) ==4)
                    cam2_row_mom = cam2_row_mom(k);
                    break;
                end
            end
        end
    end
%     cam3_row_mom = find( mom_track(:,2)== idx3);
    cam3_row_mom = [];
    if(~isempty( cam3_row_mom))
        if( mom_track(cam3_row_mom,1) ~= 1)
            cam3_row_mom =[];
        elseif(length(cam3_row_mom) >1)
            for k  =1:length((cam3_row_mom))
                if(mom_track(cam3_row_mom(k), 1 ) ==1)
                    cam3_row_mom= cam3_row_mom(k);
                    break;
                end
            end
        end
    end
    cam4_row_mom = find( mom_track(:,2)== i);
    if(~isempty( cam4_row_mom))
        if( mom_track(cam4_row_mom,1) ~= 2)
            cam4_row_mom =[];
        elseif(length(cam4_row_mom) >1)
            for k  =1:length((cam4_row_mom))
                if(mom_track(cam4_row_mom(k), 1 ) ==2)
                    cam4_row_mom= cam4_row_mom(k);
                    break;
                end
            end
        end
    end
    
    
    
    
    cam1_row_baby = find( baby_track(:,2) == idx1  );
    if(~isempty( cam1_row_baby))
        if( baby_track(cam1_row_baby,1) ~= 3)
            cam1_row_baby =[];
        elseif(length(cam1_row_baby) >1)
            for k  =1:length((cam1_row_baby))
                if(baby_track(cam1_row_baby(k), 1 ) ==3)
                    cam1_row_baby= cam1_row_baby(k);
                    break;
                end
            end
        end
    end
    cam2_row_baby = find( baby_track(:,2)== idx2);
    if(~isempty( cam2_row_baby))
        if( baby_track(cam2_row_baby,1) ~= 4)
            cam2_row_baby =[];
        elseif(length(cam2_row_baby) >1)
            for k  =1:length((cam2_row_baby))
                if(baby_track(cam2_row_baby(k), 1 ) ==4)
                    cam2_row_baby= cam2_row_baby(k);
                    break;
                end
            end
        end
    end
%     cam3_row_baby = find( baby_track(:,2)== idx3);
    cam3_row_baby = [];
    if(~isempty( cam3_row_baby))
        if( baby_track(cam3_row_baby,1) ~= 1)
            cam3_row_baby =[];
        elseif(length(cam3_row_baby) >1)
            for k  =1:length((cam3_row_baby))
                if(baby_track(cam3_row_baby(k), 1 ) ==1)
                    cam3_row_baby= cam3_row_baby(k);
                    break;
                end
            end
        end
    end
    cam4_row_baby = find( baby_track(:,2)== i);
    if(~isempty( cam4_row_baby))
        if( baby_track(cam4_row_baby,1) ~= 2)
            cam4_row_baby =[];
        elseif(length(cam4_row_baby ) >1)
            for k  =1:length((cam4_row_baby ))
                if(baby_track(cam4_row_baby (k), 1 ) ==2)
                    cam4_row_baby = cam4_row_baby (k);
                    break;
                end
            end
        end
    end

    
    drawnow update;
            cla reset;
    
    if(~isempty(cam1_row_mom))
        last_view =1;
        imshow(cam1_c);
        center_x = mom_track(cam1_row_mom,7);
        center_y = mom_track(cam1_row_mom,8);
        width = mom_track(cam1_row_mom,9);
        height = mom_track(cam1_row_mom,10);
        rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','g') ;
        if(~isempty(cam1_row_baby))
            center_x = baby_track(cam1_row_baby,7);
            center_y = baby_track(cam1_row_baby,8);
            width = baby_track(cam1_row_baby,9);
            height = baby_track(cam1_row_baby,10);
            rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','r') ;
        end
    elseif(~isempty(cam2_row_mom))
        last_view =2;
        imshow(cam2_c);
        center_x = mom_track(cam2_row_mom,7);
        center_y = mom_track(cam2_row_mom,8);
        width = mom_track(cam2_row_mom,9);
        height = mom_track(cam2_row_mom,10);
        rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','g') ;
        if(~isempty(cam2_row_baby))
            center_x = baby_track(cam2_row_baby,7);
            center_y = baby_track(cam2_row_baby,8);
            width = baby_track(cam2_row_baby,9);
            height = baby_track(cam2_row_baby,10);
            rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','r') ;
        end
    elseif(~isempty(cam3_row_mom))
        last_view =3;
        imshow(cam3_c);
        center_x = mom_track(cam3_row_mom,7);
        center_y = mom_track(cam3_row_mom,8);
        width = mom_track(cam3_row_mom,9);
        height = mom_track(cam3_row_mom,10);
        rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','g') ;
        if(~isempty(cam3_row_baby))
            center_x = baby_track(cam3_row_baby,7);
            center_y = baby_track(cam3_row_baby,8);
            width = baby_track(cam3_row_baby,9);
            height = baby_track(cam3_row_baby,10);
            rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','r') ;
        end
    elseif(~isempty(cam4_row_mom))
        last_view =4;
        imshow(cam4_c);
        center_x = mom_track(cam4_row_mom,7);
        center_y = mom_track(cam4_row_mom,8);
        width = mom_track(cam4_row_mom,9);
        height = mom_track(cam4_row_mom,10);
        rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','g') ;
        if(~isempty(cam4_row_baby))
            center_x = baby_track(cam4_row_baby,7);
            center_y = baby_track(cam4_row_baby,8);
            width = baby_track(cam4_row_baby,9);
            height = baby_track(cam4_row_baby,10);
            rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','r') ;
        end
    else
        if(last_view == 1)
            imshow(cam1_c);
            if(~isempty(cam1_row_baby))
                center_x = baby_track(cam1_row_baby,7);
                center_y = baby_track(cam1_row_baby,8);
                width = baby_track(cam1_row_baby,9);
                height = baby_track(cam1_row_baby,10);
                rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','r') ;
            end
        elseif(last_view == 2)
            imshow(cam2_c);
            if(~isempty(cam2_row_baby))
            center_x = baby_track(cam2_row_baby,7);
            center_y = baby_track(cam2_row_baby,8);
            width = baby_track(cam2_row_baby,9);
            height = baby_track(cam2_row_baby,10);
            rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','r') ;
            end
        elseif(last_view == 3)
            imshow(cam3_c);
             if(~isempty(cam3_row_baby))
            center_x = baby_track(cam3_row_baby,7);
            center_y = baby_track(cam3_row_baby,8);
            width = baby_track(cam3_row_baby,9);
            height = baby_track(cam3_row_baby,10);
            rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','r') ;
            end
        else
            imshow(cam4_c);
            if(~isempty(cam4_row_baby))
            center_x = baby_track(cam4_row_baby,7);
            center_y = baby_track(cam4_row_baby,8);
            width = baby_track(cam4_row_baby,9);
            height = baby_track(cam4_row_baby,10);
            rectangle('Position',[center_x - width/2, center_y - height/2,width,height],'LineWidth',2,'EdgeColor','r') ;
            end
        end
    end
        
    
    
    saveas(gcf,fullfile('/home/arri/Dataset/FN038_processed/reunion2/video',[num2str(seq_index),'.jpg']));
    seq_index = seq_index +1;
end
end
