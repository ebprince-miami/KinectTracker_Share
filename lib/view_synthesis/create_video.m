function [pos3d pos2d] = create_video(cam1_d2c,cam2_d2c,cam3_d2c,cam4_d2c,offset_to_p4,time3,time2,time1,baby_track,mom_track)

load('/home/arri/Dataset/code/new_stuff/code/viewpoint_az_el.mat')
load('/home/arri/Dataset/code/new_stuff/code/calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
% load('C:\Users\Explorer\Documents\Dropbox\Dropbox\Code\strange_situation\calibration\calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
p1name = 'cam3/USB-VID_045E&PID_02AE-A00365A09389104A';
p2name = 'cam4/USB-VID_045E&PID_02AE-B00362214481051B';
p3name = 'cam1/USB-VID_045E&PID_02AE-A00364A04906127A';
p4name = 'cam2/USB-VID_045E&PID_02AE-A00367A14179123A';

newviewpath = '/home/arri/Dataset/FN038_processed/reunion2/new_view1';
norm = 9.5;
offset = [-3112 -323];

fid = fopen('/home/arri/Dataset/FN038_processed/reunion2/saved_data.txt', 'a');

filepath = '/home/arri/Dataset/FN038/';
imagepath = '/home/arri/Dataset/FN038/';
% filepath = 'G:\FN001';



startidx = 35254;
endidx = 40739;

red(:,:,1:3) = 0;
red(:,:,1) = 255;
red = uint8(red);
green(:,:,1:3) = 0;
green(:,:,2) = 255;
green = uint8(green);
% baby_track_p1 = baby_track(baby_track(:,1)==3,:);
% % % % % % baby_track(baby_track(:,1)==3,3) = baby_track(baby_track(:,1)==3,3) - offset_to_p4(1);
% baby_track_p3 = baby_track(baby_track(:,1)==1,:);
% % % % % % baby_track(baby_track(:,1)==1,3) = baby_track(baby_track(:,1)==1,3) - offset_to_p4(3);
% % % % % mom_track(:,3) = mom_track(:,3) - offset_to_p4(3);
prev_baby_idx = 1;
prev_mom_idx = 1;

    new_mom_track = mom_track;
    new_baby_track = baby_track;
%     
%     new_baby_track(:,3) = new_baby_track(:,3) + offset_to_p4(3);
%     new_mom_track(:,3) = new_mom_track(:,3) + offset_to_p4(3);
    
    
pos3d = [];
pos2d = [];
for i=startidx:endidx
%     i
    load(fullfile(filepath,[p4name,'_',num2str(i),'.mat']),'Timestamp');
    Timestamp = double(Timestamp);
    load(fullfile(filepath,[p4name,'_',num2str(i),'.mat']),'DepthFrame','Timestamp');
    cam4_d = DepthFrame.DepthData;
    cam4_c = imread(strcat(imagepath,'cam2/',num2str(i),'.png'));
    
%     target = double(Timestamp - offset_to_p4(3));
%     [t idx3] = min(abs(double(time3) - target));
%     load(fullfile(filepath,[p3name,'_',num2str(idx3),'.mat']),'DepthFrame');
%     cam3_d = DepthFrame.DepthData;
%     cam3_c = imread(strcat(imagepath,'cam1/',num2str(idx3),'.png'));
    
    target = double(Timestamp - offset_to_p4(2));
    [t idx] = min(abs(double(time2) - target));
    load(fullfile(filepath,[p2name,'_',num2str(idx),'.mat']),'DepthFrame');
    cam2_d = DepthFrame.DepthData;
    cam2_c = imread(strcat(imagepath,'cam4/',num2str(idx),'.png'));
   
    target = double(Timestamp - offset_to_p4(1));
    [t idx1] = min(abs(double(time1) - target));
    load(fullfile(filepath,[p1name,'_',num2str(idx1),'.mat']),'DepthFrame');
    cam1_d = DepthFrame.DepthData;  
    cam1_c = imread(strcat(imagepath,'cam3/',num2str(idx1),'.png'));
    
    pcd1(:,:,1) = map_depthframe_to_colorframe(cam1_d.X,cam1_d2c);
    pcd1(:,:,2) = map_depthframe_to_colorframe(cam1_d.Y,cam1_d2c);
    pcd1(:,:,3) = map_depthframe_to_colorframe(cam1_d.Depth,cam1_d2c);
    pcd4(:,:,1) = map_depthframe_to_colorframe(cam4_d.X,cam4_d2c);
    pcd4(:,:,2) = map_depthframe_to_colorframe(cam4_d.Y,cam4_d2c);
    pcd4(:,:,3) = map_depthframe_to_colorframe(cam4_d.Depth,cam4_d2c);
    pcd2(:,:,1) = map_depthframe_to_colorframe(cam2_d.X,cam2_d2c);
    pcd2(:,:,2) = map_depthframe_to_colorframe(cam2_d.Y,cam2_d2c);
    pcd2(:,:,3) = map_depthframe_to_colorframe(cam2_d.Depth,cam2_d2c);
%     pcd3(:,:,1) = map_depthframe_to_colorframe(cam3_d.X,cam3_d2c);
%     pcd3(:,:,2) = map_depthframe_to_colorframe(cam3_d.Y,cam3_d2c);
%     pcd3(:,:,3) = map_depthframe_to_colorframe(cam3_d.Depth,cam3_d2c);
    
    p4 = map_pcd(pcd4,eye(3),zeros(3,1),1,cam4_c);
    p42 = map_pcd(pcd2,R_42,t_42,1,cam2_c);
%     p43 = map_pcd(pcd3,R_43,t_43,1,cam3_c);
    p41 = map_pcd(pcd1,R_41,t_41,1,cam1_c);
    
%     I = get_newview(az,el,[p4 p43 p42 p41], norm, offset);
    I = get_newview(az,el,[p4 p42 p41], norm, offset);
    imwrite(I,fullfile('/home/arri/Dataset/FN038_processed/reunion2/new_view1/',[num2str(i-startidx+1),'.png']));

    
    timestamp_diff = (new_baby_track(:,3) - double(Timestamp));
    row_num = find(abs(timestamp_diff) == min(abs(timestamp_diff)));
    row_num = row_num(1);
    baby_row_num = row_num;
    new_pos_baby = new_baby_track(row_num,4:6);
    if(new_baby_track(row_num,1) ==1)
        pbaby = map_pcd_head_pos(new_pos_baby,R_43,t_43,1,red);
    elseif(new_baby_track(row_num,1) ==3)
        pbaby = map_pcd_head_pos(new_pos_baby,R_41,t_41,1,red);     
    elseif(new_baby_track(row_num,1) ==4)
        pbaby = map_pcd_head_pos(new_pos_baby,R_42,t_42,1,red);     
    else
        pbaby = map_pcd_head_pos(new_pos_baby,eye(3,3),zeros(3,1),1,red);    %send identity matrices... 
    end
    
    
    timestamp_diff = (new_mom_track(:,3) - double(Timestamp));
    row_num = find(abs(timestamp_diff) == min(abs(timestamp_diff)));
    row_num = row_num(1);
    mom_row_num = row_num;
    new_pos_mom = new_mom_track(row_num,4:6);
    if(new_mom_track(row_num,1) ==1)
        pmom = map_pcd_head_pos(new_pos_mom,R_43,t_43,1,green);
    elseif(new_mom_track(row_num,1) ==2)
        pmom = map_pcd_head_pos(new_pos_mom,eye(3,3),zeros(3,1),1,green);
    elseif(new_mom_track(row_num,1) ==3)
        pmom = map_pcd_head_pos(new_pos_mom,R_41,t_41,1,green);
    elseif(new_mom_track(row_num,1) ==4)
        pmom = map_pcd_head_pos(new_pos_mom,R_42,t_42,1,green);
    end
    
    [I proj2d] = get_newview(az,el,[pbaby pmom], norm, offset);
    I = imread(fullfile(newviewpath,num2str(i-startidx+1,'%d.png')));
    temp = proj2d;
    proj2d(2,:) = proj2d(1,:);
    proj2d(1,:) = temp(2,:);
    cla reset;
    imshow(I);
    hold on;
    
    plot([proj2d(2,1)-5 proj2d(2,1)+15 proj2d(2,1)+15 proj2d(2,1)-5 proj2d(2,1)-5],[proj2d(1,1)-5 proj2d(1,1)-5 proj2d(1,1)+15 proj2d(1,1)+15 proj2d(1,1)-5],'r-','LineWidth',3);
    if size(proj2d,2)>1
        plot([proj2d(2,2)-5 proj2d(2,2)+15 proj2d(2,2)+15 proj2d(2,2)-5 proj2d(2,2)-5],[proj2d(1,2)-5 proj2d(1,2)-5 proj2d(1,2)+15 proj2d(1,2)+15 proj2d(1,2)-5],'g-','LineWidth',3);
    end
    hold off;
    drawnow;
    
    pos3d = [pos3d pbaby(1:3) pmom(1:3)];
    pos2d = [pos2d proj2d];
%     plot_distance(pos3d,pos2d);
    saveas(gcf,fullfile('/home/arri/Dataset/FN038_processed/reunion2/new_view1_track',[num2str(i-startidx+1),'.png']));
    
    fprintf(fid, '%1.0f %1.0f  %1.0f %1.0f  %1.0f %1.0f \t', i,new_baby_track(baby_row_num,1),new_baby_track(baby_row_num,2),  new_mom_track(mom_row_num,1),new_mom_track(mom_row_num,2), Timestamp);
    fprintf(fid, '%1.0f %1.0f  %1.0f %1.0f  %1.0f %1.0f \t',pbaby(1,1),pbaby(2,1),pbaby(3,1) ,pmom(1,1),pmom(2,1),pmom(3,1) );
    fprintf(fid, '%1.0f %1.0f %1.0f %1.0f %1.0f %1.0f  \n',proj2d(1,1),proj2d(2,1),proj2d(3,1),proj2d(1,2),proj2d(2,2),proj2d(3,2) );

end
    save('/home/arri/Dataset/FN038_processed/reunion2/pos2d.mat','pos2d');
    save('/home/arri/Dataset/FN038_processed/reunion2/pos3d.mat','pos3d');
    plot_distance(pos3d,pos2d);
end
