function plot_track_3d(track,cam1_d2c,cam2_d2c,cam3_d2c,cam4_d2c,offset_to_p3,time3,time2,time1)

load('viewpoint_az_el.mat')
load('/home/arri/Dropbox/Code/strange_situation/calibration/calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
% load('C:\Users\Explorer\Documents\Dropbox\Dropbox\Code\strange_situation\calibration\calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
p1name = 'USB-VID_045E&PID_02AE-A00365A09389104A';
p2name = 'USB-VID_045E&PID_02AE-B00362214481051B';
p3name = 'USB-VID_045E&PID_02AE-A00364A04906127A';
p4name = 'USB-VID_045E&PID_02AE-A00367A14179123A';

norm = 9.5;
offset = [-3112 -323];

red(:,:,1:3) = 0;
red(:,:,1) = 255;
red = uint8(red);

filepath = '/media/Seagate Expansion Drive/FN001';

startidx = 20889;

for i=1:2000%startidx:startidx+1617
%     load(fullfile(filepath,[p1name,'_',num2str(i),'.mat']),'Timestamp');
%     load(fullfile(filepath,[cam_name{track(i).cam},'_',num2str(track(i).frame),'.mat']),'Timestamp');
%     if track(i).cam == 1
%         track(i).frame
%     end
    
    pos(:,:,1) = track(i).x;
    pos(:,:,2) = track(i).y;
    pos(:,:,3) = double(track(i).z);
    if track(i).cam == 3
        p3d = map_pcd(pos,R_41,t_41,1,red);
    elseif track(i).cam == 1
        p3d = map_pcd(pos,R_43,t_43,1,red);
    end
    [I proj2d] = get_newview(az,el,p3d, norm, offset);
    cla reset;
    imshow(I);
    drawnow;
end

