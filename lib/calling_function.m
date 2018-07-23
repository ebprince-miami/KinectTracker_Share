load('/home/arri/Dataset/FN038/USB-VID_045E&PID_02AE-A00365A09389104A_DepthFrameToColorFrameMapping.mat');
cam1_d2c = DepthFrameToColorFrameMapping;
load('/home/arri/Dataset/FN038/USB-VID_045E&PID_02AE-B00362214481051B_DepthFrameToColorFrameMapping.mat');
cam2_d2c = DepthFrameToColorFrameMapping;
% load('/home/arri/Dataset/FN038/USB-VID_045E&PID_02AE-A00364A04906127A_DepthFrameToColorFrameMapping.mat');
% cam3_d2c = DepthFrameToColorFrameMapping;
load('/home/arri/Dataset/FN038/USB-VID_045E&PID_02AE-A00367A14179123A_DepthFrameToColorFrameMapping.mat');
cam4_d2c = DepthFrameToColorFrameMapping;

% offset_1_2 = 8505;
% offset_1_3 = -22821;
% offset_1_4 = -9218;

                
% offset_2_1 = -offset_1_2;  %p3
% offset_2_3 = (offset_1_3) - (offset_1_2); %p1
% offset_2_4 = (offset_1_4) - (offset_1_2); %p2

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
% time2 =time1;
% time3 = time1;
% % 
% track = load('/Users/karanjitcheema/Desktop/three_d_coordinates_mother.mat');
% mom_track = interpolate_tracks(track.three_d_coordinates);
% 
% track  = load('/Users/karanjitcheema/Desktop/three_d_coordinates_baby.mat');
% baby_track  = interpolate_tracks(track.three_d_coordinates_baby);
% 


track = load('/home/arri/Dataset/FN038_processed/reunion2/three_d_coordinates_mother.mat');
mom_track = interpolate_tracks(track.three_d_coordinates);

track  = load('/home/arri/Dataset/FN038_processed/reunion2/three_d_coordinates_baby.mat');
baby_track  = interpolate_tracks(track.three_d_coordinates_baby);

disp('generating new view');
[pos3d pos2d] = create_video(cam1_d2c,cam2_d2c,[],cam4_d2c,offset_to_p4,time3,time2,time1,baby_track,mom_track);