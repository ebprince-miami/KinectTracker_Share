function GetFusedImage(directory, frameno)
load('C:\Users\Explorer\Documents\Dropbox\Dropbox\Code\strange_situation\code\new_stuff\code\viewpoint_az_el.mat')
load('C:\Users\Explorer\Documents\Dropbox\Dropbox\Code\strange_situation\code\new_stuff\code\calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
% load('/home/arri/Dropbox/Code/strange_situation/code/new_stuff/code/calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
fnamePrefix{1} = strcat(directory,'/cam1/','USB-VID_045E&PID_02AE-A00364A04906127A_');
fnamePrefix{2} = strcat(directory,'/cam2/','USB-VID_045E&PID_02AE-A00367A14179123A_');
fnamePrefix{3} = strcat(directory,'/cam3/','USB-VID_045E&PID_02AE-A00365A09389104A_');
fnamePrefix{4} = strcat(directory,'/cam4/','USB-VID_045E&PID_02AE-B00362214481051B_');

imPath{1} = fullfile(directory,'cam1');
imPath{2} = fullfile(directory,'cam2');
imPath{3} = fullfile(directory,'cam3');
imPath{4} = fullfile(directory,'cam4');

offset = textread(fullfile(directory,'offset.txt'),'%d');

timestamp = cell(1,4);
DtCMapping = cell(1,4);
for i=1:4
    try
        timestamp{i} = load(fullfile(directory,['cam',num2str(i),'_timestamp.mat']));
        load([fnamePrefix{i}, 'DepthFrameToColorFrameMapping.mat']);
        DtCMapping{i} = DepthFrameToColorFrameMapping;
    catch
    end
end

offset = textread(fullfile(directory,'offset.txt'),'%d');

norm = 9.5;
viewOffset = [-3112 -323];
for i=frameno
%     i
%     load(fullfile(filepath,[p4name,'_',num2str(i),'.mat']),'Timestamp');
    Timestamp = timestamp{2}.time(i);
    load([fnamePrefix{2},num2str(i),'.mat'],'DepthFrame');
    cam4_d = DepthFrame.DepthData;
    cam4_c = imread(fullfile(imPath{2},[num2str(i),'.png']));
    
    try
    target = double(Timestamp + offset(1));
    [t idx3] = min(abs(double(timestamp{1}.time) - target));
    load([fnamePrefix{1},num2str(idx3),'.mat'],'DepthFrame');
    cam3_d = DepthFrame.DepthData;
    cam3_c = imread(fullfile(imPath{1},[num2str(idx3),'.png']));
    catch
    end
    
    try
    target = double(Timestamp + offset(4));
    [t idx] = min(abs(double(timestamp{4}.time) - target));
    load([fnamePrefix{4},num2str(idx),'.mat'],'DepthFrame');
    cam2_d = DepthFrame.DepthData;
    cam2_c = imread(fullfile(imPath{4},[num2str(idx),'.png']));
    catch
    end
   
    try
    target = double(Timestamp + offset(3));
    [t idx1] = min(abs(double(timestamp{3}.time) - target));
    load([fnamePrefix{3},num2str(idx1),'.mat'],'DepthFrame');
    cam1_d = DepthFrame.DepthData;  
    cam1_c = imread(fullfile(imPath{3},[num2str(idx1),'.png']));
    catch
    end
    
    pcd1(:,:,1) = map_depthframe_to_colorframe(cam1_d.X,DtCMapping{3});
    pcd1(:,:,2) = map_depthframe_to_colorframe(cam1_d.Y,DtCMapping{3});
    pcd1(:,:,3) = map_depthframe_to_colorframe(cam1_d.Depth,DtCMapping{3});
    pcd4(:,:,1) = map_depthframe_to_colorframe(cam4_d.X,DtCMapping{2});
    pcd4(:,:,2) = map_depthframe_to_colorframe(cam4_d.Y,DtCMapping{2});
    pcd4(:,:,3) = map_depthframe_to_colorframe(cam4_d.Depth,DtCMapping{2});
%     pcd2(:,:,1) = map_depthframe_to_colorframe(cam2_d.X,DtCMapping{4});
%     pcd2(:,:,2) = map_depthframe_to_colorframe(cam2_d.Y,DtCMapping{4});
%     pcd2(:,:,3) = map_depthframe_to_colorframe(cam2_d.Depth,DtCMapping{4});
    pcd3(:,:,1) = map_depthframe_to_colorframe(cam3_d.X,DtCMapping{1});
    pcd3(:,:,2) = map_depthframe_to_colorframe(cam3_d.Y,DtCMapping{1});
    pcd3(:,:,3) = map_depthframe_to_colorframe(cam3_d.Depth,DtCMapping{1});
    
    p4 = map_pcd(pcd4,eye(3),zeros(3,1),1,cam4_c);
%     p42 = map_pcd(pcd2,R_42,t_42,1,cam2_c);
    p43 = map_pcd(pcd3,R_43,t_43,1,cam3_c);
    p41 = map_pcd(pcd1,R_41,t_41,1,cam1_c);
    
    [I proj2d Id] = get_newview(-180,-90,[p4 p41 p42], 10);
    figure;imshow(I(200:end,250:end,:))
    figure;imshow(abs(Id(200:end,250:end))/350)
    I = get_newview(az,el,[p4 p43 p42 p41], norm, viewOffset);
%     I = get_newview(az,el,[p4 p42 p41], norm, offset);
end