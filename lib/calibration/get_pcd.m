pcd1(:,:,1) = map_depthframe_to_colorframe(I1.DepthFrame.DepthData.X,I1.dtc);
pcd1(:,:,2) = map_depthframe_to_colorframe(I1.DepthFrame.DepthData.Y,I1.dtc);
pcd1(:,:,3) = map_depthframe_to_colorframe(I1.DepthFrame.DepthData.Depth,I1.dtc);
pcd4(:,:,1) = map_depthframe_to_colorframe(I4.DepthFrame.DepthData.X,I4.dtc);
pcd4(:,:,2) = map_depthframe_to_colorframe(I4.DepthFrame.DepthData.Y,I4.dtc);
pcd4(:,:,3) = map_depthframe_to_colorframe(I4.DepthFrame.DepthData.Depth,I4.dtc);
pcd2(:,:,1) = map_depthframe_to_colorframe(I2.DepthFrame.DepthData.X,I2.dtc);
pcd2(:,:,2) = map_depthframe_to_colorframe(I2.DepthFrame.DepthData.Y,I2.dtc);
pcd2(:,:,3) = map_depthframe_to_colorframe(I2.DepthFrame.DepthData.Depth,I2.dtc);
pcd3(:,:,1) = map_depthframe_to_colorframe(I3.DepthFrame.DepthData.X,I3.dtc);
pcd3(:,:,2) = map_depthframe_to_colorframe(I3.DepthFrame.DepthData.Y,I3.dtc);
pcd3(:,:,3) = map_depthframe_to_colorframe(I3.DepthFrame.DepthData.Depth,I3.dtc);


%% loading the images and depth to color mapping
I1 = load(fullfile(dirpath,'Poster Calibration-K1/USB-VID_045E&PID_02AE-A00365A09389104A_3909.mat'));
I1.dtc = load(fullfile(dirpath,'Poster Calibration-K1/USB-VID_045E&PID_02AE-A00365A09389104A_DepthFrameToColorFrameMapping.mat'));
I1.dtc = I1.dtc.DepthFrameToColorFrameMapping;
I4 = load(fullfile(dirpath,'Poster Calibration-K4/USB-VID_045E&PID_02AE-A00367A14179123A_2143.mat'));
I4.dtc = load(fullfile(dirpath,'Poster Calibration-K4/USB-VID_045E&PID_02AE-A00367A14179123A_DepthFrameToColorFrameMapping.mat'));
I4.dtc = I4.dtc.DepthFrameToColorFrameMapping;
I2 = load(fullfile(dirpath,'Poster Calibration-K2/USB-VID_045E&PID_02AE-B00362214481051B_3855.mat'));
I2.dtc = load(fullfile(dirpath,'Poster Calibration-K2/USB-VID_045E&PID_02AE-B00362214481051B_DepthFrameToColorFrameMapping.mat'));
I2.dtc = I2.dtc.DepthFrameToColorFrameMapping;
I3 = load(fullfile(dirpath,'Poster Calibration-K3/USB-VID_045E&PID_02AE-A00364A04906127A_3971.mat'));
I3.dtc = load(fullfile(dirpath,'Poster Calibration-K3/USB-VID_045E&PID_02AE-A00364A04906127A_DepthFrameToColorFrameMapping.mat'));
I3.dtc = I3.dtc.DepthFrameToColorFrameMapping;