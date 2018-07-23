function mat_to_mesh()

datapath = 'C:\Users\Explorer\Documents\Dataset\sfm';
for i=1:13
    i
    load(fullfile(datapath,['livingroom',num2str(i)],'USB-VID_045E&PID_02AE-A00365904718133A_1.mat'),'ColorFrame','DepthFrame','DepthFrameToColorFrameMapping');
    pcd(:,:,1) = map_depthframe_to_colorframe(DepthFrame.DepthData.X,DepthFrameToColorFrameMapping);
    pcd(:,:,2) = map_depthframe_to_colorframe(DepthFrame.DepthData.Y,DepthFrameToColorFrameMapping);
    pcd(:,:,3) = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DepthFrameToColorFrameMapping);
    points = map_pcd(pcd,eye(3),[0 0 0]',ColorFrame.ColorData);
    writeply(points',fullfile(datapath,['livingroom',num2str(i),'.ply']));
end