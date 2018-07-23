function Get3DPosLSQFIT(bfile, mfile, directory, refCamID, range, savename)

savepath = fullfile(directory, savename);
mkdir(savepath);

% load('C:\Users\Explorer\Documents\Dropbox\Dropbox\Code\strange_situation\code\new_stuff\code\calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
load('D:\Dropbox\Code\strange_situation\code\new_stuff\code\calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
% load('/home/arri/Dropbox/Code/strange_situation/code/new_stuff/code/calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
fnamePrefix{1} = strcat(directory,'/cam1/','USB-VID_045E&PID_02AE-A00364A04906127A_');
fnamePrefix{2} = strcat(directory,'/cam2/','USB-VID_045E&PID_02AE-A00367A14179123A_');
fnamePrefix{3} = strcat(directory,'/cam3/','USB-VID_045E&PID_02AE-A00365A09389104A_');
fnamePrefix{4} = strcat(directory,'/cam4/','USB-VID_045E&PID_02AE-B00362214481051B_');

offset = textread(fullfile(directory,'offset.txt'),'%d');

% for drawing
blank = zeros(480, 640, 3);
separator = zeros(480, 10, 3);

load('D:\Dropbox\Code\3d_tracking\3d_tracking2\tracking_workspace.mat','TR')
sphere = TR.X;

% average head circumference for toddler is ~41cm which means r = 6.5cm
% for adult, it is ~57cm which means r = 9cm
radiusB = 65;
radiusM = 90;
sphereB = sphere * radiusB; 
sphereM = sphere * radiusM;

K = struct('fx',[],'fy',[],'cx',[],'cy',[]);
timestamp = cell(1,4);
DtCMapping = cell(1,4);
for i=1:4
    try
        timestamp{i} = load(fullfile(directory,['cam',num2str(i),'_timestamp.mat']));
        load([fnamePrefix{i}, 'DepthFrameToColorFrameMapping.mat']);
        DtCMapping{i} = DepthFrameToColorFrameMapping;
        
        load(strcat(fnamePrefix{i},int2str(1),'.mat'), 'DepthFrame');
        
        DtC = DepthFrameToColorFrameMapping;
        pcd(:,:,1) = map_depthframe_to_colorframe(DepthFrame.DepthData.X,DtC);
        pcd(:,:,2) = map_depthframe_to_colorframe(DepthFrame.DepthData.Y,DtC);
        pcd(:,:,3) = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DtC);
        [K(i).fx, K(i).fy, K(i).cx, K(i).cy] = compute_K(pcd);
    catch
    end
end

maincam = 2;

if exist(fullfile(directory,'maincam.txt'),'file')
    maincam = textread(fullfile(directory,'maincam.txt'),'%d');
end

bData = textread(fullfile(directory, bfile));
mData = textread(fullfile(directory, mfile));

bData = RemoveDuplicates(bData);
mData = RemoveDuplicates(mData);

bPos3D = zeros(length(range), 3);
mPos3D = zeros(length(range), 3);

bTimestamp = zeros(length(range), 1);
mTimestamp = zeros(length(range), 1);

x = [-1 0 1 1 1 0 -1 -1] * 2;
y = [-1 -1 -1 0 1 1 1 0] * 2;

offs = calculate_offset_value2(maincam, 2, maincam, offset);
for i=range
    i
    tic;
    bMat = [];
    mMat = [];
    
    pos3D_b = zeros(1, 3);
    pos3D_m = zeros(1, 3);
    time_b = 0;
    time_m = 0;
    
    target = timestamp{maincam}.time(i) + offs;
    
    [temp bNearest] = min(abs(bData(:, 3) - target));
    if temp <= 60
        bMat = strcat(fnamePrefix{bData(bNearest,1)},int2str(bData(bNearest,2)),'.mat');
        bImg = fullfile(directory,['cam',num2str(bData(bNearest,1))],[int2str(bData(bNearest,2)),'.png']);
    end
    
    [temp mNearest] = min(abs(mData(:, 3) - target));
    if temp <= 60
        mMat = strcat(fnamePrefix{mData(mNearest,1)},int2str(mData(mNearest,2)),'.mat');
        mImg = fullfile(directory,['cam',num2str(mData(mNearest,1))],[int2str(mData(mNearest,2)),'.png']);
    end
    
    if isempty(bMat) && isempty(mMat)
        I = [blank separator blank];
        
    elseif strcmp(bMat, mMat)
        % Plot bb
        Im = imread(bImg);
        bBB = [bData(bNearest,4), bData(bNearest,5), bData(bNearest,4)+bData(bNearest,6), bData(bNearest,5), bData(bNearest,4)+bData(bNearest,6), bData(bNearest,5)+bData(bNearest,7), bData(bNearest,4), bData(bNearest,5)+bData(bNearest,7)];
        Im = insertShape(Im, 'Polygon', bBB, 'Color', 'green', 'Opacity', 1);
        mBB = [mData(mNearest,4), mData(mNearest,5), mData(mNearest,4)+mData(mNearest,6), mData(mNearest,5), mData(mNearest,4)+mData(mNearest,6), mData(mNearest,5)+mData(mNearest,7), mData(mNearest,4), mData(mNearest,5)+mData(mNearest,7)];
        Im = insertShape(Im, 'Polygon', mBB, 'Color', 'red', 'Opacity', 1);
        
        load(bMat, 'DepthFrame');
        
        DtC = DtCMapping{bData(bNearest,1)};
        pcd(:,:,1) = map_depthframe_to_colorframe(DepthFrame.DepthData.X,DtC);
        pcd(:,:,2) = map_depthframe_to_colorframe(DepthFrame.DepthData.Y,DtC);
        pcd(:,:,3) = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DtC);
        
        % For bData
        maxY = min(480, bData(bNearest,5)+bData(bNearest,7)-1);
        maxX = min(640, bData(bNearest,4)+bData(bNearest,6)-1);
        headPCD = pcd(bData(bNearest,5):maxY,bData(bNearest,4):maxX, :);
        siz = size(headPCD,1) * size(headPCD,2);
        headPCD = [reshape(headPCD(:,:,1),[siz 1]), reshape(headPCD(:,:,2),[siz 1]), reshape(headPCD(:,:,3),[siz 1])];
        valid = find(headPCD(:,3) > 0);
        headPCD = headPCD(valid, :);
        t = track_3d(headPCD, [0 0 0], radiusB);
        if sum(t) ~= 0
            pos3D_b = t;
            pos3D_b = MapToC2(pos3D_b, bData(bNearest,1), R_43,t_43,R_41,t_41,R_42,t_42);
            % plot centroid
            Im = draw_circle(Im, K(bData(bNearest,1)), t, sphereB, 'cyan');
            time_b = bData(bNearest, 3);
        end
        
        % For mData
        maxY = min(480, mData(mNearest,5)+mData(mNearest,7)-1);
        maxX = min(640, mData(mNearest,4)+mData(mNearest,6)-1);
        headPCD = pcd(mData(mNearest,5):maxY,mData(mNearest,4):maxX, :);
        siz = size(headPCD,1) * size(headPCD,2);
        headPCD = [reshape(headPCD(:,:,1),[siz 1]), reshape(headPCD(:,:,2),[siz 1]), reshape(headPCD(:,:,3),[siz 1])];
        valid = find(headPCD(:,3) > 0);
        headPCD = headPCD(valid, :);
        t = track_3d(headPCD, [0 0 0], radiusM);
        if sum(t) ~= 0
            pos3D_m = t;
            pos3D_m = MapToC2(pos3D_m, mData(mNearest,1), R_43,t_43,R_41,t_41,R_42,t_42);
            % plot centroid
            Im = draw_circle(Im, K(mData(mNearest,1)), t, sphereM, 'magenta');
            time_m = mData(mNearest, 3);
        end
        
        I = [Im separator blank];
        
    elseif isempty(bMat)
        % Plot bb
        Im = imread(mImg);
        mBB = [mData(mNearest,4), mData(mNearest,5), mData(mNearest,4)+mData(mNearest,6), mData(mNearest,5), mData(mNearest,4)+mData(mNearest,6), mData(mNearest,5)+mData(mNearest,7), mData(mNearest,4), mData(mNearest,5)+mData(mNearest,7)];
        Im = insertShape(Im, 'Polygon', mBB, 'Color', 'red', 'Opacity', 1);
        
        load(mMat, 'DepthFrame');
        
        DtC = DtCMapping{mData(mNearest,1)};
        pcd(:,:,1) = map_depthframe_to_colorframe(DepthFrame.DepthData.X,DtC);
        pcd(:,:,2) = map_depthframe_to_colorframe(DepthFrame.DepthData.Y,DtC);
        pcd(:,:,3) = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DtC);
        
        % For mData
        maxY = min(480, mData(mNearest,5)+mData(mNearest,7)-1);
        maxX = min(640, mData(mNearest,4)+mData(mNearest,6)-1);
        headPCD = pcd(mData(mNearest,5):maxY,mData(mNearest,4):maxX, :);
        siz = size(headPCD,1) * size(headPCD,2);
        headPCD = [reshape(headPCD(:,:,1),[siz 1]), reshape(headPCD(:,:,2),[siz 1]), reshape(headPCD(:,:,3),[siz 1])];
        valid = find(headPCD(:,3) > 0);
        headPCD = headPCD(valid, :);
        t = track_3d(headPCD, [0 0 0], radiusM);
        if sum(t) ~= 0
            pos3D_m = t;
            pos3D_m = MapToC2(pos3D_m, mData(mNearest,1), R_43,t_43,R_41,t_41,R_42,t_42);
            % plot centroid
            Im = draw_circle(Im, K(mData(mNearest,1)), t, sphereM, 'magenta');
            time_m = mData(mNearest, 3);
        end
        
        I = [blank separator Im];
    
    elseif isempty(mMat)
        Im = imread(bImg);
        bBB = [bData(bNearest,4), bData(bNearest,5), bData(bNearest,4)+bData(bNearest,6), bData(bNearest,5), bData(bNearest,4)+bData(bNearest,6), bData(bNearest,5)+bData(bNearest,7), bData(bNearest,4), bData(bNearest,5)+bData(bNearest,7)];
        Im = insertShape(Im, 'Polygon', bBB, 'Color', 'green', 'Opacity', 1);
        
        load(bMat, 'DepthFrame');
        
        DtC = DtCMapping{bData(bNearest,1)};
        pcd(:,:,1) = map_depthframe_to_colorframe(DepthFrame.DepthData.X,DtC);
        pcd(:,:,2) = map_depthframe_to_colorframe(DepthFrame.DepthData.Y,DtC);
        pcd(:,:,3) = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DtC);
        
        % For bData
        maxY = min(480, bData(bNearest,5)+bData(bNearest,7)-1);
        maxX = min(640, bData(bNearest,4)+bData(bNearest,6)-1);
        headPCD = pcd(bData(bNearest,5):maxY,bData(bNearest,4):maxX, :);
        siz = size(headPCD,1) * size(headPCD,2);
        headPCD = [reshape(headPCD(:,:,1),[siz 1]), reshape(headPCD(:,:,2),[siz 1]), reshape(headPCD(:,:,3),[siz 1])];
        valid = find(headPCD(:,3) > 0);
        headPCD = headPCD(valid, :);
        t = track_3d(headPCD, [0 0 0], radiusB);
        if sum(t) ~= 0
            pos3D_b = t;
            pos3D_b = MapToC2(pos3D_b, bData(bNearest,1), R_43,t_43,R_41,t_41,R_42,t_42);
            % plot centroid
            Im = draw_circle(Im, K(bData(bNearest,1)), t, sphereB, 'cyan');
            time_b = bData(bNearest, 3);
        end
        
        I = [Im separator blank];
        
    else
        % Plot bb
        bIm = imread(bImg);
        bBB = [bData(bNearest,4), bData(bNearest,5), bData(bNearest,4)+bData(bNearest,6), bData(bNearest,5), bData(bNearest,4)+bData(bNearest,6), bData(bNearest,5)+bData(bNearest,7), bData(bNearest,4), bData(bNearest,5)+bData(bNearest,7)];
        bIm = insertShape(bIm, 'Polygon', bBB, 'Color', 'green', 'Opacity', 1);
        
        mIm = imread(mImg);
        mBB = [mData(mNearest,4), mData(mNearest,5), mData(mNearest,4)+mData(mNearest,6), mData(mNearest,5), mData(mNearest,4)+mData(mNearest,6), mData(mNearest,5)+mData(mNearest,7), mData(mNearest,4), mData(mNearest,5)+mData(mNearest,7)];
        mIm = insertShape(mIm, 'Polygon', mBB, 'Color', 'red', 'Opacity', 1);
        
        load(bMat, 'DepthFrame');
        
        DtC = DtCMapping{bData(bNearest,1)};
        pcd(:,:,1) = map_depthframe_to_colorframe(DepthFrame.DepthData.X,DtC);
        pcd(:,:,2) = map_depthframe_to_colorframe(DepthFrame.DepthData.Y,DtC);
        pcd(:,:,3) = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DtC);
        
        % For bData
        maxY = min(480, bData(bNearest,5)+bData(bNearest,7)-1);
        maxX = min(640, bData(bNearest,4)+bData(bNearest,6)-1);
        headPCD = pcd(bData(bNearest,5):maxY,bData(bNearest,4):maxX, :);
        siz = size(headPCD,1) * size(headPCD,2);
        headPCD = [reshape(headPCD(:,:,1),[siz 1]), reshape(headPCD(:,:,2),[siz 1]), reshape(headPCD(:,:,3),[siz 1])];
        valid = find(headPCD(:,3) > 0);
        headPCD = headPCD(valid, :);
        t = track_3d(headPCD, [0 0 0], radiusB);
        if sum(t) ~= 0
            pos3D_b = t;
            pos3D_b = MapToC2(pos3D_b, bData(bNearest,1), R_43,t_43,R_41,t_41,R_42,t_42);
            % plot centroid
            bIm = draw_circle(bIm, K(bData(bNearest,1)), t, sphereB, 'cyan');
            time_b = bData(bNearest, 3);
        end
        
        load(mMat, 'DepthFrame');
        
        DtC = DtCMapping{mData(mNearest,1)};
        pcd(:,:,1) = map_depthframe_to_colorframe(DepthFrame.DepthData.X,DtC);
        pcd(:,:,2) = map_depthframe_to_colorframe(DepthFrame.DepthData.Y,DtC);
        pcd(:,:,3) = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DtC);
        
        % For mData
        maxY = min(480, mData(mNearest,5)+mData(mNearest,7)-1);
        maxX = min(640, mData(mNearest,4)+mData(mNearest,6)-1);
        headPCD = pcd(mData(mNearest,5):maxY,mData(mNearest,4):maxX, :);
        siz = size(headPCD,1) * size(headPCD,2);
        headPCD = [reshape(headPCD(:,:,1),[siz 1]), reshape(headPCD(:,:,2),[siz 1]), reshape(headPCD(:,:,3),[siz 1])];
        valid = find(headPCD(:,3) > 0);
        headPCD = headPCD(valid, :);
        t = track_3d(headPCD, [0 0 0], radiusM);
        if sum(t) ~= 0
            pos3D_m = t;
            pos3D_m = MapToC2(pos3D_m, mData(mNearest,1), R_43,t_43,R_41,t_41,R_42,t_42);
            % plot centroid
            mIm = draw_circle(mIm, K(mData(mNearest,1)), t, sphereM, 'magenta');
            time_m = mData(mNearest, 3);
        end
        
        I = [bIm separator mIm];
    end
    
    toc
%     imshow(I);
%     drawnow;
    
    imwrite(I, fullfile(savepath, num2str(i-range(1)+1, '%d.jpg')), 'Quality', 95);
    bPos3D(i-range(1)+1, :) = pos3D_b;
    mPos3D(i-range(1)+1, :) = pos3D_m;
    bTimestamp(i-range(1)+1, :) = time_b;
    mTimestamp(i-range(1)+1, :) = time_m;
    
    save(fullfile(directory, [savename,'.mat']), 'bPos3D', 'mPos3D', 'bTimestamp', 'mTimestamp');
%     pos3D_b
%     pos3D_m
%     
%     plot3(pos3D_b(1), pos3D_b(2), pos3D_b(3), 'g+');
%     hold on;
%     plot3(pos3D_m(1), pos3D_m(2), pos3D_m(3), 'r+');
%     axis([-1000 1000 -1000 1000 0 2000]);
%     drawnow;
end

system(['ffmpeg -i ', fullfile(savepath, '%d.jpg'), ' -b 3500k ', savepath, '.avi']);

end

function data = RemoveDuplicates(data)

id = data(:,1) * 1e8 + data(:,2);

sel = logical(zeros(size(data,1), 1));
idxAll = 1:size(data,1);
for i=1:size(data, 1)
    dup = (id == id(i));
    dupIdx = idxAll(dup);
    sel(dupIdx(end)) = 1;
end

data = data(sel, :);
end

function val = calculate_offset_value(older_cam,new_cam, offset)
    
    if(older_cam ==1 && new_cam ==1)
        val = 0;
    elseif(older_cam ==1 && new_cam ==2)
        val = -1 * offset(1);    
    elseif(older_cam ==1 && new_cam ==3)
        val = (-1 * offset(1))  + (offset(3));    
    elseif(older_cam ==1 && new_cam ==4)
        val = (-1 * offset(1))  + (offset(4));    

    elseif(older_cam ==2 && new_cam ==1)
        val = offset(1);
    elseif(older_cam ==2 && new_cam ==2)
        val = 0;    
    elseif(older_cam ==2 && new_cam ==3)
        val = (offset(3));
    elseif(older_cam ==2 && new_cam ==4)
        val = (offset(4));
        
    elseif(older_cam ==3 && new_cam ==1)
        val = (-1 * offset(3))  + (offset(1));
    elseif(older_cam ==3 && new_cam ==2)
        val = (-1 * offset(3)) ;
    elseif(older_cam ==3 && new_cam ==3)
        val = 0;    
    elseif(older_cam ==3 && new_cam ==4)
        val = (-1 * offset(3))  + (offset(4));
    
    elseif(older_cam ==4 && new_cam ==1)
        val = (-1 * offset(4))  + (offset(1));
    elseif(older_cam ==4 && new_cam ==2)
        val = (-1 * offset(4))  ;
    elseif(older_cam ==4 && new_cam ==3)
        val = (-1 * offset(4))  + (offset(3));
    elseif(older_cam ==4 && new_cam ==4)
        val = 0;    
        
    end
end

function pos = MapToC2(pos, cam, R_43,t_43,R_41,t_41,R_42,t_42)
if(cam==1)
    pos = map_pcd_head_pos(pos,R_43,t_43,1);
elseif(cam==3)
    pos = map_pcd_head_pos(pos,R_41,t_41,1);     
elseif(cam==4)
    pos = map_pcd_head_pos(pos,R_42,t_42,1);     
else
    pos = map_pcd_head_pos(pos,eye(3,3),zeros(3,1),1);
end
end

function val = calculate_offset_value2(older_cam,new_cam, main_cam, offset)
    if (older_cam == new_cam)
        val = 0;
    elseif (older_cam == main_cam)
        val = offset(new_cam);
    elseif (new_cam == main_cam)
        val = -1 * offset(older_cam);
    else
        val = -1 * offset(older_cam) + offset(new_cam);
    end
end

function Im = draw_circle(Im, K, t, sphere, color)
xpos = t(1)./t(3) * K.fx + K.cx;
ypos = t(2)./t(3) * K.fy + K.cy;
sphere = bsxfun(@plus, sphere, t);
x = sphere(:,1)./sphere(:,3) * K.fx + K.cx;
r = (max(x) - min(x)) / 2;
Im = insertShape(Im, 'circle', [xpos ypos r], 'Color', color, 'Opacity', 1);
end