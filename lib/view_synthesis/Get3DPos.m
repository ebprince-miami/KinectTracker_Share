function Get3DPos(bfile, mfile, directory, refCamID, range, savename)
% mkdir(savepath);

% load('C:\Users\Explorer\Documents\Dropbox\Dropbox\Code\strange_situation\code\new_stuff\code\calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
load('D:\Dropbox\Code\strange_situation\code\new_stuff\code\calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
% load('/home/arri/Dropbox/Code/strange_situation/code/new_stuff/code/calib_all.mat','R_41','t_41','R_42','t_42','R_43','t_43');
fnamePrefix{1} = strcat(directory,'/cam1/','USB-VID_045E&PID_02AE-A00364A04906127A_');
fnamePrefix{2} = strcat(directory,'/cam2/','USB-VID_045E&PID_02AE-A00367A14179123A_');
fnamePrefix{3} = strcat(directory,'/cam3/','USB-VID_045E&PID_02AE-A00365A09389104A_');
fnamePrefix{4} = strcat(directory,'/cam4/','USB-VID_045E&PID_02AE-B00362214481051B_');

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
    
    target = timestamp{maincam}.time(i) + offs;
    
    [temp bNearest] = min(abs(bData(:, 3) - target));
    if temp <= 60
        bMat = strcat(fnamePrefix{bData(bNearest,1)},int2str(bData(bNearest,2)),'.mat');
    end
    
    [temp mNearest] = min(abs(mData(:, 3) - target));
    if temp <= 60
        mMat = strcat(fnamePrefix{mData(mNearest,1)},int2str(mData(mNearest,2)),'.mat');
    end
    if isempty(bMat) && isempty(mMat)
        
    elseif strcmp(bMat, mMat)
        load(bMat, 'DepthFrame');
        
        DtC = DtCMapping{bData(bNearest,1)};
        pcd(:,:,1) = map_depthframe_to_colorframe(DepthFrame.DepthData.X,DtC);
        pcd(:,:,2) = map_depthframe_to_colorframe(DepthFrame.DepthData.Y,DtC);
        pcd(:,:,3) = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DtC);
        
        % For bData
        bbLoc = [bData(bNearest,4)+round(bData(bNearest,6)/2) bData(bNearest,5)+round(bData(bNearest,7)/2)];
        
        found = 1;
        if pcd(bbLoc(2), bbLoc(1),3) == -1
            try
            newbbLoc = [];
            found = 0;
            for j=1:length(x)
                newbbLoc = [bbLoc(1)+x(j) bbLoc(2)+y(j)];
                if pcd(newbbLoc(2), newbbLoc(1),3) ~= -1
                    found = 1;
                    break;
                end
            end
            bbLoc = newbbLoc;
            catch
            end
        end
        
        if found
            pos3D_b = reshape(pcd(bbLoc(2), bbLoc(1), :), [1 3]);
            pos3D_b = MapToC2(pos3D_b, bData(bNearest,1), R_43,t_43,R_41,t_41,R_42,t_42);
        end
        
        % For mData
        bbLoc = [mData(mNearest,4)+round(mData(mNearest,6)/2) mData(mNearest,5)+round(mData(mNearest,7)/2)];
        
        found = 1;
        if pcd(bbLoc(2), bbLoc(1),3) == -1
            try
            newbbLoc = [];
            found = 0;
            for j=1:length(x)
                newbbLoc = [bbLoc(1)+x(j) bbLoc(2)+y(j)];
                if pcd(newbbLoc(2), newbbLoc(1),3) ~= -1
                    found = 1;
                    break;
                end
            end
            bbLoc = newbbLoc;
            catch
            end
        end
        
        if found
            pos3D_m = reshape(pcd(bbLoc(2), bbLoc(1), :), [1 3]);
            pos3D_m = MapToC2(pos3D_m, mData(mNearest,1), R_43,t_43,R_41,t_41,R_42,t_42);
        end
        
    elseif isempty(bMat)
        load(mMat, 'DepthFrame');
        
        DtC = DtCMapping{mData(mNearest,1)};
        pcd(:,:,1) = map_depthframe_to_colorframe(DepthFrame.DepthData.X,DtC);
        pcd(:,:,2) = map_depthframe_to_colorframe(DepthFrame.DepthData.Y,DtC);
        pcd(:,:,3) = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DtC);
        
        % For mData
        bbLoc = [mData(mNearest,4)+round(mData(mNearest,6)/2) mData(mNearest,5)+round(mData(mNearest,7)/2)];
        
        found = 1;
        if pcd(bbLoc(2), bbLoc(1),3) == -1
            try
            newbbLoc = [];
            found = 0;
            for j=1:length(x)
                newbbLoc = [bbLoc(1)+x(j) bbLoc(2)+y(j)];
                if pcd(newbbLoc(2), newbbLoc(1),3) ~= -1
                    found = 1;
                    break;
                end
            end
            bbLoc = newbbLoc;
            catch
            end
        end
        
        if found
            pos3D_m = reshape(pcd(bbLoc(2), bbLoc(1), :), [1 3]);
            pos3D_m = MapToC2(pos3D_m, mData(mNearest,1), R_43,t_43,R_41,t_41,R_42,t_42);
        end
    
    elseif isempty(mMat)
        load(bMat, 'DepthFrame');
        
        DtC = DtCMapping{bData(bNearest,1)};
        pcd(:,:,1) = map_depthframe_to_colorframe(DepthFrame.DepthData.X,DtC);
        pcd(:,:,2) = map_depthframe_to_colorframe(DepthFrame.DepthData.Y,DtC);
        pcd(:,:,3) = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DtC);
        
        % For bData
        bbLoc = [bData(bNearest,4)+round(bData(bNearest,6)/2) bData(bNearest,5)+round(bData(bNearest,7)/2)];
        
        found = 1;
        if pcd(bbLoc(2), bbLoc(1),3) == -1
            try
            newbbLoc = [];
            found = 0;
            for j=1:length(x)
                newbbLoc = [bbLoc(1)+x(j) bbLoc(2)+y(j)];
                if pcd(newbbLoc(2), newbbLoc(1),3) ~= -1
                    found = 1;
                    break;
                end
            end
            bbLoc = newbbLoc;
            catch
            end
        end
        
        if found
            pos3D_b = reshape(pcd(bbLoc(2), bbLoc(1), :), [1 3]);
            pos3D_b = MapToC2(pos3D_b, bData(bNearest,1), R_43,t_43,R_41,t_41,R_42,t_42);
        end
        
    else
        load(bMat, 'DepthFrame');
        
        DtC = DtCMapping{bData(bNearest,1)};
        pcd(:,:,1) = map_depthframe_to_colorframe(DepthFrame.DepthData.X,DtC);
        pcd(:,:,2) = map_depthframe_to_colorframe(DepthFrame.DepthData.Y,DtC);
        pcd(:,:,3) = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DtC);
        
        % For bData
        bbLoc = [bData(bNearest,4)+round(bData(bNearest,6)/2) bData(bNearest,5)+round(bData(bNearest,7)/2)];
        
        found = 1;
        if pcd(bbLoc(2), bbLoc(1),3) == -1
            try
            newbbLoc = [];
            found = 0;
            for j=1:length(x)
                newbbLoc = [bbLoc(1)+x(j) bbLoc(2)+y(j)];
                if pcd(newbbLoc(2), newbbLoc(1),3) ~= -1
                    found = 1;
                    break;
                end
            end
            bbLoc = newbbLoc;
            catch
            end
        end
        
        if found
            pos3D_b = reshape(pcd(bbLoc(2), bbLoc(1), :), [1 3]);
            pos3D_b = MapToC2(pos3D_b, bData(bNearest,1), R_43,t_43,R_41,t_41,R_42,t_42);
        end
        
        load(mMat, 'DepthFrame');
        
        DtC = DtCMapping{mData(mNearest,1)};
        pcd(:,:,1) = map_depthframe_to_colorframe(DepthFrame.DepthData.X,DtC);
        pcd(:,:,2) = map_depthframe_to_colorframe(DepthFrame.DepthData.Y,DtC);
        pcd(:,:,3) = map_depthframe_to_colorframe(DepthFrame.DepthData.Depth,DtC);
        
        % For mData
        bbLoc = [mData(mNearest,4)+round(mData(mNearest,6)/2) mData(mNearest,5)+round(mData(mNearest,7)/2)];
        
        found = 1;
        if pcd(bbLoc(2), bbLoc(1),3) == -1
            try
            newbbLoc = [];
            found = 0;
            for j=1:length(x)
                newbbLoc = [bbLoc(1)+x(j) bbLoc(2)+y(j)];
                if pcd(newbbLoc(2), newbbLoc(1),3) ~= -1
                    found = 1;
                    break;
                end
            end
            bbLoc = newbbLoc;
            catch
            end
        end
        
        if found
            pos3D_m = reshape(pcd(bbLoc(2), bbLoc(1), :), [1 3]);
            pos3D_m = MapToC2(pos3D_m, mData(mNearest,1), R_43,t_43,R_41,t_41,R_42,t_42);
        end
    end
    
    toc
%     imshow(I);
%     drawnow;
    bPos3D(i-range(1)+1, :) = pos3D_b;
    mPos3D(i-range(1)+1, :) = pos3D_m;
    
%     save(fullfile(directory, [savename,'.mat']), 'bPos3D', 'mPos3D');
%     pos3D_b
%     pos3D_m
%     
%     plot3(pos3D_b(1), pos3D_b(2), pos3D_b(3), 'g+');
%     hold on;
%     plot3(pos3D_m(1), pos3D_m(2), pos3D_m(3), 'r+');
%     axis([-1000 1000 -1000 1000 0 2000]);
%     drawnow;
end

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