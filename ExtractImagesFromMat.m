function ExtractImagesFromMat(datapath,savepath,bidx)
% datapath is directories where the kinect mat files are saved, the order
% of cam 1 - 4 has to be the same as basename
% savepath is where the new files should be saved
basename = {'USB-VID_045E&PID_02AE-A00364A04906127A';
            'USB-VID_045E&PID_02AE-A00367A14179123A';
            'USB-VID_045E&PID_02AE-0000000000000000';
            'USB-VID_045E&PID_02AE-A00366809492052A'};


mkdir(savepath);
errors = [];
for base_idx = bidx
    filelist = dir(datapath{base_idx});
    filelist = filelist(3:end);

    time = [];
    if exist(fullfile(savepath,['cam',num2str(base_idx),'_timestamp.mat']),'file')
        load(fullfile(savepath,['cam',num2str(base_idx),'_timestamp.mat']),'time');
    end
    
    spath = fullfile(savepath,num2str(base_idx,'cam%d'));
    mkdir(spath);
    
    system(['cp "', fullfile(datapath{base_idx}, [basename{base_idx},'_DepthFrameToColorFrameMapping.mat']),'" "', fullfile(spath, [basename{base_idx},'_DepthFrameToColorFrameMapping.mat']),'"']);
    for i=1:length(filelist)
        if ~isempty(regexp(filelist(i).name,basename{base_idx})) && ~isempty(regexp(filelist(i).name,'.mat')) && isempty(regexp(filelist(i).name,'DepthFrameToColorFrameMapping'))
            [base_idx i]
            separator = regexp(filelist(i).name,'_');
            idx = filelist(i).name(separator(end)+1:end-4);
            if isempty(str2num(idx)) || (exist(fullfile(spath,[idx,'.png']), 'file') && exist(fullfile(spath,filelist(i).name), 'file'))
                continue;
            end
            
            try
            load(fullfile(datapath{base_idx},filelist(i).name));
            
%             imwrite(ColorFrame.ColorData,fullfile(spath,[idx,'.jpg']),'Quality',95);
            imwrite(ColorFrame.ColorData,fullfile(spath,[idx,'.png']));
            %save(fullfile(spath,filelist(i).name),'DepthFrame','FrameNumber','SkeletonFrame','SkeletonFrameToColorFrameMapping','Timestamp');
            save(fullfile(spath,filelist(i).name),'DepthFrame','FrameNumber','Timestamp');
            time(str2num(idx)) = Timestamp;
            save(fullfile(savepath,['cam',num2str(base_idx),'_timestamp.mat']),'time');
            catch
                errors = [errors str2num(idx)];
            end
        end
    end
    save(fullfile(savepath,['cam',num2str(base_idx),'_timestamp.mat']),'time');
    save(fullfile(savepath,['cam',num2str(base_idx),'_log.mat']),'errors');

    system(['ffmpeg -i "', fullfile(savepath,['cam',num2str(base_idx)],'%d.png'), '" -b 2000k "', fullfile(savepath,['cam',num2str(base_idx),'.avi'])]);
end