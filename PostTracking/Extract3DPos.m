function Extract3DPos(datapath)

events = dir(fullfile(datapath,'*_range.txt'));

baby = dir(fullfile(datapath,'baby*.txt'));
mother = dir(fullfile(datapath,'mother*.txt'));
if isempty(mother)
    mother = dir(fullfile(datapath,'mom*.txt'));
end
toy = dir(fullfile(datapath,'toy*.txt'));

for i=1:length(events)
%     try
        events(i).name
        rangestr = textread(fullfile(datapath,events(i).name),'%s');
        sep = regexp(rangestr{1},':');
        range = [str2double(rangestr{1}(1:sep-1)):str2double(rangestr{1}(sep+1:end))];

%         savepath = fullfile(datapath, events(i).name(1:end-10));
        savename = num2str(str2num(events(i).name(end-10)),'Pos3D_R%d_orig');
%         Get3DPos(baby(i).name, mother(i).name, datapath, [], range, savename)
%         Get3DPosLSQFIT(baby(i).name, mother(i).name, datapath, [], range, savename)

        savepath = fullfile(datapath, savename);
        delete([savepath, '.avi']);
        if datapath(end)  ~= 'E'
            Get3DPosLSQTrack(baby(i).name, mother(i).name, datapath, [], range, savename)
        else
            Get3DPosLSQTrack(baby(i).name, toy(i).name, datapath, [], range, savename, 70)
        end

        load([savepath,'.mat']);
        origpath = savepath;
        savepath = fullfile(datapath, num2str(str2num(events(i).name(end-10)),'Pos3D_R%d_resampled'));
        ComputeDistanceAndValidityResampled(bPos3D, mPos3D, bTimestamp, mTimestamp, [savepath,'.mat'])
        
        %%% copy file to shared dropbox folder %%%
%         sessionName = strsplit(datapath,'\')
%         targetpath = fullfile('D:\Dropbox\COLBY3000\Jan2014',sessionName{end});
%         mkdir(targetpath);
%         targetpath = fullfile(targetpath, savename);
%         delete([targetpath, '.mat']);
%         delete([targetpath, '.avi']);
%         copyfile([savepath, '.mat'], [targetpath, '.mat']);
%         copyfile([origpath, '.avi'], [targetpath, '.avi']);
end