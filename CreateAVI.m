function CreateAVI(dirpath, idx)

dirlist = dir(fullfile(dirpath,'FN*'));
% dirlist = dirlist(3:end);

for i=idx%1:length(dirlist)%idx
    if dirlist(i).isdir
        camlist = dir(fullfile(dirpath, dirlist(i).name));
        camlist = camlist(3:end);
        for j=1:length(camlist)
            if camlist(j).isdir && ~exist(fullfile(dirpath,dirlist(i).name,[camlist(j).name,'.avi']), 'file')
                system(['ffmpeg -i "', fullfile(dirpath,dirlist(i).name,camlist(j).name,'%d.png'), '" -b 2000k "', fullfile(dirpath,dirlist(i).name,[camlist(j).name,'.avi"'])]);
            end
        end
    end
end

