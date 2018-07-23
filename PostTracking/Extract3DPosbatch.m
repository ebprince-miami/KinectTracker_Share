function Extract3DPosbatch(dirpath, idx)

sessionList = dir(fullfile(dirpath,'FN*'));

for i=idx %[1:4 6:15]%[2 3 4 7]%1:length(sessionList)
%     if sessionList(i).name(end)  == 'E'
        sessionList(i).name
        Extract3DPos(fullfile(dirpath,sessionList(i).name));
%     end
end