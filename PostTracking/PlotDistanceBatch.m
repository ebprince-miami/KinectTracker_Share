function PlotDistanceBatch(dirpath)

sessionList = dir(fullfile(dirpath,'FN*'));

for i=8%2:3
    sessionList(i).name
    PlotDistance(fullfile(dirpath,sessionList(i).name));
end