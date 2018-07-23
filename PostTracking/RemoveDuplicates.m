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