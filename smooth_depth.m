function Id = smooth_depth(Id)

idx = find(Id==0);
x =repmat( 1:size(Id,2),size(Id,1),1);
y = repmat([1:size(Id,1)]',1,size(Id,2));
x = x(idx);
y = y(idx);

% pad with zeros
pad = 300;
Id = [zeros(size(Id,1),pad) Id zeros(size(Id,1),pad)];
Id = [zeros(pad,size(Id,2));Id;zeros(pad,size(Id,2))]; 
x = x+pad;
y = y+pad;

smoothed_d = zeros(1,length(x));
for i=1:length(x)
    range = 1;
    total = [];
    neighbors = [];
    while isempty(total) % total==0
        neighbors = Id(y(i)-range:y(i)+range,x(i)-range:x(i)+range);
%         total = sum(neighbors(:));
        total = find(neighbors, 1);
        range = range+1;
    end
%     smoothed_d(i) = total / sum(neighbors(:)>0);
    smoothed_d(i) = median(neighbors(neighbors(:)>0));
end

Id = Id(pad+1:end-pad,pad+1:end-pad);
Id(idx) = smoothed_d;