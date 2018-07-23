function mapped_img = map_depthframe_to_colorframe(depthimg,mapping)

mapping = [mapping.Y(:) mapping.X(:)];
I.X = repmat([1:size(depthimg,2)],size(depthimg,1),1);
I.Y = repmat([1:size(depthimg,1)]',1,size(depthimg,2));
I = [I.Y(:) I.X(:)];

% [mapping idx] = unique(mapping,'rows');
% I = I(idx,:);
keep = (mapping(:,1)<=size(depthimg,1)) & (mapping(:,2)<=size(depthimg,2));
mapping = mapping(keep,:);
I = I(keep,:);

mapped_img = zeros(size(depthimg));
mapping = double(mapping(:,1)) + double(mapping(:,2)-1)*size(depthimg,1);
I = double(I(:,1)) + double(I(:,2)-1)*size(depthimg,1);
mapped_img(mapping) = depthimg(I);