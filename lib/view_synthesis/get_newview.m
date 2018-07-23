function [I proj2d Id] = get_newview(az,el,pcd,norm,offset)
% [az el] = view;

I = zeros(480,640,3,'uint8');
Id = zeros(480,640);

K = viewmtx(az,el);
proj = K*[pcd(1:3,:);ones(1,size(pcd,2))];
if nargin<5
    offset(1) = min(0,min(proj(1,:))) + 0.01;
    offset(2) = min(0,min(proj(2,:))) + 0.01;
end
proj(1,:) = proj(1,:) - offset(1);
proj(2,:) = proj(2,:) - offset(2);

if nargin<4
    norm = max(max(proj(2,:)./480),max(proj(1,:)./640));
end

norm
offset

proj2d = ceil((proj(1:3,:)-1e-6)./norm);
proj2d(2,:) = 481 - proj2d(2,:);
proj2d(1,:) = 641 - proj2d(1,:);
% proj2d(:,proj2d(1,:)<1) = [];
% proj2d(:,proj2d(1,:)>640) = [];
% proj2d(:,proj2d(2,:)<1) = [];
% proj2d(:,proj2d(2,:)>480) = [];
proj2d_idx = proj2d(1,:) + proj2d(2,:).*1e6;

[id idx_first] = unique(proj2d_idx,'first');
% [id idx_last] = unique(proj2d_idx,'last');


for i=1:length(id)
%     [minval closest_id] = min(proj2d(3,idx_first(i):idx_last(i)));
    
%     pos = idx_first(i) + closest_id - 1;
%     I(481-proj2d(2,idx(i)),641-proj2d(1,idx(i)),1:3) = pcd(4:6,idx(i));
    
    if proj2d(2,idx_first(i)) > 0 && proj2d(2,idx_first(i)) < 481 && proj2d(1,idx_first(i)) > 0 && proj2d(1,idx_first(i)) < 641
        I(proj2d(2,idx_first(i)),proj2d(1,idx_first(i)),1:3) = pcd(4:6,idx_first(i));
        Id(proj2d(2,idx_first(i)),proj2d(1,idx_first(i))) = proj2d(3,idx_first(i));
    end
end