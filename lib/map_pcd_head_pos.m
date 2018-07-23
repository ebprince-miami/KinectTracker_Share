function points = map_pcd(pcd,R,t,s,color,p)
    points = zeros(size(pcd,1)*size(pcd,2),3);
    
    points = pcd;
%     points(:,1) = reshape(pcd(:,:,1),[1 size(pcd,1)*size(pcd,2)]);
%     points(:,2) = reshape(pcd(:,:,2),[1 size(pcd,1)*size(pcd,2)]);
%     points(:,3) = reshape(pcd(:,:,3),[1 size(pcd,1)*size(pcd,2)]);
      
%     keep = xor(isnan(points(:,1)),1);

    % temporary commented
%     keep = points(:,3)>0;
%     points = points(keep,:);
    
    points = s*R*points' + repmat(t,[1 size(points,1) ]);
    
    if nargin>4
        rgb = zeros(size(color,1)*size(color,2),3);
    
        rgb(:,1) = reshape(color(:,:,1),[1 size(color,1)*size(color,2)]);
        rgb(:,2) = reshape(color(:,:,2),[1 size(color,1)*size(color,2)]);
        rgb(:,3) = reshape(color(:,:,3),[1 size(color,1)*size(color,2)]);
%         rgb = rgb(keep,:);
        points = [points;rgb'];
    end
end