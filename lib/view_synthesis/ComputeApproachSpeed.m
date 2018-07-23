
function ComputeApproachSpeed(bPos3D, mPos3D, savepath, bPos3DOld, mPos3DOld)
pos3d = [bPos3D';mPos3D'];
pos3d = reshape(pos3d,[3 size(bPos3D,1)*2]);
pos3dOld = [bPos3DOld';mPos3DOld'];
pos3dOld = reshape(pos3dOld,[3 size(bPos3DOld,1)*2]);
filt = CreateGaussFilter(3, 12);

for i=1:2:size(pos3d,2)
    idx = ceil(i/2);
    dist3d(idx) = norm(pos3d(1:3,i)-pos3d(1:3,i+1));
    if idx == 1
        babyApproach(idx) = 0;
        momApproach(idx) = 0;
    else
        babyApproach(idx) = dist3d(idx - 1) - norm(pos3d(1:3,i)-pos3d(1:3,i+1-2));
        momApproach(idx) = dist3d(idx - 1) - norm(pos3d(1:3,i-2)-pos3d(1:3,i+1));
    end
end

babyApproach = conv(babyApproach, filt, 'same');
momApproach = conv(momApproach, filt, 'same');

isBabyInterpolated = zeros(1,length(dist3d));
isMomInterpolated = zeros(1,length(dist3d));
for i=1:length(dist3d)
    if (sum(pos3dOld(1:3,i*2-1)==[0;0;0]) == 3)
        isBabyInterpolated(i) = 1;
    elseif (sum(pos3dOld(1:3,i*2)==[0;0;0]) == 3)
        isMomInterpolated(i) = 1;
    end
end

for i=1:length(dist3d)
    if (sum(pos3d(1:3,i*2)==[0;0;0]) == 3) || (sum(pos3d(1:3,i*2-1)==[0;0;0]) == 3)
        dist3d(i) = nan;
    end
end

load('GroundPlaneMapping.mat','R','t');
bHeight = GetHeadHeightWrtFloor(bPos3D, R, t);
mHeight = GetHeadHeightWrtFloor(mPos3D, R, t);

isBabyCloseToMom = dist3d < 900;
isBabyOnTheFloor = bHeight < 900;

save(savepath,'bPos3D','mPos3D','babyApproach','momApproach','isBabyInterpolated','isMomInterpolated','dist3d','bHeight','mHeight','isBabyCloseToMom','isBabyOnTheFloor');

    
end


function gaussFilter = CreateGaussFilter(sigma, size)

sigma = 5;
size = 30;
x = linspace(-size / 2, size / 2, size);
gaussFilter = exp(-x .^ 2 / (2 * sigma ^ 2));
gaussFilter = gaussFilter / sum (gaussFilter); % normalize

end