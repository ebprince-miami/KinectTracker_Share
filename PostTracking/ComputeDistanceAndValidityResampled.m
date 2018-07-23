function ComputeDistanceAndValidityResampled(bPos3D, mPos3D, bTimestamp, mTimestamp, savepath)

isBabyDataValid = bTimestamp > 0;
isMomDataValid = mTimestamp > 0;

bCaptureTimeDiff = bTimestamp(2:end) - bTimestamp(1:end-1);
bCaptureTimeDiff(bCaptureTimeDiff <= 0) = 33;
bMovingSpeed = sqrt(sum((bPos3D(2:end, :) - bPos3D(1:end-1, :)).^2,2)) ./ bCaptureTimeDiff;

mCaptureTimeDiff = mTimestamp(2:end) - mTimestamp(1:end-1);
mCaptureTimeDiff(mCaptureTimeDiff <= 0) = 33;
mMovingSpeed = sqrt(sum((mPos3D(2:end, :) - mPos3D(1:end-1, :)).^2,2)) ./ mCaptureTimeDiff;

bUnlikelyMove = bMovingSpeed > 1.75;
mUnlikelyMove = mMovingSpeed > 2;

isBabyDataValid([bUnlikelyMove; logical(0)]) = 0;
isMomDataValid([mUnlikelyMove; logical(0)]) = 0;

% Resample data
timeBegin = min(min(bTimestamp(bTimestamp > 0)), min(mTimestamp(mTimestamp > 0)));
timeEnd = max(max(bTimestamp(bTimestamp > 0)), max(mTimestamp(mTimestamp > 0)));
timestampResampled = timeBegin:40:timeEnd;
[bPos3DResampled, bInterpolationType] = ResamplePos3D(bPos3D, bTimestamp, isBabyDataValid, timestampResampled);
[mPos3DResampled, mInterpolationType] = ResamplePos3D(mPos3D, mTimestamp, isMomDataValid, timestampResampled);

bCaptureTimeDiff = bTimestamp(2:end) - bTimestamp(1:end-1);
bCaptureTimeDiff(bCaptureTimeDiff <= 0) = 33;
bMovingSpeed = sqrt(sum((bPos3D(2:end, :) - bPos3D(1:end-1, :)).^2,2)) ./ bCaptureTimeDiff;

mCaptureTimeDiff = mTimestamp(2:end) - mTimestamp(1:end-1);
mCaptureTimeDiff(mCaptureTimeDiff <= 0) = 33;
mMovingSpeed = sqrt(sum((mPos3D(2:end, :) - mPos3D(1:end-1, :)).^2,2)) ./ mCaptureTimeDiff;

pos3d = [bPos3DResampled';mPos3DResampled'];
pos3d = reshape(pos3d,[3 size(bPos3DResampled,1)*2]);
for i=1:2:size(pos3d,2)
    idx = ceil(i/2);
    dist3d(idx) = norm(pos3d(1:3,i)-pos3d(1:3,i+1));
    if idx == 1
        babyApproach(idx) = 0;
        momApproach(idx) = 0;
    else
        babyApproach(idx) = (dist3d(idx - 1) - norm(pos3d(1:3,i)-pos3d(1:3,i+1-2))) / 40;
        momApproach(idx) = (dist3d(idx - 1) - norm(pos3d(1:3,i-2)-pos3d(1:3,i+1))) / 40;
    end
end

load('GroundPlaneMapping.mat','R','t');
bHeightFromFloor = GetHeadHeightWrtFloor(bPos3DResampled, R, t);
mHeightFromFloor = GetHeadHeightWrtFloor(mPos3DResampled, R, t);

isBabyCloseToMom = dist3d < 800;
isBabyOnTheFloor = bHeightFromFloor < 900;

save(savepath,'bPos3DResampled','mPos3DResampled','bPos3D','mPos3D','bTimestamp','mTimestamp','babyApproach','momApproach','dist3d','bHeightFromFloor','mHeightFromFloor','isBabyCloseToMom','isBabyOnTheFloor','bInterpolationType','mInterpolationType','timestampResampled');
