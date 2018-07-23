function smoothedData = Get3DPosKalman(pos3D, timestamp)

firstValid = find(timestamp > 0, 1);
temp = timestamp - timestamp(firstValid);
init = temp < 333 & temp >= 0;

% Detection location for the first 1/3 second;
initialLocations = pos3D(init, :);

deviationFromInit = sqrt(sum(bsxfun(@minus,initialLocations, mean(initialLocations)) .^ 2, 2));

param.MotionModel = 'ConstantVelocity';
param.InitialLocation = mean(initialLocations);
param.InitialEstimateError = [var(deviationFromInit) 10^2];
param.MotionNoise = [5^2 3^2];
param.MeasurementNoise = [150^2];

kalmanFilter = configureKalmanFilter(param.MotionModel, ...
    param.InitialLocation, ...
    param.InitialEstimateError, ...
    param.MotionNoise, ...
    param.MeasurementNoise);

smoothedData = zeros(size(pos3D));
smoothedData(1, :) = param.InitialLocation;
for i=2:size(pos3D,1)
%     timeDiff = timestamp(i) - timestamp(i-1);
%     if timeDiff > 45
%         numMissing = floor(timeDiff / 33);
%         for j=1:numMissing
%             predict(kalmanFilter);
%         end
%     end
    if timestamp(i) > 0
        smoothedData(i, :) = predict(kalmanFilter);
        smoothedData(i, :) = correct(kalmanFilter, pos3D(i, :));
    else
        smoothedData(i, :) = predict(kalmanFilter);
    end
end