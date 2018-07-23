function [pos3D, timestamp] = InterpolatePos3D(pos3D, timestamp, valid)

firstValid = find(valid, 1, 'first');
lastValid = find(valid, 1, 'last');
missingData = find(valid == 0);

for i=missingData'
    if i < firstValid
        pos3D(i, :) = pos3D(firstValid, :);
        timestamp(i) = timestamp(firstValid) - 40 * (firstValid - i);
    elseif i > lastValid
        pos3D(i, :) = pos3D(lastValid, :);
        timestamp(i) = timestamp(lastValid) + 40 * (i - lastValid);
    else
        prevValid = find(valid(1:i), 1, 'last');
        nextValid = i - 1 + find(valid(i:end), 1, 'first');
        gap = nextValid - prevValid;
        pos3D(i, :) = (nextValid - i)/gap * pos3D(prevValid, :) + (i - prevValid)/gap * pos3D(nextValid, :);
        timeGap = timestamp(nextValid) - timestamp(prevValid);
        timestamp(i) =  round(timestamp(prevValid) + (i - prevValid)/gap * timeGap);
    end
end