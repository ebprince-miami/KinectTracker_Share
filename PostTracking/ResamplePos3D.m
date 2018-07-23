function [Pos3DResampled, interpolationType] = ResamplePos3D(pos3D, timestamp, valid, newtimestamp)

firstValid = find(valid, 1, 'first');
lastValid = find(valid, 1, 'last');

interpolationType = zeros(1, length(newtimestamp));
Pos3DResampled = zeros(length(newtimestamp), 3);

validTimestamp = timestamp(valid);
validPos3D = pos3D(valid, :);

for i=1:length(newtimestamp)
    if newtimestamp(i) < timestamp(firstValid)
        Pos3DResampled(i, :) = pos3D(firstValid, :);
        interpolationType(i) = 1;
    elseif newtimestamp(i) > timestamp(lastValid)
        Pos3DResampled(i, :) = pos3D(lastValid, :);
        interpolationType(i) = 3;
    else
        idxPrev = find(timestamp(valid) <= newtimestamp(i), 1, 'last');
        idxNext = find(timestamp(valid) >= newtimestamp(i), 1, 'first');
        
        timePrev = validTimestamp(idxPrev);
        timeNext = validTimestamp(idxNext);
        
        gap = timeNext - timePrev;
        if gap == 0
            Pos3DResampled(i, :) = validPos3D(idxPrev, :);
        else
            Pos3DResampled(i, :) = (timeNext - newtimestamp(i))/gap * validPos3D(idxPrev, :) + (newtimestamp(i) - timePrev)/gap * validPos3D(idxNext, :);
        end
        interpolationType(i) = 2;
    end
end