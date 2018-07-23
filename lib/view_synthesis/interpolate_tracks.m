function track_int = interpolate_tracks(track)

p1name = 'cam3/USB-VID_045E&PID_02AE-A00365A09389104A';
p2name = 'cam4/USB-VID_045E&PID_02AE-B00362214481051B';
p3name = 'cam1/USB-VID_045E&PID_02AE-A00364A04906127A';
p4name = 'cam2/USB-VID_045E&PID_02AE-A00367A14179123A';

cam_name = {p3name, p4name, p1name, p2name}; % p3, p4, p1, p2

filepath = '/home/arri/Dataset/FN038/';
% filepath = 'G:\FN001';

% column of track_int correspond to camid frameno timestamp x y z
% interpolated/not
track_int = [];%zeros(length(track),7); 

% column of prev_valid corresponds to timestamp x y z
prev_valid = [];
next_valid_id = -1;
for i=1:length(track)-1
%     i
    load(fullfile(filepath,[cam_name{track(i).cam},'_',num2str(track(i).frame),'.mat']),'Timestamp');
%     cur_time = double(Timestamp);
    cur_time = track(i).offset;
    if track(i).valid
         track_int(size(track_int,1)+1,:) = [track(i).cam track(i).frame cur_time track(i).x track(i).y double(track(i).z) track(i).bounding_box_x track(i).bounding_box_y track(i).bounding_box_width track(i).bounding_box_height 0.0];
        prev_valid = [cur_time track(i).x track(i).y double(track(i).z)];
%     else
%         if next_valid_id < i
%             next_valid_id = find_nextvalid(track,i+1);
%             load(fullfile(filepath,[cam_name{track(next_valid_id).cam},'_',num2str(track(next_valid_id).frame),'.mat']),'Timestamp');
%             next_valid = [double(Timestamp) track(next_valid_id).x track(next_valid_id).y double(track(next_valid_id).z)];
%         end
%         if next_valid_id == -1;
%             return;
%         end
%         range = next_valid(1) - prev_valid(1);
%         int_pos = (cur_time-prev_valid(1))/range*prev_valid(2:4) + (next_valid(1)-cur_time)/range*next_valid(2:4);
%         track_int(size(track_int,1)+1) = [track(i).cam track(i).frame cur_time int_pos 1];
    end
end
end

function valid_id = find_nextvalid(track, id)
    valid_id = -1;
    for i=id:length(track)
        if track(i).valid
            valid_id = i;
            return;
        end
    end
end