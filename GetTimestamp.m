function Timestamp = GetTimestamp(dirpath, camid, frameno)

basename = {'USB-VID_045E&PID_02AE-A00364A04906127A';
            'USB-VID_045E&PID_02AE-A00367A14179123A';
            'USB-VID_045E&PID_02AE-A00365A09389104A';
            'USB-VID_045E&PID_02AE-A00366809492052A'};
cams = {'cam1', 'cam2', 'cam3', 'cam4'};

load(fullfile(dirpath, cams{camid}, [basename{camid}, '_', num2str(frameno, '%d.mat')]), 'Timestamp');
Timestamp = double(Timestamp);
        