addpath(genpath('E:\InteractiveTrackingAmin'));

% K3: USB-VID_045E&PID_02AE-A00364A04906127A
% Kinect Unique ID's
% K3: A00364A04906127A
% K4: A00367A14179123A
% K1: A00365A09389104A      % OLD K1
% K2: B00362214481051B
% K2 NEW: A00366809492052A

%p1name = 'cam3/USB-VID_045E&PID_02AE-A00365A09389104A';
%p2name = 'cam4/USB-VID_045E&PID_02AE-A00366809492052A';
%p3name = 'cam1/USB-VID_045E&PID_02AE-A00364A04906127A';
%p4name = 'cam2/USB-VID_045E&PID_02AE-A00367A14179123A';

% datapath{1} = 'F:\FN55-K3-EXPORT';
% datapath{2} = 'F:\FN55-K4-EXPORT';
% datapath{3} = 'F:\FN55-K1-EXPORT';
% datapath{4} = 'F:\FN55-K2-EXPORT';

datapath{1} = 'E:\SN1001\SN1001_K3_export';
datapath{2} = '';
datapath{3} = 'E:\SN1001\SN1001_K1_export';
datapath{4} = 'E:\SN1001\SN1001_K2_export';

% Cam number from datapath
%idx = [1 2 3];
idx = [1 3 4];

savepath = 'E:\SN1001';

ExtractImagesFromMat(datapath, savepath, idx)