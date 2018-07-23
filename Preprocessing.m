
% K3: USB-VID_045E&PID_02AE-A00364A04906127A
% Kinect Unique ID's
% K3: A00364A04906127A
% K4: A00367A14179123A
% K1: A00365A09389104A      % OLD K1 and new K5 value
% K2: B00362214481051B  OLD
% K2 NEW: A00366809492052A
% K5 NEW: B00362214481051B

%p1name = 'cam3/USB-VID_045E&PID_02AE-A00365A09389104A';
%p2name = 'cam4/USB-VID_045E&PID_02AE-A00366809492052A';
%p3name = 'cam1/USB-VID_045E&PID_02AE-A00364A04906127A';
%p4name = 'cam2/USB-VID_045E&PID_02AE-A00367A14179123A';

% datapath{1} = 'F:\FN55-K3-EXPORT';
% datapath{2} = 'F:\FN55-K4-EXPORT';
% datapath{3} = 'F:\FN55-K1-EXPORT';
% datapath{4} = 'F:\FN55-K2-EXPORT';

addpath(genpath('E:\InteractiveTrackingAmin'));
datapath{1} = 'I:\SN1023_12m_SS_K3';
datapath{2} = 'I:\SN1023_12m_SS_K4';
datapath{3} = 'I:\SN1023_12m_SS_K1';
datapath{4} = '';

% Cam number from datapath
%idx = [1 2 3];
idx = [3];

savepath = 'I:\';

ExtractImagesFromMat(datapath, savepath, idx)