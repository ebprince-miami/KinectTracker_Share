function varargout = InteractiveTracking(varargin)
% InteractiveTracking MATLAB code for InteractiveTracking.fig
%      InteractiveTracking, by itself, creates a new InteractiveTracking or raises the existing
%      singleton*.
%
%      H = InteractiveTracking returns the handle to a new InteractiveTracking or the handle to
%      the existing singleton*.
%
%      InteractiveTracking('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in InteractiveTracking.M with the given input arguments.
%
%      InteractiveTracking('Property','Value',...) creates a new InteractiveTracking or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before InteractiveTracking_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to InteractiveTracking_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help InteractiveTracking

% Last Modified by GUIDE v2.5 04-May-2015 03:16:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @InteractiveTracking_OpeningFcn, ...
                   'gui_OutputFcn',  @InteractiveTracking_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before InteractiveTracking is made visible.
function InteractiveTracking_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to InteractiveTracking (see VARARGIN)
    
    addpath(genpath('.')) % added by Amin on 10142016
    
    global directory_selected directory kinect_id model frame_number isPlaying;
    global offset timestamp_cam1 timestamp_cam2 timestamp_cam3 timestamp_cam4 valid_view imhandle;
    
    imhandle = [];
    directory = [];
    
    
    directory_selected = 0;
    frame_number = 10000;
    kinect_id = 2;
    valid_view = 0;
    isPlaying = 0;
    model = [];
    
    % Choose default command line output for InteractiveTracking
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);

    % UIWAIT makes InteractiveTracking wait for user response (see UIRESUME)
    % uiwait(handles.figure1);

    % --- Outputs from this function are returned to the command line.
    function varargout = InteractiveTracking_OutputFcn(hObject, eventdata, handles) 
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;


% --- Executes on button press in pushbutton1. Start of video...
function pushbutton1_Callback (hObject, eventdata, handles)
    global directory_selected isPlaying frame_number;
    if(directory_selected == 1)        
        isPlaying = 1;
        while(isPlaying == 1)
            frame_number = frame_number + 1;
            DisplayImage(handles);
        end
    end
        
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton2.
% Adjust bounding box
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global model frame_number fnamePrefix kinect_id directory DtCMapping max_frames bData timestamp_cam1 timestamp_cam2 timestamp_cam3 timestamp_cam4 imhandle;

imfile = fullfile(directory,['cam',num2str(kinect_id),'/'],[int2str(frame_number),'.png']);
set(handles.text2, 'String',num2str([frame_number,max_frames], '%d/%d'));
set(handles.slider1,'Value',frame_number);
I = imread(imfile);
% [model] = GetHeadTemplate2(I, fnamePrefix{kinect_id}, frame_number, DtCMapping{kinect_id}, imhandle);
[model] = GetHeadTemplateColor(I, fnamePrefix{kinect_id}, frame_number, DtCMapping{kinect_id}, imhandle);
idxB = [];
if ~isempty(bData)
    idxB = find(bData(:,1)==kinect_id & bData(:,2)==frame_number);
end
if ~isempty(idxB)
    bData(idxB,4:8) = [model.bbox 1];
else
    eval(['time = timestamp_cam', num2str(kinect_id), '.time(', num2str(frame_number), ');']);
    offset = calculate_offset_value(kinect_id, 2);
    time_wrt_cam2 = time + offset;
    bData(end+1,:) = [kinect_id frame_number time_wrt_cam2 model.bbox 1];
end

% Interpolate backward
if ~isempty(bData)
    bData = RemoveDuplicates(bData);
    minIdx = frame_number - 35;
    idxB = [];
    % look for previous annotations
    for i=frame_number-1:-1:minIdx
        idxB = find(bData(:,1)==kinect_id & bData(:,2)==i & bData(:,8)==1);
        if ~isempty(idxB)
            break;
        end
    end
    if isempty(idxB) % if no annotation within a certain range
        for i=minIdx:frame_number-2
            idxB = find(bData(:,1)==kinect_id & bData(:,2)==i);
            if ~isempty(idxB)
                break;
            end
        end
    end
    if ~isempty(idxB)
        posBegin = bData(idxB,4:5) + bData(idxB,6:7)/2;
        posEnd = model.bbox(1:2) + model.bbox(3:4)/2;
        widthBegin = bData(idxB,6);
        widthEnd = model.bbox(3);
        heightBegin = bData(idxB,7);
        heightEnd = model.bbox(4);
        for i=bData(idxB,2)+1:frame_number-1
            w1 = i-bData(idxB,2);
            w2 = frame_number-i;
            pos = posBegin * w2/(w1+w2) + posEnd * w1/(w1+w2);
            width = widthBegin * w2/(w1+w2) + widthEnd * w1/(w1+w2);
            height = heightBegin * w2/(w1+w2) + heightEnd * w1/(w1+w2);
            bb = [round(pos - [width height]/2) round(width) round(height)];
            
            idx = [];
            idx = find(bData(:,1)==kinect_id & bData(:,2)==i);
            if ~isempty(idx)
                bData(idx,4:7) = bb;
            else
                eval(['t = timestamp_cam', num2str(kinect_id), '.time(', num2str(frame_number), ');']);
                offset = calculate_offset_value(kinect_id, 2);
                time_wrt_cam2 = t + offset;
                bData(end+1,:) = [kinect_id i time_wrt_cam2 bb 0];
            end
        end
    end
end

% Interpolate forward
if ~isempty(bData)
    bData = RemoveDuplicates(bData);
    maxIdx = frame_number + 35;
    idxB = [];
    % look for forward annotations
    for i=frame_number+1:maxIdx
        idxB = find(bData(:,1)==kinect_id & bData(:,2)==i & bData(:,8)==1);
        if ~isempty(idxB)
            break;
        end
    end
    if isempty(idxB) % if no annotation within a certain range
        for i=maxIdx:-1:frame_number+2
            idxB = find(bData(:,1)==kinect_id & bData(:,2)==i);
            if ~isempty(idxB)
                break;
            end
        end
    end
    if ~isempty(idxB)
        posBegin = model.bbox(1:2) + model.bbox(3:4)/2;
        posEnd = bData(idxB,4:5) + bData(idxB,6:7)/2;
        widthBegin = model.bbox(3);
        widthEnd = bData(idxB,6);
        heightBegin = model.bbox(4);
        heightEnd = bData(idxB,7);
        for i=frame_number+1:bData(idxB,2)-1
            w2 = bData(idxB,2) - i;
            w1 = i - frame_number;
            pos = posBegin * w2/(w1+w2) + posEnd * w1/(w1+w2);
            width = widthBegin * w2/(w1+w2) + widthEnd * w1/(w1+w2);
            height = heightBegin * w2/(w1+w2) + heightEnd * w1/(w1+w2);
            bb = [round(pos - [width height]/2) round(width) round(height)];
            
            idx = [];
            idx = find(bData(:,1)==kinect_id & bData(:,2)==i);
            if ~isempty(idx)
                bData(idx,4:7) = bb;
            else
                eval(['t = timestamp_cam', num2str(kinect_id), '.time(', num2str(frame_number), ');']);
                offset = calculate_offset_value(kinect_id, 2);
                time_wrt_cam2 = t + offset;
                bData(end+1,:) = [kinect_id i time_wrt_cam2 bb 0];
            end
        end
    end
end

if ~isempty(bData)
    idxB = find(bData(:,1)==kinect_id & bData(:,2)==frame_number);
    if ~isempty(idxB)
        bboxPolygon = [bData(idxB,4), bData(idxB,5), bData(idxB,4)+bData(idxB,6), bData(idxB,5), bData(idxB,4)+bData(idxB,6), bData(idxB,5)+bData(idxB,7), bData(idxB,4), bData(idxB,5)+bData(idxB,7)];
        I = insertShape(I, 'Polygon', bboxPolygon, 'Color', 'green', 'LineWidth', 4);
    end
end

if isempty(imhandle)
    imhandle = imshow(I);
else
    set(imhandle, 'CData', I);
end

drawnow update;
    

% --- Executes on button press in pushbutton5 --- GO TO SPECIFIC FRAME.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global frame_number kinect_id directory imhandle bData;
    number = inputdlg('Enter Frame Number:','Go to a specific frame number');
    if isempty(number)
        return;
    end
    frame_number = str2double(number{1});
    
    if(kinect_id == 1)
        imfile = fullfile(directory,'cam1/',[int2str(frame_number),'.png']);
    elseif(kinect_id == 2)
        imfile = fullfile(directory,'cam2/',[int2str(frame_number),'.png']);
    elseif(kinect_id == 3)
        imfile = fullfile(directory,'cam3/',[int2str(frame_number),'.png']);
    else 
        imfile = fullfile(directory,'cam4/',[int2str(frame_number),'.png']);
    end

    I = imread(imfile);
    % Draw tracked bounding box
    if ~isempty(bData)
        idxB = find(bData(:,1)==kinect_id & bData(:,2)==frame_number);
        if ~isempty(idxB)
            bboxPolygon = [bData(idxB,4), bData(idxB,5), bData(idxB,4)+bData(idxB,6), bData(idxB,5), bData(idxB,4)+bData(idxB,6), bData(idxB,5)+bData(idxB,7), bData(idxB,4), bData(idxB,5)+bData(idxB,7)];
            I = insertShape(I, 'Polygon', bboxPolygon, 'Color', 'green', 'LineWidth', 4);
        end
    end
    if isempty(imhandle)
        imhandle = imshow(I);
    else
        set(imhandle, 'CData', I);
    end
    drawnow update;
    max_frames = GetMaxFrames(kinect_id);
    set(handles.text2,'String',num2str([frame_number,max_frames], '%d/%d'));
    set(handles.slider1,'Value',frame_number);


% --- Executes on button press in pushbutton6-- Open a Directory.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global directory_selected directory frame_number fnamePrefix timestamp_cam1 timestamp_cam2 timestamp_cam3 timestamp_cam4 offset DtCMapping kinect_id max_frames bData imhandle fid;
    directory = uigetdir;
    if directory == 0 
        return;
    end
    
    try
        fclose(fid);
    catch
    end
    set(handles.text4,'String','-');
    bData = [];
    directory_selected=1;
    fnamePrefix{1} = strcat(directory,'/cam1/','USB-VID_045E&PID_02AE-A00364A04906127A_');
    fnamePrefix{2} = strcat(directory,'/cam2/','USB-VID_045E&PID_02AE-A00367A14179123A_');
    fnamePrefix{3} = strcat(directory,'/cam3/','USB-VID_045E&PID_02AE-A00365A09389104A_');
    fnamePrefix{4} = strcat(directory,'/cam4/','USB-VID_045E&PID_02AE-A00366809492052A_');
%     set(handles.text2,'String',num2str(frame_number));
    DtCMapping = cell(1,4);
    try
        timestamp_cam1 = load(fullfile(directory,'cam1_timestamp.mat'));
        load([fnamePrefix{1}, 'DepthFrameToColorFrameMapping.mat']);
        DtCMapping{1} = DepthFrameToColorFrameMapping;
    catch
    end
    try
        timestamp_cam2 = load(fullfile(directory,'cam2_timestamp.mat'));
        load([fnamePrefix{2}, 'DepthFrameToColorFrameMapping.mat']);
        DtCMapping{2} = DepthFrameToColorFrameMapping;
    catch
    end
    try
        timestamp_cam3 = load(fullfile(directory,'cam3_timestamp.mat'));
        load([fnamePrefix{3}, 'DepthFrameToColorFrameMapping.mat']);
        DtCMapping{3} = DepthFrameToColorFrameMapping;
    catch
    end
    try
        timestamp_cam4 = load(fullfile(directory,'cam4_timestamp.mat'));
        load([fnamePrefix{4}, 'DepthFrameToColorFrameMapping.mat']);
        DtCMapping{4} = DepthFrameToColorFrameMapping;
    catch
        
    end
    
%     offset_2_1 = 0;
%     offset_2_3 = -48582;
%     offset_2_4 = -34200; % This will change once we get cam4 and cam1. Right now acc to cam3.

    % value for fn026
%     offset(1) = -275764;
%         offset(2) = 0;
%         offset(3) = -78506;
%         offset(4) = 63299;
    try
        offset = textread(fullfile(directory,'offset.txt'),'%d');
    catch
        disp('no offset information');
    end
    
    % Set initial cam
    kinect_id = 2;
    if exist(fullfile(directory,'maincam.txt'),'file')
        kinect_id = textread(fullfile(directory,'maincam.txt'),'%d');
    end
    set(handles.radiobutton5,'Value',0);
    set(handles.radiobutton6,'Value',0);
    set(handles.radiobutton7,'Value',0);
    set(handles.radiobutton8,'Value',0);
    if kinect_id == 1
        set(handles.radiobutton5,'Value',1)
    elseif kinect_id == 2
        set(handles.radiobutton6,'Value',1)
    elseif kinect_id == 3
        set(handles.radiobutton7,'Value',1)
    elseif kinect_id == 4
        set(handles.radiobutton8,'Value',1)
    end
    
    % Draw image
    frame_number = 1;
    if(kinect_id == 1)
        matfile = strcat(fnamePrefix{1},int2str(frame_number),'.mat');
        imfile = fullfile(directory,'cam1/',[int2str(frame_number),'.png']);
    elseif(kinect_id == 2)
        matfile = strcat(fnamePrefix{2},int2str(frame_number),'.mat');
        imfile = fullfile(directory,'cam2/',[int2str(frame_number),'.png']);
    elseif(kinect_id == 3)
        matfile = strcat(fnamePrefix{3},int2str(frame_number),'.mat');
        imfile = fullfile(directory,'cam3/',[int2str(frame_number),'.png']);
    else 
        matfile = strcat(fnamePrefix{4},int2str(frame_number),'.mat');
        imfile = fullfile(directory,'cam4/',[int2str(frame_number),'.png']);
    end

    isPlaying = 0;
    I = imread(imfile);
    if isempty(imhandle)
        imhandle = imshow(I);
    else
        set(imhandle, 'CData', I);
    end
    drawnow update;
        
    % Frame number of current view
    max_frames = GetMaxFrames(kinect_id);
    set(handles.text2,'String',num2str([frame_number,max_frames], '%d/%d'));
    set(handles.slider1,'Value',frame_number);
    
    % Set slider properties
    set(handles.slider1, 'Min', 1);
    set(handles.slider1, 'Max', max_frames);
    set(handles.slider1, 'Value', frame_number);
    set(handles.slider1, 'SliderStep', [1/max_frames , 10/max_frames ]);

% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1

% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton2

% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton3

% --- Executes on button press in radiobutton4.
function radiobutton4_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton4


% --- Executes when selected object is changed in uipanel3.
function uipanel3_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in uipanel3 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
    global directory directory_selected kinect_id frame_number timestamp_cam1 timestamp_cam2 timestamp_cam3 timestamp_cam4 fnamePrefix imhandle isPlaying max_frames bData;
    str= (get(hObject,'Tag'));
    older_cam = kinect_id;
    kinect_id  = str(end) - '4';
    if(directory_selected ==0)
        return;
    end
    new_cam = kinect_id;
    offset = calculate_offset_value(older_cam,new_cam);
    if(older_cam ==1)
        timestamp = timestamp_cam1.time;
    elseif(older_cam ==2)
        timestamp = timestamp_cam2.time;
    elseif(older_cam ==3)
        timestamp = timestamp_cam3.time;
    elseif(older_cam ==4)
        timestamp = timestamp_cam4.time;
    end
    new_timestamp = timestamp(frame_number) + offset;
%     calculate_new_frame_number(new_timestamp);

    try
        if(kinect_id == 1)
            timestamp = timestamp_cam1.time;
        elseif(kinect_id == 2)
            timestamp = timestamp_cam2.time;
        elseif(kinect_id == 3)
            timestamp = timestamp_cam3.time;
        elseif(kinect_id == 4)
            timestamp = timestamp_cam4.time;
        end
        [temp frame_number] = min(abs(timestamp - new_timestamp));
        
        if(kinect_id == 1)
            matfile = strcat(fnamePrefix{1},int2str(frame_number),'.mat');
            imfile = fullfile(directory,'cam1/',[int2str(frame_number),'.png']);
        elseif(kinect_id == 2)
            matfile = strcat(fnamePrefix{2},int2str(frame_number),'.mat');
            imfile = fullfile(directory,'cam2/',[int2str(frame_number),'.png']);
        elseif(kinect_id == 3)
            matfile = strcat(fnamePrefix{3},int2str(frame_number),'.mat');
            imfile = fullfile(directory,'cam3/',[int2str(frame_number),'.png']);
        else 
            matfile = strcat(fnamePrefix{4},int2str(frame_number),'.mat');
            imfile = fullfile(directory,'cam4/',[int2str(frame_number),'.png']);
        end

        isPlaying = 0;
        I = imread(imfile);
        % Draw tracked bounding box
        if ~isempty(bData)
            idxB = find(bData(:,1)==kinect_id & bData(:,2)==frame_number);
            if ~isempty(idxB)
                bboxPolygon = [bData(idxB,4), bData(idxB,5), bData(idxB,4)+bData(idxB,6), bData(idxB,5), bData(idxB,4)+bData(idxB,6), bData(idxB,5)+bData(idxB,7), bData(idxB,4), bData(idxB,5)+bData(idxB,7)];
                I = insertShape(I, 'Polygon', bboxPolygon, 'Color', 'green', 'LineWidth', 4);
            end
        end
        set(imhandle, 'CData', I);
        drawnow update;
        max_frames = GetMaxFrames(kinect_id);
        set(handles.text2,'String',num2str([frame_number,max_frames], '%d/%d'));
        set(handles.slider1, 'Min', 1);
        set(handles.slider1, 'Max', max_frames);
        set(handles.slider1, 'Value',frame_number);
%         DisplayImage(handles);
    catch
        disp('Error when switching camera');
        kinect_id = older_cam;
        
        set(handles.radiobutton5,'Value',0);
        set(handles.radiobutton6,'Value',0);
        set(handles.radiobutton7,'Value',0);
        set(handles.radiobutton8,'Value',0);
        if older_cam == 1
            set(handles.radiobutton5,'Value',1)
        elseif older_cam == 2
            set(handles.radiobutton6,'Value',1)
        elseif older_cam == 3
            set(handles.radiobutton7,'Value',1)
        elseif older_cam == 4
            set(handles.radiobutton8,'Value',1)
        end
    end

    
function val = calculate_offset_value(older_cam,new_cam)
    global offset
    
    if(older_cam ==1 && new_cam ==1)
        val = 0;
    elseif(older_cam ==1 && new_cam ==2)
        val = -1 * offset(1);    
    elseif(older_cam ==1 && new_cam ==3)
        val = (-1 * offset(1))  + (offset(3));    
    elseif(older_cam ==1 && new_cam ==4)
        val = (-1 * offset(1))  + (offset(4));    

    elseif(older_cam ==2 && new_cam ==1)
        val = offset(1);
    elseif(older_cam ==2 && new_cam ==2)
        val = 0;    
    elseif(older_cam ==2 && new_cam ==3)
        val = (offset(3));
    elseif(older_cam ==2 && new_cam ==4)
        val = (offset(4));
        
    elseif(older_cam ==3 && new_cam ==1)
        val = (-1 * offset(3))  + (offset(1));
    elseif(older_cam ==3 && new_cam ==2)
        val = (-1 * offset(3)) ;
    elseif(older_cam ==3 && new_cam ==3)
        val = 0;    
    elseif(older_cam ==3 && new_cam ==4)
        val = (-1 * offset(3))  + (offset(4));
    
    elseif(older_cam ==4 && new_cam ==1)
        val = (-1 * offset(4))  + (offset(1));
    elseif(older_cam ==4 && new_cam ==2)
        val = (-1 * offset(4))  ;
    elseif(older_cam ==4 && new_cam ==3)
        val = (-1 * offset(4))  + (offset(3));
    elseif(older_cam ==4 && new_cam ==4)
        val = 0;    
        
    end
    


% --- Executes on button press in pushbutton8.
% step 1 frame
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag directory frame_number fid directory_selected imhandle tracking_started kinect_id;

if(directory_selected == 1)
    frame_number = frame_number + 1;
    set(handles.slider1,'Value',frame_number);
    DisplayImage(handles)
end

% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global isPlaying;
isPlaying = 0;


% --- Executes during object creation, after setting all properties.
function uipanel3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1


% --- Display image on InteractiveTracking
function DisplayImage(handles) 
    global fnamePrefix directory frame_number imhandle kinect_id model DtCMapping fid timestamp_cam1 timestamp_cam2 timestamp_cam3 timestamp_cam4 max_frames bData
    
    imfile = fullfile(directory,['cam',num2str(kinect_id),'/'],[int2str(frame_number),'.png']);

%             imshow(mat_data.ColorFrame.ColorData,'Parent',hf);
    set(handles.text2,'String',num2str([frame_number,max_frames], '%d/%d'));
    set(handles.slider1,'Value',frame_number);
    I = imread(imfile);
    
    if(get(handles.checkbox1, 'Value'))
        if isempty(model)
            msgbox('Please annotate the target object bounding box');
            return;
        end
%         model = GetBbox(I, fnamePrefix{kinect_id}, frame_number, model, DtCMapping{kinect_id}, 0, 1);
        model = GetBboxColor(I, fnamePrefix{kinect_id}, frame_number, model, DtCMapping{kinect_id});
        idxB = [];
        if ~isempty(bData)
            idxB = find(bData(:,1)==kinect_id & bData(:,2)==frame_number);
        end
        if ~isempty(idxB)
            bData(idxB,4:7) = model.bbox;
        else
            eval(['time = timestamp_cam', num2str(kinect_id), '.time(', num2str(frame_number), ');']);
            offset = calculate_offset_value(kinect_id, 2);
            time_wrt_cam2 = time + offset;
            bData(end+1,:) = [kinect_id frame_number time_wrt_cam2 model.bbox 0];
        end
    end
    
%     if(get(handles.checkbox2, 'Value'))
%         eval(['time = timestamp_cam',num2str(kinect_id),'.time(',num2str(frame_number),');']);
%         offset = calculate_offset_value(kinect_id, 2);
%         time_wrt_cam2 = time + offset;
%         fprintf(fid, '%1.0f %5.0f %1.0f %3.2f %3.2f %3.2f %3.2f\n', kinect_id,  frame_number, time_wrt_cam2 , model.bbox(1), model.bbox(2), model.bbox(3), model.bbox(4));
%     end
    
    % Draw tracked bounding box
    if ~isempty(bData)
        idxB = find(bData(:,1)==kinect_id & bData(:,2)==frame_number);
        if ~isempty(idxB)
            bboxPolygon = [bData(idxB,4), bData(idxB,5), bData(idxB,4)+bData(idxB,6), bData(idxB,5), bData(idxB,4)+bData(idxB,6), bData(idxB,5)+bData(idxB,7), bData(idxB,4), bData(idxB,5)+bData(idxB,7)];
            I = insertShape(I, 'Polygon', bboxPolygon, 'Color', 'green', 'LineWidth', 4);
        end
    end
    
    if isempty(imhandle)
        imhandle = imshow(I);
    else
        set(imhandle, 'CData', I);
    end
    set(handles.slider1,'Value',frame_number);
%     hold on;
    
    
%             if(tracking)
%                 plot(x_plot,y_plot,'r','LineWidth',2);
%             end

%     hold off;
    drawnow update;
    pause(0.001);
%     frame_number = frame_number + 1;


function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global fid filename directory bData

try
    fclose(fid);
catch
end
fname = (inputdlg('Filename:','Input filename'));
if isempty(fname)
    return;
end
filename = [fname{1}, '.txt'];

if exist(fullfile(directory, filename), 'file')
    msgbox(['Duplicate name detected, please use a different filename']);
    return;
end
fid = fopen(fullfile(directory, filename), 'w');
set(handles.text4,'String',filename);


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global fid bData;
try
    bData = [];
    fclose(fid);
catch
end
set(handles.text4,'String','-');


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global frame_number kinect_id directory imhandle bData isPlaying;
isPlaying = 0;
    
frame_number = double(int32(get(handles.slider1, 'Value')));
% DisplayImage(handles);
    
if(kinect_id == 1)
    imfile = fullfile(directory,'cam1/',[int2str(frame_number),'.png']);
elseif(kinect_id == 2)
    imfile = fullfile(directory,'cam2/',[int2str(frame_number),'.png']);
elseif(kinect_id == 3)
    imfile = fullfile(directory,'cam3/',[int2str(frame_number),'.png']);
else 
    imfile = fullfile(directory,'cam4/',[int2str(frame_number),'.png']);
end

I = imread(imfile);
% Draw tracked bounding box
if ~isempty(bData)
    idxB = find(bData(:,1)==kinect_id & bData(:,2)==frame_number);
    if ~isempty(idxB)
        bboxPolygon = [bData(idxB,4), bData(idxB,5), bData(idxB,4)+bData(idxB,6), bData(idxB,5), bData(idxB,4)+bData(idxB,6), bData(idxB,5)+bData(idxB,7), bData(idxB,4), bData(idxB,5)+bData(idxB,7)];
        I = insertShape(I, 'Polygon', bboxPolygon, 'Color', 'green', 'LineWidth', 4);
    end
end

if isempty(imhandle)
    imhandle = imshow(I);
else
    set(imhandle, 'CData', I);
end
drawnow update;

max_frames = GetMaxFrames(kinect_id);
set(handles.text2,'String',num2str([frame_number,max_frames], '%d/%d'));

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Get total number of frames for a particular camera
function frameno = GetMaxFrames(cam_id)
    global timestamp_cam1 timestamp_cam2 timestamp_cam3 timestamp_cam4;
    eval(['frameno = length(timestamp_cam', num2str(cam_id), '.time);']);


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SaveTrack();

% --- Save tracking result
function SaveTrack()
global fid bData filename;
fprintf(fid, '%1.0f %5.0f %1.0f %3.2f %3.2f %3.2f %3.2f %1.0f\n', bData');
msgbox(['Tracking result saved in ', filename]);
        
        
% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global bData

[filename, pathname] = uigetfile('*.txt', 'Select annotation file');
if isequal(filename,0) || isequal(pathname,0)
   return;
else
    bData = textread(fullfile(pathname, filename));
    bData = RemoveDuplicates(bData);
    bData(:,4) = max(1, bData(:,4));
    bData(:,5) = max(1, bData(:,5));
    if size(bData, 2) == 7
        bData = [bData zeros(size(bData,1), 1)];
    end
   msgbox([filename ' is loaded']);
end
