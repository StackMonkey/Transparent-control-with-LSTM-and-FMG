function varargout = main(varargin)


gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @main_OpeningFcn, ...
    'gui_OutputFcn',  @main_OutputFcn, ...
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


% --- Executes just before main is made visible.
function main_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;
clc
evalin('base','clear all')
warning('off')

%%   Initializing Handles   %%
handles = open_fun(handles);
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = main_OutputFcn(hObject, eventdata, handles)

varargout{1} = handles.output;
guidata(hObject, handles);




% --- Executes on button press in connect_exo_aider.
function connect_exo_aider_Callback(hObject, eventdata, handles)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% this function is used to conect EXO-AIDER control unit %%%
%%% that includes exoskeleton control unit and sensor bands%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
value = get(hObject,'Value');
if (value == 1)
    handles = bluetooth_connection(handles);
else
    fclose('all'); % close all open files
    delete(instrfindall); % Reset Com Port
    delete(timerfindall); % Delete Timers
    set(handles.connect_exo_aider,'Backgroundcolor','default');
    %% Disable Buttons
    set(handles.control_parameters,'enable','off')
    set(handles.sit_to_stand_control,'enable','off')
    set(handles.motor_testing,'enable','off')
    set(handles.load_lifting,'enable','off')
end
assignin('base','handles',handles);guidata(hObject, handles);



% --- Executes on button press in control_parameters.
function control_parameters_Callback(hObject, eventdata, handles)
value = get(hObject,'Value');
if (value == 1)
    %%%
    handles = control_parameters_open_fun(handles);
    %%%
    handles = control_parameters_main(handles);
else
end
assignin('base','handles',handles);guidata(hObject, handles);



% --- Executes on button press in sit_to_stand_control.
function sit_to_stand_control_Callback(hObject, eventdata, handles)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% this function is used to collect training data for Sit %%%
%%% -To-Stand task and also to perform real time testing   %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
value = get(hObject,'Value');
if (value == 1)
    %%%
    handles = sit_to_stand_control_open_fun(handles);
    %%%
    handles = sit_to_stand_control_main(handles);
else
end

assignin('base','handles',handles);guidata(hObject, handles);



% --- Executes on button press in collect_data.
function collect_data_Callback(hObject, eventdata, handles)
value = get(hObject,'Value');
if (value == 1)
else
end
assignin('base','handles',handles);guidata(hObject, handles);


% --- Executes on selection change in train_tasks_list.
function train_tasks_list_Callback(hObject, eventdata, handles)
% hObject    handle to train_tasks_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns train_tasks_list contents as cell array
%        contents{get(hObject,'Value')} returns selected item from train_tasks_list
assignin('base','handles',handles);guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function train_tasks_list_CreateFcn(hObject, eventdata, handles)
% hObject    handle to train_tasks_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in main_menu.
function main_menu_Callback(hObject, eventdata, handles)
value = get(hObject,'Value');
if (value == 1)
    %%%
    handles = main_menu_open_fun(handles);
    %%%
else
end
assignin('base','handles',handles);guidata(hObject, handles);


% --- Executes on button press in start_testing.
function start_testing_Callback(hObject, eventdata, handles)
value = get(hObject,'Value');
if (value == 1)
else
end
assignin('base','handles',handles);guidata(hObject, handles);



% --- Executes on selection change in test_tasks_list.
function test_tasks_list_Callback(hObject, eventdata, handles)
% hObject    handle to test_tasks_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns test_tasks_list contents as cell array
%        contents{get(hObject,'Value')} returns selected item from test_tasks_list
assignin('base','handles',handles);guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function test_tasks_list_CreateFcn(hObject, eventdata, handles)
% hObject    handle to test_tasks_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in train_classifier.
function train_classifier_Callback(hObject, eventdata, handles)
value = get(hObject,'Value');
if (value == 1)
else
end
assignin('base','handles',handles);guidata(hObject, handles);


% --- Executes on selection change in display_options_list.
function display_options_list_Callback(hObject, eventdata, handles)
% hObject    handle to display_options_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns display_options_list contents as cell array
%        contents{get(hObject,'Value')} returns selected item from display_options_list
assignin('base','handles',handles);guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function display_options_list_CreateFcn(hObject, eventdata, handles)
% hObject    handle to display_options_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in motor_testing.
function motor_testing_Callback(hObject, eventdata, handles)
value = get(hObject,'Value');
if (value == 1)
    %%%
    handles = motor_testing_open_fun(handles);
    %%%
    handles = motor_testing_main(handles);
else
end
assignin('base','handles',handles);guidata(hObject, handles);

% --- Executes on button press in flexion.
function flexion_Callback(hObject, eventdata, handles)
value = get(hObject,'Value');
if (value == 1)
else
end
assignin('base','handles',handles);guidata(hObject, handles);


% --- Executes on button press in extension.
function extension_Callback(hObject, eventdata, handles)
value = get(hObject,'Value');
if (value == 1)
else
end
assignin('base','handles',handles);guidata(hObject, handles);


% --- Executes on button press in load_lifting.
function load_lifting_Callback(hObject, eventdata, handles)
value = get(hObject,'Value');
if (value == 1)
    %%%
    handles = load_lifting_open_fun(handles);
else
end
assignin('base','handles',handles);guidata(hObject, handles);



function inertia_Callback(hObject, eventdata, handles)
% hObject    handle to inertia (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of inertia as text
%        str2double(get(hObject,'String')) returns contents of inertia as a double
assignin('base','handles',handles);guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function inertia_CreateFcn(hObject, eventdata, handles)
% hObject    handle to inertia (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
assignin('base','handles',handles);guidata(hObject, handles);


function damping_Callback(hObject, eventdata, handles)
% hObject    handle to damping (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of damping as text
%        str2double(get(hObject,'String')) returns contents of damping as a double
assignin('base','handles',handles);guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function damping_CreateFcn(hObject, eventdata, handles)
% hObject    handle to damping (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
assignin('base','handles',handles);guidata(hObject, handles);

function stiffness_Callback(hObject, eventdata, handles)
% hObject    handle to stiffness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of stiffness as text
%        str2double(get(hObject,'String')) returns contents of stiffness as a double
assignin('base','handles',handles);guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function stiffness_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stiffness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
assignin('base','handles',handles);guidata(hObject, handles);

% --- Executes on selection change in motors.
function motors_Callback(hObject, eventdata, handles)
% hObject    handle to motors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns motors contents as cell array
%        contents{get(hObject,'Value')} returns selected item from motors
assignin('base','handles',handles);guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function motors_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
assignin('base','handles',handles);guidata(hObject, handles);

% --- Executes on button press in update_parameters.
function update_parameters_Callback(hObject, eventdata, handles)
value = get(hObject,'Value');
if (value == 1)
    %%%
else
end
assignin('base','handles',handles);guidata(hObject, handles);



function max_assist_level_Callback(hObject, eventdata, handles)
% hObject    handle to max_assist_level (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of max_assist_level as text
%        str2double(get(hObject,'String')) returns contents of max_assist_level as a double
assignin('base','handles',handles);guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function max_assist_level_CreateFcn(hObject, eventdata, handles)
% hObject    handle to max_assist_level (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
assignin('base','handles',handles);guidata(hObject, handles);

% --- Executes on button press in update_assist_level.
function update_assist_level_Callback(hObject, eventdata, handles)
value = get(hObject,'Value');
if (value == 1)
    %%%
else
end
assignin('base','handles',handles);guidata(hObject, handles);
