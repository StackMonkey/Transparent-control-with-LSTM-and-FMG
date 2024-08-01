function handles = sit_to_stand_control_open_fun(handles)

%% tasks information
set(handles.train_tasks_list,'string',{'Calibration','Other', 'Sit_to_Stand', 'Payload_0', 'Payload_5'});

%% Invisible Buttons 
set(handles.control_parameters,'visible','off')
set(handles.sit_to_stand_control,'visible','off')
set(handles.connect_exo_aider,'visible','off')
set(handles.motor_testing,'visible','off')
set(handles.load_lifting,'visible','off')


%% Visible Buttons
set(handles.main_menu,'visible','on')
set(handles.collect_training_data,'visible','on')
set(handles.real_time_testing,'visible','on')
set(handles.data_display,'visible','on')
set(handles.display_options_list,'visible','on')
set(handles.display_options_list,'string',{'FSR', 'sEMG', 'Results'});

%% Setting button values to 0
set(handles.collect_data,'Value',0);
set(handles.train_classifier,'Value',0);
set(handles.start_testing,'Value',0);
set(handles.update_assist_level,'visible','on')
set(handles.update_assist_level,'Value',0);


%% message box
set(handles.message_box,'String','Sit-to-Stand Control')
set(handles.train_classifier,'String','Train Classifier')

cla(handles.main_axes)
reset(handles.main_axes)

handles.metricdata.test.le.results = [];
handles.metricdata.test.ri.results = [];

pause(0.01)