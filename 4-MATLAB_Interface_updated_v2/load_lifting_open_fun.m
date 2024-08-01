function handles = load_lifting_open_fun(handles)

%% tasks information
set(handles.train_tasks_list,'string',{'Calibration','Payload_A','Payload_B'});
set(handles.test_tasks_list,'string',{'Payload_A','Payload_B'});

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


%% message box
set(handles.message_box,'String','Load Lifting')
set(handles.train_classifier,'String','Train Estimator')

cla(handles.main_axes)
reset(handles.main_axes)

pause(0.01)