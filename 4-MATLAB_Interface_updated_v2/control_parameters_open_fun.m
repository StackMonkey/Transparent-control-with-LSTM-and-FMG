function handles = control_parameters_open_fun(handles)


cla(handles.main_axes)
reset(handles.main_axes)

%% tasks information
set(handles.train_tasks_list,'string',{'Calibration','Rest','Flexion', 'Extension', 'Sit_to_Stand'});
%set(handles.test_tasks_list,'string',{'Rest','Flexion', 'Extension', 'Sit_to_Stand'});

%% Invisible Buttons 
set(handles.control_parameters,'visible','off')
set(handles.sit_to_stand_control,'visible','off')
set(handles.connect_exo_aider,'visible','off')
set(handles.motor_testing,'visible','off')
set(handles.load_lifting,'visible','off')


%% Visible Buttons
set(handles.main_menu,'visible','on')
set(handles.collect_training_data,'visible','off')
set(handles.real_time_testing,'visible','off')
set(handles.data_display,'visible','off')
set(handles.display_options_list,'visible','off')
set(handles.admittance_filter,'visible','on')

%% Setting button values to 0
set(handles.update_parameters,'Value',0);


%% message box
set(handles.message_box,'String','Control Parameters')


pause(0.01)