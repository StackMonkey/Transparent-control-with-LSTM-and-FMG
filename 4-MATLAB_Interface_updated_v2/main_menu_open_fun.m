function handles = main_menu_open_fun(handles)

%% Invisible Buttons 
set(handles.control_parameters,'visible','on')
set(handles.sit_to_stand_control,'visible','on')
set(handles.connect_exo_aider,'visible','on')
set(handles.motor_testing,'visible','on')
set(handles.load_lifting,'visible','on')

%% Visible Buttons
set(handles.main_menu,'visible','off')
set(handles.collect_training_data,'visible','off')
set(handles.real_time_testing,'visible','off')
set(handles.data_display,'visible','off')
set(handles.flexion,'visible','off')
set(handles.extension,'visible','off')
set(handles.admittance_filter,'visible','off')
set(handles.update_assist_level,'visible','off')

%% Setting button values to 0
set(handles.control_parameters,'Value',0);
set(handles.sit_to_stand_control,'Value',0);
set(handles.motor_testing,'Value',0);
set(handles.load_lifting,'Value',0)
set(handles.update_assist_level,'Value',0);

%% message box
set(handles.message_box,'String','Main Menu')