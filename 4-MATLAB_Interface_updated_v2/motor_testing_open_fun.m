function handles = motor_testing_open_fun(handles)

%% Invisible Buttons 
set(handles.control_parameters,'visible','off')
set(handles.sit_to_stand_control,'visible','off')
set(handles.connect_exo_aider,'visible','off')
set(handles.collect_training_data,'visible','off')
set(handles.real_time_testing,'visible','off')
set(handles.motor_testing,'visible','off')
set(handles.load_lifting,'visible','off')


%% Visible Buttons
set(handles.main_menu,'visible','on')
set(handles.flexion,'visible','on')
set(handles.extension,'visible','on')
set(handles.data_display,'visible','on')
set(handles.display_options_list,'visible','on')

%% Setting button values to 0
set(handles.flexion,'Value',0);
set(handles.extension,'Value',0);

%% message box
set(handles.message_box,'String','Motor Testing')

set(handles.display_options_list,'string',{'Position', 'Velocity', 'Current','x-axis Force','y-axis Force','z-axis Force'});

cla(handles.main_axes)
reset(handles.main_axes)

pause(0.01)