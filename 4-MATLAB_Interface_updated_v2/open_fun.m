function handles = open_fun(handles)

%% device information
handles.metricdata.controllers_name= {'exoskeleton','left_sensor_band','right_sensor_band'};
handles.metricdata.controllers_channel = [1 1 1];

%% tasks information
set(handles.train_tasks_list,'string',{'Other', 'Sit_to_Stand', 'Payload_0', 'Payload_5'});

%% Visible Buttons 
set(handles.control_parameters,'visible','on')
set(handles.sit_to_stand_control,'visible','on')
set(handles.motor_testing,'visible','on')

%% invisible Buttons
set(handles.main_menu,'visible','off')
set(handles.collect_training_data,'visible','off')
set(handles.real_time_testing,'visible','off')
set(handles.data_display,'visible','off')
set(handles.flexion,'visible','off')
set(handles.extension,'visible','off')
set(handles.admittance_filter,'visible','off')
set(handles.update_assist_level,'visible','off')
set(handles.update_assist_level,'Value',0);


%% Disable Buttons 
set(handles.control_parameters,'enable','off')
set(handles.sit_to_stand_control,'enable','off')
set(handles.motor_testing,'enable','off')
set(handles.load_lifting,'enable','off')

%% assigning initial values
contents = cellstr(get(handles.train_tasks_list,'String'));
for i = 1:length(contents)
    handles.metricdata.raw_data.(contents{i})   = [];
end

handles.metricdata.test.raw_data = [];
handles.metricdata.test.features_data = [];
handles.metricdata.test.results = [];
handles.metricdata.test.reference = [];


%% lowpass filter for assistance level

handles.metricdata.LPfilter.old_yr = 0;
handles.metricdata.LPfilter.old_yl = 0;
handles.metricdata.LPfilter.ts = 0.10;
handles.metricdata.LPfilter.wc = 2*3.14*1;

%% highpass filter for emg data

handles.metricdata.HPfilter.old_y = [0 0 0 0 0 0 0 0];
handles.metricdata.HPfilter.old_x = [0 0 0 0 0 0 0 0];
handles.metricdata.HPfilter.ts = 0.001;
handles.metricdata.HPfilter.wc = 2*3.14*20;


handles.metricdata.test.counter_send_again = 0;
handles.metricdata.test.send_again = 1;
handles.metricdata.assistance_provided = 1;
