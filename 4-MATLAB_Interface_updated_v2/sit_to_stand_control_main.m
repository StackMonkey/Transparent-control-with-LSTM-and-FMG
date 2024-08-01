function handles = sit_to_stand_control_main(handles)


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
Serial_Port = handles.metricdata.bluetooth.exoskeleton;

%% 
main_menu = 0;
set(handles.main_menu,'Value',0);
set(handles.start_testing,'Value',0);
while(main_menu == 0)
    
    %% collect training data
    collect_data = get(handles.collect_data,'Value');
    if(collect_data == 1)
        
        flushinput(handles.metricdata.bluetooth.exoskeleton);
        flushinput(handles.metricdata.bluetooth.left_sensor_band);
        flushinput(handles.metricdata.bluetooth.right_sensor_band);
        
        pause(0.1)
        handles.metricdata.display_time = 1;
        contents = cellstr(get(handles.train_tasks_list,'String'));
        task_performed = contents{get(handles.train_tasks_list,'Value')};
        switch task_performed
            case 'Calibration'
                handles = calibrate(handles);
            otherwise
                [handles, raw_data] = record_data(handles);
                data_biceps = [mean(raw_data.(handles.metricdata.controllers_name{2})(:,81:83),2) mean(raw_data.(handles.metricdata.controllers_name{3})(:,81:83),2)];
                data_triceps = [mean(raw_data.(handles.metricdata.controllers_name{2})(:,84:88),2) mean(raw_data.(handles.metricdata.controllers_name{3})(:,84:88),2)];
                cla(handles.main_axes)
                plot(handles.main_axes,[data_biceps data_triceps]);
                data = [raw_data.(handles.metricdata.controllers_name{1}) raw_data.(handles.metricdata.controllers_name{2}) raw_data.(handles.metricdata.controllers_name{3})];
                [x,~] = ginput(2);
                if (x(1)<x(2))
                    data = data(x(1):x(2),:);
                else
                    data = data(x(2):x(1),:);
                end
                switch task_performed
                    case 'Payload_0'
                        handles.metricdata.payload_raw_data.(task_performed) = data;
                    case 'Payload_5'
                        handles.metricdata.payload_raw_data.(task_performed) = data;
                    otherwise
                        handles.metricdata.raw_data.(task_performed) = [handles.metricdata.raw_data.(task_performed);data];
                        [~,n] = size(handles.metricdata.raw_data.(task_performed));
                        handles.metricdata.raw_data.(task_performed) = [handles.metricdata.raw_data.(task_performed);-1*ones(1,n)];
                end
                cla(handles.main_axes)
                reset(handles.main_axes)
                pause(0.5)
        end
        
        
    end
    %% train data
    classifier_training = get(handles.train_classifier,'Value');
    if(classifier_training == 1)
        set(handles.train_classifier,'Backgroundcolor','green');
        pause(0.001)
        handles = tr_extract_features_fsr(handles);
        handles = tr_extract_features_semg(handles);
        handles = tr_training_classifier(handles);
        set(handles.train_classifier,'Value',0);
        set(handles.train_classifier,'Backgroundcolor','default');
        pause(0.001)
    end
    
    %% update max assist level
    update_max_assist_level = get(handles.update_assist_level,'Value');
    if(update_max_assist_level == 1)
        set(handles.start_testing,'enable','off')
        set(handles.update_assist_level,'Backgroundcolor','green');
        max_assist_value = floor(str2double(get(handles.max_assist_level,'String')));
        pause(0.5)
        text = 'Gmav';
        for i = 1:length(text)
            fwrite(Serial_Port,text(i));
            pause(0.01)
        end
        fwrite(Serial_Port,',');
        pause(0.01)
        send_float(handles,max_assist_value);
        pause(0.01)
        fwrite(Serial_Port,'T');
        pause(0.5)
        set(handles.update_assist_level,'Value',0);
        set(handles.update_assist_level,'Backgroundcolor','default');
        set(handles.start_testing,'enable','on')
        pause(0.5)
    end
    
    
    %% test classifier
    real_time_testing = get(handles.start_testing,'Value');
    if(real_time_testing == 1)
        %%set(handles.update_assist_level,'enable','off')
        set(handles.start_testing,'Backgroundcolor','green');
        set(handles.start_testing,'String','Stop Testing');
        set(handles.update_assist_level,'String','Activate');
        set(handles.update_assist_level,'Value',0);
        pause(0.001)
        loop_flag = 1;
        
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
    end
    while(real_time_testing == 1)
        if(loop_flag == 1)
            loop_flag = 0;
            data_avaiable = 1;
            pause(0.1)
            while (data_avaiable == 1)
                available_bytes = handles.metricdata.bluetooth.left_sensor_band.BytesAvailable;
                if(available_bytes > 0)
                    fread(handles.metricdata.bluetooth.left_sensor_band,available_bytes);
                else
                    data_avaiable = 0;
                end
                pause(0.1)
            end
            
            data_avaiable = 1;
            pause(0.1)
            while (data_avaiable == 1)
                available_bytes = handles.metricdata.bluetooth.right_sensor_band.BytesAvailable;
                if(available_bytes > 0)
                    fread(handles.metricdata.bluetooth.right_sensor_band,available_bytes);
                else
                    data_avaiable = 0;
                end
                pause(0.1)
            end
            
            
            data_avaiable = 1;
            pause(0.1)
            while (data_avaiable == 1)
                available_bytes = handles.metricdata.bluetooth.exoskeleton.BytesAvailable;
                if(available_bytes > 0)
                    fread(handles.metricdata.bluetooth.exoskeleton,available_bytes);
                else
                    data_avaiable = 0;
                end
                pause(0.1)
            end
            
            flushinput(handles.metricdata.bluetooth.exoskeleton);
            flushinput(handles.metricdata.bluetooth.left_sensor_band);
            flushinput(handles.metricdata.bluetooth.right_sensor_band);
            
            pause(0.1)
            fwrite(handles.metricdata.bluetooth.exoskeleton,'S');
            fwrite(handles.metricdata.bluetooth.left_sensor_band,'S');
            fwrite(handles.metricdata.bluetooth.right_sensor_band,'S');
            test_loop = 0;
            fwrite(handles.metricdata.bluetooth.exoskeleton,'C');
            handles.metricdata.display_time = 1;
            handles.metricdata.assistance_provided = 0;
            pause(2)
            fwrite(handles.metricdata.bluetooth.exoskeleton,'A');
            pause(0.001)
        end
        %%%
        [handles, test_raw_data] = te_collect_data(handles);
        test_loop = test_loop + 1;
        if(test_loop >= 10)
            test_loop = 0;
            fwrite(handles.metricdata.bluetooth.exoskeleton,'S');
            fwrite(handles.metricdata.bluetooth.left_sensor_band,'S');
            fwrite(handles.metricdata.bluetooth.right_sensor_band,'S');
        end
        handles.metricdata.test.raw_data = ...
            [handles.metricdata.test.raw_data;...
            [test_raw_data.(handles.metricdata.controllers_name{1})...
            test_raw_data.(handles.metricdata.controllers_name{2})...
            test_raw_data.(handles.metricdata.controllers_name{3})]];
        
        %%%
        [handles, test_features_data] = te_extract_features(handles,test_raw_data);
        %%%
        handles = te_test_classifier(handles,test_features_data);
        real_time_testing = get(handles.start_testing,'Value');
        handles = display_data(handles,0);
        pause(0.01)
        if(real_time_testing == 0)
            set(handles.start_testing,'Backgroundcolor','default');
            fwrite(handles.metricdata.bluetooth.exoskeleton,'D');
            %             fwrite(handles.metricdata.bluetooth.exoskeleton,'T');
            %             fwrite(handles.metricdata.bluetooth.left_sensor_band,'T');
            %             fwrite(handles.metricdata.bluetooth.right_sensor_band,'T');
            
            data_avaiable = 1;
            pause(0.1)
            while (data_avaiable == 1)
                available_bytes = handles.metricdata.bluetooth.left_sensor_band.BytesAvailable;
                if(available_bytes > 0)
                    fread(handles.metricdata.bluetooth.left_sensor_band,available_bytes);
                else
                    data_avaiable = 0;
                end
                pause(0.1)
            end
            
            data_avaiable = 1;
            pause(0.1)
            while (data_avaiable == 1)
                available_bytes = handles.metricdata.bluetooth.right_sensor_band.BytesAvailable;
                if(available_bytes > 0)
                    fread(handles.metricdata.bluetooth.right_sensor_band,available_bytes);
                else
                    data_avaiable = 0;
                end
                pause(0.1)
            end
            
            
            data_avaiable = 1;
            pause(0.1)
            while (data_avaiable == 1)
                available_bytes = handles.metricdata.bluetooth.exoskeleton.BytesAvailable;
                if(available_bytes > 0)
                    fread(handles.metricdata.bluetooth.exoskeleton,available_bytes);
                else
                    data_avaiable = 0;
                end
                pause(0.1)
            end
            
            set(handles.start_testing,'String','Start Testing');
            set(handles.update_assist_level,'enable','on')
            set(handles.update_assist_level,'String','Update assist level');
            set(handles.update_assist_level,'Value',0);
        end
        
    end
    %%
    main_menu = get(handles.main_menu,'Value');
    pause(0.01)
    
end

data_exo = handles.metricdata;
save('data.mat','data_exo');
set(handles.main_menu,'Value',0);
