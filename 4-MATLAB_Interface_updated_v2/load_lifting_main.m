function handles = sit_to_stand_control_main(handles)

main_menu = 0;

while(main_menu == 0)
    
    %% collect training data
    collect_data = get(handles.collect_data,'Value');
    if(collect_data == 1)
        
        flushinput(handles.metricdata.bluetooth.exoskeleton);
        flushinput(handles.metricdata.bluetooth.left_sensor_band);
        flushinput(handles.metricdata.bluetooth.right_sensor_band);
        
        fwrite(handles.metricdata.bluetooth.exoskeleton,'X');
        fwrite(handles.metricdata.bluetooth.left_sensor_band,'X');
        fwrite(handles.metricdata.bluetooth.right_sensor_band,'X');
        pause(0.1)
        handles.metricdata.display_time = 1;
        [handles, raw_data] = record_data(handles);
        contents = cellstr(get(handles.train_tasks_list,'String'));
        task_performed = contents{get(handles.train_tasks_list,'Value')};
        
        handles.metricdata.raw_data.(task_performed) = [handles.metricdata.raw_data.(task_performed);[raw_data.(handles.metricdata.controllers_name{1}) raw_data.(handles.metricdata.controllers_name{2}) raw_data.(handles.metricdata.controllers_name{3})]];
        [~,n] = size(handles.metricdata.raw_data.(task_performed));
        handles.metricdata.raw_data.(task_performed) = [handles.metricdata.raw_data.(task_performed);-1*ones(1,n)];
                
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
    %% test classifier
    flushinput(handles.metricdata.bluetooth.exoskeleton);
    flushinput(handles.metricdata.bluetooth.left_sensor_band);
    flushinput(handles.metricdata.bluetooth.right_sensor_band);
    real_time_testing = get(handles.start_testing,'Value');
    if(real_time_testing == 1)
        set(handles.start_testing,'Backgroundcolor','green');
        pause(0.001)
        loop_flag = 1;
    end
    while(real_time_testing == 1)
        if(loop_flag == 1)
            loop_flag = 0;
            
            flushinput(handles.metricdata.bluetooth.exoskeleton);
            flushinput(handles.metricdata.bluetooth.left_sensor_band);
            flushinput(handles.metricdata.bluetooth.right_sensor_band);
            
            fwrite(handles.metricdata.bluetooth.exoskeleton,'Y');
            fwrite(handles.metricdata.bluetooth.left_sensor_band,'Y');
            fwrite(handles.metricdata.bluetooth.right_sensor_band,'Y');
            pause(0.1)
            fwrite(handles.metricdata.bluetooth.exoskeleton,'S');
            fwrite(handles.metricdata.bluetooth.left_sensor_band,'S');
            fwrite(handles.metricdata.bluetooth.right_sensor_band,'S');
            handles.metricdata.display_time = 1;
        end
        %%%
        [handles, test_raw_data] = te_collect_data(handles);
        
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
        pause(0.001)
        if(real_time_testing == 0)
            set(handles.start_testing,'Backgroundcolor','default');
            fwrite(handles.metricdata.bluetooth.exoskeleton,'D');
        end
        
    end
    %%
    main_menu = get(handles.main_menu,'Value');
    pause(0.01)
    
end
set(handles.main_menu,'Value',0);
