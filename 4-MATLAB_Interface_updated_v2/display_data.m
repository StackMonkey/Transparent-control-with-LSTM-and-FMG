function handles = display_data(handles,raw_data)

samples_recording = 200;
%%
if(get(handles.collect_data,'Value') == 1)
    
    if(length(raw_data.(handles.metricdata.controllers_name{2})(:,1))<=samples_recording)
        fsr_data = [raw_data.(handles.metricdata.controllers_name{2})(:,81:88) raw_data.(handles.metricdata.controllers_name{3})(:,81:88)];
        emg_data = [raw_data.(handles.metricdata.controllers_name{2})(:,1:4) raw_data.(handles.metricdata.controllers_name{3})(:,1:4)];
    else
        fsr_data = [raw_data.(handles.metricdata.controllers_name{2})(end-samples_recording:end,81:88) raw_data.(handles.metricdata.controllers_name{3})(end-samples_recording:end,81:88)];
        emg_data = [raw_data.(handles.metricdata.controllers_name{2})(end-samples_recording:end,1:4) raw_data.(handles.metricdata.controllers_name{3})(end-samples_recording:end,1:4)];
    end
    handles.metricdata.display_time = handles.metricdata.display_time + 1;
    contents = cellstr(get(handles.display_options_list,'String'));
    task_performed = contents{get(handles.display_options_list,'Value')};
        switch task_performed
        case 'FSR'
            data = fsr_data;
            data = [mean(data(:,1:8),2) mean(data(:,9:16),2)];
%             data = [(data(:,1:8)) (data(:,9:16))];
        case 'sEMG'
            data = emg_data;
            
        otherwise
            data = fsr_data;
        end
    if(handles.metricdata.display_time == 5)
        
        plot(handles.main_axes,data)
        xlim(handles.main_axes,[0 samples_recording+20])
        handles.metricdata.display_time = 1;
        
    end
    
end

%%
if(get(handles.start_testing,'Value') == 1)
    
    if(length(handles.metricdata.test.features_data(:,1))<=30)
        fsr_data = handles.metricdata.test.features_data(:,1:16);
        emg_data = handles.metricdata.test.features_data(:,33:40);
        results = handles.metricdata.test.results(:,:);
    else
        fsr_data = handles.metricdata.test.features_data(end - 29:end,1:16);
        emg_data = handles.metricdata.test.features_data(end - 29:end,33:40);
        results = handles.metricdata.test.results(end - 29:end,:);
    end
    handles.metricdata.display_time = handles.metricdata.display_time + 1;
    contents = cellstr(get(handles.display_options_list,'String'));
    task_performed = contents{get(handles.display_options_list,'Value')};
    
    switch task_performed
        case 'FSR'
            data = fsr_data;
            data = [mean(data(:,1:3),2) mean(data(:,9:11),2) mean(data(:,4:8),2) mean(data(:,12:16),2)];
            
        case 'sEMG'
            data = emg_data;
            
        case 'Results'
            data = results;
    end
    if(handles.metricdata.display_time == 2)
        
        plot(handles.main_axes,data)
        xlim(handles.main_axes,[0 40])
        handles.metricdata.display_time = 1;
        
    end
    
end

%%
if(get(handles.flexion,'Value') == 1 || get(handles.extension,'Value') == 1)
    
    if(length(raw_data(:,1))<=250)
        data = raw_data;
    else
        data = raw_data(end-249:end,:);
    end
    contents = cellstr(get(handles.display_options_list,'String'));
    task_performed = contents{get(handles.display_options_list,'Value')};
    
    switch task_performed
        case 'Position'
            columns_included = [1 4 11 14];
            
        case 'Velocity'
            columns_included = [1 4 11 14]+1;
            
        case 'Current'
            columns_included = [1 4 11 14]+2;
            
        case 'x-axis Force'
            columns_included = [7 17];
            
        case 'y-axis Force'
            columns_included = [7 17]+1;
            
        case 'z-axis Force'
            columns_included = [7 17]+1;
            
        otherwise
            columns_included = [1 4 11 14];
            
    end
    
    handles.metricdata.display_time = handles.metricdata.display_time + 1;
    if(handles.metricdata.display_time == 3)
        
        plot(handles.main_axes,data(:,columns_included))
        xlim(handles.main_axes,[0 270])
        handles.metricdata.display_time = 1;
        
    end
    
end