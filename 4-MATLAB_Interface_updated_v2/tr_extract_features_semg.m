function handles = tr_extract_features_semg(handles)

%% assigning initial values
% semg_columns_a = [29:32 41:44];
% semg_columns_b = [53:56 65:68];


semg_columns_a = 21:100;
semg_columns_b = 109:188;

contents = cellstr(get(handles.train_tasks_list,'String'));
for i = 1:length(contents)
    handles.metricdata.features_data.(contents{i}).semg = [];
end

number_of_tasks_performed = numel(fieldnames(handles.metricdata.raw_data));
tasks_performed = fieldnames(handles.metricdata.raw_data);
non_overlapping_window_size = 100;
overlapping_window_size = 0;
window_time = 0.100;

for i = 1:number_of_tasks_performed
    
    %% finding number of session in a recorded data for a particular task
    data = [];
    data = handles.metricdata.raw_data.(tasks_performed{i});
    
    m = isempty(data);
    if(m == 0)
        [~,n] = size(data);
        data = [-1*ones(1,n);data];
        recording_sessions = find(mean(data,2) == -1);
        number_of_recording_sessions = length(find(mean(data,2) == -1)) - 1;
        rms_data = [];
        slope_data = [];
        index = 1;
        for j = 1:number_of_recording_sessions
            
            %% extracting data from a recorded session
            data_a = [];
            data_b = [];
            session_data = data(recording_sessions(j)+1:recording_sessions(j+1)-1,:);
            
            data1 = session_data(:,semg_columns_a);
            [m,n] = size(data1);
            data1 = (reshape(data1',[1,m*n]));
            data1 = reshape(data1,[4,floor(m*n/4)])';
            
            data_a = data1;
            
            data1 = session_data(:,semg_columns_b);
            [m,n] = size(data1);
            data1 = (reshape(data1',[1,m*n]));
            data1 = reshape(data1,[4,floor(m*n/4)])';
            
            data_b = data1;

            session_data = [data_a data_b];
            
            for data_samples = overlapping_window_size:non_overlapping_window_size:(length(session_data(:,1))-non_overlapping_window_size*2)
                %% extracting window data
                window_data = session_data((data_samples-(overlapping_window_size-1)):(data_samples+non_overlapping_window_size),:);
                %% extracting features
                rms_data(index,:)      = rms(window_data);
                if(index == 1)
                    slope_data(index,:) = (rms(window_data(floor(end/2):end,:))-rms(window_data(1:floor(end/2),:)))/window_time;                
                else
                    slope_data(index,:)    = (rms_data(index,:) - rms_data(index-1,:))/window_time;
                end
                
                index = index + 1;
            end
            
        end
        handles.metricdata.features_data.(tasks_performed{i}).semg = [handles.metricdata.features_data.(tasks_performed{i}).semg;[rms_data slope_data]];
    end
end