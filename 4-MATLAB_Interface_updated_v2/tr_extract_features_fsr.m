function handles = tr_extract_features_fsr(handles)

%% assigning initial values
% fsr_columns_a = [21:28 33:40];
% fsr_columns_b = [45:52 57:64];

fsr_columns_a = [101:108];
fsr_columns_b = [189:196];

contents = cellstr(get(handles.train_tasks_list,'String'));
for i = 1:length(contents)
    handles.metricdata.features_data.(contents{i}).fsr = [];
end

number_of_tasks_performed = numel(fieldnames(handles.metricdata.raw_data));
tasks_performed = fieldnames(handles.metricdata.raw_data);
non_overlapping_window_size = 5;
overlapping_window_size = 0;
window_time = 0.100;

for i = 1:number_of_tasks_performed
    %% finding number of session in a recorded data for a particular task
    data = [];
    data = handles.metricdata.raw_data.(tasks_performed{i});
    m = isempty(data);
    if(m == 0)
        data = handles.metricdata.raw_data.(tasks_performed{i});
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
            
            data1 = session_data(:,fsr_columns_a);
%             [m,n] = size(data1);
%             data_1_org = ones(m*2,n/2);
%             odd_rows  = (1:2:m*2)';
%             even_rows = (2:2:m*2)';
%             data_1_org(odd_rows,:) = data1(:,1:8);
%             data_1_org(even_rows,:) = data1(:,9:16);
%             data1 = data_1_org;
            data_a = data1;
            
            data1 = session_data(:,fsr_columns_b);
%             [m,n] = size(data1);
%             data_1_org = ones(m*2,n/2);
%             odd_rows  = (1:2:m*2)';
%             even_rows = (2:2:m*2)';
%             data_1_org(odd_rows,:) = data1(:,1:8);
%             data_1_org(even_rows,:) = data1(:,9:16);
%             data1 = data_1_org;
            data_b = data1;

            session_data = [data_a data_b];
            
            for data_samples = overlapping_window_size:non_overlapping_window_size:(length(session_data(:,1))-non_overlapping_window_size*2)
                %% extracting window data
                window_data = session_data((data_samples-(overlapping_window_size-1)):(data_samples+non_overlapping_window_size),:);
                %% extracting features
                rms_data(index,:)      = mean(window_data);
                slope_data(index,:)    = mean(diff(window_data))/window_time;
                index = index + 1;
            end
            
        end
        %% synthetic data
%         [ms,ns] = size(rms_data);
%         sd = rand(ms,ns)/10;
%         rms_data_sd = [rms_data+sd;rms_data-sd];
%         rms_data = [rms_data;rms_data_sd];
%         sd = rand(ms,ns)/100;
%         slope_data_sd = [slope_data+sd;slope_data-sd];
%         slope_data = [slope_data;slope_data_sd];
        handles.metricdata.features_data.(tasks_performed{i}).fsr = [handles.metricdata.features_data.(tasks_performed{i}).fsr;[rms_data slope_data]];
    end
end