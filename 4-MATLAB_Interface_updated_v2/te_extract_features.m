function [handles, test_features_data] = te_extract_features(handles,test_raw_data)

collective_test_raw_data = [test_raw_data.(handles.metricdata.controllers_name{1})...
    test_raw_data.(handles.metricdata.controllers_name{2})...
    test_raw_data.(handles.metricdata.controllers_name{3})];

%% assigning initial values
% fsr_columns_a = [21:28 33:40];
% fsr_columns_b = [45:52 57:64];
% semg_columns_a = [29:32 41:44];
% semg_columns_b = [53:56 65:68];

fsr_columns_a = 101:108;
fsr_columns_b = 189:196;
semg_columns_a = 21:100;
semg_columns_b = 109:188;

window_time = 0.100;

%% extract FSR features_data

fsr_data = collective_test_raw_data(:,fsr_columns_a);

% [m,n] = size(fsr_data);
% data_1_org = ones(m*2,n/2);
% odd_rows  = (1:2:m*2)';
% even_rows = (2:2:m*2)';
% data_1_org(odd_rows,:) = fsr_data(:,1:8);
% data_1_org(even_rows,:) = fsr_data(:,9:16);

data_1_org = fsr_data;
data_a = data_1_org;

fsr_data = collective_test_raw_data(:,fsr_columns_b);
% [m,n] = size(fsr_data);
% data_1_org = ones(m*2,n/2);
% odd_rows  = (1:2:m*2)';
% even_rows = (2:2:m*2)';
% data_1_org(odd_rows,:) = fsr_data(:,1:8);
% data_1_org(even_rows,:) = fsr_data(:,9:16);

data_1_org = fsr_data;
data_b = data_1_org;

fsr_data = [data_a data_b];

rms_data    = rms(fsr_data);
slope_data  = mean(diff(fsr_data))/window_time;
acc_data_fsr = mean(diff(diff(fsr_data))/window_time*2);
fsr_features_data = [rms_data slope_data];

%% extract semg features_data

semg_data = collective_test_raw_data(:,semg_columns_a);
[m,n] = size(semg_data);
semg_data = (reshape(semg_data',[1,m*n]));
semg_data = reshape(semg_data,[4,floor(m*n/4)])';
data_a = semg_data;
% [m,n] = size(semg_data);
% data_1_org = ones(m*2,n/2);
% odd_rows  = (1:2:m*2)';
% even_rows = (2:2:m*2)';
% data_1_org(odd_rows,:) = semg_data(:,1:4);
% data_1_org(even_rows,:) = semg_data(:,5:8);
% data_a = data_1_org;

semg_data = collective_test_raw_data(:,semg_columns_b);
[m,n] = size(semg_data);
semg_data = (reshape(semg_data',[1,m*n]));
semg_data = reshape(semg_data,[4,floor(m*n/4)])';
data_b = semg_data;
% [m,n] = size(semg_data);
% data_1_org = ones(m*2,n/2);
% odd_rows  = (1:2:m*2)';
% even_rows = (2:2:m*2)';
% data_1_org(odd_rows,:) = semg_data(:,1:4);
% data_1_org(even_rows,:) = semg_data(:,5:8);
% data_b = data_1_org;

semg_data1 = [data_a data_b];
semg_data = [data_a data_b];
for i = 1:length(semg_data1(:,1))
   
    semg_data(i,:)  = (semg_data1(i,:) - handles.metricdata.HPfilter.old_x + handles.metricdata.HPfilter.old_y)/...
        (1 + handles.metricdata.HPfilter.ts*handles.metricdata.HPfilter.wc);
    handles.metricdata.HPfilter.old_x = semg_data1(i,:);
    handles.metricdata.HPfilter.old_y = semg_data(i,:);
    
end

rms_data = rms(semg_data);
tf  = isempty(handles.metricdata.test.features_data);
if(tf == 1)
    slope_data  = mean(diff(abs(semg_data)))/window_time;
else
    old_rms_emg = handles.metricdata.test.features_data(end,33:40);
    slope_data = (rms_data - old_rms_emg)/window_time;
end

acc_data_semg = mean(diff(diff(semg_data))/window_time*2);

semg_features_data = [rms_data slope_data];

test_features_data = [fsr_features_data semg_features_data acc_data_fsr acc_data_semg];

handles.metricdata.test.features_data = [handles.metricdata.test.features_data; test_features_data];