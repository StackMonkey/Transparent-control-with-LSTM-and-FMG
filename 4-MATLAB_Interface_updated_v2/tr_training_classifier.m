function handles = tr_training_classifier(handles)

number_of_tasks_performed = numel(fieldnames(handles.metricdata.features_data));
tasks_performed = fieldnames(handles.metricdata.features_data);
training_data   = [];
training_labels = [];
ii = 1;
for i = 1:number_of_tasks_performed
    
    data_fsr = handles.metricdata.features_data.(tasks_performed{i}).fsr;
    data_semg = handles.metricdata.features_data.(tasks_performed{i}).semg;
    data = [data_fsr data_semg];
    [m, ~] = size(data);
    if(m > 1)
       
       training_data = [training_data;data];
       training_labels = [training_labels;ii*ones(length(data(:,1)),1)];
       ii = ii + 1; 
    end
    
end

handles.metricdata.train.data = training_data;
handles.metricdata.train.labels = training_labels;

training_labels_nn = [];
for i = 1:max(training_labels)
    
    training_labels_nn = [training_labels_nn training_labels==i];
    
end


%%combine
% %%for slopes only
% index_slopes = [17:32 41:48];
% %%for rms only
% index_rms = [1:16 33:40];
% %% for all
% index_all = [1:48];

%% left
%%for slopes only
index_slopes_le = [17:24];%% 41:44];
%%for rms only
index_rms_le = [1:8];%% 33:36];
%% for all
index_all_le = [1:48];

%% right
%%for slopes only
index_slopes_ri = [25:32];%% 45:48];
%%for rms only
index_rms_ri = [9:16];%% 37:40];
%% for all
index_all_ri = [1:48];


tr_data = training_data;

index_slopes = index_rms_le;
training_data = tr_data(:,index_slopes);
size(training_data)

%% training nn

[~,n_x] = size(training_data);
n_t = max(training_labels);

%% NN
trainFcn = 'trainscg';  % Levenberg-Marquardt backpropagation.

hiddenLayerSize = floor((n_x + n_t)/2);
net = patternnet(hiddenLayerSize, trainFcn);
net.divideParam.trainRatio = 50/100;
net.divideParam.valRatio = 25/100;
net.divideParam.testRatio = 25/100;
[net,~] = train(net,training_data',training_labels_nn');
handles.metricdata.train.nn_classifier_le = net;




index_slopes = index_rms_ri;
training_data = tr_data(:,index_slopes);
size(training_data)

%% training nn

[~,n_x] = size(training_data);
n_t = max(training_labels);

%% NN
trainFcn = 'trainscg';  % Levenberg-Marquardt backpropagation.

hiddenLayerSize = floor((n_x + n_t)/2);
net = patternnet(hiddenLayerSize, trainFcn);
net.divideParam.trainRatio = 50/100;
net.divideParam.valRatio = 25/100;
net.divideParam.testRatio = 25/100;
[net,~] = train(net,training_data',training_labels_nn');
handles.metricdata.train.nn_classifier_ri = net;

%% support for lifting objects.

left_lower_bound = mean(mean(handles.metricdata.payload_raw_data.Payload_0(:,101:103)),2);
left_upper_bound = mean(mean(handles.metricdata.payload_raw_data.Payload_5(:,101:103)),2);

right_lower_bound = mean(mean(handles.metricdata.payload_raw_data.Payload_0(:,189:191)),2);
right_upper_bound = mean(mean(handles.metricdata.payload_raw_data.Payload_5(:,189:191)),2);

handles.metricdata.payload_assist.m_left = (1 - 0)/(left_upper_bound - left_lower_bound);
handles.metricdata.payload_assist.c_left = 1 - handles.metricdata.payload_assist.m_left*left_upper_bound;

handles.metricdata.payload_assist.m_right = (1 - 0)/(right_upper_bound - right_lower_bound);
handles.metricdata.payload_assist.c_right = 1 - handles.metricdata.payload_assist.m_right*right_upper_bound;

