function [handles, test_raw_data] = te_collect_data(handles)

window_size = 5;
i  = 1;

%% using serial.write
%%
data = fread(handles.metricdata.bluetooth.exoskeleton,100);
% fwrite(handles.metricdata.bluetooth.exoskeleton,'S');
%     for send_loop = 1:10
%         data1(send_loop,:) = data((send_loop - 1)*20+1:20*send_loop);
%     end
data1 = (reshape(data,20,5))';
data = data1;
data1 = [];
test_raw_data.(handles.metricdata.controllers_name{1}) = data;
%%
data = fread(handles.metricdata.bluetooth.left_sensor_band,440);
% fwrite(handles.metricdata.bluetooth.left_sensor_band,'S');
%     for send_loop = 1:10
%         data1(send_loop,:) = data((send_loop - 1)*49+1:48*send_loop);
%     end
data1 = (reshape(data,88,5))'*3.3/256;
data_fsr = data1(:,81:88);
data = data1;
data1 = [];
% test_raw_data.(handles.metricdata.controllers_name{2}) = data;
%%
data = fread(handles.metricdata.bluetooth.right_sensor_band,440);%,'short');
% fwrite(handles.metricdata.bluetooth.right_sensor_band,'S');
%     for send_loop = 1:10
%         data1(send_loop,:) = data((send_loop - 1)*49+1:48*send_loop);
%     end
data1 = (reshape(data,88,5))'*3.3/256;
data_emg = data1(:,1:80);
data = data1;
data1 = [];
test_raw_data.(handles.metricdata.controllers_name{3}) = data;

test_raw_data.(handles.metricdata.controllers_name{2}) = [data_emg data_fsr];




