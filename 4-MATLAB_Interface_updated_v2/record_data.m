function [handles,raw_data] = record_data(handles)

set(handles.collect_data,'Backgroundcolor','green');
pause(0.001)
for i = 1:1
flushinput(handles.metricdata.bluetooth.exoskeleton);
flushinput(handles.metricdata.bluetooth.left_sensor_band);
flushinput(handles.metricdata.bluetooth.right_sensor_band);
end


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

for i = 1:3
    raw_data.(handles.metricdata.controllers_name{i}) = [];
end

stop_collecting = 1;
i = 0;
display_time = 0;
tic


resend_command = 0;


fsr_index = [];
for i = 1:10
fsr_index = [fsr_index i*48-7:i*48];
end


emg_index = [];
for i = 1:10
emg_index = [emg_index (i-1)*48+1:i*48-8];   
end

pause(0.1)
fwrite(handles.metricdata.bluetooth.exoskeleton,'S');
fwrite(handles.metricdata.bluetooth.left_sensor_band,'S');
fwrite(handles.metricdata.bluetooth.right_sensor_band,'S');

while(stop_collecting == 1)
    i = i + 1;
    
    %% using serial.write
    %%
%     tic
    data = fread(handles.metricdata.bluetooth.exoskeleton,100);
%     toc
    pause(0.0001)
    
%     fwrite(handles.metricdata.bluetooth.exoskeleton,'S');
%     for send_loop = 1:10
%         data1(send_loop,:) = data((send_loop - 1)*20+1:20*send_loop);
%     end
    data1 = (reshape(data,20,5))';
    data = data1;
    data1 = [];
    raw_data.(handles.metricdata.controllers_name{1}) = ...
        [raw_data.(handles.metricdata.controllers_name{1});data];
    %%
%     tic
    data = fread(handles.metricdata.bluetooth.left_sensor_band,440);
%     toc
    pause(0.0001)
%     fwrite(handles.metricdata.bluetooth.left_sensor_band,'S');
%     for send_loop = 1:10
%         data1(send_loop,:) = data((send_loop - 1)*49+1:48*send_loop);
%     end
    data1 = (reshape(data,88,5))'*3.3/256;
    data_fsr = data1(:,81:88);
    data = data1;
    data1 = [];
%     raw_data.(handles.metricdata.controllers_name{2}) = ...
%         [raw_data.(handles.metricdata.controllers_name{2});data];
    %%
    %     tic
    data = fread(handles.metricdata.bluetooth.right_sensor_band,440);%,'short');
    %     toc
    pause(0.0001)
    %     fwrite(handles.metricdata.bluetooth.right_sensor_band,'S');
    
    %     tic
    resend_command = resend_command + 1;
    if(resend_command == 10)
        resend_command = 0;
        fwrite(handles.metricdata.bluetooth.exoskeleton,'S');
        fwrite(handles.metricdata.bluetooth.left_sensor_band,'S');
        fwrite(handles.metricdata.bluetooth.right_sensor_band,'S');
    end
    %     toc
    %     for send_loop = 1:10
    %         data1(send_loop,:) = data((send_loop - 1)*49+1:48*send_loop);
    %     end
    data1 = (reshape(data,88,5))'*3.3/256;
    data_emg = data1(:,1:80);
    data = data1;
    data1 = [];
    raw_data.(handles.metricdata.controllers_name{3}) = ...
        [raw_data.(handles.metricdata.controllers_name{3});data];
    
    raw_data.(handles.metricdata.controllers_name{2}) = ...
        [raw_data.(handles.metricdata.controllers_name{2});[data_emg data_fsr]];
    
    %% used for real_time_display purpose
    handles = display_data(handles,raw_data);
    %%
    stop_collecting = get(handles.collect_data,'Value');
    pause(0.001)
    handles.metricdata.time(i) = 0;%toc;
end
toc
set(handles.collect_data,'Backgroundcolor','default');
pause(0.001)

%% highpass filter for emg data
old_y = [0 0 0 0];
old_x = [0 0 0 0];
ts = 0.001;
wc = 2*3.14*20;
data_rs = raw_data.(handles.metricdata.controllers_name{2})(:,1:80);
data_rs1 = data_rs;
for i = 1:length(data_rs(:,1))
    for j = 1:4:77
        data_rs(i,j:j+3)  = (data_rs1(i,j:j+3) - old_x + old_y)/...
            (1 + ts*wc);
        old_x = data_rs1(i,j:j+3);
        old_y = data_rs(i,j:j+3);
    end
end
raw_data.(handles.metricdata.controllers_name{2})(:,1:80) = data_rs;
old_y = [0 0 0 0];
old_x = [0 0 0 0];
data_ls = raw_data.(handles.metricdata.controllers_name{3})(:,1:80);
data_ls1 = data_ls;
for i = 1:length(data_ls(:,1))
    for j = 1:4:77
        data_ls(i,j:j+3)  = (data_ls1(i,j:j+3) - old_x + old_y)/...
            (1 + ts*wc);
        old_x = data_ls1(i,j:j+3);
        old_y = data_ls(i,j:j+3);
    end
end
raw_data.(handles.metricdata.controllers_name{3})(:,1:80) = data_ls;
% fwrite(handles.metricdata.bluetooth.exoskeleton,'T');
% fwrite(handles.metricdata.bluetooth.left_sensor_band,'T');
% fwrite(handles.metricdata.bluetooth.right_sensor_band,'T');
