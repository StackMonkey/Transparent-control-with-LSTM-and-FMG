clc
close all

a = handles.metricdata.raw_data.Other(:,109:188);
b = handles.metricdata.raw_data.Other(:,189:196);
clear data_emg
data_emg = [];

for i = 1:length(a(:,1))
    B = reshape(a(i,:),[4,20]);
    B = B';
    data_emg = [data_emg;B];
end


y = highpass(data_emg,20,1000);
col_n = 3;
subplot(3,1,1)
plot(y(:,col_n))
xlim([1 length(y(:,1))])
y_rms = [];
for i = 100:1:length(y(:,col_n))
    
    y_rms(i-99,1) = rms(y(i-99:i,col_n));
    
end
subplot(3,1,2)
plot((y_rms(:,:)))
xlim([1 length(y_rms(:,1))])
subplot(3,1,3)
plot(b(:,:))

xlim([1 length(b(:,1))])