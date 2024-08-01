function handles = send_float(handles,value)


Serial_Port = handles.metricdata.bluetooth.exoskeleton;
m = length(value);

for i = 1:m
    
   data_2_send = num2str(value(i));
   n = length(data_2_send);
   for j = 1:n
       fwrite(Serial_Port,data_2_send(j));
       pause(0.0005)
   end
   fwrite(Serial_Port,',');
   pause(0.0005)
   
end