function handles = calibrate(handles)


set(handles.collect_data,'Backgroundcolor','yellow');
pause(5)
set(handles.collect_data,'Backgroundcolor','green');
pause(1)
fwrite(handles.metricdata.bluetooth.left_sensor_band,'C');
pause(1)
fwrite(handles.metricdata.bluetooth.right_sensor_band,'C');
pause(5)
set(handles.collect_data,'Backgroundcolor','default');
pause(0.001)
set(handles.collect_data,'Value',0);