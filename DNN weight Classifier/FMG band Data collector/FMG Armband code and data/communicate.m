function Data = communicate(serial_port,command)

Data = [];

switch command
    
    case 'start'
        
        %% initiating data recording
        fwrite(serial_port,'2');
        
    case 'get_sample'
        %% reading FSR
        datafsr = (fread(serial_port,160))';
        %% reading IMU
        data_imu = (fread(serial_port,13,'float'))'; %%% float is represented by 4 bytes so in total 12*4=48bytes we read here
        
        data_fsr = [];
        %% rearranging the fsr array into 20*8 matrix
        for i = 1:8:160
            data_fsr = [data_fsr;datafsr(i:i+7)];
        end
        %% relicating IMU data array into 20*12 and catinating it with FSR data
        Data = [data_fsr*3.3/255 data_imu.*ones(20,13)];
        
    case 'stop'
        
        %% stopping data recording session
        fwrite(serial_port,'3');
        %% Clearing any data still present in the inputbuffer
        pause(0.5);
        available_bytes = 1;
        while(available_bytes == 1)
            available_bytes = serial_port.BytesAvailable;
            if(isempty(available_bytes) == 1 || available_bytes == 0)
                available_bytes = 0;
            else
                Data = fread(serial_port,serial_port.BytesAvailable);
                available_bytes = 1;
            end
            pause(0.5);
        end
        
    case 'calibrate'
        
        pause(0.001)
        fwrite(serial_port,'5');
        pause(4)
        
    case 'reset_calibration'
        
        pause(0.001)
        fwrite(serial_port,'4');
        pause(0.001);
        
    case 'shut_down'
        
        pause(0.001)
        fwrite(serial_port,'6');
        
end


