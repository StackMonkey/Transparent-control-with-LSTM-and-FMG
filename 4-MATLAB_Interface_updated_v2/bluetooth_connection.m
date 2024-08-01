function handles = bluetooth_connection(handles)

fclose('all'); % close all open files
delete(instrfindall); % Reset Com Port
delete(timerfindall); % Delete Timers
j = 1;
COM_Port = getAvailableComPort() % Looking for available COM ports

% COM_Port = {'COM35' , 'COM26', 'COM5'};

[m,~]=size(COM_Port);

for i = 1:1
    
    device_name =  handles.metricdata.controllers_name{i};
    for ii = 1:m
        try
            %% here we look for the bluetooth device
            handles.metricdata.bluetooth.communication.established = 1;
            Serial_Port = serial(COM_Port{ii},'BaudRate',250000, 'DataBits',8,'Terminator',62,'InputBufferSize',1000);
            fopen(Serial_Port);
            pause(2)
            fwrite(Serial_Port,'C');
            Recieve_bit = fscanf(Serial_Port)
            TF = isempty(Recieve_bit);
            if (TF==0)
                if (Recieve_bit(1)=='M')
                    
                    handles.metricdata.bluetooth.exoskeleton = Serial_Port;
                    
                end
                if (Recieve_bit(1)=='L')
                    
                    handles.metricdata.bluetooth.left_sensor_band = Serial_Port;
                    
                end
                if (Recieve_bit(1)=='R')
                    
                    handles.metricdata.bluetooth.right_sensor_band = Serial_Port;
                    
                end
            end
            
        catch
            %% if the bluetooth device is not avaiable
            handles.metricdata.bluetooth.communication.established = 0;
            handles.metricdata.bluetooth.missing_device{j} = device_name;
            j = j + 1;
            
        end
    end
    
end

%% here we check if all the control units are connected or not

if(handles.metricdata.bluetooth.communication.established == 0)
    %  if any one is missing message is displayed about the missing control units
    if(length(handles.metricdata.bluetooth.missing_device) == 1)
        set(handles.message_box,'String',strcat(handles.metricdata.bluetooth.missing_device,{' '},'controller is not avaliable'))
    elseif (length(handles.metricdata.bluetooth.missing_device) == 2)
        set(handles.message_box,'String',strcat(handles.metricdata.bluetooth.missing_device{1},{' '},'and',{' '},...
            handles.metricdata.bluetooth.missing_device{2},{' '},...
            'controllers are not avaliable'))
    elseif (length(handles.metricdata.bluetooth.missing_device) == 3)
        set(handles.message_box,'String',strcat(handles.metricdata.bluetooth.missing_device{1},',',{' '},...
            handles.metricdata.bluetooth.missing_device{2},{' '},'and',{' '},...
            handles.metricdata.bluetooth.missing_device{3},{' '},...
            'controllers are not avaliable'))
    end
    set(handles.connect_exo_aider,'Value',0);
    set(handles.connect_exo_aider,'enable','off');
    set(handles.connect_exo_aider,'Backgroundcolor','red');
    set(handles.message_box,'Backgroundcolor','red');
    pause(5)
    set(handles.connect_exo_aider,'Backgroundcolor','default');
    set(handles.message_box,'Backgroundcolor','white');
    set(handles.connect_exo_aider,'enable','on');
    set(handles.message_box,'String','Main Menu')
    
else
    % if all are connected message is displayed
    set(handles.connect_exo_aider,'Backgroundcolor','green');
    set(handles.message_box,'Backgroundcolor','green');
    set(handles.message_box,'String','All Controllers are connected')
    pause(2)
    set(handles.message_box,'Backgroundcolor','white');
    set(handles.message_box,'String','Main Menu')
    
    %% Enable Buttons
    set(handles.control_parameters,'enable','on')
    set(handles.sit_to_stand_control,'enable','on')
    set(handles.motor_testing,'enable','on')
    set(handles.load_lifting,'enable','on')
    
end
