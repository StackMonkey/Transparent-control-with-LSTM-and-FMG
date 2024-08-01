function handles = motor_testing_main(handles)

main_menu = 0;

while(main_menu == 0)
    %% flexion training data
    resend_command = 0;
    flexion = get(handles.flexion,'Value');
    if(flexion == 1)
        set(handles.flexion,'Backgroundcolor','green');
        set(handles.extension,'enable','off');
        set(handles.main_menu,'enable','off');
        fwrite(handles.metricdata.bluetooth.exoskeleton,'A');
        fwrite(handles.metricdata.bluetooth.exoskeleton,'C');
        fwrite(handles.metricdata.bluetooth.exoskeleton,'S');
        data = [];
        handles.metricdata.display_time = 1;
        while(flexion == 1)
            data1 = fread(handles.metricdata.bluetooth.exoskeleton,100);
            data1 = (reshape(data1,20,5))';
            data = [data;data1];
            handles = display_data(handles,data);
            flexion = get(handles.flexion,'Value');
            resend_command = resend_command + 1;
            if(resend_command == 10)
                resend_command = 0;
                fwrite(handles.metricdata.bluetooth.exoskeleton,'S');
            end
            pause(0.005)
        end
        %fwrite(handles.metricdata.bluetooth.exoskeleton,'D');
        set(handles.flexion,'Backgroundcolor','default');
        data_avaiable = 1;
        pause(0.3)
        while (data_avaiable == 1)
            available_bytes = handles.metricdata.bluetooth.exoskeleton.BytesAvailable;
            if(available_bytes > 0)
                fread(handles.metricdata.bluetooth.exoskeleton,available_bytes);
            else
                data_avaiable = 0;
            end
            pause(0.3)
        end
        set(handles.extension,'enable','on');
        set(handles.main_menu,'enable','on');
        pause(0.3)
    end
    %% extension training data
    extension = get(handles.extension,'Value');
    if(extension == 1)
        set(handles.flexion,'enable','off');
        set(handles.main_menu,'enable','off');
        pause(0.3)
        set(handles.extension,'Backgroundcolor','green');
        fwrite(handles.metricdata.bluetooth.exoskeleton,'E');
        fwrite(handles.metricdata.bluetooth.exoskeleton,'S');
        data = [];
        handles.metricdata.display_time = 1;
        while(extension == 1)
            data1 = fread(handles.metricdata.bluetooth.exoskeleton,100);
            data1 = (reshape(data1,20,5))';
            data = [data;data1];
            handles = display_data(handles,data);
            extension = get(handles.extension,'Value');
            resend_command = resend_command + 1;
            if(resend_command == 10)
                resend_command = 0;
                fwrite(handles.metricdata.bluetooth.exoskeleton,'S');
            end
            pause(0.005)
        end
        %fwrite(handles.metricdata.bluetooth.exoskeleton,'D');
        set(handles.extension,'Backgroundcolor','default');
        data_avaiable = 1;
        pause(0.3)
        while (data_avaiable == 1)
            available_bytes = handles.metricdata.bluetooth.exoskeleton.BytesAvailable;
            if(available_bytes > 0)
                fread(handles.metricdata.bluetooth.exoskeleton,available_bytes);
            else
                data_avaiable = 0;
            end
            pause(0.3)
        end
        set(handles.flexion,'enable','on');
        set(handles.main_menu,'enable','on');
        pause(0.3)
    end
    %%
    main_menu = get(handles.main_menu,'Value');
    pause(0.01)
    
end
set(handles.main_menu,'Value',0);
