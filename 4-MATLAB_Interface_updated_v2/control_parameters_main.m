function handles = control_parameters_main(handles)

main_menu = 0;
Serial_Port = handles.metricdata.bluetooth.exoskeleton;

fwrite(Serial_Port,'D');

while(main_menu == 0)
    a = 1;
    %% Update parameters
    update_parameters = get(handles.update_parameters,'Value');
    pause(0.1)
    if(update_parameters == 1)
        
        set(handles.main_menu,'enable','off')
        set(handles.update_parameters,'enable','off')
        set(handles.update_parameters,'Backgroundcolor','green')
        pause(1)
        inertia_value = str2double(get(handles.inertia,'String'));
        damping_value = str2double(get(handles.damping,'String'));
        stiffness_value = str2double(get(handles.stiffness,'String'));
        
        values = [inertia_value damping_value stiffness_value];
        
        contents = cellstr(get(handles.motors,'String'));
        selected_motor = contents{get(handles.motors,'Value')};
        switch selected_motor
            
            case 'Left'
                text = 'Glaf';
                for i = 1:length(text)
                    fwrite(Serial_Port,text(i));
                    pause(0.01)
                end
                fwrite(Serial_Port,',');
                pause(0.01)
                send_float(handles,values);
                pause(0.01)
                fwrite(Serial_Port,'T');
                
            case 'Right'
                text = 'Graf';
                for i = 1:length(text)
                    fwrite(Serial_Port,text(i));
                    pause(0.01)
                end
                fwrite(Serial_Port,',');
                pause(0.01)
                send_float(handles,values);
                pause(0.01)
                fwrite(Serial_Port,'T');
                
            case 'Both'
                
                text = 'Glaf';
                for i = 1:length(text)
                    fwrite(Serial_Port,text(i));
                    pause(0.01)
                end
                fwrite(Serial_Port,',');
                pause(0.01)
                send_float(handles,values);
                pause(0.01)
                fwrite(Serial_Port,'T');
                pause(0.1)
                
                text = 'Graf';
                for i = 1:length(text)
                    fwrite(Serial_Port,text(i));
                    pause(0.01)
                end
                fwrite(Serial_Port,',');
                pause(0.01)
                send_float(handles,values);
                pause(0.01)
                fwrite(Serial_Port,'T');
                
        end
        
        
        set(handles.update_parameters,'Value',0);
        set(handles.main_menu,'enable','on')
        set(handles.update_parameters,'enable','on')
        set(handles.update_parameters,'Backgroundcolor','default');
        pause(0.5)
        fwrite(Serial_Port,'C');
        
    end
    %%
    main_menu = get(handles.main_menu,'Value');
    pause(0.01)
    
end
set(handles.main_menu,'Value',0);
