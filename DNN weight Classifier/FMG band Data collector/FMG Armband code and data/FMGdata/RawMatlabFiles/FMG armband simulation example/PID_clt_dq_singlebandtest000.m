
clc
close all
clear all

fclose('all'); % close all open files
delete(instrfindall); % Reset Com Port
delete(timerfindall); % Delete Timers
pause('on')

%% specify gesture names and also place the images of each gesture with the same name in images folder in jpg format
gesture_names = {'open'};

%% pre-allocation of memory space otherwise data recording will slow down with time
recording_frequency = 950; %% Hz approx.
time_to_record = 200; %%seconds -- This can be changedb
time_each_loop = 20; %%msec
pre_allocated_space = floor(time_to_record*recording_frequency/time_each_loop); %% 1 loop collects the data for 20 msec

%% plotting options
plotting_time = 0.200; %% sec -- This can be changed
plotting_time = floor(plotting_time*recording_frequency/20);
plot_counter = 0;
show_data = 2; %% last 2 seconds -- This can be changed
show_data = show_data * recording_frequency;
data_2_plot = zeros(pre_allocated_space*20,22);%%此处列数或许应�?��?22

y_labels = {'FSR voltage (V)','Gravity (m/sec^2)','Angular velocity (rad/sec)',...
    'Linear acceleration (m/sec^2)','Euler angle (deg)', 'Gesture performed'};
data_indexes = [1 9 12 15 18;...
    8 11 14 17 20];
y_limits = [0 -10 -360 -20 -360;...
    3.5 10 360 20 360];

%% initiate AAL-Band
constants.prompt = 'Wear the AAL-Band, turn it on and then press enter';
input(constants.prompt);

%% Connect AAL-Band
[Serial_Port,com_stab] = Connect_AAL_BAND();

if(com_stab)
    
    %% reset amplifier gain
    [~] = communicate(Serial_Port,'reset_calibration');
    
    %% calibrate amplifier gain
    %% Close you fist with desired force before calibration starts. Keep the fist during calibration and open it when calibration is finished.
    constants.prompt = 'Close you Fist and then press enter';
    input(constants.prompt);
    disp('Calibrating')
    [~] = communicate(Serial_Port,'calibrate');
    disp('Calibrating finished')
    constants.prompt = 'Press enter to start recording';
    input(constants.prompt);
    
    
    %% recording real time data
    for number_of_gestures = 1:length(gesture_names)
        
        index = 1;
        tic
        current_time = toc;
        
        FSR_data = 0*ones(pre_allocated_space*20,8);
%         IMU_data = 0*ones(pre_allocated_space*20,13);%%此处比原本多一个
        IMU_data = 0*ones(pre_allocated_space*20,14);%%此处�?多一个
        
        [~] = communicate(Serial_Port,'start');
%         plot(1,1)
        set(gcf, 'Position',  [100, 100, 800, 400])
%         try
%             image_path = strcat('images\',gesture_names{number_of_gestures},'.jpg');
%             gesture = imread(image_path);
%             subplot(2,3,6)
%             imshow(gesture);
%         catch
%             subplot(2,3,6)
%             plot(1,1)
%         end
        
        pause(0.001)
        while (current_time < time_to_record)
            
            %% start colecting data
            Data = communicate(Serial_Port,'get_sample'); %% this command will return a data recorded in 20 msec window
            FSR_data((index-1)*20+1:index*20,:) = Data(:,1:8);
            
            x=Data(:,18);
            y=Data(:,19);
            z=Data(:,20);
            w=Data(:,21);
            test=x.*y+z.*w;
%             if (test > 0.499)
%                 euler_x=2*atand(x.*w);
%                 euler_y = 90;
%                 euler_z =0;
%             elseif (test < -0.499)
%                 euler_x=2*atand(x.*w);
%                 euler_y = -90;
%                 euler_z =0;
%             else
                euler_x=atan2d(2*(y.*w+x.*z),1-2*(z.*z+y.*y));
                euler_y = asind(2*(x.*y+z.*w));
                euler_z =atan2d(2*(x.*w+y.*z),1-2*(z.*z+x.*x));
           % end
            %%%1207 problem%l�?版本 IMU_data((index-1)*20+1:index*20,:) = [Data(:,9:21) euler_y(:,1)];
            IMU_data((index-1)*20+1:index*20,:) = [Data(:,9:21) euler_y];
            if(index*20 < show_data)
                data_2_plot = [FSR_data(1:show_data,:) IMU_data(1:show_data,:)];
            else
                data_2_plot = [FSR_data((index*20 - show_data + 1):index*20,:) IMU_data((index*20 - show_data + 1):index*20,:)];
            end
            index = index + 1;
            %% plotting
            plot_counter = plot_counter + 1;
            if(plot_counter == plotting_time)
                plot_counter = 0;
%                 color='rgbmckrb';
%                 hold on;
%                 for pltIndex = 1:8
%                     plot(data_2_plot(:, pltIndex), 'Color', color(pltIndex));
%                 end
%                 hold off;
               plot(data_2_plot(:,1:8));          
%                 h10=legend('FSR 1','FSR 2','FSR 3','FSR 4','FSR 5','FSR 6','FSR 7','FSR 8');
%                 set(h10,'position',[0.73,0.62,0.05,0.05],'units','normalized');

                xlim([1 show_data+show_data/5])
                ylim([0 3.5])
                ylabel('Activation level');
                set(gca,'xtick',[]);
                grid on;

                pause(0.001)
            end
            current_time = toc;
        end
        %% stop collecting data
        [~] = communicate(Serial_Port,'stop');
        
        raw_data.(gesture_names{number_of_gestures}).fsr_data = FSR_data(1:(index-1)*20,:);
        imu_indexes = 1:20:(index-1)*20;
        raw_data.(gesture_names{number_of_gestures}).imu_data = IMU_data(imu_indexes,:);
        
    end
    
    name = 'session_data';
    save(name,'raw_data')
    %% put AAL-Band to sleep mode
    [~] = communicate(Serial_Port,'shut_down');
    
        
else
    
    disp('AAL-Band not found')
    
end
close all
%% Commands
%% 'start'  to start collecting data
%% 'get_sample' to get data sample
%% 'stop'   to stop collecting data
%% 'calibrate'  Calibration is performed to optimize FSR amplifier gain. Close your fist with desired force before starting calibration and open your fist when calibration is ended
%% 'reset_calibration' to reset amplifier gain
%% 'shut_down'  to put AAL-Band control unit to sleep mode
%%
%% Data structure FSR_data
%% columns 1 to 8 are FSR sensors (voltage) ... recording frequency -> 1000 Hz
%% Data structure IMU_data
%% columns 1 to 3 are gravity vector in x,y and z axis (m/sec2) ... recording frequency -> 50 Hz
%% columns 4 to 6 are angular velocity vector in x,y and z axis (rad/sec) ... recording frequency -> 50 Hz
%% columns 7 to 9 are linear acceleration vector in x,y and z axis (m/sec2) ... recording frequency -> 50 Hz
%% columns 10 to 13 are quaternion angles vector in x,y,z and w ... recording frequency -> 50 Hz
