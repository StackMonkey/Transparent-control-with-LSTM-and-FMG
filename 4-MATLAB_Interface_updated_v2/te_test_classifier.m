function handles = te_test_classifier(handles,test_features_data)



% %% nn testing
% %%combine
% % %%for slopes only
% % index_slopes = [17:32 41:48];
% % %%for rms only
% % index_rms = [1:16 33:40];
% % %% for all
% % index_all = [1:48];
% te_features_data = test_features_data;
% %% left
% %%for slopes only
% index_slopes_le = [17:24];%% 41:44];
% %%for rms only
% index_rms_le = 1:8;%% 33:36];
% %% for all
% index_all_le = [1:48];
% emg_slope_left = [41:44];
% emg_rms_left = [33:36];
% %% right
% %%for slopes only
% index_slopes_ri = [25:32];%% 45:48];
% %%for rms only
% index_rms_ri = 9:16;%% 37:40];
% %% for all
% index_all_ri = [1:48];
%
% index_left = 1:3;
% index_right = 9:11;
%
% biceps1 = mean(mean(test_features_data(:,17:19),2));
% triceps1 = mean(mean(test_features_data(:,20:24),2));
% biceps2 = mean(mean(test_features_data(:,25:27),2));
% triceps2 = mean(mean(test_features_data(:,28:32),2));
%
% index_slopes = index_rms_le;
% test_features_data = te_features_data(:,index_slopes);
% result_predicted = handles.metricdata.train.nn_classifier_le(test_features_data');
% result_predicted_le = find(result_predicted == max(result_predicted));
%
% index_slopes = index_rms_ri;
% test_features_data = te_features_data(:,index_slopes);
% result_predicted = handles.metricdata.train.nn_classifier_ri(test_features_data');
% result_predicted_ri = find(result_predicted == max(result_predicted));
%
% result_predicted = 1;
% if(result_predicted_le == 2 && result_predicted_ri == 2)
%
%     result_predicted = 2;
%
% end
%
%
% number_of_intances_other = 7;
% a = 0;
% tf = isempty(handles.metricdata.test.results);
% if(tf == 0)
%     if(length(handles.metricdata.test.results(:,1))>(number_of_intances_other+3))
%
%         a = handles.metricdata.test.results(end-2:end,1);
%         a = sum((a==2*ones(3,1)));
%
%     end
% end
%
% b = 0;
% if(tf == 0)
%     if(length(handles.metricdata.test.results(:,1))>(number_of_intances_other+3))
%
%         b = handles.metricdata.test.results(end-(number_of_intances_other-1):end,1);
%         b = sum((b==1*ones(number_of_intances_other,1)));
%
%     end
% end
%



%% 3rd option
% switch result_predicted
%     case 1
%         if(handles.metricdata.assistance_provided == 1)
%             if(b==number_of_intances_other)
%                 fwrite(handles.metricdata.bluetooth.exoskeleton,'A');
%                 handles.metricdata.assistance_provided = 0;
%                 aa = handles.metricdata.assistance_provided;
%                 aa
%             end
%         end
%     case 2
%         if(handles.metricdata.assistance_provided == 0)
%             if(a<2)
%                 fwrite(handles.metricdata.bluetooth.exoskeleton,'t');
%                 handles.metricdata.assistance_provided = 1;
%                 aa = handles.metricdata.assistance_provided;
%                 aa
%             end
%         end
% end


activated = get(handles.update_assist_level,'Value');
pause(0.003)
if(activated)
    if(handles.metricdata.assistance_provided == 0)
        fwrite(handles.metricdata.bluetooth.exoskeleton,'t');
    end
    handles.metricdata.assistance_provided = 1;
    result_predicted = 2;
    
    set(handles.update_assist_level,'Backgroundcolor','green');
    
else
    
    if(handles.metricdata.assistance_provided == 1)
        fwrite(handles.metricdata.bluetooth.exoskeleton,'A');
    end
    handles.metricdata.assistance_provided = 0;
    result_predicted = 1;
    set(handles.update_assist_level,'Backgroundcolor','default');
    
end



result = result_predicted;
handles.metricdata.test.results = [handles.metricdata.test.results;result];

