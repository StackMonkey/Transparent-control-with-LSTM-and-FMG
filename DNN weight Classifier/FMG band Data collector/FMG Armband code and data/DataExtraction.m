% Malthe Large Load
MaltheLargeLoad = load('MaltheLargePayload.mat');
MaltheLargeLoadfsr = MaltheLargeLoad.raw_data.open;
MaltheLargeLoadfsr = MaltheLargeLoadfsr.fsr_data;

plot(MaltheLargeLoadfsr)
%% Malthe Large load Extract xls files flexed
Intervalslarge1 = {[4200,6400],[9700,12400],[16000,18750],[21650,24100],[28200,31000],[34530,37000],[40481,42641],[46483,49384],[52714,55300],[58900,60900]}
cutMaltheLargeLoadfsr = extract_and_plot_intervals(MaltheLargeLoadfsr,Intervalslarge1);
for i = 1:10
    filename = sprintf('dataMaltheLargeLoadfsrFlexed%04d.xls',i);
    xlswrite(filename, cutMaltheLargeLoadfsr(:,:,i))
end
%% Malthe Large load Extract xls files rest
Intervalslarge2 = {[6500,9500],[12600,15600],[19000,21500],[26000,28000],[31500,34000],[37500,40000],[43500,46000],[50000,52500],[55500,58000]}
cutMaltheLargeLoadfsr = extract_and_plot_intervals(MaltheLargeLoadfsr,Intervalslarge2);
for i = 1:9
    filename = sprintf('dataMaltheLargeLoadfsrRest%04d.xls',i);
    xlswrite(filename, cutMaltheLargeLoadfsr(:,:,i))
end
%% Malthe no load
MaltheNoLoad = load('dataMaltheNoLoad.mat');

MaltheNoLoadfsr = MaltheNoLoad.raw_data.open;

MaltheNoLoadfsr = MaltheNoLoadfsr.fsr_data;

plot(MaltheNoLoadfsr)
%% Malthe no load Extract xls files flexed
Intervals = {[565,3117],[6346,9361],[12549,15348],[18617,21511],[24743,27567],[30742,33730],[36939,39912],[42995,45861],[49136,52030],[55343,58307]};
cutMaltheNoLoadfsr = extract_and_plot_intervals(MaltheNoLoadfsr, Intervals);
for i = 1:10
    filename = sprintf('dataMaltheNoLoadFlexed%04d.xls',i);
    xlswrite(filename, cutMaltheNoLoadfsr(:,:,i))
end

%% Malthe no load Extract xls files rest
Intervals1 = {[3157,6306],[9401,12509],[15408,18577],[21561,24703],[27607,30702],[33770,36899],[39952,42955],[45901,49096],[52070,55303],[58347,61100]};
cutMaltheNoLoadfsr = extract_and_plot_intervals(MaltheNoLoadfsr, Intervals1);
for i = 1:10
    filename = sprintf('dataMaltheNoLoadRest%04d.xls',i);
    xlswrite(filename, cutMaltheNoLoadfsr(:,:,i))
end

%% Louis No Load
LouisNoLoad = load('dataLouisNoLoad.mat');

LouisNoLoadfsr = LouisNoLoad.raw_data.open;

LouisNoLoadfsr = LouisNoLoadfsr.fsr_data;

plot(LouisNoLoadfsr)
%% Louis no load Extract xls files edges
Intervals12 = {[500,1000],[3000,3700],[6200,6800], [9000,9600],[12100,12700],[15300,15900],[18000,18600], [21400,22000],[24450,25050],[27700,28300]};
cutLouisNoLoadedgesfsr = extract_and_plot_intervals(LouisNoLoadfsr, Intervals11);
for i = 1:10
    filename = sprintf('dataLouisNoLoadedges%04d.xls',i);
    xlswrite(filename, cutLouisNoLoadedgesfsr(:,:,i))
end
%% Malthe Load
MaltheLoad = load('dataMaltheLoad.mat');

MaltheLoadfsr = MaltheLoad.raw_data.open;

MaltheLoadfsr = MaltheLoadfsr.fsr_data;

plot(MaltheLoadfsr)
%% Malthe load Extract xls files flexed
Intervals2 = {[1,2850],[6130,9140],[12473,15245], [18534, 21382],[24801,28002],[30921,33672],[37223,39869], [43432,46101],[49202,52077],[55301,58061]};
cutMaltheLoadfsr = extract_and_plot_intervals(MaltheLoadfsr, Intervals2);
for i = 1:10
    filename = sprintf('dataMaltheLoadFlexed%04d.xls',i);
    xlswrite(filename, cutMaltheLoadfsr(:,:,i))
end
%% Malthe load Extract xls files rest
Intervals3 = {[2900,6090],[9180,12400],[15300,18400], [21400,24400 ],[28202,30800],[33800,37000],[40000,43000], [46200,49000],[52200,55200],[58200,60900]};
cutMaltheLoadfsr = extract_and_plot_intervals(MaltheLoadfsr, Intervals3);
for i = 1:10
    filename = sprintf('dataMaltheLoadRest%04d.xls',i);
    xlswrite(filename, cutMaltheLoadfsr(:,:,i))
end
%% Louis Load
LouisLoad = load('dataLouisLoad.mat');

LouisLoadfsr = LouisLoad.raw_data.open;

LouisLoadfsr = LouisLoadfsr.fsr_data;

plot(LouisLoadfsr)
%% Louis load Extract xls files flexed
Intervals4 = {[566,3200],[6440,9427],[12711,15539], [18535,21617],[24841,27744],[30895,34012],[37059,40021], [43280,46199],[49221,52384],[55506,58756]};
cutLouisLoadfsr = extract_and_plot_intervals(LouisLoadfsr, Intervals4);
for i = 1:10
    filename = sprintf('dataLouisLoadFlexed%04d.xls',i);
    xlswrite(filename, cutLouisLoadfsr(:,:,i))
end
%% Louis load Extract xls files Rest
Intervals4 = {[3250,6300],[9500,12600],[15600,18400], [21700,24800],[27800,30800],[34100,37000],[40100,43200], [46300,49200],[52400,55400],[58800,61100]};
cutLouisLoadfsr = extract_and_plot_intervals(LouisLoadfsr, Intervals4);
for i = 1:10
    filename = sprintf('dataLouisLoadRest%04d.xls',i);
    xlswrite(filename, cutLouisLoadfsr(:,:,i))
end
%% Louis load Extract xls files edges
Intervals11 = {[1,566],[3200,3700],[6200,6800], [9300,9900],[12500,13000],[15400,15900],[18400,19000], [21500,22000],[24500,25000],[27700,28200]};
cutLouisLoadedgesfsr = extract_and_plot_intervals(LouisLoadfsr, Intervals11);
for i = 1:10
    filename = sprintf('dataLouisLoadedges%04d.xls',i);
    xlswrite(filename, cutLouisLoadedgesfsr(:,:,i))
end
%% Emil Load
EmilLoad = load('EmilLoad.mat');
EmilLoadfsr = EmilLoad.raw_data.open;
EmilLoadfsr = EmilLoadfsr.fsr_data;

plot(EmilLoadfsr)
%% Emil Load Extract xls files Flexed
Intervals5 ={[1259,5040],[9941,14046],[19151,23213],[28494,32594],[37722,41516],[46758,50857],[56115,59812],[65188,68902],[74422,78293],[83649,87481]};
cutEmilLoadfsr = extract_and_plot_intervals(EmilLoadfsr, Intervals5);
for i = 1:10
    filename = sprintf('dataEmilLoadFlexed%04d.xls',i);
    xlswrite(filename, cutEmilLoadfsr(:,:,i));
end
%% Emil Load Extract xls files Rest
Intervals6 ={[5100,9700],[14300,19000],[23400,28400],[32700,37600],[41600,46600],[51000,56000],[60000,65000],[69000,74300],[78500,83500],[87600,91760]};
cutEmilLoadfsr = extract_and_plot_intervals(EmilLoadfsr, Intervals6);
for i = 1:10
    filename = sprintf('dataEmilLoadRest%04d.xls',i);
    xlswrite(filename, cutEmilLoadfsr(:,:,i));
end
%% Emil No Load
EmilNoLoad = load('EmilNoLoad.mat');
EmilNoLoadfsr = EmilNoLoad.raw_data.open;
EmilNoLoadfsr = EmilNoLoadfsr.fsr_data;

plot(EmilNoLoadfsr)
%% Emil No Load Extract xls files Flexed
Intervals7 = {[462,4503],[9624,13781],[18685,22799],[27773,31989],[37114,41196],[46035,50337],[55341,59577],[64668,68856],[73908,78159],[82998,87264]};
cutEmilNoLoadfsr = extract_and_plot_intervals(EmilNoLoadfsr, Intervals7);
for i = 1:10
    filename = sprintf('dataEmilNoLoadFlexed%04d.xls',i);
    xlswrite(filename, cutEmilNoLoadfsr(:,:,i));
end
%% Emil No Load Extract xls files Rest
Intervals8 = {[4600,9600],[13900,18500],[22900,27500],[32000,37000],[41300,46000],[50300,55300],[59500,64500],[68900,73500],[78200,82500],[87400,91740]};
cutEmilNoLoadfsr = extract_and_plot_intervals(EmilNoLoadfsr, Intervals8);
for i = 1:10
    filename = sprintf('dataEmilNoLoadRest%04d.xls',i);
    xlswrite(filename, cutEmilNoLoadfsr(:,:,i));
end
%% Malthe Load rising edges
Intervals9 = {[2500,3000],[5600,6200],[8700,9500], [11500,12700],[15000,16000],[18000,18500],[21000,22000], [24000,25000],[27500,28500],[30000,31100]};
cutMaltheLoadedgesfsr = extract_and_plot_intervals(MaltheLoadfsr, Intervals9);
for i = 1:10
    filename = sprintf('dataMaltheLoadEdges%04d.xls',i);
    xlswrite(filename, cutMaltheLoadedgesfsr(:,:,i));
end
%% Malthe No Load rising edges
Intervals10 = {[2800,3500],[5900,6500],[9300,9800], [12000,12700],[15100,15600],[18000,18800],[21200,21800], [24300,24800],[27500,28200],[30000,30800]};
cutMaltheNoLoadedgesfsr = extract_and_plot_intervals(MaltheNoLoadfsr, Intervals10);
for i = 1:10
    filename = sprintf('dataMaltheNoLoadEdges%04d.xls',i);
    xlswrite(filename, cutMaltheNoLoadedgesfsr(:,:,i));
end

%% Extraction Function
function [extracted_intervals_array] = extract_and_plot_intervals(signals_array, intervals)
    % Define the desired sample size for each interval
    sample_size = 500;

    % Initialize a cell array to store the extracted intervals
    extracted_intervals = cell(length(intervals), 1);

    % Loop through each specified interval and extract it from the array
    for i = 1:length(intervals)
        start_idx = intervals{i}(1);
        end_idx = intervals{i}(2);

        interval_data = signals_array(start_idx:end_idx, :);

        % If the interval size is greater than the desired sample size,
        % downsample it to the desired size
        if size(interval_data, 1) > sample_size
            downsampled_idx = round(linspace(1, size(interval_data, 1), sample_size));
            interval_data = interval_data(downsampled_idx, :);
        end

        extracted_intervals{i} = interval_data;
    end

    % Convert the cell array of extracted intervals into a 3D array
    extracted_intervals_array = cat(3, extracted_intervals{:});

    % Plot the extracted intervals
    num_intervals = size(extracted_intervals_array, 3);
    num_plots = ceil(num_intervals / 5); % Each figure will contain up to 5 intervals
    for fig = 1:num_plots
        figure;
        for i = (5*(fig-1) + 1):min(5*fig, num_intervals)
            subplot(min(5, num_intervals - 5*(fig-1)), 1, i - 5*(fig-1));
            plot(extracted_intervals_array(:, :, i));
            title(['Interval ', num2str(i)]);
            xlabel('Sample');
            ylabel('Signal Value');
        end
    end
end