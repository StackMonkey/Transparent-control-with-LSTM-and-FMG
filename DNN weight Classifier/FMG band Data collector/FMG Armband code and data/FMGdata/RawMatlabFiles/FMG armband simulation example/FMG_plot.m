
t = tiledlayout(2,2)

% Top left plot
nexttile
x_axis = 0:1/300:(length(MaltheLoadfsr)-1)/300;
plot(x_axis, MaltheLoadfsr)
title('Test subject 1: Load')
xlim([0 30])
ylim([0 3])
set(gca,'FontSize',14)


% Top right plot
nexttile
x_axis = 0:1/300:(length(MaltheNoLoadfsr)-1)/300;
plot(x_axis, MaltheNoLoadfsr)
title('Test subject 1: No load')
xlim([0 30])
ylim([0 3])
set(gca,'FontSize',14)

% Bottom left plot
nexttile
x_axis = 0:1/300:(length(LouisLoadfsr)-1)/300;
plot(x_axis, LouisLoadfsr)
title('Test subject 2: Load')
xlim([0 30])
ylim([0 3.5])
set(gca,'FontSize',14)

% Bottom right plot
nexttile
x_axis = 0:1/300:(length(LouisNoLoadfsr)-1)/300;
plot(x_axis, LouisNoLoadfsr)
title('Test subject 2: No load')
xlim([0 30])
ylim([0 3.5])
set(gca,'FontSize',14)

%Common title and labels
%title(t,'FMG time data for two test subjects','FontSize',24,'FontWeight','bold')
xlabel(t,'Seconds (s)','FontSize',20)
ylabel(t,'Magnitude','FontSize',20)
