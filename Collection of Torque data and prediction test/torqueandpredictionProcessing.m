
control = load("torqueandpredictionRecording.mat");
nocontrol = load("torqueandpredictionRecordingNoControl.mat");
control = control.sensorData;
nocontrol = nocontrol.sensorData;

plot(control(:,4))
hold on
plot(control(:,1))
%plot(control(:,2))
%plot(control(:,3))
legend('torque','prediction 1')%, 'prediction 2', 'prediction 3')
%%
plot(nocontrol(:,4))
hold on
plot(nocontrol(:,1))
%plot(nocontrol(:,2))
%plot(control(:,3))
legend('torque','prediction 1')%
%%
nocontrolwith10 = load("torqueandpredictionWith10predictionsNoControl.mat");
nocontrolwith10 = nocontrolwith10.sensorData;
plot(nocontrolwith10)
%plot(nocontrolwith10(:,4))
%legend('calculated torque','pred3')
legend('calculated torque','pred1','pred2','pred3','pred4','pred5','pred6','pred10')