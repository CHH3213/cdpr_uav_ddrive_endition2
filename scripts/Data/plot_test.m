clc;
clear;
% load('2');
% load('20220414/3uav-6N/3droneData_v8.mat');
load('20220414/2uav-9N/2droneSAData.mat');
figure();
plot(state_drone1(:,1,3));
hold on;
plot(state_drone1(:,1,2));
hold on;
plot(state_drone1(:,1,1));
hold on;
legend('z','y','x')
title("drone1")

figure();

plot(state_drone2(:,1,3));
hold on;
plot(state_drone2(:,1,2));
hold on;
plot(state_drone2(:,1,1));
hold on;
legend('z','y','x')
title("drone2")

figure();

plot(state_payload(:,1,3));
hold on;
plot(state_payload(:,1,2));
hold on;
plot(state_payload(:,1,1));
hold on;
legend('z','y','x')
title("payload")

% figure();
% plot(state_drone3(:,1,3));
% hold on;
% plot(state_drone3(:,1,2));
% hold on;
% plot(state_drone3(:,1,1));
% hold on;
% legend('z','y','x')
% title("drone3")

% save("3UAV-3/3droneData_v8.mat",'timestep','state_drone1',"state_drone2","state_drone3","state_payload","force_cable1","force_cable2", "force_cable3",...
%     "thrust_drone1","thrust_drone2","thrust_drone3","force_logger0","force_logger1","force_logger2","force_logger3","force_loggerAll","pwm2thrust1","pwm2thrust2","pwm2thrust3")