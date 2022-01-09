clc;
clear;
% load('0109/3uav2/3droneData_v8.mat');
% load('0109/2uav-big1/2droneBAData.mat');
load('0109/2uav-small1/2droneSAData.mat');
% f = sum(force_cable1(1000:1100,:),1)/100+sum(force_cable2(1000:1100,:),1)/100+sum(force_cable3(1000:1100,:),1)/100
% thrust =(norm( sum(thrust_drone1(1000:1100,:),1)/100)^1.5+norm(sum(thrust_drone2(1000:1100,:),1)/100)^1.5+norm(sum(thrust_drone3(1000:1100,:),1)/100)^1.5)*4.328
thrust =(norm( sum(thrust_drone1(1000:1100,:),1)/100)^1.5+norm(sum(thrust_drone2(1000:1100,:),1)/100)^1.5)*4.328
f = sum(force_cable1(1000:1100,:),1)/100+sum(force_cable2(1000:1100,:),1)/100