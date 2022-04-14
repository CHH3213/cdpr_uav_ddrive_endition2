clc;
clear;
% load('0109/3uav2/3droneData_v8.mat');

load('20220414/2uav-9N/2droneSAData.mat');
thrust =(norm( sum(thrust_drone1(1000:1100,:),1)/100)^1.5+norm(sum(thrust_drone2(1000:1100,:),1)/100)^1.5)*4.328
% thrust =(norm( sum(force_cable1(1000:1100,:),1)/100)^1.5+norm(sum(force_cable2(1000:1100,:),1)/100)^1.5)*4.328
f = sum(force_cable1(1000:1100,:),1)/100+sum(force_cable2(1000:1100,:),1)/100
t = thrust/2




% load('./3droneData_v8.mat');
load('20220414/3uav-9N/3droneData_v8.mat')

f1 = sum(force_cable1(1000:1100,:),1)/100+sum(force_cable2(1000:1100,:),1)/100+sum(force_cable3(1000:1100,:),1)/100
thrust1 =(norm( sum(thrust_drone1(1000:1100,:),1)/100)^1.5+norm(sum(thrust_drone2(1000:1100,:),1)/100)^1.5+norm(sum(thrust_drone3(1000:1100,:),1)/100)^1.5)*4.328
% thrust1 =(norm( sum(force_cable1(1000:1100,:),1)/100)^1.5+norm(sum(force_cable2(1000:1100,:),1)/100)^1.5+norm(sum(force_cable3(1000:1100,:),1)/100)^1.5)*4.328

t1 = thrust1/3