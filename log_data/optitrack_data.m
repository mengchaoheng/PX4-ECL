clear all;close all;clc;
format long;
d2r=pi/180;
r2d=180/pi;
opti = importdata('Take 2023-04-16 08.31.25 PM.csv');
%% data allocation, convert to NED
opti_time_s=opti.data(:,2);
opti_origin_pos=[opti.data(:,9) -opti.data(:,7) -opti.data(:,8)]/1000;
opti_origin_q=[opti.data(:,6) opti.data(:,5) -opti.data(:,3) -opti.data(:,4)];
opti_uav_pos=[opti.data(:,16) -opti.data(:,14) -opti.data(:,15)]/1000;
opti_uav_q=[opti.data(:,13) opti.data(:,12) -opti.data(:,10) -opti.data(:,11)];
%% convert data
opti_origin_Roll=quat_to_roll(opti_origin_q);
opti_origin_Pitch=quat_to_pitch(opti_origin_q);
opti_origin_Yaw=quat_to_yaw(opti_origin_q);
opti_uav_Roll=quat_to_roll(opti_uav_q);
opti_uav_Pitch=quat_to_pitch(opti_uav_q);
opti_uav_Yaw=quat_to_yaw(opti_uav_q);
%% Position
figure,
plot(opti_time_s,opti_uav_pos(:,1)-opti_origin_pos(:,1),'k-');hold on;
plot(opti_time_s,opti_uav_pos(:,2)-opti_origin_pos(:,2),'b-');hold on;
plot(opti_time_s,opti_uav_pos(:,3)-opti_origin_pos(:,3),'r-');hold on;
grid on;
% axis([0 40 -3 3]);
xlabel({'Time(s)'});
ylabel('Position (m)')
title('Position');
legend('X','Y','Z');%legend('boxoff');

%% Euler
figure,
plot(opti_time_s,opti_uav_Roll*r2d,'k-');hold on;
plot(opti_time_s,opti_uav_Pitch*r2d,'b-');hold on;
plot(opti_time_s,opti_uav_Yaw*r2d,'r-');hold on;
grid on;
% axis([0 40 -90 90]);
xlabel({'Time(s)'});
ylabel('Angular (deg)')
title('Euler Angular');
legend('Roll','Pitch','Yaw');%legend('boxoff');

save opti.mat
