clear all;
close all;
clc;
format long 
addpath(genpath(pwd));
addpath '/Users/mch/Proj/akstuki-PX4-ECL/PX4-ECL/EKF/matlab/EKF_replay/Common'
% you can run on terminal 
% ulog2csv log_8_2021-5-20-11-52-08.ulg 
% to get csv files
% =====================1==========================
% Install pyulog using pip first.https://github.com/PX4/pyulog.
% in MacOS, it maybe have been installed by the px4-dev
% =====================2==========================
% Make sure it has installed ulog2csv correctly (check the output of which ulog2csv in Linux/MacOS or where ulog2csv in Windows).
% =====================3==========================
% Change the following line in ulogviewver.m:
% command = ['!/usr/local/bin/ulog2csv ' ulgFileName '.ulg'];
% to 
% command = ['!your ulog2csv path' ulgFileName '.ulg'];
% and 
% ulgFileName = '00_41_22'; 
% to 
% ulgFileName = 'your log name'; 

% ----fig size, you have to change it for your fig

d2r=pi/180;
r2d=180/pi;
%% two offline can be use, something different. px4 v12.3.0
ulgFileName = 'log005e'; % the ulog file name  17_48_41
tmp=[ ulgFileName '.mat'];
% exist tmp var
if exist(tmp,"file")
    load(ulgFileName,'log');
else
    if ismac
        % on macOS, run " which ulog2csv " on terminal to get it.
        command = ['!/opt/homebrew/bin/ulog2csv ' ulgFileName '.ulg']; % /usr/local/bin/ is the path of ulog2csv, 
    else
        % on windows and linux just make sure you have installed pyulog
        command = ['!ulog2csv ' ulgFileName '.ulg']; % have installed ulog2csv,
    end 

	eval(command);
    log.data = csv_topics_to_d(ulgFileName);
    log.FileName = ulgFileName;
    log.version = 1.0;
    log.params = '';
    log.messages = '';
    log.info = '';
    %run add_fields_in_preprocessing.m
    save(ulgFileName,'log')
    % copy data to ecl, and then 
    delete(['*' ulgFileName '*.csv'])
end
start=1;
vehicle_attitude=log.data.vehicle_attitude_0{:,:};
vehicle_local_position=log.data.vehicle_local_position_0{:,:};
XYZ=log.data.vehicle_local_position_0{:,6:8};
V_xyz=log.data.vehicle_local_position_0{:,12:14};

q_0=vehicle_attitude(:,3);
q_1=vehicle_attitude(:,4);
q_2=vehicle_attitude(:,5);
q_3=vehicle_attitude(:,6);
Roll=quat_to_roll([q_0 q_1 q_2 q_3]);
Pitch=quat_to_pitch([q_0 q_1 q_2 q_3]);
Yaw=quat_to_yaw([q_0 q_1 q_2 q_3]);
sensor_combined=log.data.sensor_combined_0{:,:};
vehicle_visual_odometry=log.data.vehicle_visual_odometry_0{:,:};
visual_time=vehicle_visual_odometry(:,1);
visual_XYZ=vehicle_visual_odometry(:,3:5);
visual_Roll=quat_to_roll(vehicle_visual_odometry(:,6:9));
visual_Pitch=quat_to_pitch(vehicle_visual_odometry(:,6:9));
visual_Yaw=quat_to_yaw(vehicle_visual_odometry(:,6:9));
% vehicle_gps_position=log.data.vehicle_gps_position_0{:,:};
vehicle_magnetometer=log.data.vehicle_magnetometer_0{:,:};
vehicle_air_data=log.data.vehicle_air_data_0{:,:};

%%
%1666
q_init=[q_0(1440);q_1(1440);q_2(1440);q_3(1440);];
q_end=[q_0(1666);q_1(1666);q_2(1666);q_3(1666);];

Tbn = Quat2Tbn(q_init);
Tbo = Quat2Tbn(q_end);
Ton=Tbn*Tbo';
% or q_e = quatdivide(q_end', q_init');Ton=quat2rotm(q_e);
% q_b2n=q_o2n*q_b2o -> q_end=q_e*q_init -> Tbn=Ton*Tbo

%% vision
opti = importdata('Take 2023-04-25 02.57.17 PM.csv');
enable_enu=0;
if ~enable_enu
    %% NED. data allocation, convert x-y-z to front-right-down and redefinite q=[w,x,y,z]
    opti_time_s=opti.data(:,2);
    opti_origin_pos=[  opti.data(:,8) opti.data(:,7) -opti.data(:,9)]/1000; % (mm) to (m)
    opti_origin_q=[opti.data(:,6)   opti.data(:,4) opti.data(:,3) -opti.data(:,5)];
    % convert XYZ and XYZW of RigidBody
    opti_uav_pos=[ opti.data(:,15) opti.data(:,14) -opti.data(:,16)]/1000; % X=opti.data(:,14), Y=opti.data(:,15), Z=opti.data(:,16)
    opti_uav_q=[opti.data(:,13) opti.data(:,11) opti.data(:,10)  -opti.data(:,12)]; % X=opti.data(:,10), Y=opti.data(:,11), Z=opti.data(:,12) W=opti.data(:,13)

else
    %% ENU. origin, the same with ros
    opti_time_s=opti.data(:,2);
    opti_origin_pos=[  opti.data(:,7) opti.data(:,8) opti.data(:,9)]/1000; % (mm) to (m)
    opti_origin_q=[opti.data(:,6)   opti.data(:,3) opti.data(:,4) opti.data(:,5)];
    opti_uav_pos=[ opti.data(:,14) opti.data(:,15) opti.data(:,16)]/1000; % X=opti.data(:,14), Y=opti.data(:,15), Z=opti.data(:,16)
    opti_uav_q=[opti.data(:,13) opti.data(:,10) opti.data(:,11)  opti.data(:,12)]; % X=opti.data(:,10), Y=opti.data(:,11), Z=opti.data(:,12) W=opti.data(:,13)
end
%%



%% convert data
% opti_origin_Roll=quat_to_roll(opti_origin_q);
% opti_origin_Pitch=quat_to_pitch(opti_origin_q);
% opti_origin_Yaw=quat_to_yaw(opti_origin_q);
opti_uav_Roll=quat_to_roll(opti_uav_q);
opti_uav_Pitch=quat_to_pitch(opti_uav_q);
opti_uav_Yaw=quat_to_yaw(opti_uav_q);
%% to true NED
% temp=Ton*[opti_uav_Roll'; opti_uav_Pitch'; opti_uav_Yaw'];
% opti_uav_Roll=temp(1,:)';
% opti_uav_Pitch=temp(2,:)';
% opti_uav_Yaw=temp(3,:)';
% temp1=Ton*opti_uav_pos';
% opti_uav_pos=temp1';
%% move origin of front-right-down to system init point
% opti_uav_pos=opti_uav_pos-opti_uav_pos(1,:);

%% and maybe more figure, all in the variable "log.data"
fig1=figure(1);
subplot(3,1,1);
plot(opti_time_s+104.128,opti_uav_Roll*r2d,'k-');hold on;
plot(visual_time*1e-6,visual_Roll*r2d,'g-');hold on;
% plot((vehicle_attitude(start:end,1))*1e-6+0, Roll(start:end)*r2d,'r-','LineWidth',1);hold on;
grid on;
legend('optitrack','vision','PX4');
xlabel({'Time(s)'});
ylabel('Roll (deg)');
title('Euler Angle Estimates');
%% and maybe more figure, all in the variable "log.data"
subplot(3,1,2);
plot(opti_time_s+104.128,opti_uav_Pitch*r2d,'k-');hold on;
plot(visual_time*1e-6,visual_Pitch*r2d,'g-');hold on;
% plot((vehicle_attitude(start:end,1))*1e-6+0, Pitch(start:end)*r2d,'r-','LineWidth',1);hold on;
grid on;
legend('optitrack','vision','PX4');
xlabel({'Time(s)'});
ylabel('Pitch (deg)')
%% and maybe more figure, all in the variable "log.data"
subplot(3,1,3);
plot(opti_time_s+104.128,opti_uav_Yaw*r2d,'k-');hold on;
plot(visual_time*1e-6,visual_Yaw*r2d,'g-');hold on;
% plot((vehicle_attitude(start:end,1))*1e-6+0, Yaw(start:end)*r2d,'r-','LineWidth',1);hold on;
grid on;
legend('optitrack','vision','PX4');
xlabel({'Time(s)'});
ylabel('Yaw (deg)');
%% 
% % PlotToFileColorPDF(fig1,'../results/RPY.pdf',15,20); % or 'RPY.pdf'
%% and maybe more figure, all in the variable "log.data"
fig2=figure(2);
subplot(3,1,1);
plot(opti_time_s+104.128,opti_uav_pos(:,1),'k-');hold on;
plot(visual_time*1e-6,visual_XYZ(:,1),'g-');hold on;
% plot((vehicle_local_position(start:end,1))*1e-6+0, XYZ(start:end,1),'r-','LineWidth',1);hold on;
grid on;
legend('optitrack','vision','PX4');
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('X (m)');
title('Position Estimates');
%% and maybe more figure, all in the variable "log.data"
subplot(3,1,2);
plot(opti_time_s+104.128,opti_uav_pos(:,2),'k-');hold on;
plot(visual_time*1e-6,visual_XYZ(:,2),'g-');hold on;
% plot((vehicle_local_position(start:end,1))*1e-6+0, XYZ(start:end,2),'r-','LineWidth',1);hold on;
grid on;
legend('optitrack','vision','PX4');
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('Y (m)');
%% and maybe more figure, all in the variable "log.data"
subplot(3,1,3);
plot(opti_time_s+104.128,opti_uav_pos(:,3),'k-');hold on;
plot(visual_time*1e-6,visual_XYZ(:,3),'g-');hold on;
% plot((vehicle_local_position(start:end,1))*1e-6+0, XYZ(start:end,3),'r-','LineWidth',1);hold on;
grid on;
legend('optitrack','vision','PX4');
axis([-inf inf -3 1]);
xlabel({'Time(s)'});
ylabel('Z (m)');
%% 
% PlotToFileColorPDF(fig2,'../results/pos.pdf',15,20);% or 'pos.pdf'

%% and maybe more figure, all in the variable "log.data"
fig3=figure(3);
subplot(3,1,1);
plot((vehicle_local_position(start:end,1))*1e-6, V_xyz(start:end,1),'k-','LineWidth',1);hold on;
grid on;
xlabel({'Time(s)'});
ylabel('V_x (m/s)');
title('velocity Estimates');
%% and maybe more figure, all in the variable "log.data"
subplot(3,1,2);
plot((vehicle_local_position(start:end,1))*1e-6, V_xyz(start:end,2),'r-','LineWidth',1);hold on;
grid on;
xlabel({'Time(s)'});
ylabel('V_y (m/s)');
%% and maybe more figure, all in the variable "log.data"
subplot(3,1,3);
plot((vehicle_local_position(start:end,1))*1e-6, V_xyz(start:end,3),'b-','LineWidth',1);hold on;
grid on;
xlabel({'Time(s)'});
ylabel('V_z (m/s)');

%% 
% PlotToFileColorPDF(fig3,'../results/vel.pdf',15,20);% or 'vel.pdf'


% [len1,~]=size(sensor_combined);
% [len2,~]=size(vehicle_visual_odometry);
% % [len2,~]=size(vehicle_gps_position);
% [len3,~]=size(vehicle_magnetometer);
% [len4,~]=size(vehicle_air_data);
% t1=1:1:len1;
% t2=1:1:len2;
% t3=1:1:len3;
% t4=1:1:len4;
% figure,
% plot(t1,sensor_combined(:,1),'k-');hold on;
% plot(t2,vehicle_visual_odometry(:,1),'r-');hold on;
% plot(t3,vehicle_magnetometer(:,1),'b');hold on;
% plot(t4,vehicle_air_data(:,1),'k--');hold on;
% legend('sensor combined','vehicle visual odometry','vehicle magnetometer','vehicle air data');
% figure,
% plot((vehicle_attitude(start:end,1))*1e-6,vehicle_attitude(start:end,4),'k:.');hold on;
% plot((vehicle_visual_odometry(start:end,1))*1e-6,vehicle_visual_odometry(start:end,7),'r:.');hold on;
% legend('vehicle attitude','vehicle visual odometry');
