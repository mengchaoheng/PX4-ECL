clear all;
close all;
clc;
format long g
addpath(genpath(pwd));
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
ulgFileName = 'v1-13-2'; % the ulog file name  17_48_41
tmp=[ ulgFileName '.mat'];
% exist tmp var
if exist(tmp,"file")
    load(ulgFileName,'log');
else
    if ismac
        % on macOS, run " which ulog2csv " on terminal to get it.
        command = ['!/usr/local/bin/ulog2csv ' ulgFileName '.ulg']; % /usr/local/bin/ is the path of ulog2csv, 
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

% go to run ekf3 and copy results file to cover that of this proj
% cd to build and run `make && ./main/postEcl`
% then continue.

vehicle_attitude=log.data.vehicle_attitude_0{:,:};
vehicle_local_position=log.data.vehicle_local_position_0{:,:};
XYZ=log.data.vehicle_local_position_0{:,6:8};
V_xyz=log.data.vehicle_local_position_0{:,12:14};

q_0=vehicle_attitude(:,3);
q_1=vehicle_attitude(:,4);
q_2=vehicle_attitude(:,5);
q_3=vehicle_attitude(:,6);
Roll=quat_to_roll(q_0,q_1,q_2,q_3);
Pitch=quat_to_pitch(q_0,q_1,q_2,q_3);
Yaw=quat_to_yaw(q_0,q_1,q_2,q_3);


% use txt, high log rate.
% euler_estimator=load('../results/euler_estimator.txt');
% velocity_estimator=load('../results/velocity_estimator.txt');
% position_estimator=load('../results/position_estimator.txt');
% time=euler_estimator(:,1);
% ekf3_Roll=euler_estimator(:,2);
% ekf3_Pitch=euler_estimator(:,3);
% ekf3_Yaw=euler_estimator(:,4);
% ekf3_XYZ = position_estimator(:,2:4);
% time1=position_estimator(:,1);
% ekf3_V_xyz = velocity_estimator(:,2:4);

% or use csv
ECL = importdata('../results/ecloutput.csv');%https://blog.csdn.net/JingpengSun/article/details/128840264
time=ECL.data(:,1);
ekf3_Roll=ECL.data(:,26);
ekf3_Pitch=ECL.data(:,27);
ekf3_Yaw=ECL.data(:,28);
ekf3_XYZ = ECL.data(:,9:11);
ekf3_V_xyz = ECL.data(:,6:8);

start=1;
%% and maybe more figure, all in the variable "log.data"
fig1=figure;
subplot(3,1,1);
plot((vehicle_attitude(start:end,1))*1e-6, Roll(start:end)*r2d,'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_Roll*r2d,'r--','LineWidth',1);hold on;
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('Angular (deg)')
title('Roll');
legend('online','offline');%legend('boxoff');

%% and maybe more figure, all in the variable "log.data"
% figure,
subplot(3,1,2);
plot((vehicle_attitude(start:end,1))*1e-6, Pitch(start:end)*r2d,'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_Pitch*r2d,'r--','LineWidth',1);hold on;
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('Angular (deg)')
title('Pitch');
legend('online','offline');%legend('boxoff');
%% and maybe more figure, all in the variable "log.data"
% figure,
subplot(3,1,3);
plot((vehicle_attitude(start:end,1))*1e-6, Yaw(start:end)*r2d,'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_Yaw*r2d,'r--','LineWidth',1);hold on;
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('Angular (deg)')
title('Yaw');
legend('online','offline');%legend('boxoff');

%% 
PlotToFileColorPDF(fig1,'../results/RPY.pdf',10,15); % or 'RPY.pdf'
%% and maybe more figure, all in the variable "log.data"
fig2=figure;
subplot(3,1,1);
plot((vehicle_local_position(start:end,1))*1e-6, XYZ(start:end,1),'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_XYZ(:,1),'r--','LineWidth',1);hold on;
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('Position (m)')
title('X');
legend('online','offline');%legend('boxoff');

%% and maybe more figure, all in the variable "log.data"
% figure,
subplot(3,1,2);
plot((vehicle_local_position(start:end,1))*1e-6, XYZ(start:end,2),'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_XYZ(:,2),'r--','LineWidth',1);hold on;
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('Position (m)')
title('Y');
legend('online','offline');%legend('boxoff');
%% and maybe more figure, all in the variable "log.data"
% figure,
subplot(3,1,3);
plot((vehicle_local_position(start:end,1))*1e-6, XYZ(start:end,3),'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_XYZ(:,3),'r--','LineWidth',1);hold on;
grid on;
% axis([-inf inf -100 100]);
ylabel('Position (m)')
title('Z');
legend('online','offline');%legend('boxoff');

%% 
PlotToFileColorPDF(fig2,'../results/pos.pdf',10,15);% or 'pos.pdf'

%% and maybe more figure, all in the variable "log.data"
fig3=figure;
subplot(3,1,1);
plot((vehicle_local_position(start:end,1))*1e-6, V_xyz(start:end,1),'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_V_xyz(:,1),'r--','LineWidth',1);hold on;
% plot(time*1e-6, ekf3_V_xyz1(:,1),'b-','LineWidth',1);hold on;
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('velocity (m/s)')
title('V_x');
legend('online','offline');%legend('boxoff');

%% and maybe more figure, all in the variable "log.data"
% figure,
subplot(3,1,2);
plot((vehicle_local_position(start:end,1))*1e-6, V_xyz(start:end,2),'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_V_xyz(:,2),'r--','LineWidth',1);hold on;
% plot(time*1e-6, ekf3_V_xyz1(:,2),'b-','LineWidth',1);hold on;
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('velocity (m/s)')
title('V_y');
legend('online','offline');%legend('boxoff');
%% and maybe more figure, all in the variable "log.data"
% figure,
subplot(3,1,3);
plot((vehicle_local_position(start:end,1))*1e-6, V_xyz(start:end,3),'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_V_xyz(:,3),'r--','LineWidth',1);hold on;
% plot(time*1e-6, ekf3_V_xyz1(:,3),'b-','LineWidth',1);hold on;
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('velocity (m/s)')
title('V_z');
legend('online','offline');%legend('boxoff');

%% 
PlotToFileColorEPS(fig3,'../results/vel.pdf',10,15);% or 'vel.pdf'