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
ulgFileName = 'log_62_2023-4-9-13-57-58'; % the ulog file name  17_48_41 log36
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
% delete('../csv_data/*.csv');
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
Roll=quat_to_roll([q_0 q_1 q_2 q_3]);
Pitch=quat_to_pitch([q_0 q_1 q_2 q_3]);
Yaw=quat_to_yaw([q_0 q_1 q_2 q_3]);
%%
have_opti_data=exist("opti.mat","file");
if have_opti_data
load 'opti.mat';
% data allocation, using q_init convert frame of opti to NED
% Recreate the RigidBody before each flight to obtain a RigidBody frame 
% aligned with the original coordinate system in the motion capture system.
% q_init=[q_0(1);q_1(1);q_2(1);q_3(1);];
% uav_q=zeros(length(opti_uav_q),4);
% uav_pos=zeros(length(opti_uav_q),3);
% Tbn = Quat2Tbn(q_init);
% % Then we use the attitude obtained when the aircraft is initialized to 
% % rotate the frame of the motion capture system, and finally align the NED frame.
% for i=1:length(opti_uav_q)
%     uav_q(i,:) = QuatMult(q_init,opti_uav_q(i,:)');
%     uav_pos(i,:)=(Tbn * opti_uav_pos(i,:)')';
% end
% uav_Roll=quat_to_roll(uav_q);
% uav_Pitch=quat_to_pitch(uav_q);
% uav_Yaw=quat_to_yaw(uav_q);

end
%%

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

estimator_status=load('../results/estimator_status.txt');


% uint32 control_mode_flags	# Bitmask to indicate EKF logic state
% uint8 CS_TILT_ALIGN = 0		# 0 - true if the filter tilt alignment is complete
% uint8 CS_YAW_ALIGN = 1		# 1 - true if the filter yaw alignment is complete
% uint8 CS_GPS = 2		# 2 - true if GPS measurements are being fused
% uint8 CS_OPT_FLOW = 3		# 3 - true if optical flow measurements are being fused
control_mode_flags=dec2bin(estimator_status(:,2))

% uint16 innovation_check_flags # Bitmask to indicate pass/fail status of innovation consistency checks
% # 0 - true if velocity observations have been rejected
% # 1 - true if horizontal position observations have been rejected
% # 2 - true if true if vertical position observations have been rejected
% # 3 - true if the X magnetometer observation has been rejected
% # 4 - true if the Y magnetometer observation has been rejected
% # 5 - true if the Z magnetometer observation has been rejected
% # 6 - true if the yaw observation has been rejected
% # 7 - true if the airspeed observation has been rejected
% # 8 - true if the synthetic sideslip observation has been rejected
% # 9 - true if the height above ground observation has been rejected
% # 10 - true if the X optical flow observation has been rejected
% # 11 - true if the Y optical flow observation has been rejected
innovation_check_flags=dec2bin(estimator_status(:,3))

% or use csv
ECL = importdata('../results/ecloutput.csv');%https://blog.csdn.net/JingpengSun/article/details/128840264
time=ECL.data(:,1);
ekf3_Roll=ECL.data(:,26);
ekf3_Pitch=ECL.data(:,27);
ekf3_Yaw=ECL.data(:,28);
ekf3_XYZ = ECL.data(:,9:11);
ekf3_V_xyz = ECL.data(:,6:8);

start=1;

save ../EKF/matlab/EKF_replay/TestData/PX4/ecl.mat

%% and maybe more figure, all in the variable "log.data"
fig1=figure(1);
subplot(3,1,1);
plot((vehicle_attitude(start:end,1))*1e-6, Roll(start:end)*r2d,'k:.','LineWidth',1);hold on;
plot(time*1e-6, ekf3_Roll*r2d,'r-','LineWidth',1);hold on;
if have_opti_data
    plot(opti_time_s+16.33,opti_uav_Roll*r2d,'b--','LineWidth',1);hold on;
end
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('Roll (deg)')
title('Euler Angle Estimates');
if have_opti_data
    legend('PX4','offline','optitrack');
else
    legend('PX4','offline');%legend('boxoff');
end

%% and maybe more figure, all in the variable "log.data"
% figure,
subplot(3,1,2);
plot((vehicle_attitude(start:end,1))*1e-6, Pitch(start:end)*r2d,'k:.','LineWidth',1);hold on;
plot(time*1e-6, ekf3_Pitch*r2d,'r-','LineWidth',1);hold on;
if have_opti_data
    plot(opti_time_s+16.33,opti_uav_Pitch*r2d,'b--','LineWidth',1);hold on;
end
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('Pitch (deg)')
% title('Pitch');
if have_opti_data
    legend('PX4','offline','optitrack');
else
    legend('PX4','offline');%legend('boxoff');
end
%% and maybe more figure, all in the variable "log.data"
% figure,
subplot(3,1,3);
plot((vehicle_attitude(start:end,1))*1e-6, Yaw(start:end)*r2d,'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_Yaw*r2d,'r-','LineWidth',1);hold on;
if have_opti_data
    plot(opti_time_s+16.33,opti_uav_Yaw*r2d,'b--','LineWidth',1);hold on; %12.278
end
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('Yaw (deg)')
% title('Yaw');
if have_opti_data
    legend('PX4','offline','optitrack');
else
    legend('PX4','offline');%legend('boxoff');
end


%% 
% % PlotToFileColorPDF(fig1,'../results/RPY.pdf',15,20); % or 'RPY.pdf'
%% and maybe more figure, all in the variable "log.data"
fig2=figure(2);
subplot(3,1,1);
plot((vehicle_local_position(start:end,1))*1e-6, XYZ(start:end,1),'k:.','LineWidth',1);hold on;
plot(time*1e-6, ekf3_XYZ(:,1),'r-','LineWidth',1);hold on;
if have_opti_data
    plot(opti_time_s+16.33,opti_uav_pos(:,1),'b--','LineWidth',1);hold on;
end
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('X (m)')
title('Position Estimates');
if have_opti_data
    legend('PX4','offline','optitrack');
else
    legend('PX4','offline');%legend('boxoff');
end

%% and maybe more figure, all in the variable "log.data"
% figure,
subplot(3,1,2);
plot((vehicle_local_position(start:end,1))*1e-6, XYZ(start:end,2),'k:.','LineWidth',1);hold on;
plot(time*1e-6, ekf3_XYZ(:,2),'r--','LineWidth',1);hold on;
if have_opti_data
    plot(opti_time_s+16.33,opti_uav_pos(:,2),'b-','LineWidth',1);hold on;
end
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('Y (m)')
% title('Y');
if have_opti_data
    legend('PX4','offline','optitrack');
else
    legend('PX4','offline');%legend('boxoff');
end
%% and maybe more figure, all in the variable "log.data"
% figure,
subplot(3,1,3);
plot((vehicle_local_position(start:end,1))*1e-6, XYZ(start:end,3),'k:.','LineWidth',1);hold on;
plot(time*1e-6, ekf3_XYZ(:,3),'r--','LineWidth',1);hold on;
if have_opti_data
    plot(opti_time_s+16.33,opti_uav_pos(:,3),'b-','LineWidth',1);hold on;
end
grid on;
% axis([-inf inf -100 100]);
ylabel('Z (m)')
% title('Z');
if have_opti_data
    legend('PX4','offline','optitrack');
else
    legend('PX4','offline');%legend('boxoff');
end

%% 
% PlotToFileColorPDF(fig2,'../results/pos.pdf',15,20);% or 'pos.pdf'

%% and maybe more figure, all in the variable "log.data"
fig3=figure(3);
subplot(3,1,1);
plot((vehicle_local_position(start:end,1))*1e-6, V_xyz(start:end,1),'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_V_xyz(:,1),'r--','LineWidth',1);hold on;
% plot(time*1e-6, ekf3_V_xyz1(:,1),'b-','LineWidth',1);hold on;
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('V_x (m/s)')
title('velocity Estimates');
legend('PX4','offline');%legend('boxoff');

%% and maybe more figure, all in the variable "log.data"
% figure,
subplot(3,1,2);
plot((vehicle_local_position(start:end,1))*1e-6, V_xyz(start:end,2),'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_V_xyz(:,2),'r--','LineWidth',1);hold on;
% plot(time*1e-6, ekf3_V_xyz1(:,2),'b-','LineWidth',1);hold on;
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('V_y (m/s)')
% title('V_y');
legend('PX4','offline');%legend('boxoff');
%% and maybe more figure, all in the variable "log.data"
% figure,
subplot(3,1,3);
plot((vehicle_local_position(start:end,1))*1e-6, V_xyz(start:end,3),'k-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_V_xyz(:,3),'r--','LineWidth',1);hold on;
% plot(time*1e-6, ekf3_V_xyz1(:,3),'b-','LineWidth',1);hold on;
grid on;
% axis([-inf inf -100 100]);
xlabel({'Time(s)'});
ylabel('V_z (m/s)')
% title('V_z');
legend('PX4','offline');%legend('boxoff');

%% 
% PlotToFileColorPDF(fig3,'../results/vel.pdf',15,20);% or 'vel.pdf'

