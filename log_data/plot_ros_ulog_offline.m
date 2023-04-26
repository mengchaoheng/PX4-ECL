clear all;
close all;
clc;
% format long;
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
%% ulog data. two offline can be use, something different. px4 v12.3.0
ulgFileName = 'log15-20'; % the ulog file name  17_48_41, log15-20(12.3) log36, log_0_2023-4-26-19-52-20, log_6_2023-4-26-17-34-32
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


%% ros data
ros = importdata('ros15-20.csv');% ros15-20(12.3) ros19-51, ros19-51, ros, ros15-20
len_ros=length(ros.textdata);
time_ros=zeros(len_ros-1,2);
time_ros_start(1,1)=str2double(ros.textdata{2,1});
time_ros_start(1,2)=str2double(ros.textdata{2,3});
for i=2:len_ros
    time_ros(i-1,1)=str2double(ros.textdata{i,1})-time_ros_start(1,1);%time
    time_ros(i-1,2)=str2double(ros.textdata{i,3})-time_ros_start(1,2);%stamp
end
time_ros=time_ros*1e-9;
enable_enu=1;
if ~enable_enu
    %% NED. data allocation, convert x-y-z to front-right-down and redefinite q=[w,x,y,z]    
    % convert XYZ and XYZW of RigidBody
    ros_xyz=[ros.data(:,2) ros.data(:,1) -ros.data(:,3)];
    ros_q=[ros.data(:,7) ros.data(:,5) ros.data(:,4) -ros.data(:,6)];
else
    %% ENU. origin, the same with ros
    ros_xyz=ros.data(:,1:3);
    ros_q=[ros.data(:,7) ros.data(:,4:6)];
end
%% convert data
% Transform the attitude representation from frame to frame.
% The proof for this transform can be seen
% http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/

M_PI=pi;
M_PI_2=pi/2;
% Static quaternion needed for rotating between aircraft and base_link frames
% +PI rotation around X (Forward) axis transforms from Forward, Right, Down (aircraft)
% to Forward, Left, Up (base_link) frames.
% AIRCRAFT_BASELINK_Q = quaternion_from_rpy(M_PI, 0.0, 0.0);
AIRCRAFT_BASELINK_Q=eul2quat([0 0 M_PI],"ZYX");
% Transform from attitude represented WRT baselink frame to attitude represented WRT body frame
% change from expressed WRT baselnk to WRT aircraft
% q=q * AIRCRAFT_BASELINK_Q;
temp=quatmultiply(ros_q,AIRCRAFT_BASELINK_Q);

% Static quaternion needed for rotating between ENU and NED frames
% NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
% ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
% NED_ENU_Q = quaternion_from_rpy(M_PI, 0.0, M_PI_2);
NED_ENU_Q=eul2quat([M_PI_2 0 M_PI],"ZYX");
% change from expressed WRT ENU frame to WRT NED frame
% Transform from attitude represented WRT ENU frame to attitude represented WRT NED frame
% q=NED_ENU_Q * q;
ros_q=quatmultiply(NED_ENU_Q,temp);

% Use reflections instead of rotations for NED <-> ENU transformation
% to avoid NaN/Inf floating point pollution across different axes
% since in NED <-> ENU the axes are perfectly aligned.
NED_ENU_REFLECTION_XY=[0 1 0;1 0 0;0 0 1];
NED_ENU_REFLECTION_Z=diag([1 1 -1]);
% Transform data expressed in ENU to NED frame.
temp = (NED_ENU_REFLECTION_Z * ros_xyz');
ros_xyz=(NED_ENU_REFLECTION_XY * temp)';

%%
ros_Roll=quat_to_roll(ros_q);
ros_Pitch=quat_to_pitch(ros_q);
ros_Yaw=quat_to_yaw(ros_q);

%%

ECL = importdata('../results/ecloutput.csv');%https://blog.csdn.net/JingpengSun/article/details/128840264
time=ECL.data(:,1);
ekf3_Roll=ECL.data(:,26);
ekf3_Pitch=ECL.data(:,27);
ekf3_Yaw=ECL.data(:,28);
ekf3_XYZ = ECL.data(:,9:11);
ekf3_V_xyz = ECL.data(:,6:8);


%% Euler
figure,
subplot(3,1,1);
plot(time_ros(:,1)+117,ros_Roll*r2d,'r-');hold on;
plot((vehicle_attitude(start:end,1))*1e-6, Roll(start:end)*r2d,'k--','LineWidth',1);hold on;
plot((visual_time(start:end,1))*1e-6, visual_Roll(start:end)*r2d,'b-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_Roll*r2d,'g-','LineWidth',1);hold on;
grid on;
xlabel({'Time(s)'});
ylabel('Roll (deg)')
title('Euler Angle Estimates');
legend('ros','px4','visual','ekf3');
subplot(3,1,2);
plot(time_ros(:,1)+117,ros_Pitch*r2d,'r-');hold on;
plot((vehicle_attitude(start:end,1))*1e-6, Pitch(start:end)*r2d,'k--','LineWidth',1);hold on;
plot((visual_time(start:end,1))*1e-6, visual_Pitch(start:end)*r2d,'b-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_Pitch*r2d,'g-','LineWidth',1);hold on;
grid on;
xlabel({'Time(s)'});
ylabel('Pitch (deg)')
subplot(3,1,3);
plot(time_ros(:,1)+117,ros_Yaw*r2d,'r-');hold on;
plot((vehicle_attitude(start:end,1))*1e-6, Yaw(start:end)*r2d,'k--','LineWidth',1);hold on;
plot((visual_time(start:end,1))*1e-6, visual_Yaw(start:end)*r2d,'b-','LineWidth',1);hold on;
plot(time*1e-6, ekf3_Yaw*r2d,'g-','LineWidth',1);hold on;
grid on;
xlabel({'Time(s)'});
ylabel('Yaw (deg)')
%% Position
figure,
subplot(3,1,1);
plot(time_ros(:,1)+117,ros_xyz(:,1),'r-');hold on;
plot((vehicle_local_position(start:end,1))*1e-6, XYZ(start:end,1),'k-','LineWidth',1);hold on;
plot(visual_time*1e-6,visual_XYZ(:,1),'b-');hold on;
plot(time*1e-6, ekf3_XYZ(:,1),'g-','LineWidth',1);hold on;
axis([-inf inf -2.2 -1.3]);
xlabel({'Time(s)'});
ylabel('X (m)')
legend('ros','px4','visual','ekf3');
title('Position Estimates');
subplot(3,1,2);
plot(time_ros(:,1)+117,ros_xyz(:,2),'r-');hold on;
plot((vehicle_local_position(start:end,1))*1e-6, XYZ(start:end,2),'r-','LineWidth',1);hold on;
plot(visual_time*1e-6,visual_XYZ(:,2),'b-');hold on;
plot(time*1e-6, ekf3_XYZ(:,2),'g--','LineWidth',1);hold on;
axis([-inf inf 1 2]);
xlabel({'Time(s)'});
ylabel('Y (m)')
subplot(3,1,3);
plot(time_ros(:,1)+117,ros_xyz(:,3),'r-');hold on;
plot((vehicle_local_position(start:end,1))*1e-6, XYZ(start:end,3),'b-','LineWidth',1);hold on;
plot(visual_time*1e-6,visual_XYZ(:,3),'b-');hold on;
plot(time*1e-6, ekf3_XYZ(:,3),'g--','LineWidth',1);hold on;
axis([-inf inf -2.5 1]);
xlabel({'Time(s)'});
ylabel('Z (m)')
% 
% % Not the true NED frame
% save opti.mat
