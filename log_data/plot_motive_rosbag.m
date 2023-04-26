clear all;
close all;
clc;
format long;
d2r=pi/180;
r2d=180/pi;
opti = importdata('Take 2023-04-25 09.47.11 PM.csv');
ros = importdata('ros.csv');
len_ros=length(ros.textdata);
time_ros=zeros(len_ros-1,2);
time_ros_start(1,1)=str2double(ros.textdata{2,1});
time_ros_start(1,2)=str2double(ros.textdata{2,3});
for i=2:len_ros
    time_ros(i-1,1)=str2double(ros.textdata{i,1})-time_ros_start(1,1);%time
    time_ros(i-1,2)=str2double(ros.textdata{i,3})-time_ros_start(1,2);%stamp
end
time_ros=time_ros*1e-9;

enable_enu=0;
if ~enable_enu
    %% NED. data allocation, convert x-y-z to front-right-down and redefinite q=[w,x,y,z]
    opti_time_s=opti.data(:,2);
    opti_origin_pos=[  opti.data(:,8) opti.data(:,7) -opti.data(:,9)]/1000; % (mm) to (m)
    opti_origin_q=[opti.data(:,6)   opti.data(:,4) opti.data(:,3) -opti.data(:,5)];
    % convert XYZ and XYZW of RigidBody
    opti_uav_pos=[ opti.data(:,15) opti.data(:,14) -opti.data(:,16)]/1000; % X=opti.data(:,14), Y=opti.data(:,15), Z=opti.data(:,16)
    opti_uav_q=[opti.data(:,13) opti.data(:,11) opti.data(:,10)  -opti.data(:,12)]; % X=opti.data(:,10), Y=opti.data(:,11), Z=opti.data(:,12) W=opti.data(:,13)
    ros_xyz=[ros.data(:,2) ros.data(:,1) -ros.data(:,3)];
    ros_q=[ros.data(:,7) ros.data(:,5) ros.data(:,4) -ros.data(:,6)];
else
    %% ENU. origin, the same with ros
    opti_time_s=opti.data(:,2);
    opti_origin_pos=[  opti.data(:,7) opti.data(:,8) opti.data(:,9)]/1000; % (mm) to (m)
    opti_origin_q=[opti.data(:,6)   opti.data(:,3) opti.data(:,4) opti.data(:,5)];
    opti_uav_pos=[ opti.data(:,14) opti.data(:,15) opti.data(:,16)]/1000; % X=opti.data(:,14), Y=opti.data(:,15), Z=opti.data(:,16)
    opti_uav_q=[opti.data(:,13) opti.data(:,10) opti.data(:,11)  opti.data(:,12)]; % X=opti.data(:,10), Y=opti.data(:,11), Z=opti.data(:,12) W=opti.data(:,13)
    ros_xyz=ros.data(:,1:3);
    ros_q=[ros.data(:,7) ros.data(:,4:6)];
end
%%



%% convert data
% opti_origin_Roll=quat_to_roll(opti_origin_q);
% opti_origin_Pitch=quat_to_pitch(opti_origin_q);
% opti_origin_Yaw=quat_to_yaw(opti_origin_q);
opti_uav_Roll=quat_to_roll(opti_uav_q);
opti_uav_Pitch=quat_to_pitch(opti_uav_q);
opti_uav_Yaw=quat_to_yaw(opti_uav_q);
ros_Roll=quat_to_roll(ros_q);
ros_Pitch=quat_to_pitch(ros_q);
ros_Yaw=quat_to_yaw(ros_q);
%% move origin of front-right-down to system init point
% opti_uav_pos=opti_uav_pos-opti_uav_pos(1,:);

%% Euler
figure,
subplot(3,1,1);
plot(time_ros(:,1),ros_Roll*r2d,'r-');hold on;
plot(opti_time_s+5.58,opti_uav_Roll*r2d,'k-');grid on;
xlabel({'Time(s)'});
ylabel('Roll (deg)')
title('Euler Angle Estimates');
subplot(3,1,2);
plot(time_ros(:,1),ros_Pitch*r2d,'r-');hold on;
plot(opti_time_s+5.58,opti_uav_Pitch*r2d,'b-');grid on;
xlabel({'Time(s)'});
ylabel('Pitch (deg)')
subplot(3,1,3);
plot(time_ros(:,1),ros_Yaw*r2d,'r-');hold on;
plot(opti_time_s+5.58,opti_uav_Yaw*r2d,'r-');grid on;
xlabel({'Time(s)'});
ylabel('Yaw (deg)')
%% Position
figure,
subplot(3,1,1);
plot(time_ros(:,1),ros_xyz(:,1),'r-');hold on;
plot(opti_time_s+5.58,opti_uav_pos(:,1),'k-');grid on;
xlabel({'Time(s)'});
ylabel('X (m)')
title('Position Estimates');
subplot(3,1,2);
plot(time_ros(:,1),ros_xyz(:,2),'r-');hold on;
plot(opti_time_s+5.58,opti_uav_pos(:,2),'b-');grid on;
xlabel({'Time(s)'});
ylabel('Y (m)')
subplot(3,1,3);
plot(time_ros(:,1),ros_xyz(:,3),'r-');hold on;
plot(opti_time_s+5.58,opti_uav_pos(:,3),'r-');grid on;
xlabel({'Time(s)'});
ylabel('Z (m)')

% Not the true NED frame
save opti.mat
