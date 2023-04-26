clear all;
close all;
clc;
format long;
d2r=pi/180;
r2d=180/pi;
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
%%



%% convert data

ros_Roll=quat_to_roll(ros_q);
ros_Pitch=quat_to_pitch(ros_q);
ros_Yaw=quat_to_yaw(ros_q);
%% move origin of front-right-down to system init point
% opti_uav_pos=opti_uav_pos-opti_uav_pos(1,:);

%% Euler
figure,
subplot(3,1,1);
plot(time_ros(:,1),ros_Roll*r2d,'r-');hold on;
xlabel({'Time(s)'});
ylabel('Roll (deg)')
title('Euler Angle Estimates');
subplot(3,1,2);
plot(time_ros(:,1),ros_Pitch*r2d,'r-');hold on;
xlabel({'Time(s)'});
ylabel('Pitch (deg)')
subplot(3,1,3);
plot(time_ros(:,1),ros_Yaw*r2d,'r-');hold on;
xlabel({'Time(s)'});
ylabel('Yaw (deg)')
%% Position
figure,
subplot(3,1,1);
plot(time_ros(:,1),ros_xyz(:,1),'r-');hold on;
xlabel({'Time(s)'});
ylabel('X (m)')
title('Position Estimates');
subplot(3,1,2);
plot(time_ros(:,1),ros_xyz(:,2),'r-');hold on;
xlabel({'Time(s)'});
ylabel('Y (m)')
subplot(3,1,3);
plot(time_ros(:,1),ros_xyz(:,3),'r-');hold on;
xlabel({'Time(s)'});
ylabel('Z (m)')

% Not the true NED frame
save opti.mat
