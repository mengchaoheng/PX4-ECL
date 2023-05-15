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
ulgFileName = 'log_62_2023-4-9-13-57-58'; % the ulog file name  17_48_41
gps_enable=1;
mocap_enable=1;
flow_enable=1;
tmp1=[ ulgFileName, '_sensor_combined_0.csv'];
tmp2=[ ulgFileName, '_vehicle_air_data_0.csv'];
tmp3=[ ulgFileName, '_vehicle_gps_position_0.csv'];
tmp4=[ ulgFileName, '_vehicle_magnetometer_0.csv'];
tmp5=[ ulgFileName, '_vehicle_status_0.csv'];
tmp6=[ ulgFileName, '_vehicle_visual_odometry_0.csv'];
tmp7=[ ulgFileName, '_optical_flow_0.csv'];
% exist tmp var
have_data=exist(tmp1,"file") & exist(tmp2,"file") & exist(tmp3,"file") & exist(tmp4,"file") & exist(tmp5,"file");
% if ~have_data
    if ismac
        % on macOS, run " which ulog2csv " on terminal to get it.
        command = ['!/usr/local/bin/ulog2csv ' ulgFileName '.ulg']; % In xx/ulog2csv, xx is the path of python3 (3.9)
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
    % copy csv data to csv_data, and then delete them.
    movefile *_sensor_combined_0.csv ../csv_data/
    movefile *_vehicle_air_data_0.csv ../csv_data/
    if exist(tmp3,"file") && gps_enable
        movefile *_vehicle_gps_position_0.csv ../csv_data/
    end
    if exist(tmp6,"file") && mocap_enable
        movefile *_vehicle_visual_odometry_0.csv ../csv_data/
    end
    if exist(tmp7,"file") && flow_enable
        movefile *_optical_flow_0.csv ../csv_data/
    end
    movefile *_vehicle_magnetometer_0.csv ../csv_data/
    movefile *_vehicle_status_0.csv ../csv_data/
    
    delete(['*' ulgFileName '*.csv'])
    % or delete .mat
%     delete(['*' ulgFileName '*.mat'])
% end


% uint32 control_mode_flags	# Bitmask to indicate EKF logic state
% uint8 CS_TILT_ALIGN = 0		# 0 - true if the filter tilt alignment is complete
% uint8 CS_YAW_ALIGN = 1		# 1 - true if the filter yaw alignment is complete
% uint8 CS_GPS = 2		# 2 - true if GPS measurements are being fused
% uint8 CS_OPT_FLOW = 3		# 3 - true if optical flow measurements are being fused
control_mode_flags=dec2bin(log.data.estimator_status_0.control_mode_flags)

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
innovation_check_flags=dec2bin(log.data.estimator_status_0.innovation_check_flags)
