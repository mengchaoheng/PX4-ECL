#include "PostEkf.h"
#include <sstream>
#include <fstream>
#include <time.h>
#include <stdio.h>
#include<iostream>
using math::constrain;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;
bool bReadGPS=true, bmagread=true, bReadBaro=true, bReadStatus=true;
PostEkf::PostEkf(std::string filename, std::string mag_name,std::string baro_name,std::string gps_name, std::string status_name )
:_file_name(filename),_mag_name(mag_name),_baro_name(baro_name),_gps_name(gps_name),_status_name(status_name)
{
    csv_mag = CsvParser_new(_mag_name.c_str(), ",", 1);
    csv_imu = CsvParser_new(_file_name.c_str(), ",", 1);
    csv_baro = CsvParser_new(_baro_name.c_str(), ",", 1);
    csv_gps = CsvParser_new(_gps_name.c_str(), ",", 1); 
    csv_status = CsvParser_new(_status_name.c_str(), ",", 1);    
    _fp_out = fopen("../results/ecloutput.csv", "w");
    if(_fp_out==NULL) _fp_out = stdout;
}

PostEkf::~PostEkf()
{
}

void PostEkf::write_header()
{
    // write csv log head
    fprintf(_fp_out,"time_us,q0,q1,q2,q3,vn_m_s,ve_m_s,vd_m_s,n_m,e_m,d_m,\
delta_ang_bias_0,delta_ang_bias_1,delta_ang_bias_2,\
delta_vel_bias_0,delta_vel_bias_1,delta_vel_bias_2,\
mag_I_0,mag_I_1,mag_I_2,\
mag_B_0,mag_B_1,mag_B_2,\
wind_vel_0,wind_vel_1,\
roll_rad,pitch_rad,yaw_rad\n");

// fprintf(_fp_out,"q0,q1,q2,q3,vn_m_s,ve_m_s,vd_m_s,n_m,e_m,d_m, delta_ang_bias_0,delta_ang_bias_1,delta_ang_bias_2,delta_vel_bias_0,delta_vel_bias_1,delta_vel_bias_2, mag_I_0,mag_I_1,mag_I_2,mag_B_0,mag_B_1,mag_B_2, wind_vel_0,wind_vel_1 \n");

// fprintf(_fp_out,"q0,q1,q2,q3 \n");
}
std::ofstream output("../results/output.txt");
std::ofstream position_estimator("../results/position_estimator.txt");
std::ofstream velocity_estimator("../results/velocity_estimator.txt");
std::ofstream euler_estimator("../results/euler_estimator.txt");
void PostEkf::update()
{
    write_header();
    // read imu
    while ((imu_row = CsvParser_getRow(csv_imu)))
    {
        // restore data update flag
        mag_updated = false;
        baro_updated = false;
        gps_updated = false;
        status_updated = false;

        // begin follow with PX4
        bool imu_updated = false;
        imuSample imu_sample_new {};

        uint64_t imu_dt = 0; // for tracking time slip later

        // begin read imu data, like _sensor_combined_sub.update(&sensor_combined);
        char** rowFields = CsvParser_getFields(imu_row);
        int field_count = CsvParser_getNumFields(imu_row);
        if (IMU_FIELD_COUNT != field_count) 
        {
            printf("skip line, cause imu col count %d!=%d", field_count, IMU_FIELD_COUNT);
            continue;
        }
        else
        {
            /* imu have been update */
            imu_updated = true;
            receive_imu((const char**)rowFields);
        }
        CsvParser_destroy_row(imu_row);
        // end to read.

        // now we get a line imu data, and then following the px4 code.
        imu_sample_new.time_us = sensor_combined.timestamp;
        imu_sample_new.delta_ang_dt = sensor_combined.gyro_integral_dt * 1.e-6f;
        imu_sample_new.delta_ang = Vector3f{sensor_combined.gyro_rad} * imu_sample_new.delta_ang_dt;
        imu_sample_new.delta_vel_dt = sensor_combined.accelerometer_integral_dt * 1.e-6f;
        imu_sample_new.delta_vel = Vector3f{sensor_combined.accelerometer_m_s2} * imu_sample_new.delta_vel_dt;

        if (sensor_combined.accelerometer_clipping > 0) {
            imu_sample_new.delta_vel_clipping[0] = sensor_combined.accelerometer_clipping & sensor_combined.CLIPPING_X;
            imu_sample_new.delta_vel_clipping[1] = sensor_combined.accelerometer_clipping & sensor_combined.CLIPPING_Y;
            imu_sample_new.delta_vel_clipping[2] = sensor_combined.accelerometer_clipping & sensor_combined.CLIPPING_Z;
        }

        imu_dt = sensor_combined.gyro_integral_dt;

        // remove the sensor change support
        // if (_sensor_selection_sub.updated() || (_device_id_accel == 0 || _device_id_gyro == 0))

        if (imu_updated) {
            const hrt_abstime now = imu_sample_new.time_us;
            // push imu data into estimator
		    _ekf.setIMUData(imu_sample_new);
            PublishAttitude(now); // publish attitude immediately (uses quaternion from output predictor)

            // integrate time to monitor time slippage
            if (_start_time_us > 0) {
                _integrated_time_us += imu_dt;
                _last_time_slip_us = (imu_sample_new.time_us - _start_time_us) - _integrated_time_us;

            } else {
                _start_time_us = imu_sample_new.time_us;
                _last_time_slip_us = 0;
            }

            // update all other topics if they have new data
            // if (_status_sub.updated())...
            // instead 
            // UpdateVehicleStatusSample();

            // if (_vehicle_land_detected_sub.updated())

            UpdateBaroSample(); // Baro
            UpdateGpsSample(); // Gps
            UpdateMagSample(); // Mag

            if (_ekf.update()) {
                PublishLocalPosition(now);
                // publish something else...

                // log ekf
                output_csv(now);
                // or log to output.txt by iostream.
                matrix::Vector<float, 24> states = _ekf.getStateAtFusionHorizonAsVector();
                output<< states(0)<<" "<<states(1)<<" "<<states(2)<<" "<<states(3)<<" "<<states(4)<<" "<<states(5)<<" "<<states(6)<<" "<<states(7)<<" "<<states(8)<<" "<<states(9)<<" "<<states(10)<<" "<<states(11)<<" "<<states(12)<<" "<<states(13)<<" "<<states(14)<<" "<<states(15)<<" "<<states(16)<<" "<<states(17)<<" "<<states(18)<<" "<<states(19)<<" "<<states(20)<<" "<<states(21)<<" "<<states(22)<<" "<<states(23) <<" "<<std::endl;

                UpdateMagCalibration(now);
            }
        }
    }
    // now is at the end of px4 code.

    // free memory
    CsvParser_destroy(csv_imu);
    CsvParser_destroy(csv_mag);
    CsvParser_destroy(csv_baro);
    CsvParser_destroy(csv_status);
}

void PostEkf::output_csv(const hrt_abstime &timestamp)
{
    matrix::Quatf q = _ekf.getQuaternion();

    Eulerf att = Eulerf(q);

    matrix::Vector<float, 24> states = _ekf.getStateAtFusionHorizonAsVector();
    // fprintf(_fp_out,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n", states(0),states(1),states(2),states(3),states(4),states(5),states(6),states(7),states(8),states(9),states(10),states(11),states(12),states(13),states(14),states(15),states(16),states(17),states(18),states(19),states(20),states(21),states(22),states(23)); //txt
    
    fprintf(_fp_out,"%llu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", timestamp, states(0),states(1),states(2),states(3),states(4),states(5),states(6),states(7),states(8),states(9),states(10),states(11),states(12),states(13),states(14),states(15),states(16),states(17),states(18),states(19),states(20),states(21),states(22),states(23),att(0), att(1), att(2)); //csv
    // printf("att: %f,%f,%f\n", att(0), att(1), att(2));
}


void PostEkf::receive_imu(const char** row_fields)
{
    
    sensor_combined.timestamp   = atoi(row_fields[TIMESTAMP]);

    sensor_combined.gyro_rad[0] = atof(row_fields[GYRO_RAD_X]);
    sensor_combined.gyro_rad[1] = atof(row_fields[GYRO_RAD_Y]);
    sensor_combined.gyro_rad[2] = atof(row_fields[GYRO_RAD_Z]);

    sensor_combined.gyro_integral_dt  = atoi(row_fields[GYRO_INTEGRAL_DT]);
    int32_t magnetometer_timestamp_relative = atoi(row_fields[ACCELEROMETER_TIMESTAMP_RELATIVE]);

    sensor_combined.accelerometer_m_s2[0] = atof(row_fields[ACCELEROMETER_M_S2_X]);
    sensor_combined.accelerometer_m_s2[1] = atof(row_fields[ACCELEROMETER_M_S2_Y]);
    sensor_combined.accelerometer_m_s2[2] = atof(row_fields[ACCELEROMETER_M_S2_Z]);

    sensor_combined.accelerometer_integral_dt = atoi(row_fields[ACCELEROMETER_INTEGRAL_DT]);
    sensor_combined.accelerometer_clipping = atoi(row_fields[ACC_CLIP]);
    // printf("[sensor_combined]:time %llu, g1 %f, g2 %f, g3 %f, dt %u,  a1 %f, a2 %f, a3 %f, dt %u , clip %d \n", sensor_combined.timestamp, sensor_combined.gyro_rad[0], sensor_combined.gyro_rad[1], sensor_combined.gyro_rad[2], sensor_combined.gyro_integral_dt, sensor_combined.accelerometer_m_s2[0], sensor_combined.accelerometer_m_s2[1], sensor_combined.accelerometer_m_s2[2], sensor_combined.accelerometer_integral_dt,sensor_combined.accelerometer_clipping);
}
void PostEkf::receive_mag(const char** row_fields)
{
    
    magnetometer.timestamp   = atoi(row_fields[TIMESTAMP]);
    magnetometer.timestamp_sample   = atoi(row_fields[TIMESTAMP_SAMPLE]);
    magnetometer.device_id   = atoi(row_fields[DEVICE_ID]);
    magnetometer.magnetometer_ga[0] = atof(row_fields[MAGNETOMETER_GA_X]);
    magnetometer.magnetometer_ga[1] = atof(row_fields[MAGNETOMETER_GA_Y]);
    magnetometer.magnetometer_ga[2] = atof(row_fields[MAGNETOMETER_GA_Z]);
    magnetometer.calibration_count   = atoi(row_fields[CALIBRATION_COUNT]);
    // printf("[magnetometer]:time %llu, m1 %f, m2 %f, m3 %f \n", magnetometer.timestamp, magnetometer.magnetometer_ga[0], magnetometer.magnetometer_ga[1], magnetometer.magnetometer_ga[2]);
}
void PostEkf::receive_baro(const char** row_fields)
{
    
    airdata.timestamp   = atoi(row_fields[TIMESTAMP]);
    airdata.timestamp_sample   = atoi(row_fields[TIMESTAMP_SAMPLE]);
    airdata.baro_device_id   = atoi(row_fields[DEVICE_ID]);

    airdata.baro_alt_meter = atof(row_fields[BARO_ALT_METER]);
    airdata.baro_temp_celcius = atof(row_fields[BARO_TEMP_CELCIUS]);
    airdata.baro_pressure_pa = atof(row_fields[BARO_PRESSURE_PA]);
    airdata.rho = atof(row_fields[RHO]);
    // printf("[airdata]:time %llu, baro_alt_meter %f, baro_temp_celcius %f, baro_pressure_pa %f rho %f \n", airdata.timestamp, airdata.baro_alt_meter, airdata.baro_temp_celcius, airdata.baro_pressure_pa,airdata.rho);

}
void PostEkf::receive_gps(const char** row_fields)
{
    
    vehicle_gps_position.timestamp   = atoi(row_fields[TIMESTAMP]);
    vehicle_gps_position.time_utc_usec   = atoi(row_fields[TIME_UTC_USEC]);
    vehicle_gps_position.lat   = atoi(row_fields[LAT]);
    vehicle_gps_position.lon   = atoi(row_fields[LON]);
    vehicle_gps_position.alt   = atoi(row_fields[ALT]);

    vehicle_gps_position.alt_ellipsoid   = atoi(row_fields[ALT_ELLIPSOID]);
    vehicle_gps_position.s_variance_m_s = atof(row_fields[S_VARIANCE_M_S]);
    vehicle_gps_position.c_variance_rad = atof(row_fields[C_VARIANCE_RAD]);
    vehicle_gps_position.eph = atof(row_fields[EPH]);
    vehicle_gps_position.epv = atof(row_fields[EPV]);
    vehicle_gps_position.hdop = atof(row_fields[HDOP]);
    vehicle_gps_position.vdop = atof(row_fields[VDOP]);
    vehicle_gps_position.noise_per_ms   = atoi(row_fields[NOISE_PER_MS]);
    vehicle_gps_position.jamming_indicator   = atoi(row_fields[JAMMING_INDICATOR]);

    vehicle_gps_position.vel_m_s = atof(row_fields[VEL_M_S]);
    vehicle_gps_position.vel_n_m_s = atof(row_fields[VEL_N_M_S]);
    vehicle_gps_position.vel_e_m_s = atof(row_fields[VEL_E_M_S]);
    vehicle_gps_position.vel_d_m_s = atof(row_fields[VEL_D_M_S]);
    vehicle_gps_position.cog_rad = atof(row_fields[COG_RAD]);
    vehicle_gps_position.timestamp_time_relative   = atoi(row_fields[TIMESTAMP_TIMESTAMP_RELATIVE]);

    vehicle_gps_position.heading =NAN;
    vehicle_gps_position.heading_offset =NAN;

    vehicle_gps_position.fix_type   = atoi(row_fields[FIX_TYPE]);

    vehicle_gps_position.jamming_state   = atoi(row_fields[JAMMING_STATE]);


    vehicle_gps_position.vel_ned_valid = (bool) atoi(row_fields[VEL_NED_VALID]);
    vehicle_gps_position.satellites_used   = atoi(row_fields[SATELLITES_USED]);

    vehicle_gps_position.selected   = atoi(row_fields[SELECTED]);

    // printf("[vehicle_gps_position]: time %llu, time_utc_usec %llu, lat %d, lon %d, alt %d, alt_ellipsoid %d, s_variance_m_s %f, c_variance_rad %f, eph %f, epv %f, hdop %f, vdop %f, noise_per_ms %d, jamming_indicator %d, vel_m_s %f, v1 %f, v2 %f, v3 %f, cog_rad %f, timestamp_time_relative %d, fix_type %d, vel_ned_valid %d, satellites_used %d\n", vehicle_gps_position.timestamp, vehicle_gps_position.time_utc_usec, vehicle_gps_position.lat, vehicle_gps_position.lon, vehicle_gps_position.alt, vehicle_gps_position.alt_ellipsoid, vehicle_gps_position.s_variance_m_s, vehicle_gps_position.c_variance_rad, vehicle_gps_position.eph, vehicle_gps_position.epv, vehicle_gps_position.hdop, vehicle_gps_position.vdop, vehicle_gps_position.noise_per_ms, vehicle_gps_position.jamming_indicator, vehicle_gps_position.vel_m_s, vehicle_gps_position.vel_n_m_s, vehicle_gps_position.vel_e_m_s, vehicle_gps_position.vel_d_m_s, vehicle_gps_position.cog_rad, vehicle_gps_position.timestamp_time_relative, vehicle_gps_position.fix_type, vehicle_gps_position.vel_ned_valid, vehicle_gps_position.satellites_used);

}
void PostEkf::receive_status(const char** row_fields)
{
    vehicle_status.timestamp = atoi(row_fields[0]);
    vehicle_status.nav_state_timestamp = atoi(row_fields[1]);
    vehicle_status.failsafe_timestamp = atoi(row_fields[2]);
    vehicle_status.armed_time = atoi(row_fields[3]);
    vehicle_status.takeoff_time = atoi(row_fields[4]);
    vehicle_status.onboard_control_sensors_present = atoi(row_fields[5]);
    vehicle_status.onboard_control_sensors_enabled = atoi(row_fields[6]);
    vehicle_status.onboard_control_sensors_health = atoi(row_fields[7]);
    vehicle_status.nav_state = atoi(row_fields[8]);
    vehicle_status.arming_state = atoi(row_fields[9]);
    vehicle_status.hil_state = atoi(row_fields[10]);

    vehicle_status.failsafe =  (bool)  atoi(row_fields[11]);
    vehicle_status.system_type = atoi(row_fields[12]);
    vehicle_status.system_id = atoi(row_fields[13]);
    vehicle_status.component_id = atoi(row_fields[14]);
    vehicle_status.vehicle_type = atoi(row_fields[15]);

    vehicle_status.is_vtol =  (bool)  atoi(row_fields[16]);
    vehicle_status.is_vtol_tailsitter =  (bool)  atoi(row_fields[17]);
    vehicle_status.vtol_fw_permanent_stab =  (bool)  atoi(row_fields[18]);
    vehicle_status.in_transition_mode =  (bool)  atoi(row_fields[19]);
    vehicle_status.in_transition_to_fw =  (bool)  atoi(row_fields[20]);
    vehicle_status.rc_signal_lost =  (bool)  atoi(row_fields[21]);
    vehicle_status.rc_input_mode = atoi(row_fields[22]);

    vehicle_status.data_link_lost =  (bool)  atoi(row_fields[23]);
    vehicle_status.data_link_lost_counter = atoi(row_fields[24]);
    vehicle_status.high_latency_data_link_lost =  (bool)  atoi(row_fields[25]);
    vehicle_status.engine_failure =  (bool)  atoi(row_fields[26]);
    vehicle_status.mission_failure =  (bool)  atoi(row_fields[27]);
    vehicle_status.geofence_violated =  (bool)  atoi(row_fields[28]);

    vehicle_status.failure_detector_status = atoi(row_fields[29]);
    vehicle_status.latest_arming_reason = atoi(row_fields[30]);
    vehicle_status.latest_disarming_reason = atoi(row_fields[31]);

}


void PostEkf::UpdateVehicleStatusSample()
{
    //read status
    if(bReadStatus && (status_row = CsvParser_getRow(csv_status)))
    {
        // read status data
        char** rowFields = CsvParser_getFields(status_row);
        int field_count = CsvParser_getNumFields(status_row);
        if (STATUS_FIELD_COUNT != field_count) 
        {
            printf("skip line, cause mag col count %d!=%d", field_count, STATUS_FIELD_COUNT);
            return;
        }
        else
        {
            receive_status((const char**)status_row);
        }
        
        CsvParser_destroy_row(status_row);

        bReadStatus= false;
    }
    if(vehicle_status.timestamp <sensor_combined.timestamp)
    {
        status_updated = true;
        bReadStatus = true;
    }
    if(status_updated)
    {
        // update all other topics if they have new data
        const bool is_fixed_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

        // only fuse synthetic sideslip measurements if conditions are met
        _ekf.set_fuse_beta_flag(is_fixed_wing && (_param_ekf3_fuse_beta == 1));

        // let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
        _ekf.set_is_fixed_wing(is_fixed_wing);

        // _preflt_checker.setVehicleCanObserveHeadingInFlight(vehicle_status.vehicle_type !=
        //         vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

        _armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

        // update standby (arming state) flag
        // const bool standby = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY);

        // if (_standby != standby) {
        //     _standby = standby;

        //     // reset preflight checks if transitioning in or out of standby arming state
        //     _preflt_checker.reset();
        // }
    }
}

void PostEkf::UpdateBaroSample()
{
    //read baro
    if(bReadBaro && (baro_row = CsvParser_getRow(csv_baro)))
    {
        // read baro data
        char** rowFields = CsvParser_getFields(baro_row);
        int field_count = CsvParser_getNumFields(baro_row);
        if (BARO_FIELD_COUNT != field_count) 
        {
            printf("skip line, cause mag col count %d!=%d", field_count, BARO_FIELD_COUNT);
            return;
        }
        else{
            receive_baro((const char**)rowFields);
        }
        CsvParser_destroy_row(baro_row);

        bReadBaro= false;
    }
    if(airdata.timestamp <sensor_combined.timestamp)
    {
        baro_updated = true;
        bReadBaro = true;
    }
    if(baro_updated)
    {
        // set baro data to ekf, follow px4
        _ekf.set_air_density(airdata.rho);

		_ekf.setBaroData(baroSample{airdata.timestamp_sample, airdata.baro_alt_meter});

		// PX4_INFO("New airdata.baro_alt_meter: %f\n", static_cast<double>(airdata.baro_alt_meter));
		last_airdata.rho=airdata.rho;
		last_airdata.baro_alt_meter=airdata.baro_alt_meter;

		_device_id_baro = airdata.baro_device_id;
    }
}

void PostEkf::UpdateGpsSample()
{
    // read gps
    if(bReadGPS && (gps_row = CsvParser_getRow(csv_gps))) 
    {
        // read gps data
        char** rowFields = CsvParser_getFields(gps_row);
        int field_count = CsvParser_getNumFields(gps_row);
        if (GPS_FIELD_COUNT != field_count) 
        {
            printf("skip line, cause mag col count %d!=%d", field_count, GPS_FIELD_COUNT);
            return;
        }
        else{
            receive_gps((const char**)rowFields);
        }
        CsvParser_destroy_row(gps_row);

        bReadGPS = false;
    }
    if(vehicle_gps_position.timestamp  < sensor_combined.timestamp)
    {
        gps_updated = true;
        bReadGPS = true;
    }
    if(gps_updated)
    {
        // set gps data to ekf
        gps_message gps_msg{
			.time_usec = vehicle_gps_position.timestamp,
			.lat = vehicle_gps_position.lat,
			.lon = vehicle_gps_position.lon,
			.alt = vehicle_gps_position.alt,
			.yaw = vehicle_gps_position.heading,
			.yaw_offset = vehicle_gps_position.heading_offset,
			.fix_type = vehicle_gps_position.fix_type,
			.eph = vehicle_gps_position.eph,
			.epv = vehicle_gps_position.epv,
			.sacc = vehicle_gps_position.s_variance_m_s,
			.vel_m_s = vehicle_gps_position.vel_m_s,
			.vel_ned = Vector3f{
				vehicle_gps_position.vel_n_m_s,
				vehicle_gps_position.vel_e_m_s,
				vehicle_gps_position.vel_d_m_s
			},
			.vel_ned_valid = vehicle_gps_position.vel_ned_valid,
			.nsats = vehicle_gps_position.satellites_used,
			.pdop = sqrtf(vehicle_gps_position.hdop *vehicle_gps_position.hdop
				      + vehicle_gps_position.vdop * vehicle_gps_position.vdop),
		};
		_ekf.setGpsData(gps_msg);

		_gps_time_usec = gps_msg.time_usec;
		_gps_alttitude_ellipsoid = vehicle_gps_position.alt_ellipsoid;

		// save to last
		last_vehicle_gps_position.timestamp = vehicle_gps_position.timestamp;
		last_vehicle_gps_position.lat = vehicle_gps_position.lat;
		last_vehicle_gps_position.lon = vehicle_gps_position.lon;
		last_vehicle_gps_position.alt = vehicle_gps_position.alt;
		last_vehicle_gps_position.heading = vehicle_gps_position.heading;
		last_vehicle_gps_position.heading_offset = vehicle_gps_position.heading_offset;
		last_vehicle_gps_position.fix_type = vehicle_gps_position.fix_type;
		last_vehicle_gps_position.eph = vehicle_gps_position.eph;
		last_vehicle_gps_position.epv = vehicle_gps_position.epv;
		last_vehicle_gps_position.s_variance_m_s = vehicle_gps_position.s_variance_m_s;
		last_vehicle_gps_position.vel_m_s = vehicle_gps_position.vel_m_s;
		last_vehicle_gps_position.vel_n_m_s = vehicle_gps_position.vel_n_m_s;
		last_vehicle_gps_position.vel_e_m_s = vehicle_gps_position.vel_e_m_s;
		last_vehicle_gps_position.vel_d_m_s = vehicle_gps_position.vel_d_m_s;
		last_vehicle_gps_position.vel_ned_valid = vehicle_gps_position.vel_ned_valid;
		last_vehicle_gps_position.satellites_used = vehicle_gps_position.satellites_used;
		last_vehicle_gps_position.hdop = vehicle_gps_position.hdop;
		last_vehicle_gps_position.vdop = vehicle_gps_position.vdop;
    }
}

void PostEkf::UpdateMagSample()
{
    //read mag
    if(bmagread && (mag_row = CsvParser_getRow(csv_mag)))
    {
        //read mag data
        char** rowFields = CsvParser_getFields(mag_row);
        int field_count = CsvParser_getNumFields(mag_row);
        if (MAG_FIELD_COUNT != field_count) 
        {
        printf("skip line, cause mag col count %d!=%d", field_count, MAG_FIELD_COUNT);
        return;
        }
        else{
            receive_mag((const char**)rowFields);
        }
        CsvParser_destroy_row(mag_row);

        bmagread = false;
    }
    // for replay, the timestamp of imu, that is sensor_combined.timestamp, can be used to the "time" stream.
    if(magnetometer.timestamp < sensor_combined.timestamp)
    {
        mag_updated = true; //else, we have to waitting for time.
        bmagread = true;
    }
    if(mag_updated) // is time to fusion
    {
        // set mag data to ekf, follow the px4

        bool reset = false;

		// check if magnetometer has changed
		if (magnetometer.device_id != _device_id_mag) {
			if (_device_id_mag != 0) {
				printf("mag sensor ID changed to %d \n", magnetometer.device_id);
			}

			reset = true;

		} else if (magnetometer.calibration_count > _mag_calibration_count) {
			// existing calibration has changed, reset saved mag bias
			printf("%d - mag id \n", _device_id_mag);
			reset = true;
		}

		if (reset) {
			_ekf.resetMagBias();
			_device_id_mag = magnetometer.device_id;
			_mag_calibration_count = magnetometer.calibration_count;

			// reset magnetometer bias learning
			_mag_cal_total_time_us = 0;
			_mag_cal_last_us = 0;
			_mag_cal_available = false;
		}

        _ekf.setMagData(magSample{magnetometer.timestamp, Vector3f{magnetometer.magnetometer_ga}});

		last_magnetometer.magnetometer_ga[0]=magnetometer.magnetometer_ga[0];
		last_magnetometer.magnetometer_ga[1]=magnetometer.magnetometer_ga[1];
		last_magnetometer.magnetometer_ga[2]=magnetometer.magnetometer_ga[2];
		last_magnetometer.device_id = magnetometer.device_id;
		last_magnetometer.calibration_count = magnetometer.calibration_count;

    }
}
void PostEkf::PublishAttitude(const hrt_abstime &timestamp)
{
    if (_ekf.attitude_valid()) {
		// generate vehicle attitude quaternion data
		vehicle_attitude_s att;
		att.timestamp_sample = timestamp;
		const Quatf q{_ekf.calculate_quaternion()};
		q.copyTo(att.q);

		_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);
		att.timestamp = timestamp;
		// _attitude_pub.publish(att);
        // The rotation of the tangent plane vs. geographical north
        matrix::Eulerf euler(q);

        euler_estimator<< timestamp <<" "<<euler(0) <<" "<<euler(1) <<" "
            <<euler(2)<<" "<<std::endl;	
        
	}
}

void PostEkf::PublishLocalPosition(const hrt_abstime &timestamp)
{
    vehicle_local_position_s lpos;
	// generate vehicle local position data
	lpos.timestamp_sample = timestamp;

	// Position of body origin in local NED frame
	const Vector3f position{_ekf.getPosition()};
	lpos.x = position(0);
	lpos.y = position(1);
	lpos.z = position(2);

	// Velocity of body origin in local NED frame (m/s)
	const Vector3f velocity{_ekf.getVelocity()};
	lpos.vx = velocity(0);
	lpos.vy = velocity(1);
	lpos.vz = velocity(2);

	// vertical position time derivative (m/s)
	lpos.z_deriv = _ekf.getVerticalPositionDerivative();

	// Acceleration of body origin in local frame
	const Vector3f vel_deriv{_ekf.getVelocityDerivative()};
	lpos.ax = vel_deriv(0);
	lpos.ay = vel_deriv(1);
	lpos.az = vel_deriv(2);

	// TODO: better status reporting
	lpos.xy_valid = _ekf.local_position_is_valid();
	lpos.z_valid = true;
	lpos.v_xy_valid = _ekf.local_position_is_valid();
	lpos.v_z_valid = true;

	// Position of local NED origin in GPS / WGS84 frame
	if (_ekf.global_origin_valid()) {
		lpos.ref_timestamp = _ekf.global_origin().timestamp;
		lpos.ref_lat = math::degrees(_ekf.global_origin().lat_rad); // Reference point latitude in degrees
		lpos.ref_lon = math::degrees(_ekf.global_origin().lon_rad); // Reference point longitude in degrees
		lpos.ref_alt = _ekf.getEkfGlobalOriginAltitude();           // Reference point in MSL altitude meters
		lpos.xy_global = true;
		lpos.z_global = true;

	} else {
		lpos.ref_timestamp = 0;
		lpos.ref_lat = static_cast<double>(NAN);
		lpos.ref_lon = static_cast<double>(NAN);
		lpos.ref_alt = NAN;
		lpos.xy_global = false;
		lpos.z_global = false;
	}

	Quatf delta_q_reset;
	_ekf.get_quat_reset(&delta_q_reset(0), &lpos.heading_reset_counter);

	lpos.heading = Eulerf(_ekf.getQuaternion()).psi();
	lpos.delta_heading = Eulerf(delta_q_reset).psi();

	// Distance to bottom surface (ground) in meters
	// constrain the distance to ground to _rng_gnd_clearance
	lpos.dist_bottom = math::max(_ekf.getTerrainVertPos() - lpos.z, _param_ekf3_min_rng );
	lpos.dist_bottom_valid = _ekf.isTerrainEstimateValid();
	lpos.dist_bottom_sensor_bitfield = _ekf.getTerrainEstimateSensorBitfield();

	if (!_had_valid_terrain) {
		_had_valid_terrain = lpos.dist_bottom_valid;
	}

	// only consider ground effect if compensation is configured and the vehicle is armed (props spinning)
	if ((_param_ekf3_gnd_eff_dz  > 0.0f) && _armed && lpos.dist_bottom_valid) {
		// set ground effect flag if vehicle is closer than a specified distance to the ground
		_ekf.set_gnd_effect_flag(lpos.dist_bottom < _param_ekf3_gnd_max_hgt );

		// if we have no valid terrain estimate and never had one then use ground effect flag from land detector
		// _had_valid_terrain is used to make sure that we don't fall back to using this option
		// if we temporarily lose terrain data due to the distance sensor getting out of range
	}

	_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);
	_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

	// get state reset information of position and velocity
	_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
	_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
	_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
	_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

	// get control limit information
	_ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max);

	// convert NaN to INFINITY
	if (!PX4_ISFINITE(lpos.vxy_max)) {
		lpos.vxy_max = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.vz_max)) {
		lpos.vz_max = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_min)) {
		lpos.hagl_min = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_max)) {
		lpos.hagl_max = INFINITY;
	}

	// publish vehicle local position data
	lpos.timestamp = timestamp  ;
	// _local_position_pub.publish(lpos);
    // Position of body origin in local NED frame
    // Local Position NED
    position_estimator<< timestamp <<" "<<position(0) <<" "<<position(1) <<" "
    << position(2) <<" "<<std::endl;

    // Velocity of body origin in local NED frame (m/s)
    velocity_estimator<< timestamp <<" "<<velocity(0) <<" "<<velocity(1) <<" "
    << velocity(2) <<" "<<std::endl;


}

void PostEkf::UpdateMagCalibration(const hrt_abstime &timestamp)
{
    // Check if conditions are OK for learning of magnetometer bias values
	// the EKF is operating in the correct mode and there are no filter faults
	if (_ekf.control_status_flags().in_air && _ekf.control_status_flags().mag_3D && (_ekf.fault_status().value == 0)) {

		if (_mag_cal_last_us != 0) {
			_mag_cal_total_time_us += timestamp - _mag_cal_last_us;

			// Start checking mag bias estimates when we have accumulated sufficient calibration time
			if (_mag_cal_total_time_us > 30_s) {
				_mag_cal_last_bias = _ekf.getMagBias();
				_mag_cal_last_bias_variance = _ekf.getMagBiasVariance();
				_mag_cal_available = true;
			}
		}

		_mag_cal_last_us = timestamp;

	} else {
		// conditions are NOT OK for learning magnetometer bias, reset timestamp
		// but keep the accumulated calibration time
		_mag_cal_last_us = 0;

		if (_ekf.fault_status().value != 0) {
			// if a filter fault has occurred, assume previous learning was invalid and do not
			// count it towards total learning time.
			_mag_cal_total_time_us = 0;
		}
	}

	if (!_armed) {
		// update stored declination value
		if (!_mag_decl_saved) {
			float declination_deg;

			if (_ekf.get_mag_decl_deg(&declination_deg)) {
				_param_ekf3_mag_decl=(declination_deg);
				_mag_decl_saved = true;

				// if (!_multi_mode) {
				// 	_param_ekf3_mag_decl.commit_no_notification();
				// }
			}
		}
	}
}