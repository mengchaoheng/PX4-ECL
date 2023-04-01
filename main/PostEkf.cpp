#include "PostEkf.h"

PostEkf::PostEkf(std::string filename, std::string mag_name,std::string baro_name,std::string gps_name)
:_file_name(filename),_mag_name(mag_name),_baro_name(baro_name),_gps_name(gps_name)
{
    _fp_out = fopen("ecloutput.csv", "w");
    if(_fp_out==NULL) _fp_out = stdout;
}

PostEkf::~PostEkf()
{
}

void PostEkf::write_header()
{
    fprintf(_fp_out,"q0,q1,q2,q3,vn_m_s,ve_m_s,vd_m_s,n_m,e_m,d_m,\
delta_ang_bias_0,delta_ang_bias_1,delta_ang_bias_2,\
delta_vel_bias_0,delta_vel_bias_1,delta_vel_bias_2,\
mag_I_0,mag_I_1,mag_I_2,\
mag_B_0,mag_B_1,mag_B_2,\
wind_vel_0,wind_vel_1,\
roll_rad,pitch_rad,yaw_rad\n");
}

void PostEkf::update()
{
    // write_header();

    // read status
    CsvParser* csv_status = CsvParser_new(_file_name.c_str(), ",", 1);
    CsvRow* status_row = nullptr;

    while ((status_row = CsvParser_getRow(csv_status)))
    {
       char** rowFields = CsvParser_getFields(status_row);
       int field_count = CsvParser_getNumFields(status_row);
       if (FIELD_COUNT != field_count) 
       {
          printf("skip line, cause mag col count %d!=%d", field_count, FIELD_COUNT);
          continue;
       }
        receive_status((const char**)rowFields);

       CsvParser_destroy_row(status_row);
    }
    CsvParser_destroy(csv_status);



}

void PostEkf::output_csv()
{
    matrix::Quatf q = _ekf_core.getQuaternion();

    Eulerf att = Eulerf(q);

    matrix::Vector<float, 24> states = _ekf_core.getStateAtFusionHorizonAsVector();
    for(int i=0;i<24;++i)
    {
        fprintf(_fp_out,"%f,", states(i));
    }

    fprintf(_fp_out,"%f,%f,%f\n", att(0), att(1), att(2));
}


void PostEkf::set_imu(const char** row_fields)
{
    imudata record = {0};
    record.timestamp   = atoi(row_fields[TIMESTAMP]);
    record.gyro_rad(0) = atof(row_fields[GYRO_RAD_X]);
    record.gyro_rad(1) = atof(row_fields[GYRO_RAD_Y]);
    record.gyro_rad(2) = atof(row_fields[GYRO_RAD_Z]);
    record.gyro_integral_dt  = atoi(row_fields[GYRO_INTEGRAL_DT]);
    int32_t magnetometer_timestamp_relative = atoi(row_fields[ACCELEROMETER_TIMESTAMP_RELATIVE]);
    record.accelerometer_m_s2(0) = atof(row_fields[ACCELEROMETER_M_S2_X]);
    record.accelerometer_m_s2(1) = atof(row_fields[ACCELEROMETER_M_S2_Y]);
    record.accelerometer_m_s2(2) = atof(row_fields[ACCELEROMETER_M_S2_Z]);
    record.accelerometer_integral_dt = atoi(row_fields[ACCELEROMETER_INTEGRAL_DT]);
    printf("[imu]:time %llu, g1 %f, g2 %f, g3 %f, dt %u,  a1 %f, a2 %f, a3 %f, dt %u \n", record.timestamp, record.gyro_rad(0), record.gyro_rad(1), record.gyro_rad(2), record.gyro_integral_dt, record.accelerometer_m_s2(0), record.accelerometer_m_s2(1), record.accelerometer_m_s2(2), record.accelerometer_integral_dt);
}
void PostEkf::set_mag(const char** row_fields)
{
    magdata record = {0};
    record.timestamp   = atoi(row_fields[TIMESTAMP]);
    record.magnetometer_ga(0) = atof(row_fields[MAGNETOMETER_GA_X]);
    record.magnetometer_ga(1) = atof(row_fields[MAGNETOMETER_GA_Y]);
    record.magnetometer_ga(2) = atof(row_fields[MAGNETOMETER_GA_Z]);
    printf("[mag]:time %llu, m1 %f, m2 %f, m3 %f \n", record.timestamp, record.magnetometer_ga(0), record.magnetometer_ga(1), record.magnetometer_ga(2));
}
void PostEkf::set_baro(const char** row_fields)
{
    barodata record = {0};
    record.timestamp   = atoi(row_fields[TIMESTAMP]);
    record.baro_alt_meter = atof(row_fields[BARO_ALT_METER]);
    record.baro_temp_celcius = atof(row_fields[BARO_TEMP_CELCIUS]);
    record.baro_pressure_pa = atof(row_fields[BARO_PRESSURE_PA]);
    record.rho = atof(row_fields[RHO]);
    printf("[baro]:time %llu, baro_alt_meter %f, baro_temp_celcius %f, baro_pressure_pa %f rho %f \n", record.timestamp, record.baro_alt_meter, record.baro_temp_celcius, record.baro_pressure_pa,record.rho);

}
void PostEkf::receive_status(const char** row_fields)
{
    vehicle_status_s vehicle_status = {0}; 
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

    printf("[status]: time %llu, nav_state_timestamp %llu, failsafe_timestamp %llu, armed_time %llu, takeoff_time %llu, onboard_control_sensors_present %d, onboard_control_sensors_enabled %d, onboard_control_sensors_health %d, nav_state %d, arming_state %d, hil_state %d, failsafe %f, system_type %d, system_id %d, component_id %d, vehicle_type %d, is_vtol %f, is_vtol_tailsitter %f, vtol_fw_permanent_stab %f, in_transition_mode %f, in_transition_to_fw %d, rc_signal_lost %f, rc_input_mode %d\n", vehicle_status.timestamp, vehicle_status.nav_state_timestamp, vehicle_status.failsafe_timestamp, vehicle_status.armed_time, vehicle_status.takeoff_time, vehicle_status.onboard_control_sensors_present, vehicle_status.onboard_control_sensors_enabled, vehicle_status.onboard_control_sensors_health, vehicle_status.nav_state, vehicle_status.arming_state, vehicle_status.hil_state,(float) vehicle_status.failsafe, vehicle_status.system_type, vehicle_status.system_id, vehicle_status.component_id, vehicle_status.vehicle_type, (float)vehicle_status.is_vtol, (float)vehicle_status.is_vtol_tailsitter, (float)vehicle_status.vtol_fw_permanent_stab, (float) vehicle_status.in_transition_mode, vehicle_status.in_transition_to_fw, (float)vehicle_status.rc_signal_lost, vehicle_status.rc_input_mode);

}
void PostEkf::set_gps(const char** row_fields)
{
    gpsdata record = {0};
    record.timestamp   = atoi(row_fields[TIMESTAMP]);
    record.time_utc_usec   = atoi(row_fields[TIME_UTC_USEC]);
    record.lat   = atoi(row_fields[LAT]);
    record.lon   = atoi(row_fields[LON]);
    record.alt   = atoi(row_fields[ALT]);
    record.alt_ellipsoid   = atoi(row_fields[ALT_ELLIPSOID]);
    record.s_variance_m_s = atof(row_fields[S_VARIANCE_M_S]);
    record.c_variance_rad = atof(row_fields[C_VARIANCE_RAD]);
    record.eph = atof(row_fields[EPH]);
    record.epv = atof(row_fields[EPV]);
    record.hdop = atof(row_fields[HDOP]);
    record.vdop = atof(row_fields[VDOP]);
    record.noise_per_ms   = atoi(row_fields[NOISE_PER_MS]);
    record.jamming_indicator   = atoi(row_fields[JAMMING_INDICATOR]);
    record.vel_m_s = atof(row_fields[VEL_M_S]);
    record.vel_n_m_s = atof(row_fields[VEL_N_M_S]);
    record.vel_e_m_s = atof(row_fields[VEL_E_M_S]);
    record.vel_d_m_s = atof(row_fields[VEL_D_M_S]);
    record.cog_rad = atof(row_fields[COG_RAD]);
    record.timestamp_time_relative   = atoi(row_fields[TIMESTAMP_TIMESTAMP_RELATIVE]);
    record.fix_type   = atoi(row_fields[FIX_TYPE]);
    record.vel_ned_valid = (bool) atoi(row_fields[VEL_NED_VALID]);
    record.satellites_used   = atoi(row_fields[SATELLITES_USED]);

    printf("[gps]: time %llu, time_utc_usec %llu, lat %d, lon %d, alt %d, alt_ellipsoid %d, s_variance_m_s %f, c_variance_rad %f, eph %f, epv %f, hdop %f, vdop %f, noise_per_ms %d, jamming_indicator %d, vel_m_s %f, v1 %f, v2 %f, v3 %f, cog_rad %f, timestamp_time_relative %d, fix_type %d, vel_ned_valid %d, satellites_used %d\n", record.timestamp, record.time_utc_usec, record.lat, record.lon, record.alt, record.alt_ellipsoid, record.s_variance_m_s, record.c_variance_rad, record.eph, record.epv, record.hdop, record.vdop, record.noise_per_ms, record.jamming_indicator, record.vel_m_s, record.vel_n_m_s, record.vel_e_m_s, record.vel_d_m_s, record.cog_rad, record.timestamp_time_relative, record.fix_type, record.vel_ned_valid, record.satellites_used);

}

// void PostEkf::set_px4logstruct(const char** row_fields)
// {
//     LOGStruct record = {0};

//    record.imu_data.time_us      = atoi(row_fields[TIMESTAMP]);
//    record.imu_data.delta_ang(0) = atof(row_fields[GYRO_RAD_X]);
//    record.imu_data.delta_ang(1) = atof(row_fields[GYRO_RAD_Y]);
//    record.imu_data.delta_ang(2) = atof(row_fields[GYRO_RAD_Z]);
//    record.imu_data.delta_ang_dt = atof(row_fields[GYRO_INTEGRAL_DT]);
//    record.imu_data.delta_vel(0) = record.imu_data.delta_ang_dt*atof(row_fields[ACCELEROMETER_M_S2_X]);
//    record.imu_data.delta_vel(1) = record.imu_data.delta_ang_dt*atof(row_fields[ACCELEROMETER_M_S2_Y]);
//    record.imu_data.delta_vel(2) = record.imu_data.delta_ang_dt*atof(row_fields[ACCELEROMETER_M_S2_Z]);
//    record.imu_data.delta_vel_dt = atof(row_fields[ACCELEROMETER_INTEGRAL_DT]);
//    int magnetometer_timestamp_relative = atoi(row_fields[MAGNETOMETER_TIMESTAMP_RELATIVE]);
//    record.mag_data.time_us    = record.imu_data.time_us + magnetometer_timestamp_relative;
//    record.mag_data.mag(0)     = atof(row_fields[MAGNETOMETER_GA_X]);
//    record.mag_data.mag(1)     = atof(row_fields[MAGNETOMETER_GA_Y]);
//    record.mag_data.mag(2)     = atof(row_fields[MAGNETOMETER_GA_Z]);
//    int baro_timestamp_relative = atoi(row_fields[BARO_TIMESTAMP_RELATIVE]);
//    record.baro_data.time_us   = record.imu_data.time_us + baro_timestamp_relative;
//    record.baro_data.hgt       = atof(row_fields[BARO_ALT_METER]);
//    record.baro_temp_celcius   = atof(row_fields[BARO_TEMP_CELCIUS]);

//     // printf("[imu]:time %llu, g1 %f, g2 %f, g3 %f, dt %llu,  a1 %f, a2 %f, a3 %f, dt %llu \n", record.imu_data.time_us, record.imu_data.delta_ang(0), record.imu_data.delta_ang(1), record.imu_data.delta_ang(2), record.imu_data.delta_ang_dt, record.imu_data.delta_vel(0), record.imu_data.delta_vel(1), record.imu_data.delta_vel(2), record.imu_data.delta_vel_dt);

//     _ekf_core.setIMUData( record.imu_data);
//     _ekf_core.setMagData( record.mag_data);
//     _ekf_core.setBaroData(record.baro_data);
// }