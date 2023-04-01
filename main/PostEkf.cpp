#include "PostEkf.h"

PostEkf::PostEkf(std::string filename)
:_file_name(filename)
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
    write_header();

    CsvParser* csvparser = CsvParser_new(_file_name.c_str(), ",", 1);
    CsvRow* row = nullptr;

    while ((row = CsvParser_getRow(csvparser)))
     {
       char** rowFields = CsvParser_getFields(row);
       int field_count = CsvParser_getNumFields(row);
       if (FIELD_COUNT != field_count) 
       {
          printf("skip line, cause col count %d!=%d", field_count, FIELD_COUNT);
          continue;
       }

       set_px4logstruct((const char**)rowFields);

    //    _ekf_core.update();

    //    output_csv();

       CsvParser_destroy_row(row);
    }
    CsvParser_destroy(csvparser);
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

void PostEkf::set_px4logstruct(const char** row_fields)
{
    // LOGStruct record = {0};

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

    // printf("[imu]:time %llu, g1 %f, g2 %f, g3 %f, dt %llu,  a1 %f, a2 %f, a3 %f, dt %llu \n", record.imu_data.time_us, record.imu_data.delta_ang(0), record.imu_data.delta_ang(1), record.imu_data.delta_ang(2), record.imu_data.delta_ang_dt, record.imu_data.delta_vel(0), record.imu_data.delta_vel(1), record.imu_data.delta_vel(2), record.imu_data.delta_vel_dt);


// user

    // imudata record = {0};
    // record.timestamp   = atoi(row_fields[TIMESTAMP]);
    // record.gyro_rad(0) = atof(row_fields[GYRO_RAD_X]);
    // record.gyro_rad(1) = atof(row_fields[GYRO_RAD_Y]);
    // record.gyro_rad(2) = atof(row_fields[GYRO_RAD_Z]);
    // record.gyro_integral_dt  = atoi(row_fields[GYRO_INTEGRAL_DT]);
    // int32_t magnetometer_timestamp_relative = atoi(row_fields[ACCELEROMETER_TIMESTAMP_RELATIVE]);
    // record.accelerometer_m_s2(0) = atof(row_fields[ACCELEROMETER_M_S2_X]);
    // record.accelerometer_m_s2(1) = atof(row_fields[ACCELEROMETER_M_S2_Y]);
    // record.accelerometer_m_s2(2) = atof(row_fields[ACCELEROMETER_M_S2_Z]);
    // record.accelerometer_integral_dt = atoi(row_fields[ACCELEROMETER_INTEGRAL_DT]);
    // printf("[imu]:time %llu, g1 %f, g2 %f, g3 %f, dt %u,  a1 %f, a2 %f, a3 %f, dt %u \n", record.timestamp, record.gyro_rad(0), record.gyro_rad(1), record.gyro_rad(2), record.gyro_integral_dt, record.accelerometer_m_s2(0), record.accelerometer_m_s2(1), record.accelerometer_m_s2(2), record.accelerometer_integral_dt);


    // magdata record = {0};
    // record.timestamp   = atoi(row_fields[TIMESTAMP]);
    // record.magnetometer_ga(0) = atof(row_fields[MAGNETOMETER_GA_X]);
    // record.magnetometer_ga(1) = atof(row_fields[MAGNETOMETER_GA_Y]);
    // record.magnetometer_ga(2) = atof(row_fields[MAGNETOMETER_GA_Z]);
    // printf("[mag]:time %llu, m1 %f, m2 %f, m3 %f \n", record.timestamp, record.magnetometer_ga(0), record.magnetometer_ga(1), record.magnetometer_ga(2));

    // barodata record = {0};
    // record.timestamp   = atoi(row_fields[TIMESTAMP]);
    // record.baro_alt_meter = atof(row_fields[BARO_ALT_METER]);
    // record.baro_temp_celcius = atof(row_fields[BARO_TEMP_CELCIUS]);
    // record.baro_pressure_pa = atof(row_fields[BARO_PRESSURE_PA]);
    // record.rho = atof(row_fields[RHO]);
    // printf("[baro]:time %llu, baro_alt_meter %f, baro_temp_celcius %f, baro_pressure_pa %f rho %f \n", record.timestamp, record.baro_alt_meter, record.baro_temp_celcius, record.baro_pressure_pa,record.rho);


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

    printf("[gps]: time %llu, time_utc_usec %llu, lat %d, lon %d, alt %d, alt_ellipsoid %d, s_variance_m_s %f, c_variance_rad %f, eph %f, epv %f, hdop %f, vdop %f, noise_per_ms %d, jamming_indicator %d, vel_m_s %f, v1 %f, v2 %f, v3 %f, cog_rad %f, timestamp_time_relative %d, fix_type %d, vel_ned_valid %d, satellites_used %d\n", record.timestamp, record.time_utc_usec, record.lat, record.lon, record.alt, record.alt_ellipsoid, record.s_variance_m_s, record.c_variance_rad, record.eph, record.epv, record.hdop, record.vdop, record.noise_per_ms, record.jamming_indicator, record.vel_m_s, record.vel_n_m_s, record.vel_e_m_s, record.vel_d_m_s, record.cog_rad, record.timestamp_time_relative, record.fix_type, record.vel_ned_valid, record.satellites_used);;

    


    


    // _ekf_core.setIMUData( record.imu_data);
    // _ekf_core.setMagData( record.mag_data);
    // _ekf_core.setBaroData(record.baro_data);
}