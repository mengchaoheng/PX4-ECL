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

       _ekf_core.update();

       output_csv();

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
    LOGStruct record = {0};

   record.imu_data.time_us      = atoi(row_fields[TIMESTAMP]);
   record.imu_data.delta_ang(0) = atof(row_fields[GYRO_RAD_X]);
   record.imu_data.delta_ang(1) = atof(row_fields[GYRO_RAD_Y]);
   record.imu_data.delta_ang(2) = atof(row_fields[GYRO_RAD_Z]);
   record.imu_data.delta_ang_dt = atof(row_fields[GYRO_INTEGRAL_DT]);
   record.imu_data.delta_vel(0) = record.imu_data.delta_ang_dt*atof(row_fields[ACCELEROMETER_M_S2_X]);
   record.imu_data.delta_vel(1) = record.imu_data.delta_ang_dt*atof(row_fields[ACCELEROMETER_M_S2_Y]);
   record.imu_data.delta_vel(2) = record.imu_data.delta_ang_dt*atof(row_fields[ACCELEROMETER_M_S2_Z]);
   record.imu_data.delta_vel_dt = atof(row_fields[ACCELEROMETER_INTEGRAL_DT]);
   int magnetometer_timestamp_relative = atoi(row_fields[MAGNETOMETER_TIMESTAMP_RELATIVE]);
   record.mag_data.time_us    = record.imu_data.time_us + magnetometer_timestamp_relative;
   record.mag_data.mag(0)     = atof(row_fields[MAGNETOMETER_GA_X]);
   record.mag_data.mag(1)     = atof(row_fields[MAGNETOMETER_GA_Y]);
   record.mag_data.mag(2)     = atof(row_fields[MAGNETOMETER_GA_Z]);
   int baro_timestamp_relative = atoi(row_fields[BARO_TIMESTAMP_RELATIVE]);
   record.baro_data.time_us   = record.imu_data.time_us + baro_timestamp_relative;
   record.baro_data.hgt       = atof(row_fields[BARO_ALT_METER]);
   record.baro_temp_celcius   = atof(row_fields[BARO_TEMP_CELCIUS]);

    _ekf_core.setIMUData( record.imu_data);
    _ekf_core.setMagData( record.mag_data);
    _ekf_core.setBaroData(record.baro_data);
}