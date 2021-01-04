#include <stdio.h>
#include "EKF/ekf.h"

extern "C"
{
#include "csvparser.h"
}

enum {
   TIMESTAMP,
   GYRO_RAD_X,
   GYRO_RAD_Y,
   GYRO_RAD_Z,
   GYRO_INTEGRAL_DT,
   ACCELEROMETER_TIMESTAMP_RELATIVE,
   ACCELEROMETER_M_S2_X,
   ACCELEROMETER_M_S2_Y,
   ACCELEROMETER_M_S2_Z,
   ACCELEROMETER_INTEGRAL_DT,
   MAGNETOMETER_TIMESTAMP_RELATIVE,
   MAGNETOMETER_GA_X,
   MAGNETOMETER_GA_Y,
   MAGNETOMETER_GA_Z,
   BARO_TIMESTAMP_RELATIVE,
   BARO_ALT_METER,
   BARO_TEMP_CELCIUS,
   FIELD_COUNT,
};

struct PX4LOGStruct
{
   imuSample imu_data;
   magSample mag_data;
   baroSample baro_data;
   double baro_temp_celcius;
};

void set_px4logstruct(PX4LOGStruct * record, const char** row_fields);

Ekf _ekf_core;
int main(int argc, char** argv)
{
   const unsigned int MAX_PATH = 260u;
   char file_name[MAX_PATH] = { 0 };
   if (argc <= 1) {
      printf("please input the px4 csv log file.\n");
      fgets(file_name, MAX_PATH, stdin);
   }
   else {
      strcpy(file_name, argv[1]);
   }
    printf("start ecl main...\n");
    CsvParser* csvparser = CsvParser_new(file_name, ",", 1);
    CsvRow* row = nullptr;
    PX4LOGStruct record = {0};

    while ((row = CsvParser_getRow(csvparser))) {
       char** rowFields = CsvParser_getFields(row);
       int field_count = CsvParser_getNumFields(row);
       if (FIELD_COUNT != field_count) {
          printf("skip line, cause col count %d!=%d", field_count, FIELD_COUNT);
          continue;
       }

       set_px4logstruct(&record, (const char**)rowFields);

       _ekf_core.setIMUData(record.imu_data);
       _ekf_core.setMagData(record.mag_data);
       _ekf_core.setBaroData(record.baro_data);

       _ekf_core.update();

       Vector3f pos = _ekf_core.getPosition();

       printf("%f,%f,%f\n", pos(0), pos(1), pos(2));

       CsvParser_destroy_row(row);
    }
    CsvParser_destroy(csvparser);

    printf("finished.\n");
    
    return 0;
}

void set_px4logstruct(PX4LOGStruct * record, const char** row_fields)
{
   record->imu_data.time_us = atoi(row_fields[TIMESTAMP]);
   record->imu_data.delta_ang(0) = atof(row_fields[GYRO_RAD_X]);
   record->imu_data.delta_ang(1) = atof(row_fields[GYRO_RAD_Y]);
   record->imu_data.delta_ang(2) = atof(row_fields[GYRO_RAD_Z]);
   record->imu_data.delta_ang_dt = atof(row_fields[GYRO_INTEGRAL_DT]);
   record->imu_data.delta_vel(0) = atof(row_fields[ACCELEROMETER_M_S2_X]);
   record->imu_data.delta_vel(1) = atof(row_fields[ACCELEROMETER_M_S2_Y]);
   record->imu_data.delta_vel(2) = atof(row_fields[ACCELEROMETER_M_S2_Z]);
   record->imu_data.delta_vel_dt = atof(row_fields[ACCELEROMETER_INTEGRAL_DT]);
   int magnetometer_timestamp_relative = atoi(row_fields[MAGNETOMETER_TIMESTAMP_RELATIVE]);
   record->mag_data.time_us = record->imu_data.time_us + magnetometer_timestamp_relative;
   record->mag_data.mag(0) = atof(row_fields[MAGNETOMETER_GA_X]);
   record->mag_data.mag(1) = atof(row_fields[MAGNETOMETER_GA_Y]);
   record->mag_data.mag(2) = atof(row_fields[MAGNETOMETER_GA_Z]);
   int baro_timestamp_relative = atoi(row_fields[BARO_TIMESTAMP_RELATIVE]);
   record->baro_data.time_us = record->imu_data.time_us + baro_timestamp_relative;
   record->baro_data.hgt = atof(row_fields[BARO_ALT_METER]);
   record->baro_temp_celcius = atof(row_fields[BARO_TEMP_CELCIUS]);
}