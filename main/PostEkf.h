#pragma once

#ifndef __POST_EKF_2021__
#define __POST_EKF_2021__

#include <string>
#include "EKF/ekf.h"
#include "csvparser.h"

// TODO: create different type of data type
/*
enum {
    TYPE_PX4,
    TYPR_COUNT,
}
*/

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

struct LOGStruct
{
   imuSample imu_data;
   magSample mag_data;
   baroSample baro_data;
   double baro_temp_celcius;
};

class PostEkf
{
private:
    std::string _file_name = {""};
    Ekf _ekf_core;
    FILE * _fp_out = NULL;
public:
    PostEkf(std::string filename);
    PostEkf() = delete;
    ~PostEkf();

    void update();

private:
    void write_header();
    void output_csv();
    void set_px4logstruct(const char** row_fields);
};

#endif // __POST_EKF_2021__