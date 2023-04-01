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

// enum {
//    TIMESTAMP,
//    GYRO_RAD_X,
//    GYRO_RAD_Y,
//    GYRO_RAD_Z,
//    GYRO_INTEGRAL_DT,
//    ACCELEROMETER_TIMESTAMP_RELATIVE,
//    ACCELEROMETER_M_S2_X,
//    ACCELEROMETER_M_S2_Y,
//    ACCELEROMETER_M_S2_Z,
//    ACCELEROMETER_INTEGRAL_DT,
//    MAGNETOMETER_TIMESTAMP_RELATIVE,
//    MAGNETOMETER_GA_X,
//    MAGNETOMETER_GA_Y,
//    MAGNETOMETER_GA_Z,
//    BARO_TIMESTAMP_RELATIVE,
//    BARO_ALT_METER,
//    BARO_TEMP_CELCIUS,
//    FIELD_COUNT,
// };
#define IMU_FIELD_COUNT 10
#define GPS_FIELD_COUNT 23
#define MAG_FIELD_COUNT 4
#define BARO_FIELD_COUNT 5

#define FIELD_COUNT 23


#define   TIMESTAMP 0
#define   GYRO_RAD_X 1
#define   GYRO_RAD_Y 2
#define   GYRO_RAD_Z 3
#define   GYRO_INTEGRAL_DT 4
#define   ACCELEROMETER_TIMESTAMP_RELATIVE 5
#define   ACCELEROMETER_M_S2_X 6
#define   ACCELEROMETER_M_S2_Y 7
#define   ACCELEROMETER_M_S2_Z 8
#define   ACCELEROMETER_INTEGRAL_DT 9


#define   MAGNETOMETER_GA_X 1
#define   MAGNETOMETER_GA_Y 2
#define   MAGNETOMETER_GA_Z 3

#define   BARO_ALT_METER 1
#define   BARO_TEMP_CELCIUS 2
#define   BARO_PRESSURE_PA 3
#define   RHO 4

#define   TIME_UTC_USEC 1
#define   LAT 2
#define   LON 3
#define   ALT 4
#define   ALT_ELLIPSOID 5
#define   S_VARIANCE_M_S 6
#define   C_VARIANCE_RAD 7
#define   EPH 8
#define   EPV 9
#define   HDOP 10
#define   VDOP 11
#define   NOISE_PER_MS 12
#define   JAMMING_INDICATOR 13
#define   VEL_M_S 14
#define   VEL_N_M_S 15
#define   VEL_E_M_S 16
#define   VEL_D_M_S 17
#define   COG_RAD 18
#define   TIMESTAMP_TIMESTAMP_RELATIVE 19
#define   FIX_TYPE 20
#define   VEL_NED_VALID 21
#define   SATELLITES_USED 22
struct imudata {
    // // Sensor readings in SI-unit form.
    // // These fields are scaled and offset-compensated where possible and do not
    // // change with board revisions and sensor updates.
    uint64_t timestamp;				// time since system start (microseconds)
    // // gyro timstamp is equal to the timestamp of the message
    Vector3f gyro_rad;			// average angular rate measured in the FRD body frame XYZ-axis in rad/s over the last gyro sampling period
    uint32_t gyro_integral_dt;		// gyro measurement sampling period in microseconds

    int32_t accelerometer_timestamp_relative;	// timestamp + accelerometer_timestamp_relative = Accelerometer timestamp
    Vector3f accelerometer_m_s2;		// average value acceleration measured in the FRD body frame XYZ-axis in m/s^2 over the last accelerometer sampling period
    uint32_t accelerometer_integral_dt;	// accelerometer measurement sampling period in microseconds
};

struct gpsdata {
	// // GPS position in WGS84 coordinates.
    // // the auto-generated field 'timestamp' is for the position & velocity (microseconds)
    uint64_t timestamp;				// time since system start (microseconds)
    uint64_t time_utc_usec;		// Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0 
    int32_t lat;			// Latitude in 1E-7 degrees
    int32_t lon;			// Longitude in 1E-7 degrees 
    int32_t alt;			// Altitude in 1E-3 meters above MSL, (millimetres)
    int32_t alt_ellipsoid; 		// Altitude in 1E-3 meters bove Ellipsoid, (millimetres)

    float s_variance_m_s;		// GPS speed accuracy estimate, (metres/sec)
    float c_variance_rad;		// GPS course accuracy estimate, (radians) 
     

    float eph;			// GPS horizontal position accuracy (metres)
    float epv;			// GPS vertical position accuracy (metres)

    float hdop;			// Horizontal dilution of precision
    float vdop;			// Vertical dilution of precision

    int32_t noise_per_ms;		// GPS noise per millisecond
    int32_t jamming_indicator;		// indicates jamming is occurring

    float vel_m_s;			// GPS ground speed, (metres/sec) 
    float vel_n_m_s;		// GPS North velocity, (metres/sec) 
    float vel_e_m_s;		// GPS East velocity, (metres/sec)
    float vel_d_m_s;		// GPS Down velocity, (metres/sec) 
    float cog_rad;			// Course over ground (NOT heading, but direction of movement), -PI..PI, (radians) 
    bool vel_ned_valid;		// True if NED velocity is valid 

    int32_t timestamp_time_relative;	// timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
    
    uint8_t fix_type; // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.  

    uint8_t satellites_used;		// Number of satellites used 
};

struct magdata {
	uint64_t timestamp;            // time since system start (microseconds)
    uint64_t timestamp_sample;     // the timestamp of the raw data (microseconds)
    uint32_t device_id;            // unique device ID for the selected magnetometer
    Vector3f magnetometer_ga;  // Magnetic field in the FRD body frame XYZ-axis in Gauss
    uint8_t calibration_count;     // Calibration changed counter. Monotonically increases whenever calibration changes.
};

struct barodata {
    uint64_t timestamp;            // time since system start (microseconds)
    uint64_t timestamp_sample;     // the timestamp of the raw data (microseconds)
	float baro_alt_meter;			// Altitude above MSL calculated from temperature compensated baro sensor data using an ISA corrected for sea level pressure SENS_BARO_QNH.
    float baro_temp_celcius;		// Temperature in degrees celsius
    float baro_pressure_pa;		// Absolute pressure in pascals

    float rho;						// air density
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