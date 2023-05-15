#pragma once

#ifndef __POST_EKF_2021__
#define __POST_EKF_2021__

#include <string>
#include "EKF/ekf.h"
#include "csvparser.h"

#include "PreFlightChecker.hpp"
//  #include "Utility/estimator_innovations.h"
// TODO: create different type of data type
/*
enum {
    TYPE_PX4,
    TYPR_COUNT,
}
*/

/**
 * Absolute time, in microsecond units.
 *
 * Absolute time is measured from some arbitrary epoch shortly after
 * system startup.  It should never wrap or go backwards.
 */
typedef uint64_t	hrt_abstime;
//can't be use to control, just for data flow logi.
//#define    NAN          __builtin_nanf("0x7fc00000")

#define PX4_ISFINITE(x) std::isfinite(x) //ISFINITE判断

// User-defined integer literals for different time units.
// The base unit is hrt_abstime in microseconds

constexpr hrt_abstime operator "" _s(unsigned long long seconds)
{
	return hrt_abstime(seconds * 1000000ULL);
}


#define IMU_FIELD_COUNT 11
#define MAG_FIELD_COUNT 7
#define EV_FIELD_COUNT 63
#define BARO_FIELD_COUNT 7
#define  FLOW_FIELD_COUNT 17
#define GPS_FIELD_COUNT 27
#define STATUS_FIELD_COUNT 32

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
#define   ACC_CLIP  10

#define   MAGNETOMETER_GA_X 3
#define   MAGNETOMETER_GA_Y 4
#define   MAGNETOMETER_GA_Z 5
#define   CALIBRATION_COUNT 6

#define   TIMESTAMP_SAMPLE 1 
#define   DEVICE_ID 2 
#define   BARO_ALT_METER 3
#define   BARO_TEMP_CELCIUS 4
#define   BARO_PRESSURE_PA 5
#define   RHO 6

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
//NAN for heading
#define   FIX_TYPE 22
#define   JAMMING_STATE 23
#define   VEL_NED_VALID 24
#define   SATELLITES_USED 25
#define   SELECTED 26

struct vehicle_attitude_s {
    uint64_t timestamp;
	uint64_t timestamp_sample;
	float q[4];
	float delta_q_reset[4];
	uint8_t quat_reset_counter;
	uint8_t _padding0[7]; // required for logger
};
struct vehicle_local_position_s {
    uint64_t timestamp;
	uint64_t timestamp_sample;
	uint64_t ref_timestamp;
	double ref_lat;
	double ref_lon;
	float x;
	float y;
	float z;
	float delta_xy[2];
	float delta_z;
	float vx;
	float vy;
	float vz;
	float z_deriv;
	float delta_vxy[2];
	float delta_vz;
	float ax;
	float ay;
	float az;
	float heading;
	float delta_heading;
	float ref_alt;
	float dist_bottom;
	float eph;
	float epv;
	float evh;
	float evv;
	float vxy_max;
	float vz_max;
	float hagl_min;
	float hagl_max;
	bool xy_valid;
	bool z_valid;
	bool v_xy_valid;
	bool v_z_valid;
	uint8_t xy_reset_counter;
	uint8_t z_reset_counter;
	uint8_t vxy_reset_counter;
	uint8_t vz_reset_counter;
	uint8_t heading_reset_counter;
	bool xy_global;
	bool z_global;
	bool dist_bottom_valid;
	uint8_t dist_bottom_sensor_bitfield;
	uint8_t _padding0[3]; // required for logger
    static constexpr uint8_t DIST_BOTTOM_SENSOR_NONE = 0;
	static constexpr uint8_t DIST_BOTTOM_SENSOR_RANGE = 1;
	static constexpr uint8_t DIST_BOTTOM_SENSOR_FLOW = 2;
};

struct estimator_status_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float vibe[3];
	float output_tracking_error[3];
	uint32_t control_mode_flags;
	uint32_t filter_fault_flags;
	float pos_horiz_accuracy;
	float pos_vert_accuracy;
	float mag_test_ratio;
	float vel_test_ratio;
	float pos_test_ratio;
	float hgt_test_ratio;
	float tas_test_ratio;
	float hagl_test_ratio;
	float beta_test_ratio;
	float time_slip;
	uint32_t accel_device_id;
	uint32_t gyro_device_id;
	uint32_t baro_device_id;
	uint32_t mag_device_id;
	uint16_t gps_check_fail_flags;
	uint16_t innovation_check_flags;
	uint16_t solution_status_flags;
	uint8_t reset_count_vel_ne;
	uint8_t reset_count_vel_d;
	uint8_t reset_count_pos_ne;
	uint8_t reset_count_pod_d;
	uint8_t reset_count_quat;
	bool pre_flt_fail_innov_heading;
	bool pre_flt_fail_innov_vel_horiz;
	bool pre_flt_fail_innov_vel_vert;
	bool pre_flt_fail_innov_height;
	bool pre_flt_fail_mag_field_disturbed;
	uint8_t health_flags;
	uint8_t timeout_flags;

	static constexpr uint8_t GPS_CHECK_FAIL_GPS_FIX = 0;
	static constexpr uint8_t GPS_CHECK_FAIL_MIN_SAT_COUNT = 1;
	static constexpr uint8_t GPS_CHECK_FAIL_MAX_PDOP = 2;
	static constexpr uint8_t GPS_CHECK_FAIL_MAX_HORZ_ERR = 3;
	static constexpr uint8_t GPS_CHECK_FAIL_MAX_VERT_ERR = 4;
	static constexpr uint8_t GPS_CHECK_FAIL_MAX_SPD_ERR = 5;
	static constexpr uint8_t GPS_CHECK_FAIL_MAX_HORZ_DRIFT = 6;
	static constexpr uint8_t GPS_CHECK_FAIL_MAX_VERT_DRIFT = 7;
	static constexpr uint8_t GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR = 8;
	static constexpr uint8_t GPS_CHECK_FAIL_MAX_VERT_SPD_ERR = 9;
	static constexpr uint8_t CS_TILT_ALIGN = 0;
	static constexpr uint8_t CS_YAW_ALIGN = 1;
	static constexpr uint8_t CS_GPS = 2;
	static constexpr uint8_t CS_OPT_FLOW = 3;
	static constexpr uint8_t CS_MAG_HDG = 4;
	static constexpr uint8_t CS_MAG_3D = 5;
	static constexpr uint8_t CS_MAG_DEC = 6;
	static constexpr uint8_t CS_IN_AIR = 7;
	static constexpr uint8_t CS_WIND = 8;
	static constexpr uint8_t CS_BARO_HGT = 9;
	static constexpr uint8_t CS_RNG_HGT = 10;
	static constexpr uint8_t CS_GPS_HGT = 11;
	static constexpr uint8_t CS_EV_POS = 12;
	static constexpr uint8_t CS_EV_YAW = 13;
	static constexpr uint8_t CS_EV_HGT = 14;
	static constexpr uint8_t CS_BETA = 15;
	static constexpr uint8_t CS_MAG_FIELD = 16;
	static constexpr uint8_t CS_FIXED_WING = 17;
	static constexpr uint8_t CS_MAG_FAULT = 18;
	static constexpr uint8_t CS_ASPD = 19;
	static constexpr uint8_t CS_GND_EFFECT = 20;
	static constexpr uint8_t CS_RNG_STUCK = 21;
	static constexpr uint8_t CS_GPS_YAW = 22;
	static constexpr uint8_t CS_MAG_ALIGNED = 23;
};
struct sensor_combined_s {
    // // Sensor readings in SI-unit form.
    // // These fields are scaled and offset-compensated where possible and do not
    // // change with board revisions and sensor updates.
    uint64_t timestamp;				// time since system start (microseconds)
    // // gyro timstamp is equal to the timestamp of the message
    float gyro_rad[3];			// average angular rate measured in the FRD body frame XYZ-axis in rad/s over the last gyro sampling period
    uint32_t gyro_integral_dt;		// gyro measurement sampling period in microseconds

    int32_t accelerometer_timestamp_relative;	// timestamp + accelerometer_timestamp_relative = Accelerometer timestamp
    float accelerometer_m_s2[3];		// average value acceleration measured in the FRD body frame XYZ-axis in m/s^2 over the last accelerometer sampling period
    uint32_t accelerometer_integral_dt;	// accelerometer measurement sampling period in microseconds
    uint8_t accelerometer_clipping;  // bitfield indicating if there was any accelerometer clipping (per axis) during the sampling period
	uint8_t _padding0[3]; // required for logger

    static constexpr int32_t RELATIVE_TIMESTAMP_INVALID = 2147483647;
	static constexpr uint8_t CLIPPING_X = 1;
	static constexpr uint8_t CLIPPING_Y = 2;
	static constexpr uint8_t CLIPPING_Z = 4;
};

struct vehicle_gps_position_s {
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
    
    int32_t timestamp_time_relative;	// timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
    float heading; //heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
	float heading_offset; // heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])
    uint8_t fix_type; // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.  
	uint8_t jamming_state;
    bool vel_ned_valid;		// True if NED velocity is valid 

    uint8_t satellites_used;		// Number of satellites used 
    uint8_t selected;
	uint8_t _padding0[3]; // required for logger
};

struct vehicle_magnetometer_s {
	uint64_t timestamp;            // time since system start (microseconds)
    uint64_t timestamp_sample;     // the timestamp of the raw data (microseconds)
    uint32_t device_id;            // unique device ID for the selected magnetometer
    float magnetometer_ga[3];  // Magnetic field in the FRD body frame XYZ-axis in Gauss
    uint8_t calibration_count;     // Calibration changed counter. Monotonically increases whenever calibration changes.
    uint8_t _padding0[7]; // required for logger
};

struct vehicle_odometry_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float x;
	float y;
	float z;
	float q[4];
	float q_offset[4];
	float pose_covariance[21];
	float vx;
	float vy;
	float vz;
	float rollspeed;
	float pitchspeed;
	float yawspeed;
	float velocity_covariance[21];
	uint8_t local_frame;
	uint8_t velocity_frame;

	static constexpr uint8_t COVARIANCE_MATRIX_X_VARIANCE = 0;
	static constexpr uint8_t COVARIANCE_MATRIX_Y_VARIANCE = 6;
	static constexpr uint8_t COVARIANCE_MATRIX_Z_VARIANCE = 11;
	static constexpr uint8_t COVARIANCE_MATRIX_ROLL_VARIANCE = 15;
	static constexpr uint8_t COVARIANCE_MATRIX_PITCH_VARIANCE = 18;
	static constexpr uint8_t COVARIANCE_MATRIX_YAW_VARIANCE = 20;
	static constexpr uint8_t COVARIANCE_MATRIX_VX_VARIANCE = 0;
	static constexpr uint8_t COVARIANCE_MATRIX_VY_VARIANCE = 6;
	static constexpr uint8_t COVARIANCE_MATRIX_VZ_VARIANCE = 11;
	static constexpr uint8_t COVARIANCE_MATRIX_ROLLRATE_VARIANCE = 15;
	static constexpr uint8_t COVARIANCE_MATRIX_PITCHRATE_VARIANCE = 18;
	static constexpr uint8_t COVARIANCE_MATRIX_YAWRATE_VARIANCE = 20;
	static constexpr uint8_t LOCAL_FRAME_NED = 0;
	static constexpr uint8_t LOCAL_FRAME_FRD = 1;
	static constexpr uint8_t LOCAL_FRAME_OTHER = 2;
	static constexpr uint8_t BODY_FRAME_FRD = 3;

};


struct vehicle_air_data_s {
    uint64_t timestamp;            // time since system start (microseconds)
    uint64_t timestamp_sample;     // the timestamp of the raw data (microseconds)
    uint32_t baro_device_id;
	float baro_alt_meter;			// Altitude above MSL calculated from temperature compensated baro sensor data using an ISA corrected for sea level pressure SENS_BARO_QNH.
    float baro_temp_celcius;		// Temperature in degrees celsius
    float baro_pressure_pa;		// Absolute pressure in pascals

    float rho;						// air density
    uint8_t _padding0[4]; // required for logger
};

struct optical_flow_s {

	uint64_t timestamp;
	float pixel_flow_x_integral;
	float pixel_flow_y_integral;
	float gyro_x_rate_integral;
	float gyro_y_rate_integral;
	float gyro_z_rate_integral;
	float ground_distance_m;
	uint32_t integration_timespan;
	uint32_t time_since_last_sonar_update;
	float max_flow_rate;
	float min_ground_distance;
	float max_ground_distance;
	uint16_t frame_count_since_last_readout;
	int16_t gyro_temperature;
	uint8_t sensor_id;
	uint8_t quality;
	uint8_t mode;
	static constexpr uint8_t MODE_UNKNOWN = 0;
	static constexpr uint8_t MODE_BRIGHT = 1;
	static constexpr uint8_t MODE_LOWLIGHT = 2;
	static constexpr uint8_t MODE_SUPER_LOWLIGHT = 3;

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
    std::string _mag_name = {""};
    std::string _baro_name = {""};
    std::string _gps_name = {""};
    std::string _status_name = {""};
	std::string _visual_odometry_name = {""};
	std::string _optical_flow_name = {""};
    Ekf _ekf;
    FILE * _fp_out = NULL;
public:
    PostEkf(std::string filename, std::string mag_name, std::string baro_name, std::string gps_name, std::string status_name, std::string visual_odometry_name, std::string optical_flow_name);
    PostEkf() = delete;
    ~PostEkf();

    void update();

private:
	optical_flow_s optical_flow = {0};
    sensor_combined_s sensor_combined = {0};
    vehicle_magnetometer_s magnetometer = {0};
    vehicle_magnetometer_s last_magnetometer = {0};
	vehicle_odometry_s ev_odom = {0};
	vehicle_odometry_s last_ev_odom = {0};
    vehicle_air_data_s airdata = {0};
    vehicle_air_data_s last_airdata = {0};
    vehicle_gps_position_s vehicle_gps_position = {0};
    vehicle_gps_position_s last_vehicle_gps_position = {0};
    vehicle_status_s vehicle_status = {0};
    //flag
    bool mag_updated = false;
	bool ev_odom_updated = false;
    bool baro_updated = false;
    bool gps_updated = false;
    bool status_updated = false;
	bool optical_flow_updated = false;
    CsvParser* csv_imu;
    CsvRow* imu_row = nullptr;
    CsvParser* csv_mag;
    CsvRow* mag_row= nullptr; 
	CsvParser* csv_ev;
    CsvRow* ev_row= nullptr;
    CsvParser* csv_baro;
    CsvRow* baro_row = nullptr;
    CsvParser* csv_gps;
    CsvRow* gps_row = nullptr;
    CsvParser* csv_status;
    CsvRow* status_row = nullptr;
	CsvParser* csv_optical_flow;
    CsvRow* optical_flow_row = nullptr;

    void write_header();
    void output_csv(const hrt_abstime &timestamp);
    void receive_imu(const char** row_fields);
    void receive_mag(const char** row_fields);
	void receive_ev(const char** row_fields);
    void receive_baro(const char** row_fields);
    void receive_gps(const char** row_fields);
    void receive_status(const char** row_fields);
	void receive_optical_flow(const char** row_fields);


    // follow px4
    // time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)
    uint8_t _mag_calibration_count{0};
    // uint8_t _imu_calibration_count{0};
	uint32_t _device_id_accel{0};
	uint32_t _device_id_baro{0};
	uint32_t _device_id_gyro{0};
	uint32_t _device_id_mag{0};

    uint64_t _gps_time_usec{0};
	int32_t _gps_alttitude_ellipsoid{0};			///< altitude in 1E-3 meters (millimeters) above ellipsoid

    // Used to check, save and use learned magnetometer biases
	hrt_abstime _mag_cal_last_us{0};	///< last time the EKF was operating a mode that estimates magnetomer biases (uSec)

    Vector3f _mag_cal_last_bias{};	///< last valid XYZ magnetometer bias estimates (Gauss)
    Vector3f _mag_cal_last_bias_variance{};	///< variances for the last valid magnetometer XYZ bias estimates (Gauss**2)
    bool _mag_cal_available{false};	///< true when an unsaved valid calibration for the XYZ magnetometer bias is available
    // Used to control saving of mag declination to be used on next startup
	bool _mag_decl_saved = false;	///< true when the magnetic declination has been saved

	hrt_abstime _mag_cal_total_time_us{0};	///< accumulated calibration time since the last save

    bool _had_valid_terrain{false};			///< true if at any time there was a valid terrain estimate

    float _param_ekf3_fuse_beta{0};
    float _param_ekf3_min_rng{0.1f};
    float _param_ekf3_gnd_eff_dz{4.0f};
    float _param_ekf3_gnd_max_hgt{0.5f};
    float _param_ekf3_mag_decl{0.f};
	float  _param_ekf2_evv_noise{0.1};
	int _param_ekf2_ev_noise_md{0};
	float  _param_ekf2_evp_noise{0.05};
	float  _param_ekf2_eva_noise{0.05};

    bool _armed{false};
	bool _standby{false}; // standby arming state

    void PublishAttitude(const hrt_abstime &timestamp);
	// void PublishEkfDriftMetrics(const hrt_abstime &timestamp);
	// void PublishEventFlags(const hrt_abstime &timestamp);
	// void PublishGlobalPosition(const hrt_abstime &timestamp);
	void PublishInnovations(const hrt_abstime &timestamp, const imuSample &imu);
	// void PublishInnovationTestRatios(const hrt_abstime &timestamp);
	// void PublishInnovationVariances(const hrt_abstime &timestamp);
	void PublishLocalPosition(const hrt_abstime &timestamp);
	// void PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu);
	// void PublishOdometryAligned(const hrt_abstime &timestamp, const vehicle_odometry_s &ev_odom);
	// void PublishOpticalFlowVel(const hrt_abstime &timestamp, const optical_flow_s &optical_flow);
	// void PublishSensorBias(const hrt_abstime &timestamp);
	// void PublishStates(const hrt_abstime &timestamp);
	void PublishStatus(const hrt_abstime &timestamp);
	// void PublishStatusFlags(const hrt_abstime &timestamp);
	// void PublishWindEstimate(const hrt_abstime &timestamp);
	// void PublishYawEstimatorStatus(const hrt_abstime &timestamp);

    void UpdateVehicleStatusSample();
    void UpdateBaroSample();
	void UpdateGpsSample();
	void UpdateMagSample();
	bool UpdateExtVisionSample();
    void UpdateMagCalibration(const hrt_abstime &timestamp);
	bool UpdateFlowSample();
	PreFlightChecker _preflt_checker;
	parameters *_params;
};

#endif // __POST_EKF_2021__