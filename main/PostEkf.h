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

/**
 * Absolute time, in microsecond units.
 *
 * Absolute time is measured from some arbitrary epoch shortly after
 * system startup.  It should never wrap or go backwards.
 */
typedef uint64_t	hrt_abstime;
//can't be use to control, just for data flow logi.
#define    NAN          __builtin_nanf("0x7fc00000")

#define PX4_ISFINITE(x) std::isfinite(x) //ISFINITE判断

// User-defined integer literals for different time units.
// The base unit is hrt_abstime in microseconds

constexpr hrt_abstime operator "" _s(unsigned long long seconds)
{
	return hrt_abstime(seconds * 1000000ULL);
}

struct vehicle_status_s {
    uint64_t timestamp;
	uint64_t nav_state_timestamp;
	uint64_t failsafe_timestamp;

	uint64_t onboard_control_sensors_present;
	uint64_t onboard_control_sensors_enabled;
	uint64_t onboard_control_sensors_health;

	uint64_t armed_time;
	uint64_t takeoff_time;
	uint8_t nav_state;

	uint8_t arming_state;
	uint8_t hil_state;
	bool failsafe;

	uint8_t system_type;
	uint8_t system_id;
	uint8_t component_id;

	uint8_t vehicle_type;
	bool is_vtol;
	bool is_vtol_tailsitter;

	bool vtol_fw_permanent_stab;
	bool in_transition_mode;
	bool in_transition_to_fw;

	bool rc_signal_lost;
	bool data_link_lost;
	uint8_t data_link_lost_counter;
	
	bool high_latency_data_link_lost;
	bool engine_failure;
	bool mission_failure;
	bool geofence_violated;

	uint8_t failure_detector_status;
	uint8_t latest_arming_reason;
	uint8_t latest_disarming_reason;
    static constexpr uint8_t ARMING_STATE_INIT = 0;
	static constexpr uint8_t ARMING_STATE_STANDBY = 1;
	static constexpr uint8_t ARMING_STATE_ARMED = 2;
	static constexpr uint8_t ARMING_STATE_STANDBY_ERROR = 3;
	static constexpr uint8_t ARMING_STATE_SHUTDOWN = 4;
	static constexpr uint8_t ARMING_STATE_IN_AIR_RESTORE = 5;
	static constexpr uint8_t ARMING_STATE_MAX = 6;
	static constexpr uint8_t FAILURE_NONE = 0;
	static constexpr uint8_t FAILURE_ROLL = 1;
	static constexpr uint8_t FAILURE_PITCH = 2;
	static constexpr uint8_t FAILURE_ALT = 4;
	static constexpr uint8_t FAILURE_EXT = 8;
	static constexpr uint8_t FAILURE_ARM_ESC = 16;
	static constexpr uint8_t FAILURE_HIGH_WIND = 32;
	static constexpr uint8_t FAILURE_BATTERY = 64;
	static constexpr uint8_t FAILURE_IMBALANCED_PROP = 128;
	static constexpr uint8_t HIL_STATE_OFF = 0;
	static constexpr uint8_t HIL_STATE_ON = 1;
	static constexpr uint8_t NAVIGATION_STATE_MANUAL = 0;
	static constexpr uint8_t NAVIGATION_STATE_ALTCTL = 1;
	static constexpr uint8_t NAVIGATION_STATE_POSCTL = 2;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_MISSION = 3;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_LOITER = 4;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_RTL = 5;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_LANDENGFAIL = 8;
	static constexpr uint8_t NAVIGATION_STATE_UNUSED = 9;
	static constexpr uint8_t NAVIGATION_STATE_ACRO = 10;
	static constexpr uint8_t NAVIGATION_STATE_UNUSED1 = 11;
	static constexpr uint8_t NAVIGATION_STATE_DESCEND = 12;
	static constexpr uint8_t NAVIGATION_STATE_TERMINATION = 13;
	static constexpr uint8_t NAVIGATION_STATE_OFFBOARD = 14;
	static constexpr uint8_t NAVIGATION_STATE_STAB = 15;
	static constexpr uint8_t NAVIGATION_STATE_UNUSED2 = 16;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_TAKEOFF = 17;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_LAND = 18;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_PRECLAND = 20;
	static constexpr uint8_t NAVIGATION_STATE_ORBIT = 21;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22;
	static constexpr uint8_t NAVIGATION_STATE_MAX = 23;
	static constexpr uint8_t VEHICLE_TYPE_UNKNOWN = 0;
	static constexpr uint8_t VEHICLE_TYPE_ROTARY_WING = 1;
	static constexpr uint8_t VEHICLE_TYPE_FIXED_WING = 2;
	static constexpr uint8_t VEHICLE_TYPE_ROVER = 3;
	static constexpr uint8_t VEHICLE_TYPE_AIRSHIP = 4;
	static constexpr uint8_t ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0;
	static constexpr uint8_t ARM_DISARM_REASON_RC_STICK = 1;
	static constexpr uint8_t ARM_DISARM_REASON_RC_SWITCH = 2;
	static constexpr uint8_t ARM_DISARM_REASON_COMMAND_INTERNAL = 3;
	static constexpr uint8_t ARM_DISARM_REASON_COMMAND_EXTERNAL = 4;
	static constexpr uint8_t ARM_DISARM_REASON_MISSION_START = 5;
	static constexpr uint8_t ARM_DISARM_REASON_SAFETY_BUTTON = 6;
	static constexpr uint8_t ARM_DISARM_REASON_AUTO_DISARM_LAND = 7;
	static constexpr uint8_t ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8;
	static constexpr uint8_t ARM_DISARM_REASON_KILL_SWITCH = 9;
	static constexpr uint8_t ARM_DISARM_REASON_LOCKDOWN = 10;
	static constexpr uint8_t ARM_DISARM_REASON_FAILURE_DETECTOR = 11;
	static constexpr uint8_t ARM_DISARM_REASON_SHUTDOWN = 12;
	static constexpr uint8_t ARM_DISARM_REASON_UNIT_TEST = 13;
};

#define IMU_FIELD_COUNT 13
#define MAG_FIELD_COUNT 7
#define BARO_FIELD_COUNT 8
#define GPS_FIELD_COUNT 27
#define STATUS_FIELD_COUNT 31

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
#define   ACC_CALI_COUNT  11
#define   GYRO_CALI_COUNT  12

#define   MAG_DEVICE_ID 2 
#define   MAGNETOMETER_GA_X 3
#define   MAGNETOMETER_GA_Y 4
#define   MAGNETOMETER_GA_Z 5
#define   MAG_CALI_COUNT 6

#define   TIMESTAMP_SAMPLE 1 
#define   BARO_DEVICE_ID 2 
#define   BARO_ALT_METER 3
#define   BARO_TEMP_CELCIUS 4
#define   BARO_PRESSURE_PA 5
#define   RHO 6
#define   BARO_CALI_COUNT 6

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
	bool heading_good_for_control;
	bool xy_global;
	bool z_global;
	bool dist_bottom_valid;
	uint8_t dist_bottom_sensor_bitfield;
    static constexpr uint8_t DIST_BOTTOM_SENSOR_NONE = 0;
	static constexpr uint8_t DIST_BOTTOM_SENSOR_RANGE = 1;
	static constexpr uint8_t DIST_BOTTOM_SENSOR_FLOW = 2;
};
struct sensor_combined_s {
    uint64_t timestamp;
	float gyro_rad[3];
	uint32_t gyro_integral_dt;
	int32_t accelerometer_timestamp_relative;
	float accelerometer_m_s2[3];
	uint32_t accelerometer_integral_dt;
	uint8_t accelerometer_clipping;
	uint8_t accel_calibration_count;
	uint8_t gyro_calibration_count;

    static constexpr int32_t RELATIVE_TIMESTAMP_INVALID = 2147483647;
	static constexpr uint8_t CLIPPING_X = 1;
	static constexpr uint8_t CLIPPING_Y = 2;
	static constexpr uint8_t CLIPPING_Z = 4;
};

struct vehicle_gps_position_s {
	uint64_t timestamp;
	uint64_t time_utc_usec;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	int32_t alt_ellipsoid;
	float s_variance_m_s;
	float c_variance_rad;
	float eph;
	float epv;
	float hdop;
	float vdop;
	int32_t noise_per_ms;
	int32_t jamming_indicator;
	float vel_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float vel_d_m_s;
	float cog_rad;
	int32_t timestamp_time_relative;
	float heading;
	float heading_offset;
	uint8_t fix_type;
	uint8_t jamming_state;
	bool vel_ned_valid;
	uint8_t satellites_used;
	uint8_t selected;
};

struct vehicle_magnetometer_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	uint32_t device_id;
	float magnetometer_ga[3];
	uint8_t calibration_count;
};

struct vehicle_air_data_s {
    uint64_t timestamp;
	uint64_t timestamp_sample;
	uint32_t baro_device_id;
	float baro_alt_meter;
	float baro_temp_celcius;
	float baro_pressure_pa;
	float rho;
	uint8_t calibration_count;
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
    Ekf _ekf;
    FILE * _fp_out = NULL;
public:
    PostEkf(std::string filename, std::string mag_name,std::string baro_name,std::string gps_name, std::string status_name);
    PostEkf() = delete;
    ~PostEkf();

    void update();

private:
    sensor_combined_s sensor_combined = {0};
    vehicle_magnetometer_s magnetometer = {0};
    vehicle_magnetometer_s last_magnetometer = {0};
    vehicle_air_data_s airdata = {0};
    vehicle_air_data_s last_airdata = {0};
    vehicle_gps_position_s vehicle_gps_position = {0};
    vehicle_gps_position_s last_vehicle_gps_position = {0};
    vehicle_status_s vehicle_status = {0};
    //flag
    bool mag_updated = false;
    bool baro_updated = false;
    bool gps_updated = false;
    bool status_updated = false;
    CsvParser* csv_imu;
    CsvRow* imu_row = nullptr;
    CsvParser* csv_mag;
    CsvRow* mag_row= nullptr; 
    CsvParser* csv_baro;
    CsvRow* baro_row = nullptr;
    CsvParser* csv_gps;
    CsvRow* gps_row = nullptr;
    CsvParser* csv_status;
    CsvRow* status_row = nullptr;

    void write_header();
    void output_csv(const hrt_abstime &timestamp);
    void receive_imu(const char** row_fields);
    void receive_mag(const char** row_fields);
    void receive_baro(const char** row_fields);
    void receive_gps(const char** row_fields);
    void receive_status(const char** row_fields);


    // follow px4
	struct InFlightCalibration {
		hrt_abstime last_us{0};         ///< last time the EKF was operating a mode that estimates accelerometer biases (uSec)
		hrt_abstime total_time_us{0};   ///< accumulated calibration time since the last save
		bool cal_available{false};      ///< true when an unsaved valid calibration for the XYZ accelerometer bias is available
	};

	InFlightCalibration _accel_cal{};
	InFlightCalibration _gyro_cal{};
	InFlightCalibration _mag_cal{};

    // time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	uint8_t _accel_calibration_count{0};
	uint8_t _baro_calibration_count{0};
	uint8_t _gyro_calibration_count{0};
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

    float _param_ekf2_fuse_beta{0};
    float _param_ekf2_min_rng{0.1f};
    float _param_ekf2_gnd_eff_dz{4.0f};
    float _param_ekf2_gnd_max_hgt{0.5f};
    float _param_ekf2_mag_decl{0.f};
	int32_t _param_ekf2_aid_mask{1};

    bool _armed{false};

    void PublishAttitude(const hrt_abstime &timestamp);
	// void PublishEkfDriftMetrics(const hrt_abstime &timestamp);
	// void PublishEventFlags(const hrt_abstime &timestamp);
	// void PublishGlobalPosition(const hrt_abstime &timestamp);
	// void PublishInnovations(const hrt_abstime &timestamp, const imuSample &imu);
	// void PublishInnovationTestRatios(const hrt_abstime &timestamp);
	// void PublishInnovationVariances(const hrt_abstime &timestamp);
	void PublishLocalPosition(const hrt_abstime &timestamp);
	// void PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu);
	// void PublishOdometryAligned(const hrt_abstime &timestamp, const vehicle_odometry_s &ev_odom);
	// void PublishOpticalFlowVel(const hrt_abstime &timestamp, const optical_flow_s &optical_flow);
	// void PublishSensorBias(const hrt_abstime &timestamp);
	// void PublishStates(const hrt_abstime &timestamp);
	// void PublishStatus(const hrt_abstime &timestamp);
	// void PublishStatusFlags(const hrt_abstime &timestamp);
	// void PublishWindEstimate(const hrt_abstime &timestamp);
	// void PublishYawEstimatorStatus(const hrt_abstime &timestamp);

    void UpdateVehicleStatusSample();
    void UpdateBaroSample();
	void UpdateGpsSample();
	void UpdateMagSample();
	void UpdateAccelCalibration(const hrt_abstime &timestamp);
	void UpdateGyroCalibration(const hrt_abstime &timestamp);
    void UpdateMagCalibration(const hrt_abstime &timestamp);


};

#endif // __POST_EKF_2021__