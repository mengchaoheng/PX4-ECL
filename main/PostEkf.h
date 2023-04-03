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



struct vehicle_status_s {
    uint64_t timestamp;
	uint64_t nav_state_timestamp;
	uint64_t failsafe_timestamp;
	uint64_t armed_time;
	uint64_t takeoff_time;
	uint32_t onboard_control_sensors_present;
	uint32_t onboard_control_sensors_enabled;
	uint32_t onboard_control_sensors_health;
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
	uint8_t rc_input_mode;
	bool data_link_lost;
	uint8_t data_link_lost_counter;
	bool high_latency_data_link_lost;
	bool engine_failure;
	bool mission_failure;
	bool geofence_violated;
	uint8_t failure_detector_status;
	uint8_t latest_arming_reason;
	uint8_t latest_disarming_reason;
	uint8_t _padding0[4]; // required for logger
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
	static constexpr uint8_t HIL_STATE_OFF = 0;
	static constexpr uint8_t HIL_STATE_ON = 1;
	static constexpr uint8_t NAVIGATION_STATE_MANUAL = 0;
	static constexpr uint8_t NAVIGATION_STATE_ALTCTL = 1;
	static constexpr uint8_t NAVIGATION_STATE_POSCTL = 2;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_MISSION = 3;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_LOITER = 4;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_RTL = 5;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_LANDENGFAIL = 8;
	static constexpr uint8_t NAVIGATION_STATE_AUTO_LANDGPSFAIL = 9;
	static constexpr uint8_t NAVIGATION_STATE_ACRO = 10;
	static constexpr uint8_t NAVIGATION_STATE_UNUSED = 11;
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
	static constexpr uint8_t NAVIGATION_STATE_MAX = 22;
	static constexpr uint8_t RC_IN_MODE_DEFAULT = 0;
	static constexpr uint8_t RC_IN_MODE_OFF = 1;
	static constexpr uint8_t RC_IN_MODE_GENERATED = 2;
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

#define IMU_FIELD_COUNT 11
#define MAG_FIELD_COUNT 7
#define BARO_FIELD_COUNT 7
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
    void UpdateMagCalibration(const hrt_abstime &timestamp);


};

#endif // __POST_EKF_2021__