#pragma once




struct estimator_innovations_s {
	unsigned long long timestamp;
	unsigned long long timestamp_sample;
	float gps_hvel[2];
	float gps_vvel;
	float gps_hpos[2];
	float gps_vpos;
	float ev_hvel[2];
	float ev_vvel;
	float ev_hpos[2];
	float ev_vpos;
	float rng_vpos;
	float baro_vpos;
	float aux_hvel[2];
	float aux_vvel;
	float flow[2];
	float heading;
	float mag_field[3];
	float drag[2];
	float airspeed;
	float beta;
	float hagl;
};

