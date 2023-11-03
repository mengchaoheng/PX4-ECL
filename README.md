# ECL

**Very lightweight Estimation & Control Library.**

[![DOI](https://zenodo.org/badge/22634/PX4/PX4-ECL.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/PX4-ECL)

This library solves the estimation & control problems of a number of robots and drones. It accepts GPS, vision and inertial sensor inputs. It is extremely lightweight and efficient and yet has the rugged field-proven performance.

The library is BSD 3-clause licensed.

## EKF Documentation

  * [EKF Documentation and Tuning Guide](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html)

## Building EKF

### Prerequisites:

  * Matrix: A lightweight, BSD-licensed matrix math library: https://github.com/px4/matrix - it is automatically included as submodule.


By following the steps mentioned below you can create a static library which can be included in projects:

```
make
// OR
mkdir build/
cd build/
cmake ..
make
```

## Testing ECL
By following the steps you can run the unit tests:

```
make test
```
## Offline ECL
By running the following command, you can use the ulg file of px4 to run the ekf algorithm offline.

1. Put the .ulg file in the log_data folder and run the gen_csv_from_ulg.m file (need to pre-install python and ulog2csv). 
> Note that you must use a log format compatible with px4 v1.12.3, otherwise you need to modify the program and recompile to get the data correctly.

2. build ECL lib.
```
mkdir build/
cd build/
cmake ..
make
```
The appliction would be in `build\main\postEcl`, Run postEcl application
```
cd build/
./main/postEcl
```
3. run main.m in the log_data files. Make sure the correct ulg file name is set in gen_csv_from_ulg.m and main.m.

The output file would be in result files.
```
ecloutput.csv          
euler_estimator.txt    
output.txt             
position_estimator.txt 
velocity_estimator.txt
```
The offline version of ecl makes it easier to port to other platforms. You only need the ulg file to test the performance of ekf, and then you can port it directly.

The following is a comparison between the online version of ekf and the offline version of ekf:

<img src="./log_data/RPY.png" width="70%" height="70%" />

<img src="./log_data/pos.png" width="70%" height="70%" />

<img src="./log_data/vel.png" width="70%" height="70%" />

## data 

- 17_48_41.ulg
the imu, mag, baro, gps data from sitl used to test ekf.
1. version
> Airframe:	10016 \
Hardware:	PX4_SITL \
Software Version:	6bcf1fb7 \
branch: ekf-df \
Estimator:	EKF2
2. fusion control
```C++
// measurement source control
	int32_t fusion_mode{MASK_USE_GPS};		///< bitmasked integer that selects which aiding sources will be used
	int32_t vdist_sensor_type{VDIST_SENSOR_BARO};	///< selects the primary source for height data
```
3. step
```Console
gen_csv_from_ulg.m 
cd build 
cmake && make 
./main/postEcl
```

- log36.ulg and ros.csv
the optitrack data used to test ekf.
1. version
> Airframe:	Generic Quadcopter \
Quadrotor x (4001) \
Hardware:	PX4_FMU_V5 (V550) \
Software Version:	c6186ac0 \
branch: off \
OS Version:	NuttX, v8.2.0 \
Estimator:	EKF2
2. fusion control
```C++
// measurement source control
	int32_t fusion_mode{24};		///< bitmasked integer that selects which aiding sources will be used
	int32_t vdist_sensor_type{3};	///< selects the primary source for height data
```
3. step
```Console
gen_csv_from_ulg.m # log36
cd build 
cmake && make # log36
./main/postEcl
main.m # log36, this is an old version, maybe need save opti.mat before, or
plot_ros_ulog_offline.m # this can use ros.csv by a new way to convert data, or
plot_ros_ulog.m # plot ros and ulog data.
plot_ros.m # only plot ros data， simply convert data.
```
## matlab replay
1. cd to `PX4-ECL/EKF/matlab/EKF_replay` and run `px4_replay_import.m` , you need to run gen_csv_from_ulg before and decide use gps data or not. The name of files need to be change. This step will generate `*_data.mat` file which used in replay.

```
sensors_file = '../../../csv_data/17_48_41_sensor_combined_0';
air_data_file = '../../../csv_data/17_48_41_vehicle_air_data_0';
magnetometer_file = '../../../csv_data/17_48_41_vehicle_magnetometer_0';
% or don't use it
% gps_file = '../../../csv_data/17_48_41_vehicle_gps_position_0';
```

2. cd to `PX4-ECL/EKF/matlab/EKF_replay/Filter` and run `replay_px4_data.m`. Then the result as follow:

<img src="./EKF/matlab/EKF_replay/OutputPlots/rpy.png" width="100%" height="100%" />

### Change Indicator / Unit Tests
Change indication is the concept of running the EKF on different data-sets and compare the state of the EKF to a previous version. If a contributor makes a functional change that is run during the change_indication tests, this will produce a different output of the EKF's state. As the tests are run in CI, this checks if a contributor forgot to run the checks themselves and add the [new EKF's state outputs](https://github.com/PX4/ecl/blob/master/test/change_indication/iris_gps.csv) to the pull request.

The unit tests include a check to see if the pull request results in a difference to the [output data csv file](https://github.com/PX4/ecl/blob/master/test/change_indication/iris_gps.csv) when replaying the [sensor data csv file](https://github.com/PX4/ecl/blob/master/test/replay_data/iris_gps.csv). If a pull request results in an expected difference, then it is important that the output reference file be re-generated and included as part of the pull request. A non-functional pull request should not result in changes to this file, however the default test case does not exercise all sensor types so this test passing is a necessary, but not sufficient requirement for a non-functional pull request.

The functionality that supports this test consists of:
* Python scripts that extract sensor data from ulog files and writes them to a sensor data csv file. The default [sensor data csv file](https://github.com/PX4/ecl/blob/master/test/replay_data/iris_gps.csv) used by the unit test was generated from a ulog created from an iris SITL flight.
* A [script file](https://github.com/PX4/ecl/blob/master/test/test_EKF_withReplayData.cpp) using functionality provided by  the [sensor simulator](https://github.com/PX4/ecl/blob/master/test/sensor_simulator/sensor_simulator.cpp), that loads sensor data from the [sensor data csv file](https://github.com/PX4/ecl/blob/master/test/replay_data/iris_gps.csv) , replays the EKF with it and logs the EKF's state and covariance data to the [output data csv file](https://github.com/PX4/ecl/blob/master/test/replay_data/iris_gps.csv).
* CI action that checks if the logs of the test running with replay data is changing. This helps to see if there are functional changes.

#### How to run the Change Indicator test during development on your own logs:

* create sensor_data.csv file from ulog file 'cd test/sensor_simulator/
python3 createSensorDataFile.py <path/to/ulog> ../replay_data/<descriptive_name>.csv'
* Setup the test file to use the EKF with the created sensor data by copy&paste an existing test case in [test/test_EKF_withReplayData.cpp](https://github.com/PX4/ecl/blob/master/test/test_EKF_withReplayData.cpp) and adapt the paths to load the right sensor data and write it to the right place, eg
_sensor_simulator.loadSensorDataFromFile("../../../test/replay_data/<descriptive_name>.csv");
_ekf_logger.setFilePath("../../../test/change_indication/<descriptive_name>.csv");
* You can feed the EKF with the data in the csv file, by running '_sensor_simulator.runReplaySeconds(duration_in_seconds)'. Be aware that replay sensor data will only be available when the corresponding sensor simulation are running. By default only imu, baro and mag sensor simulators are running. You can start a sensor simulation by calling _sensor_simulator._<sensor>.start(). Be also aware that you still have to setup the EKF yourself. This includes setting the bit mask (fusion_mode in common.h) according to what you intend to fuse.
* In between _sensor_simulator.runReplaySeconds(duration_in_seconds) calls, write the state and covariances to the change_indication file by including a _ekf_logger.writeStateToFile(); line.
* Run the EKF with your data and all the other tests by running 'make test' from the ecl directory. The [default output data csv file](https://github.com/PX4/ecl/blob/master/test/change_indication/iris_gps.csv) changes can then be included in the PR if differences are causing the CI test to fail.

#### Known Issues
If compiler versions other than GCC 7.5 are used to generate the output data file, then is is possible that the file will cause CI failures due to small numerical differences to file generated by the CI test.
