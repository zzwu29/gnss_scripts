# gnss_scripts

**Original Authors:** CAO Shaozu (shaozu.cao AT gmail.com)

modifier: zzwu29

The *gnss_comm* package contains basic definitions and utility functions for GNSS raw measurement processing. 

## 1. Prerequisites

### 1.1 C++11 Compiler
This package requires some features of C++11.

### 1.2 ROS
This package is developed under [ROS Kinetic](http://wiki.ros.org/kinetic) environment.

### 1.3 Eigen
Our code uses [Eigen 3.3.3](https://gitlab.com/libeigen/eigen/-/archive/3.3.3/eigen-3.3.3.zip) for matrix manipulation. After downloading and unzipping the Eigen source code package, you may install it with the following commands:

```
cd eigen-3.3.3/
mkdir build
cd build
cmake ..
sudo make install
```

### 1.4 Glog
We use google's glog library for message output. If you are using Ubuntu, install it by:
```
sudo apt-get install libgoogle-glog-dev
```
If you are on other OS or just want to build it from source, please follow [these instructions](https://github.com/google/glog#building-glog-with-cmake) to install it.


## 2. Build gnss_comm library
Clone the repository to your catkin workspace (for example `~/catkin_ws/`):
```
cd ~/catkin_ws/src/
git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git
```
Then build the package with:
```
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```
If you encounter any problem during the building of *gnss_comm*, try with docker in [the next section](#docker_section).

## 3. <a name="docker_section"></a>Docker Support
To simplify the building process, we add docker in our code. Docker is like a sandbox so it can isolate our code from your local environment. To run with docker, first make sure [ros](http://wiki.ros.org/ROS/Installation) and [docker](https://docs.docker.com/get-docker/) are installed on your machine. Then add your account to `docker` group by `sudo usermod -aG docker $USER`. **Relaunch the terminal or logout and re-login if you get `Permission denied` error**, type:
```
cd ~/catkin_ws/src/gnss_comm/docker
make build
```
The docker image `gnss_comm:latest` should be successfully built after a while. Then you can check all available docker images in your local machine by `docker image ls` command.

## 4. Dataset Details
The dataset is released in the form of rosbag and currently there are two rosbags available:

| name | duration | size | link | 
| :--: | :------: | :--: | :--: |
| sports_field | 25min | 20.5GB | [OneDrive](https://hkustconnect-my.sharepoint.com/:u:/g/personal/scaoad_connect_ust_hk/Eb5EuCmYR4RBnXbpjV8EfNQBORvT0WPtGXMbwPa4Rmx6Lg?e=SYIHUv) |
| complex_environment | 32min | 26.1GB | [OneDrive](https://hkustconnect-my.sharepoint.com/:u:/g/personal/scaoad_connect_ust_hk/EalZKULm8QFPqNZlf53C31QBmcQ1KUsWnOQ6N2rIefNBYA?e=QUbvHe) |
| urban_driving | 41min | 33.4GB | [OneDrive](https://hkustconnect-my.sharepoint.com/:u:/g/personal/scaoad_connect_ust_hk/EX1pIitB4iFIqXjf2JZ_LYMBbENBrC1Rnl18KQKGeH3T0A?e=eFTp1f) |

The data items within the rosbag are listed below:
| topic | type | frequency | description |
| :---: | :--: | :-------: | :---------: |
| /cam0/image_raw | sensor_msgs/Image | 20Hz | right camera |
| /cam1/image_raw | sensor_msgs/Image | 20Hz | left camera |
| /imu0 | sensor_msgs/Imu | 200Hz | IMU |
| /external_trigger | gvins/LocalSensorExternalTrigger | - | publish when VI-Sensor is trigger. [definition](https://github.com/HKUST-Aerial-Robotics/GVINS/blob/main/estimator/msg/LocalSensorExternalTrigger.msg)
| /ublox_driver/receiver_lla | sensor_msgs/NavSatFix | 10Hz | Receiver's GNSS solution (brief). |
| /ublox_driver/receiver_pvt | gnss_comm/GnssPVTSolnMsg | 10Hz | Receiver's GNSS solution (verbose). [definition](https://github.com/HKUST-Aerial-Robotics/gnss_comm/blob/main/msg/GnssPVTSolnMsg.msg) |
| /ublox_driver/range_meas | gnss_comm/GnssMeasMsg | 10Hz | GNSS raw measurement. [definition](https://github.com/HKUST-Aerial-Robotics/gnss_comm/blob/main/msg/GnssMeasMsg.msg) | 
| /ublox_driver/ephem | gnss_comm/GnssEphemMsg | - | The broadcast ephemeris of GPS, Galileo and BeiDou. [definition](https://github.com/HKUST-Aerial-Robotics/gnss_comm/blob/main/msg/GnssEphemMsg.msg) |
| /ublox_driver/glo_ephem | gnss_comm/GnssGloEphemMsg | - | The broadcast ephemeris of GLONASS. [definition](https://github.com/HKUST-Aerial-Robotics/gnss_comm/blob/main/msg/GnssGloEphemMsg.msg) | 
| /ublox_driver/iono_params | gnss_comm/StampedFloat64Array | - | The broadcast ionospheric parameters. [definition](https://github.com/HKUST-Aerial-Robotics/gnss_comm/blob/main/msg/StampedFloat64Array.msg) | 
| /ublox_driver/time_pulse_info | gnss_comm/GnssTimePulseInfoMsg | 1Hz | The time information of next PPS signal. [definition](https://github.com/HKUST-Aerial-Robotics/gnss_comm/blob/main/msg/GnssTimePulseInfoMsg.msg).

## 5. Toolkit

**The toolkit provided in this package requires [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) library.**
### 5.1. Convert GNSS raw measurement to RINEX File

```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
rosrun gvins_dataset_toolkit bag2rinex INPUT_BAG_FILEPATH OUTPUT_RINEX_FILEPATH
```
The observation RINEX file should be generated after a while. The corresponding GNSS ephemeris RINEX file can be found in `GVINS-Dataset/data/ephemeris_rinex/`. 

### 5.2 Save RTK solution to csv file

```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
rosrun gvins_dataset_toolkit bag2rtk_solution INPUT_BAG_FILEPATH OUTPUT_RINEX_FILEPATH
```
Each record in the generated csv file is in the form of:
```
gnss_ts_ns, ecef_px, ecef_py, ecef_pz, enu_vx, enu_vy, enu_vz, fix_type, valid_fix, diff_soln, carr_soln
```
, with each item described in the following:
| name | description | 
| :--: | :---------: |
| gnss_ts_ns | GNSS time of the navigation epoch (expressed as Unix timestamp in ns) |
| ecef_p* | The x, y, z component of the position in ECEF frame |
| enu_v* | The x, y, z component of the velocity in ENU frame |
| fix_type | GNSS fix type (0=no fix, 1=dead reckoning only, 2=2D-fix, 3=3D-fix, 4=GNSS+dead reckoning combined, 5=time only fix) |
| valid_fix | if fix valid (1=valid fix) |
| diff_soln | if differential correction were applied (1=applied) |
| carr_soln | carrier phase range solution status (0=no carrier phase, 1=float, 2=fix) |


## 6. Acknowledgements
Many of the definitions and utility functions in this package are adapted from [RTKLIB](http://www.rtklib.com/).

[Project gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm)

[Project GVINS-Dataset](https://github.com/HKUST-Aerial-Robotics/GVINS-Dataset)

## 7. License
The source code is released under [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html) license.
