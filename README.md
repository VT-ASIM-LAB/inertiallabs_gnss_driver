CARMA Inertial Labs GNSS/INS Driver
===================================

This is a fork of the [Inertial Labs ROS](https://us.inertiallabs.com:31443/projects/INS/repos/inertiallabs-ros-pkgs/browse) package that is used for connecting to [Inertial Labs](https://inertiallabs.com/products/) GNSS/INS, IMU-P, AHRS, and AHRS-10 products. This fork has been modified to allow for building a Docker image that can serve as both a GNSS/INS driver as well as a standalone INS driver for the [CARMA Platform](https://github.com/usdot-fhwa-stol/carma-platform).

Ubuntu 20.04 Installation
-------------------------
Assuming the CARMA Platform is installed at `~/carma_ws/src`,
```
cd ~/carma_ws/src
git clone https://github.com/VT-ASIM-LAB/inertiallabs_gnss_driver.git
cd inertiallabs_gnss_driver/docker
sudo ./build-image.sh -d
```
After the Docker image is successfully built, add the following lines to the appropriate `docker-compose.yml` file in the `carma-config` directory, and make sure that the current user (and not `root`) is the owner of `/dev/ttyUSB0`.
```
inertiallabs-gnss-driver:
  image: usdotfhwastoldev/carma-inertiallabs-gnss-driver:develop
  container_name: inertiallabs-gnss-driver
  network_mode: host
  privileged: true
  devices:
    - /dev/ttyUSB0:/dev/ttyUSB0
  volumes_from:
    - container:carma-config:ro
  environment:
    - ROS_IP=127.0.0.1
  volumes:
    - /opt/carma/logs:/opt/carma/logs
    - /opt/carma/.ros:/home/carma/.ros
    - /opt/carma/vehicle/calibration:/opt/carma/vehicle/calibration
  command: bash -c '. ./devel/setup.bash && export ROS_NAMESPACE=$${CARMA_INTR_NS} && wait-for-it.sh localhost:11311 -- roslaunch /opt/carma/vehicle/config/drivers.launch drivers:=inertiallabs_gnss'
```
Finally, add the following lines to the `drivers.launch` file in the same directory as `docker-compose.yml`.
```
<include if="$(arg inertiallabs_gnss)" file="$(find inertiallabs_ins)/launch/ins.launch">
</include>
```
`ins.launch` is used as an example here and can be replaced with any other launch file in the `inertiallabs_ins/launch` directory.

ROS API (stable)
----------------

### inertiallabs_ins

#### Nodes
* `il_ins`

#### Topics
* ``: .
* ``: .
* `il_ins/discovery`: publishes the CARMA [DriverStatus](https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/cav_msgs/msg/DriverStatus.msg) message.

#### Services
* ``

#### Parameters
* ``: .
* `parameter`: .

Examples
--------

See the `ins.launch` file in the `inertiallabs_ins/launch` directory that is used to launch an Inertial Labs INS-D GNSS/INS system.

Original Inertial Labs ROS Documentation
========================================

ROS Packages for Inertial Labs Products. Current supported devices INS, IMU-P, AHRS, AHRS-10.

[![alt text](https://readthedocs.org/projects/docs/badge/?version=latest "Documentation Status")](https://gitlab.com/oblivione/inertiallabs_ros_pkgs)

![Picture of IMU](http://inertiallabs.com/static/assets/img/products/INS-D.jpg)

The `inertiallabs_ros_pkgs` package is a linux ROS driver for GPS-Aided Inertial Navigation Systems (INS), IMU-P, AHRS and AHRS-10 of [Inertial Labs](https://inertiallabs.com/). The package is developed based on the official [SDK v0.2](https://gitlab.com/oblivione/inertiallabs_sdk) for Linux.

The package is tested on Ubuntu 16.04 LTS & 18.04 LTS  with ROS Kinetic & ROS Melodic . You can install ROS from [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).

License
-------

* The license for the official SDK is the MIT license which is included in the `ins_ros/inertiallabs_sdk`
* The license for the other codes is Apache 2.0 whenever not specified.

Compiling
---------

This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compiling a catkin package will work.

```
$ cd <your_work_space>/src
$ git clone https://us.inertiallabs.com:31443/scm/ins/inertiallabs-ros-pkgs.git
$ cd <your_work_space>
$ catkin_make_isolated
$ source devel_isolated/setup.bash
```

Why `catkin_make_isolated`?

We developed a package for multiple devices with same dependencies, so to avoid conflicts we are compiling using `catkin_make_isolated`. If you need
to use `ins` only, so you get the subpackages like `inertiallabs_ins`, `inertiallabs_sdk`, `intertiallabs_msgs` and `intertiallabs_ros_pkgs` to your workspace and use your own build system to compile it. You have to change the `intertiallabs_ros_pkgs/intertiallabs_ros_pkgs/package.xml`.

```
<exec_depend>inertiallabs_ins</exec_depend>
```

### Node

example rosnodes:

for ins OPVT2AHR packet via USB serial port:
```
rosrun inertiallabs_ins il_ins url:=serial:/dev/ttyUSB0:460800 _ins_output_format:=0x58
```

for ins OPVT packet via UDP (INS hostname is used):
```
rosrun inertiallabs_ins il_ins _ins_url:=udp:INS-F2001234:23 _ins_output_format:=0x52
```

for ins OPVT packet via UDP (INS IP address is used):
```
rosrun inertiallabs_ins il_ins _ins_url:=udp:192.168.0.249:23 _ins_output_format:=0x52
```

## Example Usage

### Parameters

`url` (`string`, `default: serial:/dev/ttyUSB0:115200`)

Port the device is connected to. Can be serial:[path to device]:[baudrate], tcp:[hostname or address]:[tcp server port], or udp:[hostname or address]:[udp server port]. Inertial Labs Driver supports serial connection

`ins_output_format` (`int`, `82`)

The output data format of the INS data according to IL INS ICD.

```
 IL_SENSOR_DATA             0x50
 IL_OPVT                    0x52
 IL_MINIMAL_DATA            0x53
 IL_QPVT                    0x56
 IL_OPVT2A                  0x57
 IL_OPVT2AHR                0x58
 IL_OPVT2AW                 0x59
 IL_OPVTAD                  0x61
 MRU_OPVTHSSHR              0x64
 IL_OPVT_RAWIMU_DATA        0x66
 IL_OPVT_GNSSEXT_DATA       0x67
 IL_USER_DEFINED_DATA       0x95
```

### Published Topics - feel free to modify using fields from IL::INSDataStruct

`/Inertial_Labs/sensor_data` (`ins_ros/sensor_data`)

Publish Gyro(x,y,z), Accelation(x,y,z), Magnetic (x,y,z), Temprature, Input Voltage, Pressure, Barometric height.

`/Inertial_Labs/ins_data` (`ins_ros/ins_data`)

Publish GPS INS Time, GPS IMU Time, Millisecond of the week, Latitude, Longitude, Altitude, Heading, Pitch, Roll, Orientation quaternion, East Velocity, North Velocity, Up Velocity values, Solution status, Position STD, Heading STD, Unit Status.

`/Inertial_Labs/gps_data` (`ins_ros/gps_data`)

Publish Latitude, Longitude, Altitude, Ground Speed, Track Direction,  Vertical Speed values.

`/Inertial_Labs/gnss_data` (`ins_ros/gnss_data`)

Publish GNSS service Info 1, Info 2, Satellites Used, Velocity Latency, Heading status, Heading, Pitch, GDOP, PDOP, HDOP, VDOP, TDOP, New GNSS Flag, Age of differenctiol correction.

`/Inertial_Labs/marine_data` (`ins_ros/marine_data`)

Publish  Heave, Surge, Sway, Heave Velocity, Surge Velocity, Sway Velocity, Significant Wave Height.


## FAQ

1. Why can't the driver open my serial device?\
Make sure you have ownership of the device in `/dev`.

2. Why do I see a permission error during the initialization process of the driver?\
Most often, this is because the baud rate you set does not match the package size to be received. Try increase the baud rate.

3. Why is the IMU data output rate much lower than what is set?\
This may be due to a recent change in the FTDI USB-Serial driver in the Linux kernel, the following shell script might help:
    ```
    bash
    # Reduce latency in the FTDI serial-USB kernel driver to 1ms
    # This is required due to https://github.com/torvalds/linux/commit/c6dce262
    for file in $(ls /sys/bus/usb-serial/devices/); do
      value=`cat /sys/bus/usb-serial/devices/$file/latency_timer`
      if [ $value -gt 1 ]; then
        echo "Setting low_latency mode for $file"
        sudo sh -c "echo 1 > /sys/bus/usb-serial/devices/$file/latency_timer"
      fi
    done
    ```
4. Why is a field value always zero?\
Most likely, because this field is not provided in the selected INS data packet. The most versatile data packet is User-Defined Data, which allows to order any set of fields

## Bug Report

Opening an issue is preferred. You can also send an E-mail to support@inertiallabs.com.
