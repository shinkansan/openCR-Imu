# openCR IMU by shinkansan

## Dependencies
rosserial-python
turtolebotmsg

all files are included in release zip file 

## Downloads
[release](https://github.com/DGIST-ARTIV/openCR-Imu/releases/)


## out topic
* /opencr/imu_yaw <Float64>
  
## How to Connect IMU
`sudo chmod 777 /dev/ttyACM0`

## How to Start IMU driver
`roslaunch opencrImu imu_driver.launch`




