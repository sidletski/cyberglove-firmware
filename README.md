# Cyberglove Firmware

[SparkFun 9DoF Razor IMU M0](https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide)

## Content

The repository contains two firmware: one that reads values from IMU sensors directly through the I2C bus (stable / is able to work with the server), and one that calculates the quaternion based on functions from the SparkFunMPU9250-DMP.h library (experimental)

## Compiling

For successful compilation and firmware upload you need to choose SparkFun 9DoF Razor IMU M0 board in the _Tools/Board manager_
