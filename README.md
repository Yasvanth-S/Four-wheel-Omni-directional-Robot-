# **Quicksilver - Omniwheel robot with + configuration** 

## **Simulation, SLAM and Navigation part**
1. Gazebo urdf done with ros control.
2. Sensors used are rplidar and imu.
3. Urdf with four wheel omni drive.
4. Odom and Tf is published.
5. Gmapping is done.
6. 3 custom gazebo worlds for mapping.
7. Navigation stack implemented need to tune the parameters for omniwheel drive

## **Hardware part**

### **Components**
1. Jetson nano
2. wifi and bluetooth module for jetson
3. USB 2.0 to TTL UART serial converter Module
4. RMCS 2303 motor driver
5. RHINO 12V DC 60RPM 40KGCM IG32 HEAVY DUTY PLANETARY GEARED MOTOR
6. 16000mAh 6S 25C/50C (22.2V) Lithium Polymer Battery
7. DC Buck Converter
8. Rp lidar A2M8
9. Joystick

### **Programming**
1. USB setup rules for UART.
2. startup file.
3. motor communication code using modbus.
4. Joy node and Twist to motor.
5. Odom and tf publish using wheel encoders.
6. Gmapping is done and code will be uploaded soon for gmapping and odom.

### **USB name after setting up rules**

                        | S.NO |  VID | PID  |         NAME        |
                        |:----:|:----:|:----:|:-------------------:|
                        |  1   | 0001 | 0001 | quicksilver wheel 1 |
                        |  2   | 0002 | 0002 | quicksilver wheel 2 |
                        |  3   | 0003 | 0003 | quicksilver wheel 3 |
                        |  4   | 0004 | 0004 | quicksilver wheel 4 |

## **Gmapping Image**
![Gmapping](https://github.com/Yasvanth-S/quicksilver/blob/master/assets/Gmapping.png?raw=true)

##**Navigation Stack Simulation**
![Navigation](https://github.com/Yasvanth-S/quicksilver/blob/master/assets/navigation.gif?raw=true)

##**Navigation Stack Obstacle Avoidance Simulation**
![Obstacle Avoidance](https://github.com/Yasvanth-S/quicksilver/blob/master/assets/obstacle_avoidance.gif?raw=true)
