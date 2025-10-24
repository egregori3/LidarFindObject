git clone git@github.com:Slamtec/rplidar_sdk.git

cd ~/rplidar_sdk/app/ultra_simple

rm *

git clone git@github.com:egregori3/LidarFindObject.git .

(base) eric@raspberrypi:~/rplidar_sdk/app/ultra_simple $ ls
calibration.cpp  filter.cpp  LidarFindObject.h  main.cpp  Makefile  run.cpp  server.cpp

~/rplidar_sdk $ make

```
(base) eric@raspberrypi:~/rplidar_sdk/app/ultra_simple $ make
 CXX  main.cpp
 CXX  calibration.cpp
 CXX  filter.cpp
 CXX  server.cpp
 CXX  run.cpp
mkdir -p `dirname /home/eric/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a`
 pack main.o->libsl_lidar_sdk.a
 pack calibration.o->libsl_lidar_sdk.a
 pack filter.o->libsl_lidar_sdk.a
 pack server.o->libsl_lidar_sdk.a
 pack run.o->libsl_lidar_sdk.a
 LD   /home/eric/rplidar_sdk/output/Linux/Release/ultra_simple
 ```
 
 Execute code:
 ```
 (base) eric@raspberrypi:~/rplidar_sdk/output/Linux/Release $ ./ultra_simple
Find Object Using LIDAR
SDK Version: 2.1.0
Compiled: Oct 23 2025 19:00:51

Using default serial port /dev/ttyUSB0


Usage: -s <filename> to save calibration file
       -l <filename> to load calibration file [min angle, max angle]
       -x <filename> to load calibration file [min angle, max angle] output X,Y
       -f <filename> to load calibration file [min angle, max angle] output filtered data
```
Load calibration data from (null)
Error opening file for reading: (null)
