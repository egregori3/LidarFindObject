/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 * Pulled from https://github.com/Slamtec/rplidar_sdk
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <limits>
#include <cmath>

#define DISTANCE_COUNT (360*10)
#define MAX_CALIBRATION_SCANS 100
#define NOISE_BUFFER_SIZE 300
#define MAX_DISTANCE 10000

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;



bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

void print_usage()
{
       printf("\n\nUsage: -s <filename> to save calibration file\n");
        printf("       -l <filename> to load calibration file\n");
}

void run_mode(FILE * file, ILidarDriver * drv)
{
    sl_result    op_result;
    int          distances[DISTANCE_COUNT] = {0};
 
    // load calibration data from file
    if (file) 
    {
        int angle;
        int distance;
        while (fscanf(file, "%d %d\n", &angle, &distance) == 2) 
        {
            if (angle >= 0 && angle < DISTANCE_COUNT) 
            {
                if((distance > NOISE_BUFFER_SIZE) && (distance < MAX_DISTANCE))
                    distances[angle] = distance - NOISE_BUFFER_SIZE;
                else
                    distances[angle] = 0;
            }
        }
    }

    while (!ctrl_c_pressed) 
    {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) 
        {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) 
            {
                float angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                float dist = nodes[pos].dist_mm_q2/4.0f;
                int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                if (quality > 0 && dist > 0) 
                {
                    int inter_angle = (int)round(angle * 10.0f);
                    if (inter_angle >= 0 && inter_angle < DISTANCE_COUNT )
                    {
                        int calib_dist = distances[inter_angle];
                        if(dist < calib_dist)
                            printf("Angle: %03.2f Dist: %08.2f calib_dist: %d\n", angle, dist, calib_dist);
                    }
                }
            }
        }
    }
}

void calibrate_mode(FILE * file, ILidarDriver * drv)
{
    sl_result    op_result;
    int          distances[DISTANCE_COUNT] = {0};
    std::fill(distances, distances + DISTANCE_COUNT, std::numeric_limits<int>::max());
    int          flags[DISTANCE_COUNT] = {0};
    int          angle_count = 0;
    int          scan_count = 0;

    // Calibration mode implementation
    // fetech result and print it out...
    while (!ctrl_c_pressed) 
    {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) 
        {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) 
            {
                float angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                float dist = nodes[pos].dist_mm_q2/4.0f;
                int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                printf("%s theta: %03.2f Dist: %08.2f Q: %d Count: %d\n", 
                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                    angle, dist, quality, angle_count);
                if (quality >0) 
                {
                    int inter_angle = (int)round(angle * 10.0f);
                    if (inter_angle < 0) inter_angle = 0;
                    if (inter_angle >= DISTANCE_COUNT) inter_angle = DISTANCE_COUNT - 1;
                    if (dist < distances[inter_angle]) 
                    {
                        distances[inter_angle] = (int)dist;
                        if(!flags[inter_angle]) 
                        {
                            flags[inter_angle] = 1;
                            angle_count++;
                            if(angle_count == DISTANCE_COUNT) {
                                printf("All angles have been calibrated\n");
                                ctrl_c_pressed = true;
                                break;
                            }
                        }
                    }
                }   
            }
        }

        if(scan_count++ > MAX_CALIBRATION_SCANS) 
        {
            printf("Maximum calibration scans reached\n");
            break;
        }

        if (ctrl_c_pressed){ 
            break;
        }
    }

    if (file) 
    {
        // save calibration data to file
        for (int pos = 0; pos < DISTANCE_COUNT; ++pos) 
        {
            fprintf(file, "%d %d\n", pos, distances[pos]);
        }
        fclose(file);
        file = NULL;
        printf("Calibration data saved\n");
    }
}   

int main(int argc, const char * argv[]) 
{
    const char * comm_port = NULL;
    FILE       * file = NULL;
    sl_u32       baudrateArray[2] = {115200, 256000};
    IChannel*    _channel;
    int          save_flag = 0;

    printf("Find Object Using LIDAR\nSDK Version: %s\n", SL_LIDAR_SDK_VERSION);
    printf("Compiled: %s %s\n", __DATE__, __TIME__);

#ifdef _WIN32
		// use default com port
		comm_port = "\\\\.\\com3";
#elif __APPLE__
		comm_port = "/dev/tty.SLAB_USBtoUART";
#else
		comm_port = "/dev/ttyUSB0";
#endif	 

    printf("\nUsing default serial port %s\n", comm_port);

    if (argc == 3) 
    {
        if (strcmp(argv[1], "-s") == 0) 
        {
            // create file argv[2] to save calibration data
            const char * calibration_filename = argv[2];
            printf("Save calibration data to %s\n", calibration_filename);
            // open file for writing
            file = fopen(calibration_filename, "w");
            if (!file) {
                fprintf(stderr, "Error opening file for writing: %s\n", calibration_filename);
                return -1;
            }

            save_flag = 1;
        }
        else if (strcmp(argv[1], "-l") == 0) 
        {
            // load calibration data from file argv[2]
            const char * calibration_filename = argv[2];
            printf("Load calibration data from %s\n", calibration_filename);
            // open file for reading
            file = fopen(calibration_filename, "r");
            if (!file) {
                fprintf(stderr, "Error opening file for reading: %s\n", calibration_filename);
                return -1;
            }
        }
        else
        {
            print_usage();
            return -1;
        }
    }
    else
    {
        print_usage();
        return -1;
    }

    // create the driver instance
	ILidarDriver * drv = *createLidarDriver();

    if (!drv) 
    {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
	for(size_t i = 0; i < baudRateArraySize; ++i)
	{
		_channel = (*createSerialPortChannel(comm_port, baudrateArray[i]));
        if (SL_IS_OK((drv)->connect(_channel))) 
        {
            sl_result op_result = drv->getDeviceInfo(devinfo);
            if (SL_IS_OK(op_result)) 
            {
	            connectSuccess = true;
                break;
            }
            else
            {
                break;
            }
        }
	}
 
    if (!connectSuccess) 
    {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", comm_port);
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);
    
    drv->setMotorSpeed();
    // start scan...
    drv->startScan(0,1);


    if (file)
    {
        if(!save_flag) 
        {
            run_mode(file, drv);
        }
        else 
        {
            calibrate_mode(file, drv);
        }
    }
    else 
    {
        printf("\n\nNo file specified\n\n");
    }


    // done!
on_finished:
    if(drv) {
        drv->stop();
	    delay(200);
        drv->setMotorSpeed(0);
        delete drv;
        drv = NULL;
    }
    return 0;
}

