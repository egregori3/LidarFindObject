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
#include <vector>

#include "LidarFindObject.h"

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

void getLIDARMode(ILidarDriver * drv)
{
   // Get and display all supported scan modes
    {
        std::vector<sl::LidarScanMode> scanModes;
        sl_result scan_op_result = drv->getAllSupportedScanModes(scanModes);
        
        if (SL_IS_OK(scan_op_result)) {
            printf("\nSupported Scan Modes (%d total):\n", (int)scanModes.size());
            printf("ID | Name                 | Max Dist | Ans Type | Sample Rate\n");
            printf("---|----------------------|----------|----------|------------\n");
            
            for (size_t i = 0; i < scanModes.size(); ++i) {
                const auto& mode = scanModes[i];
                printf("%2d | %-20s | %8.1f | %8d | %8.1f Hz\n", 
                       mode.id,
                       mode.scan_mode,
                       mode.max_distance,
                       mode.ans_type,
                       mode.us_per_sample ? (1000000.0f / mode.us_per_sample) : 0.0f);
            }
            printf("\n");
        } else {
            printf("Failed to get scan modes: %x\n", scan_op_result);
        }

         // Get current scan mode info
        sl_u16 current_mode_id;
        sl_result mode_result = drv->getTypicalScanMode(current_mode_id);
        if (SL_IS_OK(mode_result)) 
        {
            // Find the matching scan mode from the list
            for (const auto& mode : scanModes) {
                if (mode.id == current_mode_id) {
                    printf("Current scan mode: %s (ID: %d, Sample Rate: %.1f Hz)\n", 
                           mode.scan_mode, 
                           mode.id,
                           mode.us_per_sample ? (1000000.0f / mode.us_per_sample) : 0.0f);
                    break;
                }
            }
        }
        else 
        {
            printf("Failed to get current scan mode: %x\n", mode_result);
        }
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
       printf("       -l <filename> to load calibration file [min angle, max angle]\n");
       printf("       -x <filename> to load calibration file [min angle, max angle] output X,Y\n");
       printf("       -f <filename> to load calibration file [min angle, max angle] output filtered data\n");
}

int main(int argc, const char * argv[]) 
{
    const char * comm_port = NULL;
    FILE       * file = NULL;
    const char * calibration_filename = NULL;
    sl_u32       baudrateArray[2] = {115200, 256000};
    IChannel*    _channel;
    int          save_flag = 0;
    int          min_angle = 0;
    int          max_angle = DISTANCE_COUNT;
    int          output_data_type = OUTPUT_POLAR;

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
    // argc 2 illegal
    // argc 3 -> argv[1] = mode (-s or -l)
    //        -> argv[2] = calibration filename
    // argc 4  illegal
    // argc 5 -> argv[1] = mode (-s or -l)
    //        -> argv[2] = calibration filename
    //        -> argv[3] = min angle
    //        -> argv[4] = max angle
    
    switch(argc)
    {
        case 1:
        case 2:
        case 4:
        default:
            print_usage();
            break;
        case 3:
        case 5:
            calibration_filename = argv[2];
            if (strcmp(argv[1], "-s") == 0) 
                save_flag = 1;
            else if (strcmp(argv[1], "-x") == 0)
                output_data_type = OUTPUT_CARTESIAN;
            else if (strcmp(argv[1], "-f") == 0)
                output_data_type = OUTPUT_FILTERED; 
            break;
    }

    if( argc == 5) // -s filename min_angle max_angle or -l filename min_angle max_angle
    {
        min_angle = atoi(argv[3]);
        max_angle = atoi(argv[4]);
        max_angle *= 10;
        min_angle *= 10;
        if (min_angle < 0 || min_angle >= DISTANCE_COUNT || max_angle <= min_angle || max_angle > DISTANCE_COUNT)
        {
            printf("Invalid angle range [%d, %d]\n", min_angle/10, max_angle/10);
            return -1;
        }
    }

    if (save_flag) 
    {
        // create file argv[2] to save calibration data
        printf("Save calibration data to %s\n", calibration_filename);
        // open file for writing
        file = fopen(calibration_filename, "w");
        if (!file) {
            fprintf(stderr, "Error opening file for writing: %s\n", calibration_filename);
            return -1;
        }
    }
    else 
    {
        // load calibration data from file argv[2]
        printf("Load calibration data from %s\n", calibration_filename);
        // open file for reading
        file = fopen(calibration_filename, "r");
        if (!file) {
            fprintf(stderr, "Error opening file for reading: %s\n", calibration_filename);
            return -1;
        }
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

    getLIDARMode(drv);

    signal(SIGINT, ctrlc);
    
    drv->setMotorSpeed();
        
    // start scan...
    // Parameters: startScan(force_scan, use_typical_scan, options, scan_mode)
    // force_scan=0: Don't force scan if motor is not ready
    // use_typical_scan=1: Use the device's typical scan mode (usually Standard)
    drv->startScan(0,1);


    if (file)
    {
        if(!save_flag) 
        {
            run_mode(file, drv, min_angle, max_angle, output_data_type);
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

