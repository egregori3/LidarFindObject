
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

extern bool ctrl_c_pressed;

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

