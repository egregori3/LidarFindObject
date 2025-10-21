
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

using namespace sl;

extern bool ctrl_c_pressed;

void run_mode(FILE * file, ILidarDriver * drv, int min_angle, int max_angle)
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

    printf("Min angle: %d, Max angle: %d ", min_angle/10, max_angle/10);

    while (!ctrl_c_pressed) 
    {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) 
        {
            std::string result = filter_algorithm(true, 0, 0); // Indicate new scan
            if(is_server_running() && !result.empty())
            {
                printf("\n%s\n", result.c_str());
                send_server_data(result);
            }

            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) 
            {
                float angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                float dist = nodes[pos].dist_mm_q2/4.0f;
                int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                if (quality > 0 && dist > 0) 
                {
                    int integer_angle = (int)round(angle * 10.0f);
                    if(((min_angle < max_angle) && (integer_angle >= min_angle && integer_angle < max_angle)) ||
                       ((min_angle > max_angle) && (integer_angle >= min_angle || integer_angle < max_angle)))
                    {
                        int calib_dist = distances[integer_angle];
                        if(dist < calib_dist)
                        {
                            filter_algorithm(false, angle, dist);
# if 0
                            if(output_data_type == OUTPUT_CARTESIAN)
                            {
                                CartesianPoint point = polar_to_cartesian_scaled(integer_angle, dist);
                                printf("X: %08.2f Y: %08.2f\n", point.x, point.y);
                            }
                            else if(output_data_type == OUTPUT_POLAR)
                            {
                                printf("Angle: %03.2f Dist: %08.2f calib_dist: %d\n", angle, dist, calib_dist);
                            }
                            else if(output_data_type == OUTPUT_FILTERED)
                            {
                                CartesianPoint point = polar_to_cartesian_scaled(integer_angle, dist);
                                filter_algorithm(false, &point.x, &point.y);
                            }
#endif
                        }
                    }
                }
            }
        }
    }
}


