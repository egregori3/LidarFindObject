
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

// Structure to hold Cartesian coordinates
struct CartesianPoint {
    float x;
    float y;
};

// Convert polar coordinates to Cartesian coordinates
// angle: angle in degrees (0-360)
// distance: distance in mm
// returns: CartesianPoint with x, y coordinates in mm
CartesianPoint polar_to_cartesian(float angle_degrees, float distance_mm)
{
    CartesianPoint point;
    
    // Convert degrees to radians
    float angle_radians = angle_degrees * M_PI / 180.0f;
    
    // Standard polar to Cartesian conversion
    // x = r * cos(θ), y = r * sin(θ)
    point.x = distance_mm * cos(angle_radians);
    point.y = distance_mm * sin(angle_radians);
    
    return point;
}

// Overloaded version that takes angle in degrees*10 format (as used in your arrays)
CartesianPoint polar_to_cartesian_scaled(int angle_scaled, float distance_mm)
{
    // Convert scaled angle (degrees * 10) back to degrees
    float angle_degrees = angle_scaled / 10.0f;
    return polar_to_cartesian(angle_degrees, distance_mm);
}

// Filter output algorithm
// returns true if x and y are output
// new scan indicates start of new scan
bool filter_algorithm(bool new_scan, float *x, float *y)
{
    static float average_x = 0.0f;
    static float average_y = 0.0f;
    static int count = 0;

    // Pablos's average X and Y algorithm
    if(new_scan) // Start of scan
    {
        if(count > 0)
        {
            *x = average_x / count;
            *y = average_y / count;
            average_x = 0.0f;
            average_y = 0.0f;
            count = 0;
            return true;
        }
        else
        {
            // No valid points collected in the last scan
            *x = 0.0f;
            *y = 0.0f;
            return false;
        }
    }
    else // During scan, accumulate points
    {
        if(x && y)
        {
            average_x += *x;
            average_y += *y;
            count++;
        }
    }
    return false;
}

void run_mode(FILE * file, ILidarDriver * drv, int min_angle, int max_angle, int output_data_type = OUTPUT_POLAR)
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

    printf("Min angle: %d, Max angle: %d, Output Data_Type: ", min_angle/10, max_angle/10);
    if(output_data_type == OUTPUT_POLAR)
        printf("Polar\n");
    else if(output_data_type == OUTPUT_CARTESIAN)
        printf("Cartesian\n");
    else if(output_data_type == OUTPUT_FILTERED)
        printf("Filtered\n");

    while (!ctrl_c_pressed) 
    {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) 
        {
            if(output_data_type == OUTPUT_FILTERED)
            {
                float average_x = 0.0f;
                float average_y = 0.0f;
                if(filter_algorithm(true, &average_x, &average_y))
                {
                    if(is_server_running())
                    {
                        char buffer[100];
                        snprintf(buffer, sizeof(buffer), "X: %03.2f Y: %08.2f\n", average_x, average_y);
                        send_server_data(std::string(buffer));
                    }
                    printf("X: %03.2f Y: %08.2f\n", average_x, average_y);
                }
            }

            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) 
            {
                float angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                float dist = nodes[pos].dist_mm_q2/4.0f;
                int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                if (quality > 0 && dist > 0) 
                {
                    int inter_angle = (int)round(angle * 10.0f);
                    if (inter_angle >= min_angle && inter_angle < max_angle )
                    {
                        int calib_dist = distances[inter_angle];
                        if(dist < calib_dist)
                        {
                            if(output_data_type == OUTPUT_CARTESIAN)
                            {
                                CartesianPoint point = polar_to_cartesian_scaled(inter_angle, dist);
                                printf("X: %08.2f Y: %08.2f\n", point.x, point.y);
                            }
                            else if(output_data_type == OUTPUT_POLAR)
                            {
                                printf("Angle: %03.2f Dist: %08.2f calib_dist: %d\n", angle, dist, calib_dist);
                            }
                            else if(output_data_type == OUTPUT_FILTERED)
                            {
                                CartesianPoint point = polar_to_cartesian_scaled(inter_angle, dist);
                                filter_algorithm(false, &point.x, &point.y);
                            }
                        }
                    }
                }
            }
        }
    }
}


