
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
    float angle;
    float distance;
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
    point.angle = angle_degrees;
    point.distance = distance_mm;
    
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
std::string filter_algorithm(bool new_scan, float angle, float dist )
{
    static std::vector<CartesianPoint> points;
    std::string result = "No Object Detected\n";

    // Pablos's average X and Y algorithm
    CartesianPoint point = polar_to_cartesian_scaled(angle, dist);
    if(new_scan) // Start of scan
    {
        if(points.size() > 0)
        {
            char buffer[100];
            float sum_x = 0.0f;
            float sum_y = 0.0f;
            int start, stop, count;

            // Find min distance points around center
            float min_distance = std::numeric_limits<float>::max();

            for (const auto& p : points) 
            {
                if (p.distance < min_distance) 
                {
                    min_distance = p.distance;
                }
            }

            count = points.size();
            if(count > 20)
            {
                start = count / 2 - 10; // to avoid division by zero
                stop = count / 2 + 10;
            }
            else
            {
                start = 0;
                stop = count;
            }
            count = 0;
            result = "";
            for (const auto& p : points) 
            {
                sum_x += p.x;
                sum_y += p.y;
                if( count >= start && count < stop )
                {
                    // Add to result string as well
                    if(p.distance == min_distance)
                    {
                        snprintf(buffer, sizeof(buffer), "*angle:%.2f, dist:%.2f, X: %.2f, Y: %.2f\n", p.angle, p.distance, p.x, p.y);
                    }
                    else
                    {
                        snprintf(buffer, sizeof(buffer), "angle:%.2f, dist:%.2f, X: %.2f, Y: %.2f\n", p.angle, p.distance, p.x, p.y);
                    }
                    result += std::string(buffer);
                }
                count++;
            }
            float x = sum_x / points.size();
            float y = sum_y / points.size();
            snprintf(buffer, sizeof(buffer), "AVGX: %.2f, AVGY: %.2f\n", x, y);
            result += std::string(buffer);
            // Reset for next scan
            points.clear();
        }
        else
        {
            // No valid points collected in the last scan

        }
    }
    else // During scan, accumulate points
    {
        points.push_back(point);
    }
    // return empty string object if no output
    return result;
}

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
                    if (integer_angle >= min_angle && integer_angle < max_angle )
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


