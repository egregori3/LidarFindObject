#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <limits>
#include <cmath>
#include <vector>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"


#define DISTANCE_COUNT (360*10)
#define MAX_CALIBRATION_SCANS 100
#define NOISE_BUFFER_SIZE 300
#define MAX_DISTANCE 10000
#define OUTPUT_POLAR 0
#define OUTPUT_CARTESIAN 1
#define OUTPUT_FILTERED 2



void calibrate_mode(FILE * file, sl::ILidarDriver * drv);
void run_mode(FILE * file, sl::ILidarDriver * drv, int min_angle, int max_angle, int output_data_type);

// External function declarations for server API

bool get_server_data(std::string& data_out);
bool send_server_data(const std::string& data);
bool is_server_running();
void cleanup_server();
int start_server();

