#pragma once

#include <deque>
#include "IMU/imudata.h"

std::deque<ORB_SLAM2::IMUData> get_imu_data();
