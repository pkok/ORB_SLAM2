#pragma once

#include <deque>
#include "IMU/imudata.h"

namespace ORB_SLAM2 {
  namespace IMU {
    std::deque<ORB_SLAM2::IMUData> parse_euroc(const std::string& csv_filename);
  }
}
