#pragma once

#include "nav2_amcl/pf/pf_vector.hpp"
#include "nav2_amcl/map/map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LocalizationMonitor
{
public:
  LocalizationMonitor(
    const map_t * map,
    int max_beams);

  double computeScore(
    const pf_vector_t & pose,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & data,
    pf_vector_t laser_offset);

private:
  const map_t * map_;
  int max_beams_;
};