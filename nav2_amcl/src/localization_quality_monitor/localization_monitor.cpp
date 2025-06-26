#include "nav2_amcl/localization_quality_monitor/localization_monitor.hpp"
#include <cmath>

LocalizationMonitor::LocalizationMonitor(
  const map_t * map,
  int max_beams)
: map_(map),
  max_beams_(max_beams)
{}

double LocalizationMonitor::computeScore(
  const pf_vector_t & raw_pose,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & data,
  pf_vector_t laser_offset)
{
  pf_vector_t pose = pf_vector_coord_add(laser_offset, raw_pose);
  double score = 0.0;

  int number_of_beams = data->ranges.size();
  int step = (number_of_beams - 1) / (max_beams_ - 1);
  if (step < 1) step = 1;

  for (int i = 0; i < number_of_beams; i += step) {
    double obs = data->ranges[i];
    double bearing = data->angle_min + i * data->angle_increment;
  
    if (obs >= data->range_max || std::isnan(obs)) continue;

    double hx = pose.v[0] + obs * cos(pose.v[2] + bearing);
    double hy = pose.v[1] + obs * sin(pose.v[2] + bearing);

    int mi = MAP_GXWX(map_, hx);
    int mj = MAP_GYWY(map_, hy);

    double dz = (!MAP_VALID(map_, mi, mj)) ?
      map_->max_occ_dist :
      map_->cells[MAP_INDEX(map_, mi, mj)].occ_dist;

    score += dz;
  }

  return score;
}
