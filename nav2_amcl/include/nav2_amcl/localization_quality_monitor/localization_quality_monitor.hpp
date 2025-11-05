#ifndef NAV2_AMCL__LOCALIZATION_QUALITY_MONITOR__LOCALIZATION_QUALITY_MONITOR_HPP_
#define NAV2_AMCL__LOCALIZATION_QUALITY_MONITOR__LOCALIZATION_QUALITY_MONITOR_HPP_

#include "nav2_amcl/map/map.hpp"
#include "nav2_amcl/pf/pf_vector.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2_amcl
{
/**
* @class LocalizationQualityMonitor
* @brief A class to evaluate the quality of an estimated pose by comparing
*        actual laser scan data against what would be expected if the robot
*        was truly at that pose on the map.
*/
class LocalizationQualityMonitor
{
public:
  /**
   * @brief Constructor
   * @param map Pointer to the static occupancy grid map used for localization
   * @param max_beams Maximum number of laser beams to consider when evaluating score.
   */
  LocalizationQualityMonitor(
    const map_t * map,
    int max_beams);

  /**
   * @brief Computes the localization quality score for a given pose hypothesis.
   */
  double computeScore(
    const pf_vector_t & pose,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & data,
    pf_vector_t laser_offset);

private:
  const map_t * map_;
  int max_beams_;
};

}  // namespace nav2_amcl

#endif // NAV2_AMCL__LOCALIZATION_QUALITY_MONITOR__LOCALIZATION_QUALITY_MONITOR_HPP_