#ifndef NAV2_AMCL__THRESHOLD_STATE_MACHINE_HPP_
#define NAV2_AMCL__THRESHOLD_STATE_MACHINE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace nav2_amcl
{

class LocalizationQualityStateMachine
{
public:
  LocalizationQualityStateMachine(double low_strict, double low_soft, double high_strict, double high_soft, double delay_in_sec);

  std::string update(double avg_weight);

private:
  void reset_timer();
  void check_timer(const std::string &timer_type, double avg_weight);

  double low_strict_, low_soft_, high_strict_, high_soft_, delay_in_sec_;
  std::string state_;
  std::chrono::steady_clock::time_point timer_start_;
  std::string current_timer_type_;
};

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__THRESHOLD_STATE_MACHINE_HPP_
