#include "nav2_amcl/localization_quality_monitor/localization_quality_state_machine.hpp"

namespace nav2_amcl
{

LocalizationQualityStateMachine::LocalizationQualityStateMachine(double low_strict, double low_soft, double high_strict, double high_soft, double delay_in_sec)
: low_strict_(low_strict), low_soft_(low_soft), high_strict_(high_strict), high_soft_(high_soft), delay_in_sec_(delay_in_sec)
{
  state_ = "localized";
}

std::string LocalizationQualityStateMachine::update(double avg_weight)
{
  if (avg_weight > high_strict_)
  {
    if (state_ != "mislocalized")
    {
      state_ = "mislocalized";
      RCLCPP_WARN(rclcpp::get_logger("localization_quality_monitor"), "State changed to mislocalized due to strict high threshold (weight: %.5f)", avg_weight);
    }
    reset_timer();
  }
  else if (avg_weight > high_soft_)
  {
    check_timer("mislocalizing", avg_weight);
  }
  else if (avg_weight < low_strict_)
  {
    if (state_ != "localized")
    {
      state_ = "localized";
      RCLCPP_WARN(rclcpp::get_logger("localization_quality_monitor"), "State changed to localized due to strict low threshold (weight: %.5f)", avg_weight);
    }
    reset_timer();
  }
  else if (avg_weight < low_soft_)
  {
    check_timer("recovering", avg_weight);
  }
  else
  {
    reset_timer();
  }

  return state_;
}

void LocalizationQualityStateMachine::reset_timer()
{
  timer_start_ = std::chrono::steady_clock::time_point();
  current_timer_type_.clear();
}

void LocalizationQualityStateMachine::check_timer(const std::string &timer_type, double avg_weight)
{
  if (current_timer_type_ != timer_type)
  {
    timer_start_ = std::chrono::steady_clock::now();
    current_timer_type_ = timer_type;
  }
  else
  {
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - timer_start_).count();
    if (elapsed > delay_in_sec_)
    {
      if (timer_type == "mislocalizing" && state_ != "mislocalized")
      {
        state_ = "mislocalized";
        RCLCPP_WARN(rclcpp::get_logger("localization_quality_monitor"), "State changed to mislocalized after delay (weight: %.5f)", avg_weight);
      }
      else if (timer_type == "recovering" && state_ != "localized")
      {
        state_ = "localized";
        RCLCPP_INFO(rclcpp::get_logger("localization_quality_monitor"), "State changed to localized after delay (weight: %.5f)", avg_weight);
      }
    }
  }
}

}  // namespace nav2_amcl