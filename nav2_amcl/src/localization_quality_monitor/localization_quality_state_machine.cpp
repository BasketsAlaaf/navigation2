#include "nav2_amcl/localization_quality_monitor/localization_quality_state_machine.hpp"

namespace nav2_amcl
{

LocalizationQualityStateMachine::LocalizationQualityStateMachine(
    double low_strict,
    double low_soft,
    double high_strict,
    double high_soft,
    double delay_in_sec)
: low_strict_(low_strict),
  low_soft_(low_soft),
  high_strict_(high_strict),
  high_soft_(high_soft),
  delay_in_sec_(delay_in_sec)
{
  // Initial state assumes robot is localized
  state_ = "localized";
}

// Update the state machine based on the current average particle filter weight
std::string LocalizationQualityStateMachine::update(double avg_weight)
{
  // Immediate transition to "mislocalized" if above strict high threshold
  if (avg_weight > high_strict_) {
    if (state_ != "mislocalized") {
      state_ = "mislocalized";
      RCLCPP_WARN(
        rclcpp::get_logger("localization_quality_monitor"), "State changed to mislocalized due to"
        " strict high threshold (weight: %.5f)", avg_weight);
    }
    reset_timer();
  }
  // Soft high threshold: may transition to "mislocalized" after delay
  else if (avg_weight > high_soft_) {
    check_timer("mislocalizing", avg_weight);
  }
  // Immediate transition to "localized" if below strict low threshold
  else if (avg_weight < low_strict_) {
    if (state_ != "localized") {
      state_ = "localized";
      RCLCPP_WARN(
        rclcpp::get_logger("localization_quality_monitor"), "State changed to localized due to"
        " strict low threshold (weight: %.5f)", avg_weight);
    }
    reset_timer();
  }
  // Soft low threshold: may transition to "localized" after delay
  else if (avg_weight < low_soft_) {
    check_timer("recovering", avg_weight);
  }
  else {
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
  // Start new timer if the type changed
  if (current_timer_type_ != timer_type) {
    timer_start_ = std::chrono::steady_clock::now();
    current_timer_type_ = timer_type;
  }
  else {
    // Calculate elapsed time since timer started
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now()
                   - timer_start_).count();
    // If elapsed time exceeds delay, change state based on timer type
    if (elapsed > delay_in_sec_) {
      if (timer_type == "mislocalizing" && state_ != "mislocalized") {
        state_ = "mislocalized";
        RCLCPP_WARN(
            rclcpp::get_logger("localization_quality_monitor"), "State changed to mislocalized "
            "after delay (weight: %.5f)", avg_weight);
      }
      else if (timer_type == "recovering" && state_ != "localized") {
        state_ = "localized";
        RCLCPP_INFO(rclcpp::get_logger("localization_quality_monitor"), "State changed to localized "
        "after delay (weight: %.5f)", avg_weight);
      }
    }
  }
}

}  // namespace nav2_amcl