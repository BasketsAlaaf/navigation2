#ifndef NAV2_AMCL__LOCALIZATION_QUALITY_MONITOR__LOCALIZATION_QUALITY_STATE_MACHINE_HPP_
#define NAV2_AMCL__LOCALIZATION_QUALITY_MONITOR__LOCALIZATION_QUALITY_STATE_MACHINE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace nav2_amcl
{
/**
 * @brief State machine for evaluating localization quality based on particle filter likelihood scores.
 *
 * This class tracks the current localization state of the robot using defined thresholds and
 * delays for soft transitions. It supports immediate transitions (strict thresholds) and delayed
 * transitions (soft thresholds) for smoother behavior.
 *
 * States are represented as strings, e.g., "localized", "mislocalized", "recovering", "mislocalizing".
 */
class LocalizationQualityStateMachine
{
public:
  /**
   * @brief Constructor
   *
   * @param low_strict Threshold below which the robot is immediately considered well-localized
   * @param low_soft Threshold below which the robot may become localized after a delay
   * @param high_strict Threshold above which the robot is immediately considered mislocalized
   * @param high_soft Threshold above which the robot may become mislocalized after a delay
   * @param delay_in_sec Delay (seconds) for soft-threshold transitions
   */
  LocalizationQualityStateMachine(
    double low_strict,
    double low_soft,
    double high_strict,
    double high_soft,
    double delay_in_sec);

  /**
   * @brief Update the state machine based on the current score.
   *
   * This function evaluates the score and transitions the state if thresholds are exceeded.
   *
   * @param avg_weight Average particle filter score from the likelihood scoring
   * @return std::string The current state after update
   */
  std::string update(double avg_weight);

private:
  /**
   * @brief Reset the soft-threshold timer.
   */
  void reset_timer();

  /**
   * @brief Check if the soft-threshold timer has expired and update state if needed.
   *
   * @param timer_type Type of timer, e.g., "recovering" or "mislocalizing"
   * @param avg_weight Current average particle filter weight
   */
  void check_timer(const std::string &timer_type, double avg_weight);

  // Thresholds and delay for evaluating localization quality
  double low_strict_, low_soft_, high_strict_, high_soft_, delay_in_sec_;
  // Current state of the robot
  std::string state_;
  std::chrono::steady_clock::time_point timer_start_;
  std::string current_timer_type_;
};

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__LOCALIZATION_QUALITY_MONITOR__LOCALIZATION_QUALITY_STATE_MACHINE_HPP_
