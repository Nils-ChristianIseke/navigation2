// Copyright (c) 2022 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>
#include <unordered_map>
#include <functional>
#include "nav2_regulated_pure_pursuit_controller/parameter_handler.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::string & plugin_name, rclcpp::Logger & logger,
  const double costmap_size_x)
{
  node_ = node;
  plugin_name_ = plugin_name;
  logger_ = logger;
  const std::vector<std::pair<std::string, rclcpp::ParameterValue>> parameters = {
    {plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5)},
    {plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6)},
    {plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3)},
    {plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9)},
    {plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5)},
    {plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8)},
    {plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1)},
    {plugin_name_ + ".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(false)},
    {plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05)},
    {plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.6)},
    {plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot", rclcpp::ParameterValue(1.0)},
    {plugin_name_ + ".use_regulated_linear_velocity_scaling", rclcpp::ParameterValue(true)},
    {plugin_name_ + ".use_cost_regulated_linear_velocity_scaling", rclcpp::ParameterValue(true)},
    {plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6)},
    {plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0)},
    {plugin_name_ + ".inflation_cost_scaling_factor", rclcpp::ParameterValue(3.0)},
    {plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(0.90)},
    {plugin_name_ + ".regulated_linear_scaling_min_speed", rclcpp::ParameterValue(0.25)},
    {plugin_name_ + ".use_fixed_curvature_lookahead", rclcpp::ParameterValue(false)},
    {plugin_name_ + ".curvature_lookahead_dist", rclcpp::ParameterValue(0.6)},
    {plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true)},
    {plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.785)},
    {plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2)},
    {plugin_name_ + ".use_cancel_deceleration", rclcpp::ParameterValue(false)},
    {plugin_name_ + ".cancel_deceleration", rclcpp::ParameterValue(3.2)},
    {plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(false)},
    {plugin_name_ + ".max_robot_pose_search_dist", rclcpp::ParameterValue(costmap_size_x / 2.0)},
    {plugin_name_ + ".interpolate_curvature_after_goal", rclcpp::ParameterValue(false)},
    {plugin_name_ + ".use_collision_detection", rclcpp::ParameterValue(true)}
  };

  for (const auto & param : parameters) {
    declare_parameter_if_not_declared(node, param.first, param.second);
  }

  const std::vector<std::pair<std::string, std::reference_wrapper<double>>> double_params = {
    {plugin_name_ + ".desired_linear_vel", params_.desired_linear_vel},
    {plugin_name_ + ".lookahead_dist", params_.lookahead_dist},
    {plugin_name_ + ".min_lookahead_dist", params_.min_lookahead_dist},
    {plugin_name_ + ".max_lookahead_dist", params_.max_lookahead_dist},
    {plugin_name_ + ".lookahead_time", params_.lookahead_time},
    {plugin_name_ + ".rotate_to_heading_angular_vel", params_.rotate_to_heading_angular_vel},
    {plugin_name_ + ".transform_tolerance", params_.transform_tolerance},
    {plugin_name_ + ".min_approach_linear_velocity", params_.min_approach_linear_velocity},
    {plugin_name_ + ".approach_velocity_scaling_dist", params_.approach_velocity_scaling_dist},
    {plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
      params_.max_allowed_time_to_collision_up_to_carrot},
    {plugin_name_ + ".cost_scaling_dist", params_.cost_scaling_dist},
    {plugin_name_ + ".cost_scaling_gain", params_.cost_scaling_gain},
    {plugin_name_ + ".inflation_cost_scaling_factor", params_.inflation_cost_scaling_factor},
    {plugin_name_ + ".regulated_linear_scaling_min_radius",
      params_.regulated_linear_scaling_min_radius},
    {plugin_name_ + ".regulated_linear_scaling_min_speed",
      params_.regulated_linear_scaling_min_speed},
    {plugin_name_ + ".curvature_lookahead_dist", params_.curvature_lookahead_dist},
    {plugin_name_ + ".rotate_to_heading_min_angle", params_.rotate_to_heading_min_angle},
    {plugin_name_ + ".max_angular_accel", params_.max_angular_accel},
    {plugin_name_ + ".cancel_deceleration", params_.cancel_deceleration},
    {plugin_name_ + ".max_robot_pose_search_dist", params_.max_robot_pose_search_dist}
  };

  for (const auto & param : double_params) {
    node->get_parameter(param.first, param.second.get());
  }

  params_.base_desired_linear_vel = params_.desired_linear_vel;

  if (params_.approach_velocity_scaling_dist > costmap_size_x / 2.0) {
    RCLCPP_WARN(
      logger_, "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }

  const std::vector<std::pair<std::string, std::reference_wrapper<bool>>> bool_params = {
    {plugin_name_ + ".use_velocity_scaled_lookahead_dist",
      params_.use_velocity_scaled_lookahead_dist},
    {plugin_name_ + ".use_regulated_linear_velocity_scaling",
      params_.use_regulated_linear_velocity_scaling},
    {plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
      params_.use_cost_regulated_linear_velocity_scaling},
    {plugin_name_ + ".use_fixed_curvature_lookahead", params_.use_fixed_curvature_lookahead},
    {plugin_name_ + ".use_rotate_to_heading", params_.use_rotate_to_heading},
    {plugin_name_ + ".use_cancel_deceleration", params_.use_cancel_deceleration},
    {plugin_name_ + ".allow_reversing", params_.allow_reversing},
    {plugin_name_ + ".interpolate_curvature_after_goal", params_.interpolate_curvature_after_goal},
    {plugin_name_ + ".use_collision_detection", params_.use_collision_detection}
  };

  for (const auto & param : bool_params) {
    node->get_parameter(param.first, param.second.get());
  }
  if (params_.max_robot_pose_search_dist < 0.0) {
    RCLCPP_WARN(
      logger_, "Max robot search distance is negative, setting to max to search"
      " every point on path for the closest value.");
    params_.max_robot_pose_search_dist = std::numeric_limits<double>::max();
  }

  node->get_parameter(
    plugin_name_ + ".interpolate_curvature_after_goal",
    params_.interpolate_curvature_after_goal);
  if (!params_.use_fixed_curvature_lookahead && params_.interpolate_curvature_after_goal) {
    RCLCPP_WARN(
      logger_, "For interpolate_curvature_after_goal to be set to true, "
      "use_fixed_curvature_lookahead should be true, it is currently set to false. Disabling.");
    params_.interpolate_curvature_after_goal = false;
  }
  node->get_parameter(
    plugin_name_ + ".use_collision_detection",
    params_.use_collision_detection);

  if (params_.inflation_cost_scaling_factor <= 0.0) {
    RCLCPP_WARN(
      logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
      "it should be >0. Disabling cost regulated linear velocity scaling.");
    params_.use_cost_regulated_linear_velocity_scaling = false;
  }

  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &ParameterHandler::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParameterHandler::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

ParameterHandler::~ParameterHandler()
{
  auto node = node_.lock();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
}
rcl_interfaces::msg::SetParametersResult ParameterHandler::validateParameterUpdatesCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".inflation_cost_scaling_factor") {
        if (parameter.as_double() <= 0.0) {
          RCLCPP_WARN(
            logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
            "it should be >0. Ignoring parameter update.");
          result.successful = false;
        }
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".allow_reversing") {
        if (params_.use_rotate_to_heading && parameter.as_bool()) {
          RCLCPP_WARN(
            logger_, "Both use_rotate_to_heading and allow_reversing "
            "parameter cannot be set to true. Rejecting parameter update.");
          result.successful = false;
        }
      }
    }
  }
  return result;
}
void ParameterHandler::updateParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  static const std::unordered_map<std::string, std::function<void(double)>> double_parameters = {
    {plugin_name_ + ".inflation_cost_scaling_factor", [this](double value) {
        params_.inflation_cost_scaling_factor = value;
      }},
    {plugin_name_ + ".desired_linear_vel", [this](double value) {
        params_.desired_linear_vel = value; params_.base_desired_linear_vel = value;
      }},
    {plugin_name_ + ".lookahead_dist", [this](double value) {params_.lookahead_dist = value;}},
    {plugin_name_ + ".max_lookahead_dist", [this](double value) {
        params_.max_lookahead_dist = value;
      }},
    {plugin_name_ + ".min_lookahead_dist", [this](double value) {
        params_.min_lookahead_dist = value;
      }},
    {plugin_name_ + ".lookahead_time", [this](double value) {params_.lookahead_time = value;}},
    {plugin_name_ + ".rotate_to_heading_angular_vel", [this](double value) {
        params_.rotate_to_heading_angular_vel = value;
      }},
    {plugin_name_ + ".min_approach_linear_velocity", [this](double value) {
        params_.min_approach_linear_velocity = value;
      }},
    {plugin_name_ + ".curvature_lookahead_dist", [this](double value) {
        params_.curvature_lookahead_dist = value;
      }},
    {plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot", [this](double value) {
        params_.max_allowed_time_to_collision_up_to_carrot = value;
      }},
    {plugin_name_ + ".cost_scaling_dist", [this](double value) {
        params_.cost_scaling_dist = value;
      }},
    {plugin_name_ + ".cost_scaling_gain", [this](double value) {
        params_.cost_scaling_gain = value;
      }},
    {plugin_name_ + ".regulated_linear_scaling_min_radius", [this](double value) {
        params_.regulated_linear_scaling_min_radius = value;
      }},
    {plugin_name_ + ".regulated_linear_scaling_min_speed", [this](double value) {
        params_.regulated_linear_scaling_min_speed = value;
      }},
    {plugin_name_ + ".max_angular_accel", [this](double value) {
        params_.max_angular_accel = value;
      }},
    {plugin_name_ + ".cancel_deceleration", [this](double value) {
        params_.cancel_deceleration = value;
      }},
    {plugin_name_ + ".rotate_to_heading_min_angle", [this](double value) {
        params_.rotate_to_heading_min_angle = value;
      }},
  };

  static const std::unordered_map<std::string, std::function<void(bool)>> bool_parameters = {
    {plugin_name_ + ".use_velocity_scaled_lookahead_dist", [this](bool value) {
        params_.use_velocity_scaled_lookahead_dist = value;
      }},
    {plugin_name_ + ".use_regulated_linear_velocity_scaling", [this](bool value) {
        params_.use_regulated_linear_velocity_scaling = value;
      }},
    {plugin_name_ + ".use_fixed_curvature_lookahead", [this](bool value) {
        params_.use_fixed_curvature_lookahead = value;
      }},
    {plugin_name_ + ".use_cost_regulated_linear_velocity_scaling", [this](bool value) {
        params_.use_cost_regulated_linear_velocity_scaling = value;
      }},
    {plugin_name_ + ".use_collision_detection", [this](bool value) {
        params_.use_collision_detection = value;
      }},
    {plugin_name_ + ".use_rotate_to_heading", [this](bool value) {
        params_.use_rotate_to_heading = value;
      }},
    {plugin_name_ + ".use_cancel_deceleration", [this](bool value) {
        params_.use_cancel_deceleration = value;
      }},
    {plugin_name_ + ".allow_reversing", [this](bool value) {params_.allow_reversing = value;}},
  };

  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      auto it = double_parameters.find(name);
      if (it != double_parameters.end()) {
        it->second(parameter.as_double());
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      auto it = bool_parameters.find(name);
      if (it != bool_parameters.end()) {
        it->second(parameter.as_bool());
      }
    }
  }
}

}  // namespace nav2_regulated_pure_pursuit_controller
