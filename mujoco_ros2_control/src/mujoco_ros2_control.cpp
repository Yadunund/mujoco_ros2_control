// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "hardware_interface/types/resource_manager_params.hpp"

#include "pluginlib/class_loader.hpp"

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"
#include "mujoco_ros2_control/mujoco_system_interface.hpp"

namespace mujoco_ros2_control
{
// Custom ResourceManager that loads Mujoco model before initializing hardware
class MujocoResourceManager : public hardware_interface::ResourceManager
{
public:
  MujocoResourceManager(
    rclcpp::Node::SharedPtr & node,
    const rclcpp::Clock::SharedPtr & clock,
    const rclcpp::Logger & logger,
    mjModel*& mj_model,
    mjData*& mj_data)
  : hardware_interface::ResourceManager(clock, logger),
    node_(node),
    logger_(logger),
    mj_model_(mj_model),
    mj_data_(mj_data),
    mj_system_loader_("mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface")
  {}

  bool load_and_initialize_components(
    const hardware_interface::ResourceManagerParams & params) override
  {
    RCLCPP_INFO(logger_, "MujocoResourceManager::load_and_initialize_components() called");

    // Mark components as being loaded - this sets the initialization flag
    components_are_loaded_and_initialized_ = true;

    if (params.robot_description.empty())
    {
      RCLCPP_ERROR(logger_, "Robot description is empty");
      components_are_loaded_and_initialized_ = false;
      return false;
    }

    RCLCPP_INFO(logger_, "Robot description received, length: %zu", params.robot_description.length());

    // Load Mujoco model if not already loaded
    if (!mj_model_)
    {
      RCLCPP_INFO(logger_, "Loading Mujoco model...");
      auto model_path = node_->get_parameter("mujoco_model_path").as_string();
      char error[1000] = "Could not load model";

      if (model_path.size() > 4 && model_path.substr(model_path.size() - 4) == ".mjb")
      {
        mj_model_ = mj_loadModel(model_path.c_str(), 0);
      }
      else
      {
        mj_model_ = mj_loadXML(model_path.c_str(), 0, error, 1000);
      }

      if (!mj_model_)
      {
        RCLCPP_ERROR_STREAM(logger_, "Failed to load Mujoco model: " << error);
        components_are_loaded_and_initialized_ = false;
        return false;
      }

      mj_data_ = mj_makeData(mj_model_);
      RCLCPP_INFO(logger_, "Mujoco model loaded successfully");
    }
    else
    {
      RCLCPP_INFO(logger_, "Mujoco model already loaded");
    }

    // Parse hardware info from URDF
    RCLCPP_INFO(logger_, "Parsing hardware info from URDF...");
    const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(params.robot_description);
    RCLCPP_INFO(logger_, "Found %zu hardware components", hardware_info.size());

    // Load each hardware component
    for (const auto & individual_hardware_info : hardware_info)
    {
      std::string robot_hw_sim_type_str = individual_hardware_info.hardware_plugin_name;
      RCLCPP_INFO(logger_, "Loading hardware interface: %s", robot_hw_sim_type_str.c_str());

      // Load hardware
      std::unique_ptr<MujocoSystemInterface> mjSimSystem;
      try
      {
        RCLCPP_INFO(logger_, "Creating plugin instance...");
        mjSimSystem = std::unique_ptr<MujocoSystemInterface>(
          mj_system_loader_.createUnmanagedInstance(robot_hw_sim_type_str));
        RCLCPP_INFO(logger_, "Plugin instance created successfully");
      }
      catch (pluginlib::PluginlibException & ex)
      {
        RCLCPP_ERROR_STREAM(logger_, "The plugin failed to load. Error: " << ex.what());
        continue;
      }

      // Initialize simulation required resource from the hardware info
      RCLCPP_INFO(logger_, "Initializing simulation interface...");
      if (!mjSimSystem->init_sim(mj_model_, mj_data_, individual_hardware_info))
      {
        RCLCPP_FATAL(logger_, "Could not initialize robot simulation interface");
        components_are_loaded_and_initialized_ = false;
        return false;
      }

      RCLCPP_INFO(logger_, "Initialized hardware interface %s !", robot_hw_sim_type_str.c_str());

      // Import component to resource manager
      hardware_interface::HardwareComponentParams hw_params;
      hw_params.hardware_info = individual_hardware_info;
      hw_params.logger = logger_;
      hw_params.clock = params.clock;
      RCLCPP_INFO(logger_, "Importing component to resource manager...");
      import_component(std::move(mjSimSystem), hw_params);
      RCLCPP_INFO(logger_, "Component imported successfully");
    }

    RCLCPP_INFO(logger_, "All hardware components loaded successfully");
    return true;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  mjModel*& mj_model_;
  mjData*& mj_data_;
  pluginlib::ClassLoader<MujocoSystemInterface> mj_system_loader_;
};

MujocoRos2Control::MujocoRos2Control(
  rclcpp::Node::SharedPtr &node, mjModel *mujoco_model, mjData *mujoco_data,
  const rclcpp::NodeOptions &cm_node_options)
    : node_(node),
      mj_model_(mujoco_model),
      mj_data_(mujoco_data),
      cm_node_options_(cm_node_options),
      logger_(rclcpp::get_logger(node_->get_name() + std::string(".mujoco_ros2_control"))),
      control_period_(rclcpp::Duration(1, 0)),
      last_update_sim_time_ros_(0, 0, RCL_ROS_TIME)
{
}

MujocoRos2Control::~MujocoRos2Control()
{
  stop_cm_thread_ = true;
  cm_executor_->remove_node(controller_manager_);
  cm_executor_->cancel();

  if (cm_thread_.joinable()) cm_thread_.join();
}

std::string MujocoRos2Control::get_robot_description()
{
  // Getting robot description from parameter first. If not set trying from topic
  std::string robot_description;

  if (node_->has_parameter("robot_description"))
  {
    robot_description = node_->get_parameter("robot_description").as_string();
    return robot_description;
  }

  RCLCPP_WARN(
    logger_,
    "Failed to get robot_description from parameter. Will listen on the ~/robot_description "
    "topic...");

  auto robot_description_sub = node_->create_subscription<std_msgs::msg::String>(
    "robot_description", rclcpp::QoS(1).transient_local(),
    [&](const std_msgs::msg::String::SharedPtr msg)
    {
      if (!msg->data.empty() && robot_description.empty()) robot_description = msg->data;
    });

  while (robot_description.empty() && rclcpp::ok())
  {
    rclcpp::spin_some(node_);
    RCLCPP_INFO(node_->get_logger(), "Waiting for robot description message");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  return robot_description;
}

void MujocoRos2Control::init()
{
  RCLCPP_INFO(logger_, "MujocoRos2Control::init() started");
  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  RCLCPP_INFO(logger_, "Clock publisher created");

  // Create the controller manager
  RCLCPP_INFO(logger_, "Loading controller_manager");

  RCLCPP_INFO(logger_, "Creating MujocoResourceManager");
  std::unique_ptr<hardware_interface::ResourceManager> resource_manager =
    std::make_unique<MujocoResourceManager>(
      node_, node_->get_clock(), logger_, mj_model_, mj_data_);
  RCLCPP_INFO(logger_, "MujocoResourceManager created");

  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
    std::move(resource_manager), cm_executor_, "controller_manager", node_->get_namespace(), cm_node_options_);
  cm_executor_->add_node(controller_manager_);

  if (!controller_manager_->has_parameter("update_rate"))
  {
    RCLCPP_ERROR_STREAM(logger_, "controller manager doesn't have an update_rate parameter");
    return;
  }

  auto update_rate = controller_manager_->get_parameter("update_rate").as_int();
  control_period_ = rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / static_cast<double>(update_rate))));

  // Force setting of use_sim_time parameter
  controller_manager_->set_parameter(
    rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  stop_cm_thread_ = false;
  auto spin = [this]()
  {
    while (rclcpp::ok() && !stop_cm_thread_)
    {
      cm_executor_->spin_once();
    }
  };
  cm_thread_ = std::thread(spin);

  // Wait for resource manager to be fully initialized
  while (!controller_manager_->is_resource_manager_initialized())
  {
    RCLCPP_WARN(logger_, "Waiting for ResourceManager to load and initialize hardware...");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  RCLCPP_INFO(logger_, "ResourceManager initialized successfully");
}

void MujocoRos2Control::update()
{
  // Get the simulation time and period
  auto sim_time = mj_data_->time;
  int sim_time_sec = static_cast<int>(sim_time);
  int sim_time_nanosec = static_cast<int>((sim_time - sim_time_sec) * 1000000000);

  rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  publish_sim_time(sim_time_ros);

  mj_step1(mj_model_, mj_data_);

  if (sim_period >= control_period_)
  {
    controller_manager_->read(sim_time_ros, sim_period);
    controller_manager_->update(sim_time_ros, sim_period);
    last_update_sim_time_ros_ = sim_time_ros;
  }

  // use same time as for read and update call - this is how it is done in ros2_control_node
  controller_manager_->write(sim_time_ros, sim_period);

  mj_step2(mj_model_, mj_data_);
}

void MujocoRos2Control::publish_sim_time(rclcpp::Time sim_time)
{
  // TODO(sangteak601)
  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time;
  clock_publisher_->publish(sim_time_msg);
}

}  // namespace mujoco_ros2_control
