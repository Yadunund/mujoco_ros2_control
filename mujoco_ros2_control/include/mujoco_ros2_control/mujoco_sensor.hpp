// Copyright 2024 Aldrin Nugroho
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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SENSOR_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SENSOR_HPP_

#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/system_interface.hpp>
#include <string>
#include <hardware_interface/hardware_info.hpp>

namespace mujoco_ros2_control
{

template <typename T>
struct SensorData
{
  std::string name;
  T data;
  int mj_sensor_index;
};

class SensorBase
{
public:
  virtual ~SensorBase() = default;
  virtual bool init(const hardware_interface::ComponentInfo &sensor_info, const mjModel *mj_model) = 0;
  virtual void registerStateIface(std::vector<hardware_interface::StateInterface> &state_interfaces) = 0;
  virtual void read(const mjData *mj_data) = 0;

protected:
  std::string name_;
};

class IMUSensor : public SensorBase
{
public:
  bool init(const hardware_interface::ComponentInfo &sensor_info, const mjModel *mj_model) override;
  void registerStateIface(std::vector<hardware_interface::StateInterface> &state_interfaces) override;
  void read(const mjData *mj_data) override;

  SensorData<Eigen::Vector3d> angular_velocity;
  SensorData<Eigen::Quaterniond> orientation;
  SensorData<Eigen::Vector3d> linear_velocity;
  SensorData<Eigen::Vector3d> linear_acceleration;
};

class FTSensor : public SensorBase
{
public:
  bool init(const hardware_interface::ComponentInfo &sensor_info, const mjModel *mj_model) override;
  void registerStateIface(std::vector<hardware_interface::StateInterface> &state_interfaces) override;
  void read(const mjData *mj_data) override;

  SensorData<Eigen::Vector3d> force;
  SensorData<Eigen::Vector3d> torque;
  std::vector<std::string> iface_lists;
};

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SENSOR_HPP_
