#include <algorithm>
#include <atomic>
#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/transport/Node.hh>
#include <ignition/msgs/joint_trajectory.pb.h>
#include <ignition/plugin/Register.hh>

namespace linkerhand_gazebo_plugin
{

class OnlineJointController final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    if (!model.Valid(ecm)) {
      ignerr << "Linker Hand 控制插件必须挂载到 model 实体。" << std::endl;
      return;
    }

    if (!sdf->HasElement("joint_name")) {
      ignerr << "Linker Hand 控制插件缺少 joint_name 配置。" << std::endl;
      return;
    }

    auto joint_element = sdf->FindElement("joint_name");
    while (joint_element) {
      const std::string joint_name = joint_element->Get<std::string>();
      const auto joint_entity = model.JointByName(ecm, joint_name);
      if (joint_entity == gz::sim::kNullEntity) {
        ignerr << "Gazebo 模型中不存在关节 [" << joint_name << "]。" << std::endl;
        return;
      }

      const auto index = joints_.size();
      joint_indices_.emplace(joint_name, index);
      joints_.push_back({joint_name, joint_entity});
      targets_.push_back(0.0);

      if (!ecm.Component<gz::sim::components::JointPosition>(joint_entity)) {
        ecm.CreateComponent(
          joint_entity,
          gz::sim::components::JointPosition());
      }
      joint_element = joint_element->GetNextElement("joint_name");
    }

    topic_ = sdf->Get<std::string>("topic", "/gazebo_joint_trajectory").first;
    position_gain_ = sdf->Get<double>("position_gain", 8.0).first;
    max_velocity_ = sdf->Get<double>("max_velocity", 3.0).first;
    if (position_gain_ <= 0.0 || max_velocity_ <= 0.0) {
      ignerr << "position_gain 和 max_velocity 必须大于 0。" << std::endl;
      return;
    }

    if (!node_.Subscribe(topic_, &OnlineJointController::OnTrajectory, this)) {
      ignerr << "无法订阅 Gazebo 轨迹话题 [" << topic_ << "]。" << std::endl;
      return;
    }

    configured_.store(true);
    ignmsg << "Linker Hand 在线控制插件已加载，关节数=" << joints_.size()
           << "，话题=" << topic_ << "。" << std::endl;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!configured_.load() || info.paused) {
      return;
    }

    std::vector<double> targets;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      if (!has_target_) {
        return;
      }
      targets = targets_;
    }

    const double step_seconds = std::chrono::duration<double>(info.dt).count();
    if (step_seconds <= 0.0) {
      return;
    }

    for (std::size_t index = 0; index < joints_.size(); ++index) {
      const auto * position =
        ecm.Component<gz::sim::components::JointPosition>(joints_[index].entity);
      if (!position || position->Data().empty()) {
        continue;
      }

      const double error = targets[index] - position->Data()[0];
      const double velocity_command = std::clamp(
        position_gain_ * error,
        -max_velocity_,
        max_velocity_);
      const double next_position =
        position->Data()[0] + velocity_command * step_seconds;

      gz::sim::Joint joint(joints_[index].entity);
      joint.ResetPosition(ecm, {next_position});
      joint.ResetVelocity(ecm, {0.0});
    }
  }

private:
  struct ControlledJoint
  {
    std::string name;
    gz::sim::Entity entity{gz::sim::kNullEntity};
  };

  void OnTrajectory(const ignition::msgs::JointTrajectory & message)
  {
    if (!configured_.load() || message.points_size() == 0) {
      return;
    }

    const auto & point = message.points(message.points_size() - 1);
    if (message.joint_names_size() != point.positions_size()) {
      return;
    }

    std::vector<double> next_targets(targets_.size(), 0.0);
    std::vector<bool> received(targets_.size(), false);
    for (int index = 0; index < message.joint_names_size(); ++index) {
      const auto joint = joint_indices_.find(message.joint_names(index));
      if (joint == joint_indices_.end()) {
        continue;
      }

      const double value = point.positions(index);
      if (!std::isfinite(value)) {
        return;
      }
      next_targets[joint->second] = value;
      received[joint->second] = true;
    }

    if (std::find(received.begin(), received.end(), false) != received.end()) {
      return;
    }

    std::lock_guard<std::mutex> lock(target_mutex_);
    targets_ = std::move(next_targets);
    if (!has_target_) {
      ignmsg << "Linker Hand 在线控制插件已收到首个完整轨迹目标。"
             << std::endl;
    }
    has_target_ = true;
  }

  gz::transport::Node node_;
  std::string topic_;
  std::vector<ControlledJoint> joints_;
  std::unordered_map<std::string, std::size_t> joint_indices_;

  std::mutex target_mutex_;
  std::vector<double> targets_;
  bool has_target_{false};
  std::atomic<bool> configured_{false};

  double position_gain_{8.0};
  double max_velocity_{3.0};
};

}  // namespace linkerhand_gazebo_plugin

IGNITION_ADD_PLUGIN(
  linkerhand_gazebo_plugin::OnlineJointController,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  linkerhand_gazebo_plugin::OnlineJointController,
  "linkerhand_gazebo_plugin::OnlineJointController")
