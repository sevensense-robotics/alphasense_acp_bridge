#ifndef SEV_ACP_ROS_CONVERSIONS_CONVERSIONS_H_
#define SEV_ACP_ROS_CONVERSIONS_CONVERSIONS_H_
#include <stdexcept>
#include <string>
#include <tuple>
#include "atlas_msgs/PositioningUpdate.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "state_machine_msgs/State.h"
#include "state_machine_msgs/Status.h"
#include "std_msgs/Header.h"

// #include "sev/acp/serialization.h"
#include "sev/acp/types.h"

namespace sev::acp::udp_ros_bridge {

template <typename RosType>
const std::string& suffix_lookup();

template <>
inline const std::string& suffix_lookup<geometry_msgs::PoseStamped>() {
  static const std::string suffix{"ros_pose"};
  return suffix;
}

template <>
inline const std::string& suffix_lookup<atlas_msgs::PositioningUpdate>() {
  static const std::string suffix{"positioning_update"};
  return suffix;
}

struct ConversionError : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

std_msgs::Header convertToRosStamped(sev::acp::MessageHeader const& mh);

sev::acp::MessageHeader convertToAcp(const std_msgs::Header& header);

// sev::acp::Pose
std::tuple<sev::acp::Pose> convertToAcp(
    const geometry_msgs::PoseStamped& ros_message);

std::tuple<geometry_msgs::PoseStamped> convertToRosStamped(
    const sev::acp::Pose& p);

// sev::acp::OperationState
std::tuple<sev::acp::OperationState> convertToAcp(
    const state_machine_msgs::State& status);

std::tuple<state_machine_msgs::State> convertToRosStamped(
    const sev::acp::OperationState& os);

// sev::acp::Notifications
std::tuple<sev::acp::Notifications> convertToAcp(
    const state_machine_msgs::Status& status);

std::tuple<state_machine_msgs::Status> convertToRosStamped(
    const sev::acp::Notifications& notifications);

// sev::acp::PoseFloat
// sev::acp::PoseInt
std::tuple<atlas_msgs::PositioningUpdate> convertToRosStamped(
    const sev::acp::PoseInt& acp_pose);

std::tuple<sev::acp::PoseFloat, sev::acp::PoseInt> convertToAcp(
    const atlas_msgs::PositioningUpdate& ros_pose);

std::tuple<geometry_msgs::PoseStamped, atlas_msgs::PositioningUpdate>
convertToRosStamped(const sev::acp::PoseFloat& acp_pose);

// sev::acp::WheelOdometryInt
// sev::acp::WheelOdometryIntegrated
std::tuple<geometry_msgs::TwistStamped> convertToRosStamped(
    const sev::acp::WheelOdometryInt& woi);

std::tuple<sev::acp::WheelOdometryIntegrated, sev::acp::WheelOdometryInt>
convertToAcp(const geometry_msgs::TwistStamped& ros_message);

std::tuple<geometry_msgs::TwistStamped> convertToRosStamped(
    const sev::acp::WheelOdometryIntegrated& woi);

}  // namespace sev::acp::udp_ros_bridge

#endif  // SEV_ACP_ROS_CONVERSIONS_CONVERSIONS_H_
