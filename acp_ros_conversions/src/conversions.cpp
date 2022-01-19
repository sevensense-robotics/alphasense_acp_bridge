#include "sev/acp_ros_conversions/conversions.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <utility>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "minkindr_conversions/kindr_msg.h"
#include "std_msgs/Header.h"

#include "sev/acp/types.h"

#include <Eigen/Core>

namespace sev_conversions {

using Transformation = kindr::minimal::QuatTransformation;
using Position3D = kindr::minimal::Position;

// Conversion from rotation matrix to roll, pitch, yaw angles.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
RotationMatrixToRollPitchYaw(const Eigen::MatrixBase<Derived>& rot) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);
  Eigen::Matrix<typename Derived::Scalar, 3, 1> rpy;
  rpy(1, 0) =
      atan2(-rot(2, 0), sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0)));
  if (std::abs(cos(rpy(1, 0))) >
      static_cast<typename Derived::Scalar>(1.0e-12)) {
    rpy(2, 0) = atan2(rot(1, 0) / cos(rpy(1, 0)), rot(0, 0) / cos(rpy(1, 0)));
    rpy(0, 0) = atan2(rot(2, 1) / cos(rpy(1, 0)), rot(2, 2) / cos(rpy(1, 0)));
  } else if (sin(rpy(1, 0)) > static_cast<typename Derived::Scalar>(0)) {
    rpy(2, 0) = static_cast<typename Derived::Scalar>(0);
    rpy(0, 0) = atan2(rot(0, 1), rot(1, 1));
  } else {
    rpy(2, 0) = static_cast<typename Derived::Scalar>(0);
    rpy(0, 0) = -atan2(rot(0, 1), rot(1, 1));
  }
  return rpy;
}

// Conversion from roll, pitch, yaw to rotation matrix.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
RollPitchYawToRotationMatrix(const Eigen::MatrixBase<Derived>& roll_pitch_yaw) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 1);
  Eigen::Matrix<typename Derived::Scalar, 3, 3> rotation_matrix_x;
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0.);
  typename Derived::Scalar one = static_cast<typename Derived::Scalar>(1.);
  rotation_matrix_x << one, zero, zero, zero, cos(roll_pitch_yaw(0)),
      -sin(roll_pitch_yaw(0)), zero, sin(roll_pitch_yaw(0)),
      cos(roll_pitch_yaw(0));

  Eigen::Matrix<typename Derived::Scalar, 3, 3> rotation_matrix_y;
  rotation_matrix_y << cos(roll_pitch_yaw(1)), zero, sin(roll_pitch_yaw(1)),
      zero, one, zero, -sin(roll_pitch_yaw(1)), 0, cos(roll_pitch_yaw(1));

  Eigen::Matrix<typename Derived::Scalar, 3, 3> rotation_matrix_z;
  rotation_matrix_z << cos(roll_pitch_yaw(2)), -sin(roll_pitch_yaw(2)), zero,
      sin(roll_pitch_yaw(2)), cos(roll_pitch_yaw(2)), zero, zero, zero, one;

  return rotation_matrix_z * rotation_matrix_y * rotation_matrix_x;
}

using TimePoint = std::chrono::time_point<std::chrono::system_clock>;
using Duration = std::chrono::nanoseconds;
constexpr double kNanosecondsToSeconds = 1e-9;
constexpr double kSecondsToNanoSeconds = 1e9;

inline TimePoint toTimePoint(const int64_t timestamp_ns) {
  // Making sure that conversion between TimePoint and int64_t comes at no
  // runtime cost.
  static_assert(std::is_same<TimePoint::rep, int64_t>::value, "");
  static_assert(
      std::is_same<
          std::chrono::system_clock::period, std::ratio<1, 1000000000>>::value,
      "");
  return TimePoint{std::chrono::nanoseconds{timestamp_ns}};
}

inline TimePoint toTimePoint(const ros::Time& ros_time) {
  const int64_t timestamp_ns = ros_time.toNSec();
  return toTimePoint(timestamp_ns);
}

inline int64_t toNs(const TimePoint timestamp) {
  return timestamp.time_since_epoch().count();
}

inline ros::Time toRosTime(const TimePoint& time_point) {
  const int64_t timestamp_ns = toNs(time_point);
  const int32_t ros_timestamp_sec = timestamp_ns * kNanosecondsToSeconds;
  const int32_t ros_timestamp_nsec =
      timestamp_ns % static_cast<int64_t>(kSecondsToNanoSeconds);
  return ros::Time(ros_timestamp_sec, ros_timestamp_nsec);
}

inline ros::Time convertNanosecondsInt64ToRosTime(
    const int64_t timestamp_nanoseconds) {
  return toRosTime(toTimePoint(timestamp_nanoseconds));
}

inline int64_t convertRosTimeToNanosecondsInt64(
    const ros::Time& ros_timestamp) {
  return toNs(toTimePoint(ros_timestamp));
}
}  // namespace sev_conversions

namespace sev::acp::udp_ros_bridge {
constexpr double M_TO_MM = 1000;
#ifndef M_PI
#error "M_PI is not defined (issue with _USE_MATH_DEFINES?)"
#endif
constexpr double RAD_TO_CDEG = 100 * 180 / M_PI;

std_msgs::Header convertToRosStamped(sev::acp::MessageHeader const& mh) {
  std_msgs::Header retval;
  retval.seq = mh.seq;
  retval.stamp =
      sev_conversions::convertNanosecondsInt64ToRosTime(mh.timestamp);
  return retval;
}

sev::acp::MessageHeader convertToAcp(const std_msgs::Header& header) {
  sev::acp::MessageHeader mh;
  mh.seq = header.seq;
  mh.timestamp =
      sev_conversions::convertRosTimeToNanosecondsInt64(header.stamp);
  return mh;
}

auto getRotationMatrix(const geometry_msgs::Pose& ros_pose) {
  // Protection against invalid data.
  const auto compute_sq_norm = [](const auto& o) {
    return o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w;
  };
  const auto norm_sq = compute_sq_norm(ros_pose.orientation);

  // Copied from minkindr.
  // The decision boundary should be at the (binary represented) same number,
  // therefore using abs() would lead to slightly different decision than in
  // kindr's CHECK.
  if ((norm_sq > (1. - 1.e-4)) && (norm_sq < (1. + 1.e-4))) {
    sev_conversions::Transformation T_M_I;
    tf::poseMsgToKindr(ros_pose, &T_M_I);
    return T_M_I.getRotation().getRotationMatrix();
  } else {
    std::stringstream ss;
    ss << "Encountered non-normalized orientation during conversion:\n";
    ss << ros_pose.orientation;
    throw ConversionError(ss.str());
  }
}

geometry_msgs::Pose poseFromMatrix(
    const std::array<double, 3>& position_array,
    const sev_conversions::Transformation::RotationMatrix& rotation_matrix) {
  // TODO(pseyfert): Simplify until poseKindrToMsg.
  if (sev_conversions::Transformation::Rotation::isValidRotationMatrix(
          rotation_matrix)) {
    sev_conversions::Position3D position(&position_array[0]);
    const auto quat_trafo = sev_conversions::Transformation(
        sev_conversions::Transformation::Rotation{std::move(rotation_matrix)},
        std::move(position));
    geometry_msgs::Pose ros_pose;
    tf::poseKindrToMsg(std::move(quat_trafo), &ros_pose);
    return ros_pose;
  } else {
    std::stringstream ss;
    ss << "Encountered invalid rotation during conversion:\n";
    ss << rotation_matrix;
    throw ConversionError(ss.str());
  }
}

geometry_msgs::Pose poseFromMatrix(
    const std::array<double, 3>& position_array, const sev::acp::Rot3& R_m_r) {
  // TODO(pseyfert): Simplify until poseKindrToMsg.
  sev_conversions::Transformation::RotationMatrix rotation_matrix;
  for (std::size_t i = 0; i < 9; ++i) {
    rotation_matrix.data()[i] = R_m_r.mat[i];
  }
  return poseFromMatrix(position_array, rotation_matrix);
}

std::tuple<sev::acp::Pose> convertToAcp(
    const geometry_msgs::PoseStamped& ros_message) {
  sev::acp::Pose out_pose;
  const auto& ros_pose = ros_message.pose;
  out_pose.position[0] = ros_pose.position.x;
  out_pose.position[1] = ros_pose.position.y;
  out_pose.position[2] = ros_pose.position.z;

  for (std::size_t i = 0; i < 3; ++i) {
    out_pose.velocity[i] = std::numeric_limits<double>::quiet_NaN();
  }

  const auto rotation_matrix = getRotationMatrix(ros_pose);

  for (std::size_t i = 0; i < 9; ++i) {
    out_pose.R_m_r.mat[i] = rotation_matrix.data()[i];
  }
  out_pose.yaw =
      sev_conversions::RotationMatrixToRollPitchYaw(rotation_matrix)[2];

  out_pose.header = convertToAcp(ros_message.header);
  return std::make_tuple(std::move(out_pose));
}

std::tuple<geometry_msgs::PoseStamped> convertToRosStamped(
    const sev::acp::Pose& acp_pose) {
  geometry_msgs::PoseStamped ros_pose;

  std::array<double, 3> position;
  for (std::size_t i = 0; i < position.size(); i++) {
    position[i] = acp_pose.position[i];
  }
  ros_pose.pose = poseFromMatrix(position, acp_pose.R_m_r);
  // TODO(pseyfert): acp_pose.velocity is ignored.

  ros_pose.header = convertToRosStamped(acp_pose.header);
  return std::make_tuple(std::move(ros_pose));
}

std::tuple<sev::acp::WheelOdometryIntegrated, sev::acp::WheelOdometryInt>
convertToAcp(const geometry_msgs::TwistStamped& ros_message) {
  const auto& linear = ros_message.twist.linear;
  const auto& angular = ros_message.twist.angular;
  const auto header = convertToAcp(ros_message.header);

  const auto woi_int = [&]() {
    sev::acp::WheelOdometryInt woi;
    const auto buffer =
        std::array<int, 6>{linear.x * M_TO_MM,      linear.y * M_TO_MM,
                           linear.z * M_TO_MM,      angular.x * RAD_TO_CDEG,
                           angular.y * RAD_TO_CDEG, angular.z * RAD_TO_CDEG};
    // Can't use std::copy because woi.twist is packed and STL algorithms don't
    // handle packed structs.
    for (std::size_t i = 0; i < buffer.size(); ++i) {
      woi.twist[i] = buffer[i];
    }
    woi.header = header;
    return woi;
  }();
  const auto woi_integrated = [&]() {
    sev::acp::WheelOdometryIntegrated woi;
    const auto buffer = std::array{linear.x,  linear.y,  linear.z,
                                   angular.x, angular.y, angular.z};
    // Can't use std::copy because woi.twist is packed and STL algorithms don't
    // handle packed structs.
    for (std::size_t i = 0; i < buffer.size(); ++i) {
      woi.twist[i] = buffer[i];
    }
    woi.header = header;
    return woi;
  }();
  return {std::move(woi_integrated), std::move(woi_int)};
}

std::tuple<geometry_msgs::TwistStamped> convertToRosStamped(
    const sev::acp::WheelOdometryIntegrated& woi) {
  geometry_msgs::TwistStamped ros_message;

  std::size_t i = 0;
  ros_message.twist.linear.x = woi.twist[i++];
  ros_message.twist.linear.y = woi.twist[i++];
  ros_message.twist.linear.z = woi.twist[i++];
  ros_message.twist.angular.x = woi.twist[i++];
  ros_message.twist.angular.y = woi.twist[i++];
  ros_message.twist.angular.z = woi.twist[i++];

  ros_message.header = convertToRosStamped(woi.header);
  return std::make_tuple(std::move(ros_message));
}

std::tuple<sev::acp::OperationState> convertToAcp(
    const state_machine_msgs::State& status) {
  sev::acp::OperationState acp;
  acp.task = status.state_int;
  acp.stage = status.substate_int;
  // TODO(pseyfert): Fill with meaningful data.
  for (std::size_t i = 0; i < 16; ++i) {
    acp.task_uuid[i] = 0;
  }
  acp.optimization_progress = -1;
  acp.header = convertToAcp(status.header);
  return std::make_tuple(std::move(acp));
}

std::tuple<state_machine_msgs::State> convertToRosStamped(
    const sev::acp::OperationState& os) {
  state_machine_msgs::State state;
  state.state_int = os.task;
  state.substate_int = os.stage;
  // TODO(pseyfert): Fill with meaningful data.
  // state.state =
  // state.substate =
  state.header = convertToRosStamped(os.header);
  return std::make_tuple(std::move(state));
}

// sev::acp::Notifications
std::tuple<sev::acp::Notifications> convertToAcp(
    const state_machine_msgs::Status& status) {
  sev::acp::Notifications acp;
  acp.module_id = status.module;
  acp.status_code = status.status_code;
  acp.severity = status.severity;
  // TODO(pseyfert): Fill with meaningful data.
  acp.logId = -1;
  acp.logEntryId = -1;
  acp.header = convertToAcp(status.header);
  return std::make_tuple(std::move(acp));
}

std::tuple<state_machine_msgs::Status> convertToRosStamped(
    const sev::acp::Notifications& notifications) {
  state_machine_msgs::Status status;
  status.severity = notifications.severity;
  status.module = notifications.module_id;
  status.status_code = notifications.status_code;
  // TODO(pseyfert): Fill with meaningful data.
  // status.status_text =
  status.header = convertToRosStamped(notifications.header);
  return std::make_tuple(std::move(status));
}

std::tuple<atlas_msgs::PositioningUpdate> convertToRosStamped(
    const sev::acp::PoseInt& acp_pose) {
  atlas_msgs::PositioningUpdate retval;
  retval.header = convertToRosStamped(acp_pose.header);
  std::array<double, 3> position;
  for (std::size_t i = 0; i < 3; ++i) {
    position[i] = acp_pose.position_mm[i] / M_TO_MM;
  }
  Eigen::Matrix<double, 3, 1> angles{
      acp_pose.roll_cdeg / RAD_TO_CDEG,
      acp_pose.pitch_cdeg / RAD_TO_CDEG,
      acp_pose.yaw_cdeg / RAD_TO_CDEG,
  };
  const auto rotation_matrix =
      sev_conversions::RollPitchYawToRotationMatrix(angles);
  retval.pose = poseFromMatrix(position, rotation_matrix);
  retval.quality.level = acp_pose.quality;
  retval.is_relocalization_event = acp_pose.relocalization;
  return std::make_tuple(std::move(retval));
}

std::tuple<sev::acp::PoseFloat, sev::acp::PoseInt> convertToAcp(
    const atlas_msgs::PositioningUpdate& ros_msg) {
  const auto rotation_matrix = getRotationMatrix(ros_msg.pose);
  const auto angles =
      sev_conversions::RotationMatrixToRollPitchYaw(rotation_matrix);
  const auto header = convertToAcp(ros_msg.header);

  const auto pose_int = [&]() {
    sev::acp::PoseInt retval;

    retval.position_mm[0] = ros_msg.pose.position.x * M_TO_MM;
    retval.position_mm[1] = ros_msg.pose.position.y * M_TO_MM;
    retval.position_mm[2] = ros_msg.pose.position.z * M_TO_MM;

    for (std::size_t i = 0; i < 3; ++i) {
      // TODO(pseyfert): Fill with meaningful data.
      retval.velocity_mmps[i] = 0;
    }
    retval.quality = ros_msg.quality.level;

    retval.roll_cdeg = angles[0] * RAD_TO_CDEG;
    retval.pitch_cdeg = angles[1] * RAD_TO_CDEG;
    retval.yaw_cdeg = angles[2] * RAD_TO_CDEG;

    retval.relocalization = ros_msg.is_relocalization_event ? 1 : 0;

    retval.header = header;
    return retval;
  }();

  const auto pose_float = [&]() {
    sev::acp::PoseFloat retval;

    retval.position[0] = ros_msg.pose.position.x;
    retval.position[1] = ros_msg.pose.position.y;
    retval.position[2] = ros_msg.pose.position.z;

    for (std::size_t i = 0; i < 3; ++i) {
      retval.velocity[i] = std::numeric_limits<double>::quiet_NaN();
    }
    retval.quality = ros_msg.quality.level;

    for (std::size_t i = 0; i < 9; ++i) {
      retval.rot.mat[i] = rotation_matrix.data()[i];
    }
    retval.roll = angles[0];
    retval.pitch = angles[1];
    retval.yaw = angles[2];

    retval.relocalization = ros_msg.is_relocalization_event ? 1 : 0;

    retval.header = header;
    return retval;
  }();

  return {std::move(pose_float), std::move(pose_int)};
}

std::tuple<geometry_msgs::PoseStamped, atlas_msgs::PositioningUpdate>
convertToRosStamped(const sev::acp::PoseFloat& acp_pose) {
  atlas_msgs::PositioningUpdate positioning_update;
  geometry_msgs::PoseStamped ros_pose;

  std::array<double, 3> position;
  for (std::size_t i = 0; i < position.size(); i++) {
    position[i] = acp_pose.position[i];
  }
  positioning_update.pose = poseFromMatrix(position, acp_pose.rot);
  ros_pose.pose = poseFromMatrix(position, acp_pose.rot);

  positioning_update.header = convertToRosStamped(acp_pose.header);
  ros_pose.header = convertToRosStamped(acp_pose.header);
  positioning_update.quality.level = acp_pose.quality;
  positioning_update.is_relocalization_event = acp_pose.relocalization;
  return std::make_tuple(std::move(ros_pose), std::move(positioning_update));
}

std::tuple<geometry_msgs::TwistStamped> convertToRosStamped(
    const sev::acp::WheelOdometryInt& woi) {
  geometry_msgs::TwistStamped ros_message;

  std::size_t i = 0;
  ros_message.twist.linear.x = woi.twist[i++] / M_TO_MM;
  ros_message.twist.linear.y = woi.twist[i++] / M_TO_MM;
  ros_message.twist.linear.z = woi.twist[i++] / M_TO_MM;
  ros_message.twist.angular.x = woi.twist[i++] / RAD_TO_CDEG;
  ros_message.twist.angular.y = woi.twist[i++] / RAD_TO_CDEG;
  ros_message.twist.angular.z = woi.twist[i++] / RAD_TO_CDEG;

  ros_message.header = convertToRosStamped(woi.header);
  return std::make_tuple(std::move(ros_message));
}
}  // namespace sev::acp::udp_ros_bridge
