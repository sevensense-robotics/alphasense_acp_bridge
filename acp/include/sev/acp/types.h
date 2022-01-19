#ifndef SEV_ACP_TYPES_H_
#define SEV_ACP_TYPES_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "sev/acp/serialization.h"

namespace sev::acp {

using flag_t = uint8_t;

inline bool equal_or_both_nan(double lhs, double rhs) {
  return lhs == rhs || (std::isnan(lhs) && std::isnan(rhs));
}

struct __attribute__((__packed__)) Rot3 {
  double mat[9];
  friend bool operator==(const Rot3& lhs, const Rot3& rhs) {
    for (std::size_t i = 0; i < 9; ++i) {
      if (!equal_or_both_nan(lhs.mat[i], rhs.mat[i])) {
        return false;
      }
    }
    return true;
  }
  friend bool operator!=(const Rot3& lhs, const Rot3& rhs) {
    return !(lhs == rhs);
  }
};

template <typename DERIVED, MessageType MESSAGE_TYPE>
struct __attribute__((__packed__)) AbstractBase {
  constexpr static MessageType message_type() {
    return MESSAGE_TYPE;
  }
  template <std::size_t SIZE>
  int write(std::array<char, SIZE>& dest) const {
    return write(reinterpret_cast<unsigned char*>(dest.data()), SIZE);
  }
  template <std::size_t SIZE>
  int write(std::array<unsigned char, SIZE>& dest) const {
    return write(dest.data(), SIZE);
  }
  int write(
      unsigned char* dest,
      std::size_t max_buffer = message_size<DERIVED>()) const {
    return write_fun<DERIVED>(
        static_cast<DERIVED const*>(this), dest, max_buffer);
  }
  template <typename CALLABLE>
  int write_callable(CALLABLE&& callable) const {
    return write_lambda<DERIVED, CALLABLE>(
        static_cast<DERIVED const*>(this), std::forward<CALLABLE>(callable));
  }
  int read(unsigned char const* src, std::size_t size) {
    return read_fun<DERIVED>(static_cast<DERIVED*>(this), src, size);
  }
  friend bool operator==(const DERIVED& lhs, const DERIVED& rhs) {
    auto lhs_begin = reinterpret_cast<const char*>(&lhs);
    auto lhs_end = lhs_begin + sizeof(DERIVED);
    auto rhs_begin = reinterpret_cast<const char*>(&rhs);
    return std::equal(lhs_begin, lhs_end, rhs_begin);
  }
};

struct __attribute__((__packed__)) Pose
    : public AbstractBase<Pose, acp::MessageType::kPose> {
  MessageHeader header;
  double yaw;
  Rot3 R_m_r;
  double velocity[3];  // in map (...)
  double position[3];  // robot center () in map
};

struct __attribute__((__packed__)) OperationState
    : public AbstractBase<OperationState, acp::MessageType::kOperationState> {
  MessageHeader header;
  int32_t task;                   //  Enum Task
  int32_t stage;                  // Enum SubTask
  char task_uuid[16];             // all 0 iff task = idle / error
  int32_t optimization_progress;  // only valid for stage = OPTIMIZING,
                                  // otherwise -1
};

struct __attribute__((__packed__)) Notifications
    : public AbstractBase<Notifications, acp::MessageType::kNotification> {
  MessageHeader header;
  int32_t module_id;
  int32_t status_code;
  int32_t severity;  // Enum Severity
  int32_t logId;     //  -1 if not available  or UUID?
  int32_t logEntryId;
};
// Ensure the empty base class doesn't add size.
static_assert(
    sizeof(Notifications) == 5 * sizeof(int32_t) + sizeof(MessageHeader));
// Ensure there are no padding bits at the end of the MessageHeader.
static_assert(
    sizeof(Notifications) ==
    5 * sizeof(int32_t) + sizeof(int64_t) + sizeof(uint32_t));

struct __attribute__((__packed__)) WheelOdometryIntegrated
    : public AbstractBase<
          WheelOdometryIntegrated, acp::MessageType::kWheelOdometryIntegrated> {
  MessageHeader header;
  double twist[6];  // x,y,z, Y,P, R,
};

struct __attribute__((__packed__)) WheelOdometryPose
    : public AbstractBase<
          WheelOdometryPose, acp::MessageType::kWheelOdometryPose> {
  MessageHeader header;
  Rot3 rot;
  double position[3];
};

struct __attribute__((__packed__)) WheelOdometryWheelTicks
    : public AbstractBase<
          WheelOdometryWheelTicks, acp::MessageType::kWheelOdometryWheelTicks> {
  MessageHeader header;
  int32_t left_wheel_ticks;
  int32_t right_wheel_ticks;
};

struct __attribute__((__packed__)) WheelOdometryWheelVel
    : public AbstractBase<
          WheelOdometryWheelVel, acp::MessageType::kWheelOdometryWheelVel> {
  MessageHeader header;
  double left_wheel_vel;
  double right_wheel_vel;
};

struct __attribute__((__packed__)) PoseInt
    : public AbstractBase<PoseInt, acp::MessageType::kPoseInt> {
  MessageHeader header;
  int32_t position_mm[3];
  int32_t velocity_mmps[3];
  int32_t pitch_cdeg;
  int32_t roll_cdeg;
  int32_t yaw_cdeg;
  uint8_t quality;
  flag_t relocalization;
};

struct __attribute__((__packed__)) PoseFloat
    : public AbstractBase<PoseFloat, acp::MessageType::kPoseFloat> {
  MessageHeader header;
  double position[3];
  double velocity[3];
  Rot3 rot;
  double pitch;
  double roll;
  double yaw;
  uint8_t quality;
  flag_t relocalization;
};

struct __attribute__((__packed__)) WheelOdometryInt
    : public AbstractBase<
          WheelOdometryInt, acp::MessageType::kWheelOdometryInt> {
  MessageHeader header;
  int32_t twist[6];  // x,y,z, Y,P,R
};

static_assert(
    message_size<WheelOdometryInt>() == 40,
    "WheelOdometryInt size unexpected.");

}  // namespace sev::acp

#endif  // SEV_ACP_TYPES_H_
