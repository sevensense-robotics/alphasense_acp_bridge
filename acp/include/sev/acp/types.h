#ifndef SEV_ACP_TYPES_H_
#define SEV_ACP_TYPES_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdint.h>
#include <utility>

namespace sev {
namespace acp {

typedef uint8_t flag_t;

struct __attribute__((__packed__)) MessageHeader {
  uint32_t seq;       // over all messages sent ?!
  int64_t timestamp;  // epoch nanoseconds
  friend bool operator==(const MessageHeader& lhs, const MessageHeader& rhs) {
    return lhs.seq == rhs.seq && lhs.timestamp == rhs.timestamp;
  }
  friend bool operator!=(const MessageHeader& lhs, const MessageHeader& rhs) {
    return !(lhs == rhs);
  }
};

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


template <typename DERIVED>
struct __attribute__((__packed__)) AbstractBase {
  friend bool operator==(const DERIVED& lhs, const DERIVED& rhs) {
    const char* lhs_begin = reinterpret_cast<const char*>(&lhs);
    const char* lhs_end = lhs_begin + sizeof(DERIVED);
    const char* rhs_begin = reinterpret_cast<const char*>(&rhs);
    return std::equal(lhs_begin, lhs_end, rhs_begin);
  }
};

struct __attribute__((__packed__)) Pose : public AbstractBase<Pose> {
  MessageHeader header;
  double yaw;
  Rot3 R_m_r;
  double velocity[3];  // in map (...)
  double position[3];  // robot center () in map
};

struct __attribute__((__packed__)) OperationState
    : public AbstractBase<OperationState> {
  MessageHeader header;
  int32_t task;                   //  Enum Task
  int32_t stage;                  // Enum SubTask
  char task_uuid[16];             // all 0 iff task = idle / error
  int32_t optimization_progress;  // only valid for stage = OPTIMIZING,
                                  // otherwise -1
};

struct __attribute__((__packed__)) Notifications
    : public AbstractBase<Notifications> {
  MessageHeader header;
  int32_t module_id;
  int32_t status_code;
  int32_t severity;  // Enum Severity
  int32_t logId;     //  -1 if not available  or UUID?
  int32_t logEntryId;
};

struct __attribute__((__packed__)) WheelOdometryIntegrated
    : public AbstractBase<WheelOdometryIntegrated> {
  MessageHeader header;
  double twist[6];  // x,y,z, R,P,Y
};

struct __attribute__((__packed__)) WheelOdometryPose
    : public AbstractBase<WheelOdometryPose> {
  MessageHeader header;
  Rot3 rot;
  double position[3];
};

struct __attribute__((__packed__)) WheelOdometryWheelTicks
    : public AbstractBase<WheelOdometryWheelTicks> {
  MessageHeader header;
  int32_t left_wheel_ticks;
  int32_t right_wheel_ticks;
};

struct __attribute__((__packed__)) WheelOdometryWheelVel
    : public AbstractBase<WheelOdometryWheelVel> {
  MessageHeader header;
  double left_wheel_vel;
  double right_wheel_vel;
};

struct __attribute__((__packed__)) PoseInt : public AbstractBase<PoseInt> {
  MessageHeader header;
  int32_t position_mm[3];
  int32_t velocity_mmps[3];
  int32_t pitch_cdeg;
  int32_t roll_cdeg;
  int32_t yaw_cdeg;
  uint8_t quality;
  flag_t relocalization;
};

struct __attribute__((__packed__)) PoseFloat : public AbstractBase<PoseFloat> {
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
    : public AbstractBase<WheelOdometryInt> {
  MessageHeader header;
  int32_t twist[6];  // x,y,z, R,P,Y
};

}  // namespace acp
}  // namespace sev

#endif  // SEV_ACP_TYPES_H_
