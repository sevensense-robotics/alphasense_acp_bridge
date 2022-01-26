#include <atomic>
#include <bits/exception.h>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <type_traits>
#include <variant>

#include "sev/acp/serialization.h"
#include "sev/acp/types.h"
#include "sev/acp_udp/port.h"
#include "sev/acp_udp/receiver.h"

std::atomic_bool keep_going = true;

template <class>
inline constexpr bool always_false_v = false;

void sighand(int sig) {
  keep_going = false;
}

int main() {
  signal(SIGINT, sighand);
  sev::acp::udp::UdpReceiver r(sev::acp::udp::default_port);
  while (keep_going) {
    try {
      auto something = r.receive_Messages<
          false, sev::acp::Pose, sev::acp::OperationState,
          sev::acp::Notifications, sev::acp::WheelOdometryIntegrated,
          sev::acp::WheelOdometryPose, sev::acp::WheelOdometryWheelTicks,
          sev::acp::WheelOdometryWheelVel, sev::acp::PoseInt,
          sev::acp::PoseFloat, sev::acp::WheelOdometryInt>(&keep_going);

      std::visit(
          [](auto&& arg) {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, sev::acp::MessageType>) {
              printf("Received an unexpected message\n");
              printf("Message type %x\n", static_cast<int32_t>(arg));
            } else {
              printf(
                  "Received a %s\n",
                  sev::acp::message_type_name_lookup(
                      sev::acp::MessageTypeIdLookup<
                          std::decay_t<decltype(arg)>>::message_type)
                      .c_str());
              if constexpr (std::is_same_v<T, sev::acp::Pose>) {
                printf(
                    "Message seq %x, time %lx\n", arg.header.seq,
                    arg.header.timestamp);
                printf(
                    "Received LEGACY Position %f, %f, %f\n", arg.position[0],
                    arg.position[1], arg.position[2]);
              } else if constexpr (std::is_same_v<
                                       T, sev::acp::OperationState>) {
                printf(
                    "Message seq %x, time %lx\n", arg.header.seq,
                    arg.header.timestamp);
                printf("Task: %d, Sub-Task: %d\n", arg.task, arg.stage);
              } else if constexpr (std::is_same_v<T, sev::acp::Notifications>) {
                printf(
                    "Message seq %x, time %lx\n", arg.header.seq,
                    arg.header.timestamp);
                printf(
                    "Module: %d, StatusCode: %d, Severity: %d, LogID: %d, "
                    "LogEntryID: %d\n",
                    arg.module_id, arg.status_code, arg.severity, arg.logId,
                    arg.logEntryId);
              } else if constexpr (std::is_same_v<
                                       T, sev::acp::WheelOdometryIntegrated>) {
                printf(
                    "Message seq %x, time %lx\n", arg.header.seq,
                    arg.header.timestamp);
                printf(
                    "Twist: %f, %f, %f, %f, %f, %f\n", arg.twist[0],
                    arg.twist[1], arg.twist[2], arg.twist[3], arg.twist[4],
                    arg.twist[5]);
              } else if constexpr (std::is_same_v<
                                       T, sev::acp::WheelOdometryPose>) {
                printf(
                    "Message seq %x, time %lx\n", arg.header.seq,
                    arg.header.timestamp);
                printf(
                    "Received Position %f, %f, %f\n", arg.position[0],
                    arg.position[1], arg.position[2]);
              } else if constexpr (std::is_same_v<
                                       T, sev::acp::WheelOdometryWheelTicks>) {
                printf(
                    "Message seq %x, time %lx\n", arg.header.seq,
                    arg.header.timestamp);
                printf(
                    "Left: %d, Right: %d.\n", arg.left_wheel_ticks,
                    arg.right_wheel_ticks);
              } else if constexpr (std::is_same_v<
                                       T, sev::acp::WheelOdometryWheelVel>) {
                printf(
                    "Message seq %x, time %lx\n", arg.header.seq,
                    arg.header.timestamp);
                printf(
                    "Left: %f, Right: %f.\n", arg.left_wheel_vel,
                    arg.right_wheel_vel);
              } else if constexpr (std::is_same_v<T, sev::acp::PoseInt>) {
                printf(
                    "Message seq %x, time %lx\n", arg.header.seq,
                    arg.header.timestamp);
                printf(
                    "Received Integer Position (mm) %d, %d, %d\n",
                    arg.position_mm[0], arg.position_mm[1], arg.position_mm[2]);
              } else if constexpr (std::is_same_v<T, sev::acp::PoseFloat>) {
                printf(
                    "Message seq %x, time %lx\n", arg.header.seq,
                    arg.header.timestamp);
                printf(
                    "Received floating point Position %f, %f, %f\n",
                    arg.position[0], arg.position[1], arg.position[2]);
              } else if constexpr (std::is_same_v<
                                       T, sev::acp::WheelOdometryInt>) {
                printf(
                    "Message seq %x, time %lx\n", arg.header.seq,
                    arg.header.timestamp);
                printf(
                    "Twist: %d, %d, %d, %d, %d, %d\n", arg.twist[0],
                    arg.twist[1], arg.twist[2], arg.twist[3], arg.twist[4],
                    arg.twist[5]);
              } else {
                static_assert(always_false_v<T>, "non-exhaustive visitor!");
              }
            }
          },
          something);
    } catch (std::exception& e) {
      std::cerr << e.what() << std::endl;
    } catch (int& i) {
      std::cerr << "integer i = " << i << " got thrown\n";
    }
  }

  return 0;
}
