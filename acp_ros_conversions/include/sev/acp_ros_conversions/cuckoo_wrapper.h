#ifndef SEV_ACP_ROS_CONVERSIONS_CUCKOO_WRAPPER_H_
#define SEV_ACP_ROS_CONVERSIONS_CUCKOO_WRAPPER_H_
#include <algorithm>
#include <array>
#include <cuckoo_time_translator/DeviceTimeTranslator.h>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <stdint.h>
#include "cuckoo_time_translator/ClockParameters.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "sev/acp/serialization.h"

namespace sev::acp::udp_ros_bridge {
struct MessageTimes {
  uint64_t transmitted;
  ros::Time received;
};

class TimeTranslator {
  std::mutex translator_mutex_;
  cuckoo_time_translator::UnwrappedDeviceTimeTranslator time_translator_;
  std::array<
      uint64_t, static_cast<std::size_t>(sev::acp::MessageType::kLast) + 1>
      last_timestamp_per_type_;
  uint64_t last_timestamp_{0};

 public:
  explicit TimeTranslator(ros::NodeHandle* n)
      : time_translator_{
            cuckoo_time_translator::ClockParameters(1.e9), n->getNamespace(),
            cuckoo_time_translator::Defaults()
                .setFilterAlgorithm(
                    cuckoo_time_translator::FilterAlgorithm::ConvexHull)
                .setSwitchTimeSecs(100)} {
    for (auto& element : last_timestamp_per_type_) {
      element = 0;
    }
  }

  template <typename AcpType>
  std::optional<ros::Time> translate(const MessageTimes message_times) {
    const auto retval = compute_republish_time<AcpType>(message_times);
    if (retval) {
      reset_last_timestamps<AcpType>(message_times);
    }
    return retval;
  }

 private:
  template <typename AcpType>
  void reset_last_timestamps(const MessageTimes message_times) {
    const std::scoped_lock<std::mutex> translator_lock(translator_mutex_);
    const auto last_timestamp_typed_it = [&]() {
      auto tmp = last_timestamp_per_type_.begin();
      std::advance(
          tmp,
          static_cast<std::size_t>(MessageTypeIdLookup<AcpType>::message_type));
      return tmp;
    }();
    last_timestamp_ = std::max(last_timestamp_, message_times.transmitted);
    *last_timestamp_typed_it =
        std::max(*last_timestamp_typed_it, message_times.transmitted);
  }
  template <typename AcpType>
  std::optional<ros::Time> compute_republish_time(
      const MessageTimes message_times) {
    const auto last_timestamp_typed_it = [&]() {
      auto tmp = last_timestamp_per_type_.begin();
      std::advance(
          tmp,
          static_cast<std::size_t>(MessageTypeIdLookup<AcpType>::message_type));
      return tmp;
    }();
    const std::scoped_lock<std::mutex> translator_lock(translator_mutex_);
    if (message_times.transmitted > last_timestamp_) {
      try {
        return time_translator_.update(
            message_times.transmitted, message_times.received, 0);
      } catch (const std::runtime_error& e) {
        std::cout << "WARNING: Resetting Cuckoo because Cuckoo failed to "
                     "translate timestamps:\n"
                  << e.what();
        time_translator_.resetTranslation();
        return message_times.received;
      }
    } else {
      if (message_times.transmitted <= *last_timestamp_typed_it) {
        std::cout << "WARNING: Receiving a new message of type "
                  << sev::acp::message_type_name_lookup(
                         MessageTypeIdLookup<AcpType>::message_type)
                  << " but its timestamp is not newer "
                     "than a previous message of the same "
                     "type. Dropping message.";
        return std::nullopt;
      } else {
        return time_translator_.translate(message_times.transmitted);
      }
    }
  }
};
}  // namespace sev::acp::udp_ros_bridge

#endif  // SEV_ACP_ROS_CONVERSIONS_CUCKOO_WRAPPER_H_
