#ifndef SEV_ACP_SERIALIZATION_BASE_H_
#define SEV_ACP_SERIALIZATION_BASE_H_

#include <cstddef>
#include <cstdint>  // TODO(pseyfert): review necessity here.
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>

namespace sev {
namespace acp {
enum class MessageType : int32_t {
  kPose = 0,
  kOperationState = 1,
  kNotification = 2,
  kWheelOdometryIntegrated = 3,
  kWheelOdometryPose = 4,
  kWheelOdometryWheelTicks = 5,
  kWheelOdometryWheelVel = 6,
  kPoseInt = 7,
  kPoseFloat = 8,
  kWheelOdometryInt = 9,
  kLast = kWheelOdometryInt,
  //
  // kUninitialized = 99
};

inline const std::string& message_type_lookup(acp::MessageType mt) {
  static const std::map<MessageType, std::string> message_type_lookup{
      {MessageType::kPose, "Pose"},
      {MessageType::kOperationState, "OperationState"},
      {MessageType::kNotification, "Notification"},
      {MessageType::kWheelOdometryIntegrated, "WheelOdometryIntegrated"},
      {MessageType::kWheelOdometryPose, "WheelOdometryPose"},
      {MessageType::kWheelOdometryWheelTicks, "WheelOdometryWheelTicks"},
      {MessageType::kWheelOdometryWheelVel, "WheelOdometryWheelVel"},
      {MessageType::kPoseInt, "PoseInt"},
      {MessageType::kPoseFloat, "PoseFloat"},
      {MessageType::kWheelOdometryInt, "WheelOdometryInt"},
  };
  try {
    return message_type_lookup.at(mt);
  } catch (const std::out_of_range&) {
    throw std::logic_error("Encountered MessageType without name lookup.");
  }
}

struct __attribute__((__packed__)) MessageHeader {
  uint32_t seq;       // over all messages sent ?!
  int64_t timestamp;  // epoch nanoseconds
  int read(unsigned char const* src) {
    const auto retval = sizeof(decltype(*this));
    std::memcpy(reinterpret_cast<char*>(this), src, retval);
    return retval;
  }
  friend bool operator==(const MessageHeader& lhs, const MessageHeader& rhs) {
    return lhs.seq == rhs.seq && lhs.timestamp == rhs.timestamp;
  }
  friend bool operator!=(const MessageHeader& lhs, const MessageHeader& rhs) {
    return !(lhs == rhs);
  }
};
template <typename T>
struct serializable_trait {
  constexpr static bool value = false;
};

template <>
struct serializable_trait<struct MessageHeader> {
  constexpr static bool value = true;
};
template <>
struct serializable_trait<struct Rot3> {
  constexpr static bool value = true;
};
template <>
struct serializable_trait<struct Pose> {
  constexpr static bool value = true;
};
template <>
struct serializable_trait<struct OperationState> {
  constexpr static bool value = true;
};
template <>
struct serializable_trait<struct Notifications> {
  constexpr static bool value = true;
};
template <>
struct serializable_trait<struct WheelOdometryIntegrated> {
  constexpr static bool value = true;
};
template <>
struct serializable_trait<struct WheelOdometryPose> {
  constexpr static bool value = true;
};
template <>
struct serializable_trait<struct WheelOdometryWheelTicks> {
  constexpr static bool value = true;
};
template <>
struct serializable_trait<struct WheelOdometryWheelVel> {
  constexpr static bool value = true;
};
template <>
struct serializable_trait<struct PoseInt> {
  constexpr static bool value = true;
};
template <>
struct serializable_trait<struct PoseFloat> {
  constexpr static bool value = true;
};
template <>
struct serializable_trait<struct WheelOdometryInt> {
  constexpr static bool value = true;
};

template <
    typename T,
    typename = typename std::enable_if<serializable_trait<T>::value>::type>
constexpr std::size_t message_size() {
  return sizeof(MessageType) + sizeof(T);
}
template <
    typename T, typename Callable,
    typename = typename std::enable_if<
        serializable_trait<typename std::decay<T>::type>::value>::type
    // ,
    // typename = typename std::enable_if_t<std::is_invocable_r<
    //     int, #<{(|decay?|)}># std::decay_t<Callable>, #<{(|char?void?|)}>#
    //     const T*,
    //     #<{(|size_t?|)}># int>::value>
    >
int write_lambda(const T* obj, Callable&& c) {
  std::size_t written = 0;
  MessageType t = T::message_type();
  while (written < sizeof(MessageType)) {
    written +=
        c(reinterpret_cast<const char*>(&t) + written, sizeof(MessageType));
  }
  while (written < message_size<T>()) {
    // Write starting at obj+written, and write at most what's left to be
    // written.
    written +=
        c(reinterpret_cast<const char*>(obj) + written - sizeof(MessageType),
          message_size<T>() - written);
  }
  if (written != message_size<T>()) {
    throw std::length_error(
        "Number of bytes written doesn't match the message size for that "
        "message type");
  }
  return written;
}

template <
    typename T,
    typename = typename std::enable_if<serializable_trait<T>::value>::type>
int write_fun(
    const T* obj, unsigned char* dest,
    std::size_t max_buffer = message_size<T>()) {
  std::size_t written = 0;
  if (message_size<T>() > max_buffer) {
    throw std::runtime_error(
        std::string("Message size for this type (") +
        std::to_string(message_size<T>()) +
        std::string(") is larger than the write-bytes limit (") +
        std::to_string(max_buffer) + std::string(")."));
  }
  return write_lambda(
      obj, [dest, &written](const char* start, std::size_t size) {
        if (written + size > message_size<T>()) {
          throw std::length_error(
              std::string("Trying to write more bytes (") +
              std::to_string(written + size) +
              std::string(") than necessary for this message type (") +
              std::to_string(message_size<T>()) + std::string(")."));
        }
        std::memcpy(dest + written, start, size);
        written += size;
        return size;
      });
}

template <
    typename T, typename Callable,
    typename = typename std::enable_if<serializable_trait<T>::value>::type>
int read_lambda(T* dest, Callable&& c) {
  std::size_t read = 0;
  MessageType read_type;
  while (read < sizeof(MessageType)) {
    read += c(
        reinterpret_cast<char*>(&read_type) + read, sizeof(MessageType) - read);
  }
  if (read_type != T::message_type()) {
    throw std::runtime_error(
        "Message contains a different type than the one that's being "
        "deserialized to.");
  }
  while (read < message_size<T>()) {
    read +=
        c(reinterpret_cast<char*>(dest) + read - sizeof(MessageType),
          message_size<T>() - read);
  }
  return read;
}

template <
    typename T,
    typename = typename std::enable_if<serializable_trait<T>::value>::type>
int read_fun(T* obj, const unsigned char* src, std::size_t max_read_size) {
  std::size_t already_done = 0;
  return read_lambda(
      obj,
      [&already_done, src, max_read_size](char* dest, std::size_t read_size) {
        if (max_read_size < already_done + read_size) {
          throw std::length_error(
              std::string("Trying to read until byte ") +
              std::to_string(already_done + read_size) +
              std::string(", which is past the read-bytes limit (.") +
              std::to_string(max_read_size) + std::string(")."));
        }
        std::memcpy(dest, src + already_done, read_size);
        already_done += read_size;
        return read_size;
      });
}

}  // namespace acp
}  // namespace sev

#endif  // SEV_ACP_SERIALIZATION_BASE_H_
