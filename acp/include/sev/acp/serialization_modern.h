#ifndef SEV_ACP_SERIALIZATION_MODERN_H_
#define SEV_ACP_SERIALIZATION_MODERN_H_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>

#include "serialization_base.h"

namespace sev::acp {
template <typename... T>
constexpr bool serializable_v = (serializable_trait<T>::value && ...);

namespace detail {
template <typename DesiredType, typename... AllTypes>
bool read_one_type(
    MessageType read_type, const unsigned char* data, std::size_t size,
    std::variant<AllTypes...>* target) {
  if (read_type != MessageTypeIdLookup<DesiredType>::message_type) {
    return false;
  }
  if (size < message_size<DesiredType>()) {
    throw std::length_error(
        std::string("The byte-read limit (") + std::to_string(size) +
        std::string(") indicates a message size that's too small for the "
                    "message type that's being deserialized (") +
        std::to_string(message_size<DesiredType>()) + std::string(")."));
  }
  if (target == nullptr) {
    throw std::invalid_argument("Can't deserialize into a NULL object.");
  }
  *target = DesiredType{};
  read_fun(&std::get<DesiredType>(*target), data, size);
  return true;
}

// TODO(pseyfert): There must be a nicer way to end the recursion.
template <typename T>
constexpr bool validate_message_types_distinctive() {
  return true;
}
template <typename T1, typename T2, typename... Ts>
constexpr bool validate_message_types_distinctive() {
  return (MessageTypeIdLookup<T1>::message_type !=
          MessageTypeIdLookup<T2>::message_type) &&
         ((MessageTypeIdLookup<T1>::message_type !=
           MessageTypeIdLookup<Ts>::message_type) &&
          ...) &&
         validate_message_types_distinctive<T2, Ts...>();
}
}  // namespace detail

template <
    typename... T, typename = typename std::enable_if_t<serializable_v<T...>>>
std::variant<MessageType, T...> read_variant_with_fallback(
    const unsigned char* src, std::size_t size) {
  static_assert(detail::validate_message_types_distinctive<T...>());
  std::variant<MessageType, T...> retval;
  MessageType read_type;
  if (size < sizeof(MessageType)) {
    throw std::length_error(
        std::string("The byte-read limit is too low (") + std::to_string(size) +
        std::string(") to read the message type."));
  }
  std::memcpy(reinterpret_cast<char*>(&read_type), src, sizeof(MessageType));
  bool success =
      (detail::read_one_type<T>(read_type, src, size, &retval) || ...);
  if (!success) {
    return read_type;
  }
  return retval;
}

template <
    typename... T, typename = typename std::enable_if_t<serializable_v<T...>>>
std::variant<T...> read_variant(const unsigned char* src, std::size_t size) {
  static_assert(detail::validate_message_types_distinctive<T...>());
  auto deser = read_variant_with_fallback<T...>(src, size);
  if (std::holds_alternative<acp::MessageType>(deser)) {
    auto message_type = std::get<acp::MessageType>(deser);
    auto what = std::string("Encountered a message of type ") +
                message_type_name_lookup(message_type) +
                ", which is not expected.";
    throw std::runtime_error(what);
  }
  // This is casting variant<ruled_out, T...> -> variant<T...>.
  // The MessageType variant has been ruled out at this point.
  return std::visit(
      [](auto const& input) -> std::variant<T...> {
        using U = std::decay_t<decltype(input)>;
        if constexpr (std::is_same_v<U, MessageType>) {
          throw std::logic_error("Reached unreachable code.");
        } else {
          return input;
        }
      },
      deser);
}

// Asserts from types.h.

// Ensure the empty base class doesn't add size.
static_assert(
    sizeof(Notifications) == 5 * sizeof(int32_t) + sizeof(MessageHeader),
    "Crosscheck in message size computation failed.");
// Ensure there are no padding bits at the end of the MessageHeader.
static_assert(
    sizeof(Notifications) ==
        5 * sizeof(int32_t) + sizeof(int64_t) + sizeof(uint32_t),
    "Crosscheck in message size computation failed.");

}  // namespace sev::acp

#endif  // SEV_ACP_SERIALIZATION_MODERN_H_
