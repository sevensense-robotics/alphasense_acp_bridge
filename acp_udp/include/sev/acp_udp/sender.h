#ifndef SEV_ACP_UDP_SENDER_H_
#define SEV_ACP_UDP_SENDER_H_

#include <arpa/inet.h>
#include <array>
#include <errno.h>
#include <memory>
#include <netinet/in.h>
#include <stdexcept>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <system_error>
#include <type_traits>
#include <unistd.h>

#include "sev/acp/serialization_base.h"
#include "sev/acp_udp/port.h"

namespace sev {
namespace acp {
namespace udp {
struct UdpSender {
 public:
  explicit UdpSender(const char* address) : UdpSender{address, default_port} {};
  UdpSender(const char* address, unsigned short port) {
    // Creating socket file descriptor
    if ((*sockfd_ptr = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      throw std::system_error(errno, std::system_category());
    }

    memset(&servaddr, 0, sizeof(servaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = inet_addr(address);
  }
  UdpSender(const UdpSender&) = delete;
  UdpSender& operator=(const UdpSender&) = delete;
  UdpSender(UdpSender&&) = default;
  UdpSender& operator=(UdpSender&&) = default;
  ~UdpSender() {
    // Will be false in a moved-from state.
    if (sockfd_ptr) {
      close(*sockfd_ptr);
    }
  }

 private:
  std::unique_ptr<int> sockfd_ptr{std::make_unique<int>()};
  struct sockaddr_in servaddr;

 protected:
  template <typename T>
  ssize_t send_buffer(T&& buffer) const {
    auto bytes_sent = sendto(
        *sockfd_ptr, reinterpret_cast<const char*>(buffer.data()),
        buffer.size(), MSG_CONFIRM, (const struct sockaddr*)&servaddr,
        sizeof(servaddr));
    static_assert(
        std::is_same<decltype(bytes_sent), ssize_t>::value,
        "Unexpected return type from sendto.");
    if (bytes_sent <= 0) {
      throw std::system_error(errno, std::system_category());
    }
    return bytes_sent;
  }

 public:
  template <typename Message>
  void send_message(Message&& m) const {
    constexpr int message_size = acp::message_size<std::decay_t<Message>>();
    std::array<unsigned char, message_size> a;
    write_fun<typename std::decay<Message>::type>(
        static_cast<typename std::decay<Message>::type const*>(&m), a.data(),
        a.size());
    // m.write(static_cast<unsigned char*>(a.data()));

    auto sent_bytes = send_buffer(a);
    if (sent_bytes != message_size) {
      throw std::runtime_error(
          std::string("Sent bytes (") + std::to_string(sent_bytes) +
          std::string("do not match message size (") +
          std::to_string(message_size) + std::string(")."));
    }
  }
};
}  // namespace udp
}  // namespace acp
}  // namespace sev

#endif  // SEV_ACP_UDP_SENDER_H_
