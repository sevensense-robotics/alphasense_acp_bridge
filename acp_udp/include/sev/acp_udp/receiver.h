#ifndef SEV_ACP_UDP_RECEIVER_H_
#define SEV_ACP_UDP_RECEIVER_H_

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <errno.h>
#include <optional>
#include <poll.h>
#include <signal.h>
#include <system_error>
#include <variant>

#include <arpa/inet.h>
#include <iostream>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "sev/acp/serialization.h"
#include "sev/acp_udp/port.h"

namespace sev::acp::udp {
struct UdpReceiver {
 public:
  UdpReceiver() : UdpReceiver(default_port){};

  explicit UdpReceiver(unsigned short port) {
    // Creating socket file descriptor
    if ((sock_pollfd.fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      throw std::system_error(errno, std::system_category());
    }

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;  // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    // Bind the socket with the server address
    if (bind(
            sock_pollfd.fd, (const struct sockaddr*)&servaddr,
            sizeof(servaddr)) < 0) {
      throw std::system_error(errno, std::system_category());
    }

    sock_pollfd.events = POLLIN;
  }

 private:
  pollfd sock_pollfd;

 protected:
  template <typename T>
  std::size_t receive_bytes(T&& buffer, std::atomic_bool* keep_listening) {
    while (*keep_listening) {
      int poll_res = poll(&sock_pollfd, 1 /*nfds*/, 100 /*timeout ms*/);
      if (poll_res == -1) {
        throw std::system_error(errno, std::system_category());
      }
      if (poll_res == 0) {
        continue;
      }

      int bytes_received =
          recv(sock_pollfd.fd, buffer.data(), buffer.size(), MSG_WAITALL);
      if (bytes_received < 0) {
        throw std::system_error(errno, std::system_category());
      }
      return bytes_received;
    }
    return 0;
  }

 public:
  template <typename Message>
  std::optional<Message> receive_single_message(
      std::atomic_bool* keep_listening) {
    constexpr auto size = acp::message_size<Message>();

    std::array<unsigned char, size> recv_buf;
    const size_t len = receive_bytes(recv_buf, keep_listening);

    if (len == 0) {
      return std::nullopt;
    }

    const auto received = acp::read_variant_with_fallback<Message>(
        static_cast<const unsigned char*>(recv_buf.data()), len);

    if (std::holds_alternative<Message>(received)) {
      return std::get<Message>(received);
    }
    return std::nullopt;
  }

  template <bool throw_exception = false, typename... Message>
  auto receive_Messages(std::atomic_bool* keep_listening) {
    constexpr auto max_size = std::max({acp::message_size<Message>()...});

    std::array<unsigned char, max_size> recv_buf;
    const size_t len = receive_bytes(recv_buf, keep_listening);

    if constexpr (throw_exception) {
      return acp::read_variant<Message...>(
          static_cast<const unsigned char*>(recv_buf.data()), len);
    } else {
      return acp::read_variant_with_fallback<Message...>(
          static_cast<const unsigned char*>(recv_buf.data()), len);
    }
  }
};
}  // namespace sev::acp::udp

#endif  // SEV_ACP_UDP_RECEIVER_H_
