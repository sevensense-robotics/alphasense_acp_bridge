#include "sev/acp_external_bridge/cli.h"
#include <boost/program_options.hpp>
#include <iostream>
#include <sev/acp_udp/port.h>

namespace po = boost::program_options;

std::optional<Config> parse_args(int ac, char** av) {  // NOLINT
  Config config;
  po::options_description desc("Alphasense Embedded UDP bridge");
  desc.add_options()
      // clang-format off
    (
     "help",
     "Show help message."
    )
    (
      "pose-port",
      po::value<uint16_t>(&config.receive_port)
          ->default_value(sev::acp::udp::default_port)
          ->value_name("PORT"),
      "Port on which the bridge listens for pose messages."
    )
    (
      "alphasense-port",
      po::value<uint16_t>(&config.ae_port)
          ->default_value(sev::acp::udp::default_port)
          ->value_name("PORT"),
      "Port on which the Alphasense Embedded is listening for Odometry data."
    )
    (
      "alphasense-ip",
      po::value<std::string>(&config.ae_address)
          ->value_name("IP"),
      "IP Address of Alphasense Embedded."
    )
    (
      "pose-topic",
      po::value<std::string>(&config.pose_prefix)
          ->default_value("")
          ->value_name("PREFIX"),
      "ROS topic prefix on which poses will be published. "
      "Leave empty to disable."
    )
    (
      "notification-topic",
      po::value<std::string>(&config.notification_topic)
          ->default_value("")
          ->value_name("TOPIC"),
      "ROS topic on which notifications will be published. "
      "Leave empty to disable."
    )
    (
      "operation-state-topic",
      po::value<std::string>(&config.operation_state_topic)
          ->default_value("")
          ->value_name("TOPIC"),
      "ROS topic on which operation states will be published. "
      "Leave empty to disable."
    )
    (
      "odometry-topic",
      po::value<std::string>(&config.subscribe_topic)
          ->default_value("")
          ->value_name("TOPIC"),
      "ROS topic with odometry data, that the bridge should listen to."
      "Leave empty to disable."
    )
    ;  // NOLINT
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);
  if (vm.count("help")) {
    std::cout << desc << '\n';
    return std::nullopt;
  }
  return config;
}
