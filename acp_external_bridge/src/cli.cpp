#include "sev/acp_external_bridge/cli.h"
#include <boost/program_options.hpp>
#include <iostream>
#include <sev/acp_udp/port.h>
#include "sev/acp_ros_conversions/conversions.h"

namespace po = boost::program_options;

std::optional<Config> parse_args(int ac, char** av) {  // NOLINT
  Config config;
  po::options_description desc("Alphasense Position UDP bridge");
  desc.add_options()
      // clang-format off
    (
     "help",
     "Show help message."
    )
    (
      "pose-port",
      po::value<uint16_t>(&config.legacy_receive_port)
          ->value_name("PORT"),
      "Port on which the bridge listens (not only for pose messages). "
      "DEPRECATED: Use --local-port instead."
    )
    (
      "local-port",
      po::value<uint16_t>(&config.receive_port)
          ->default_value(sev::acp::udp::default_port)
          ->value_name("PORT"),
      "Port on which the bridge listens (e.g. for pose messages)."
    )
    (
      "alphasense-port",
      po::value<uint16_t>(&config.ae_port)
          ->default_value(sev::acp::udp::default_port)
          ->value_name("PORT"),
      "Port on which the Alphasense hardware is listening for Odometry data."
    )
    (
      "ros-frame-id",
      po::value<std::string>(&config.frame_id)
          ->default_value("")
          ->value_name("FRAME_ID"),
      "Frame ID with which poses get published."
    )
    (
      "alphasense-ip",
      po::value<std::string>(&config.ae_address)
          ->value_name("IP"),
      "IP Address of Alphasense hardware."
    )
    (
      "pose-topic",
      po::value<std::string>(&config.publish_topic)
          ->value_name("PREFIX"),
      "ROS topic prefix on which poses will be published. "
      "DEPRECATED: Configure --ros-pose-topic and --positioning-update-topic "
      "separately instead. E.g. --ros-pose-topic PREFIX/ros_pose "
      "--positioning-update-topic PREFIX/positioning_update will have the "
      "same effect."
    )
    (
      "ros-pose-topic",
      po::value<std::string>(&config.ros_pose_topic)
          ->default_value("/alphasense_position/T_G_O_propagated")
          ->value_name("TOPIC"),
      "ROS topic on which geometry_msgs poses will be published."
    )
    (
      "positioning-update-topic",
      po::value<std::string>(&config.positioning_update_topic)
          ->default_value("/alphasense_position/T_G_O_propagated_update")
          ->value_name("TOPIC"),
      "ROS topic on which positioning updates will be published."
    )
    (
      "notification-topic",
      po::value<std::string>(&config.notification_topic)
          ->default_value("/alphasense_position/notifications")
          ->value_name("TOPIC"),
      "ROS topic on which notifications will be published."
    )
    (
      "operation-state-topic",
      po::value<std::string>(&config.operation_state_topic)
          ->default_value("/alphasense_position/operation_state")
          ->value_name("TOPIC"),
      "ROS topic on which operation states will be published."
    )
    (
      "odometry-topic",
      po::value<std::string>(&config.subscribe_topic)
          ->default_value("/robot/velocity")
          ->value_name("TOPIC"),
      "ROS topic with odometry data, that the bridge should listen to."
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
  if (vm.count("pose-port")) {
    config.receive_port = config.legacy_receive_port;
    std::cout << "Deprecation warning: --pose-port will be replaced by "
                 "--local-port in a future release.\nUntil then, --pose-port"
                 "takes precedence over --local-port.\n";
  }
  if (vm.count("pose-topic")) {
    config.ros_pose_topic =
        config.publish_topic + "/" +
        sev::acp::udp_ros_bridge::suffix_lookup<geometry_msgs::PoseStamped>();
    config.positioning_update_topic = config.publish_topic + "/" +
                                      sev::acp::udp_ros_bridge::suffix_lookup<
                                          atlas_msgs::PositioningUpdate>();
    std::cout << "Deprecation warning: Use '--positioning-update-topic "
              << config.positioning_update_topic << " --ros-pose-topic "
              << config.ros_pose_topic
              << "' instead of --pose-topic."
                 "\nUsing --pose-topic overrides the settings of "
                 "--ros-pose-topic and --positioning-update-topic.\n";
  }
  return config;
}
