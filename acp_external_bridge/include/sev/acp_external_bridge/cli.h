#ifndef SEV_ACP_EXTERNAL_BRIDGE_CLI_H_
#define SEV_ACP_EXTERNAL_BRIDGE_CLI_H_
#include <optional>
#include <string>

struct Config {
  uint16_t legacy_receive_port;
  uint16_t receive_port;
  std::string publish_topic;
  std::string positioning_update_topic;
  std::string ros_pose_topic;
  std::string subscribe_topic;
  std::string notification_topic;
  std::string operation_state_topic;
  std::string ae_address;
  uint16_t ae_port;
  std::string frame_id;
};

std::optional<Config> parse_args(int ac, char** av);  // NOLINT
#endif  // SEV_ACP_EXTERNAL_BRIDGE_CLI_H_
