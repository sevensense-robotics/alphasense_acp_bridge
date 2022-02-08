#include <algorithm>
#include <atomic>
#include <csignal>
#include <functional>
#include <iterator>
#include <optional>
#include <thread>
#include <type_traits>
#include <vector>

#include <cuckoo_time_translator/DeviceTimeTranslator.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"
#include "sev/acp_external_bridge/cli.h"
#include "sev/acp_ros_conversions/conversions.h"
#include "sev/acp_ros_conversions/cuckoo_wrapper.h"
#include "sev/acp_udp/port.h"
#include "sev/acp_udp/receiver.h"
#include "sev/acp_udp/sender.h"

std::atomic_bool keep_listening;

void sighand(int) {
  keep_listening = false;
  ros::requestShutdown();
}

// TODO(pseyfert): Deduplicate.
namespace sev::acp::udp_ros_bridge {
namespace detail {
template <typename... RosType>
void update_timestamps(
    std::tuple<RosType...>* ros_tuple, const ros::Time republish_time) {
  std::apply(
      [republish_time](auto&... ros_tuple_element) {
        auto update_timestamp = [republish_time](auto& x) {
          x.header.stamp = republish_time;
        };
        (..., update_timestamp(ros_tuple_element));
      },
      *ros_tuple);
}
template <typename... RosType>
auto get_time(const std::tuple<RosType...>& ros_message) {
  return std::get<0>(ros_message).header.stamp.toNSec();
}

template <typename RosType>
ros::Publisher instantiate_publisher(
    ros::NodeHandle* n, const std::string& topic) {
  return n->advertise<RosType>(topic, 10);
}
}  // namespace detail

class UdpReceiverWrapper {
  ros::Publisher notification_pub_;
  ros::Publisher operation_state_pub_;
  ros::Publisher ros_pub_;
  ros::Publisher positioning_update_pub_;

  sev::acp::udp::UdpReceiver receiver_;
  sev::acp::udp_ros_bridge::TimeTranslator time_translator_;

 public:
  UdpReceiverWrapper(ros::NodeHandle* n, const Config& config)
      : notification_pub_{detail::instantiate_publisher<
            state_machine_msgs::Status>(n, config.notification_topic)},
        operation_state_pub_{
            detail::instantiate_publisher<state_machine_msgs::State>(
                n, config.operation_state_topic)},
        ros_pub_{detail::instantiate_publisher<geometry_msgs::PoseStamped>(
            n, config.ros_pose_topic)},
        positioning_update_pub_{
            detail::instantiate_publisher<atlas_msgs::PositioningUpdate>(
                n, config.positioning_update_topic)},
        receiver_{config.receive_port},
        time_translator_{n} {}

  void spin(std::atomic_bool* keep_listening) {
    while (ros::ok()) {
      try {
        const auto message = receiver_.receive_Messages<
            false, sev::acp::Pose, sev::acp::PoseFloat,
            sev::acp::OperationState, sev::acp::Notifications>(keep_listening);

        const auto received_time = ros::Time::now();
        std::visit(
            [this, received_time](auto&& acp_object) {
              using AcpType = std::decay_t<decltype(acp_object)>;
              if constexpr (!std::is_same_v<AcpType, sev::acp::MessageType>) {
                // This condition is false when we receive a message type other
                // than those in the list of .receive_Messages<false, ...>.
                try {
                  auto ros_message_tuple = convertToRosStamped(acp_object);
                  const MessageTimes message_times{
                      detail::get_time(ros_message_tuple), received_time};

                  const auto republish_time =
                      this->time_translator_.translate<AcpType>(message_times);

                  if (republish_time) {
                    detail::update_timestamps(
                        &ros_message_tuple, *republish_time);
                    if constexpr (std::is_same_v<
                                      AcpType, sev::acp::Notifications>) {
                      notification_pub_.publish(
                          std::get<state_machine_msgs::Status>(
                              ros_message_tuple));
                    } else if constexpr (std::is_same_v<
                                             AcpType,
                                             sev::acp::OperationState>) {
                      operation_state_pub_.publish(
                          std::get<state_machine_msgs::State>(
                              ros_message_tuple));
                    } else {
                      std::apply(
                          [&ros_pub = this->ros_pub_,
                           &positioning_update_pub =
                               this->positioning_update_pub_](auto... msg) {
                            auto publish = [&ros_pub,
                                            &positioning_update_pub](auto x) {
                              if constexpr (std::is_same_v<
                                                std::decay_t<decltype(x)>,
                                                atlas_msgs::
                                                    PositioningUpdate>) {
                                positioning_update_pub.publish(x);
                              } else if constexpr (
                                  std::is_same_v<
                                      std::decay_t<decltype(x)>,
                                      geometry_msgs::PoseStamped>) {
                                ros_pub.publish(x);
                              }
                            };
                            (..., publish(msg));
                          },
                          ros_message_tuple);
                    }  // constexpr else
                  }    // if republish_time
                } catch (const ConversionError& e) {
                  std::cout
                      << "ERROR: Encountered conversion error:\n"
                      << e.what() << "\nduring reception of message type "
                      << sev::acp::message_type_lookup(
                             acp_object.message_type())
                      << "\twith seq: " << acp_object.header.seq
                      << "\tand timestamp: " << acp_object.header.timestamp;
                }
              }
            },
            message);
      } catch (const std::length_error& e) {
        if (!(*keep_listening)) {
          return;
        } else {
          std::rethrow_exception(std::current_exception());
        }
      }
    }
  }
};
}  // namespace sev::acp::udp_ros_bridge

int main(int argc, char** argv) {
  // NB: Call ros::init before loadDefaultConfigTree to remove ros-specific
  //     arguments.
  keep_listening = true;
  ros::init(argc, argv, "ap_udp_bridge");

  std::optional<Config> config = parse_args(argc, argv);
  if (!config) {
    return 1;
  }

  ros::NodeHandle n;

  std::signal(SIGINT, sighand);

  sev::acp::udp::UdpSender sender =
      sev::acp::udp::UdpSender{config->ae_address.c_str(), config->ae_port};
  ros::Subscriber subscriber = n.subscribe<geometry_msgs::TwistStamped>(
      config->subscribe_topic, 10,
      std::function{
          [&sender](
              const geometry_msgs::TwistStamped::ConstPtr& ros_message_ptr) {
            try {
              const auto to_be_sent =
                  std::get<sev::acp::WheelOdometryIntegrated>(
                      sev::acp::udp_ros_bridge::convertToAcp(*ros_message_ptr));
              sender.send_message(to_be_sent);
            } catch (const sev::acp::udp_ros_bridge::ConversionError& e) {
              std::cout << "ERROR: " << e.what();
            }
          }});
  std::thread send_thread = std::thread{[&]() { ros::spin(); }};

  // Launch receive thread.
  // We want the UdpReceiver running in a different thread than the signal
  // handler.
  std::thread receive_thread([&]() {
    sev::acp::udp_ros_bridge::UdpReceiverWrapper receiver_wrapper(&n, *config);

    receiver_wrapper.spin(&keep_listening);
  });
  receive_thread.join();

  send_thread.join();

  return 0;
}
