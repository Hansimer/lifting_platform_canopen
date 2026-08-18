// CSP Initialization Node for lifting_platform_canopen
// Flow:
// 1. Wait for joint_states to confirm motor communication
// 2. Call init service (CANopen bus init → Operation Enabled)
// 3. Switch to CSP mode

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;

class CspInitNode : public rclcpp::Node
{
public:
  CspInitNode()
  : rclcpp::Node("csp_init_node"),
    joint_state_received_(false),
    init_done_(false)
  {
    // Parameters
    this->declare_parameter<std::string>("controller_name", "cia402_device_1_controller");
    this->declare_parameter<int>("max_retries", 3);
    this->declare_parameter<std::string>("xr_input_topic", "/raybot/teleop/xr_input");
    this->declare_parameter<std::string>(
      "forward_commands_topic",
      "/lifting_platform/lifting_forward_position_controller/commands");
    this->declare_parameter<int>("lift_axis_index", 3);
    this->declare_parameter<double>("lift_increment", 0.01);
    this->declare_parameter<bool>("edge_trigger", true);
    this->declare_parameter<double>("min_position", -0.2);
    this->declare_parameter<double>("max_position", 0.8);

    controller_name_ = this->get_parameter("controller_name").as_string();
    max_retries_ = this->get_parameter("max_retries").as_int();
    xr_input_topic_ = this->get_parameter("xr_input_topic").as_string();
    forward_commands_topic_ = this->get_parameter("forward_commands_topic").as_string();
    lift_axis_index_ = this->get_parameter("lift_axis_index").as_int();
    lift_increment_ = this->get_parameter("lift_increment").as_double();
    edge_trigger_ = this->get_parameter("edge_trigger").as_bool();
    min_position_ = this->get_parameter("min_position").as_double();
    max_position_ = this->get_parameter("max_position").as_double();

    RCLCPP_INFO(this->get_logger(), "CSP Init Node started");
    RCLCPP_INFO(this->get_logger(), "  Controller: %s", controller_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Max retries: %d", max_retries_);

    // Service names - use RELATIVE names so they are expanded with this node's
    // namespace (/lifting_platform/cia402_device_1_controller/init). Absolute
    // names would resolve to the root namespace and collide with other ROS2
    // systems on the same network / domain.
    std::string init_srv = controller_name_ + "/init";
    std::string csp_mode_srv = controller_name_ + "/position_mode";
    std::string recover_srv = controller_name_ + "/recover";

    // Create a separate callback group for service clients
    service_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Service clients
    init_client_ = this->create_client<Trigger>(init_srv, rmw_qos_profile_default, service_cbg_);
    csp_mode_client_ = this->create_client<Trigger>(csp_mode_srv, rmw_qos_profile_default, service_cbg_);
    recover_client_ = this->create_client<Trigger>(recover_srv, rmw_qos_profile_default, service_cbg_);

    // Subscribe to joint_states to confirm motor communication is alive
    // (absolute name -> /lifting_platform/joint_states, published by the
    //  joint_state_broadcaster running in the lifting_platform namespace)
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/lifting_platform/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Track the current updown position (feedback) for the lift command.
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          if (msg->name[i] == "updown") {
            current_updown_position_ = msg->position[i];
            updown_position_valid_ = true;
            // RCLCPP_INFO(this->get_logger(),"motor position: %f (raw)",current_updown_position_);

            break;
          }
        }
        if (!msg->position.empty() && !joint_state_received_) {
          joint_state_received_ = true;
          RCLCPP_INFO(this->get_logger(),
            "Joint state received, motor position: %d (raw)",
            static_cast<int32_t>(msg->position[0]));
          start_init_sequence();
        }
      });

    // Publish target positions to the (forward) position controller.
    // This is a TOPIC (not a service): forward_command_controller exposes
    // its command interface through the "commands" topic of type Float64MultiArray.
    forward_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      forward_commands_topic_, 10);

    // Subscribe to the XR teleop Joy input for incremental lifting.
    // axes[4] ==  1 -> command the platform to current + lift_increment_
    // axes[4] == -1 -> command the platform to current - lift_increment_
    // Use SensorDataQoS (BEST_EFFORT): the teleop node publishes Joy as a
    // low-latency stream, and a RELIABLE subscription would be incompatible
    // with its BEST_EFFORT publisher (no messages would be received).
    xr_input_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      xr_input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CspInitNode::xr_input_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "CSP Init Node ready. Waiting for joint_states...");
    RCLCPP_INFO(this->get_logger(),
      "XR lift control: topic=%s axis[%d] increment=%.3f edge_trigger=%s limits=[%.3f, %.3f]",
      xr_input_topic_.c_str(), lift_axis_index_, lift_increment_,
      edge_trigger_ ? "true" : "false", min_position_, max_position_);
  }

private:
  bool call_trigger(rclcpp::Client<Trigger>::SharedPtr client, const std::string & desc)
  {
    auto request = std::make_shared<Trigger::Request>();
    auto future = client->async_send_request(request);

    auto status = future.wait_for(10s);
    if (status == std::future_status::timeout) {
      RCLCPP_ERROR(this->get_logger(), "Service call '%s' timed out", desc.c_str());
      return false;
    }

    auto result = future.get();
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "Service '%s' succeeded", desc.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Service '%s' returned failure: %s",
        desc.c_str(), result->message.c_str());
    }
    return result->success;
  }

  void wait_for_service(rclcpp::Client<Trigger>::SharedPtr client, const std::string & name)
  {
    if (!client->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Service %s not available!", name.c_str());
    }
  }

  // Called for every /raybot/teleop/xr_input (sensor_msgs/Joy) message.
  // axes[lift_axis_index_] ==  1 -> publish current_updown_position_ + lift_increment_
  // axes[lift_axis_index_] == -1 -> publish current_updown_position_ - lift_increment_
  void xr_input_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // RCLCPP_WARN(this->get_logger(),"this is xr callback");
    if (msg->axes.size() <= static_cast<size_t>(lift_axis_index_)) {
      return;
    }

    const double axis_val = msg->axes[lift_axis_index_];
    // RCLCPP_WARN(this->get_logger(),
    //     "axs value in pico is %f",axis_val);

    // Command direction: +1 = up, -1 = down, 0 = neutral.
    int dir = 0;
    if (std::abs(axis_val - 1.0) < 1e-6) {
      dir = 1;
    } else if (std::abs(axis_val + 1.0) < 1e-6) {
      dir = -1;
    }

    if (edge_trigger_) {
      // Rising-edge only: send exactly one command per press / direction change.
      // Joy messages are usually streamed continuously while a button is held,
      // so without edge detection the target would keep advancing at topic rate.
      if (dir == prev_dir_) {
        return;
      }
      prev_dir_ = dir;
      if (dir == 0) {
        return;
      }
    } else if (dir == 0) {
      return;
    }

    if (!updown_position_valid_) {
      RCLCPP_WARN(this->get_logger(),
        "updown position not available yet, skip lift command");
      return;
    }

    const double target = current_updown_position_ + lift_increment_ * dir;

    // Soft limit protection: block the command if the target would go beyond
    // the configured [min_position_, max_position_] range.
    if (target < min_position_ || target > max_position_) {
      RCLCPP_WARN(this->get_logger(),
        "Soft limit reached: current=%.3f target=%.3f out of [%.3f, %.3f], command ignored",
        current_updown_position_, target, min_position_, max_position_);
      return;
    }

    auto cmd = std_msgs::msg::Float64MultiArray();
    cmd.data.push_back(target);

    forward_cmd_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(),
      "XR axis[%d]=%d -> command updown: current=%.3f target=%.3f (%+.3f)",
      lift_axis_index_, dir, current_updown_position_, target, lift_increment_ * dir);
  }

  void start_init_sequence()
  {
    if (init_done_) { return; }
    init_done_ = true;

    RCLCPP_INFO(this->get_logger(), "========== CSP Init Sequence Start ==========");

    // Step 1: Init CANopen bus
    RCLCPP_INFO(this->get_logger(), "[Step 1/2] Waiting for init service...");
    wait_for_service(init_client_, controller_name_ + "/init");
    RCLCPP_INFO(this->get_logger(), "[Step 1/2] Initializing CANopen motor...");
    if (!call_trigger(init_client_, "init")) {
      RCLCPP_WARN(this->get_logger(), "Init failed, trying recover...");
      wait_for_service(recover_client_, controller_name_ + "/recover");
      if (!call_trigger(recover_client_, "recover")) {
        RCLCPP_ERROR(this->get_logger(), "Recover failed. Aborting.");
        return;
      }
      rclcpp::sleep_for(1s);
      if (!call_trigger(init_client_, "init (retry)")) {
        RCLCPP_ERROR(this->get_logger(), "Init failed after recover. Aborting.");
        return;
      }
    }
    RCLCPP_INFO(this->get_logger(), "[Step 1/2] Init completed.");

    rclcpp::sleep_for(1s);

      //Step 2: Switch to CSP mode
    RCLCPP_INFO(this->get_logger(), "[Step 2/2] Waiting for cyclic_position_mode service...");
    wait_for_service(csp_mode_client_, controller_name_ + "/cyclic_position_mode");
    RCLCPP_INFO(this->get_logger(), "[Step 2/2] Switching to CSP mode...");
    bool csp_ok = false;
    for (int retry = 0; retry < max_retries_; retry++) {
      if (call_trigger(csp_mode_client_, "cyclic_position_mode")) {
        csp_ok = true;
        break;
      }
      RCLCPP_WARN(this->get_logger(), "CSP switch failed, retry %d/%d...", retry + 1, max_retries_);
      rclcpp::sleep_for(1s);
    }
    if (!csp_ok) {
      RCLCPP_ERROR(this->get_logger(), "CSP mode switch failed after %d attempts!", max_retries_);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[Step 2/2] CSP mode activated!");

    RCLCPP_INFO(this->get_logger(), "========== CSP Init Sequence Complete ==========");
    RCLCPP_INFO(this->get_logger(), "Motor in CSP mode. lifting_platform_controller ready for commands.");
  }

  // Member variables
  std::string controller_name_;
  int max_retries_;
  bool joint_state_received_;
  bool init_done_;

  // XR teleop / lift command
  std::string xr_input_topic_;
  std::string forward_commands_topic_;
  int lift_axis_index_;
  double lift_increment_;
  bool edge_trigger_;

  // Soft position limits (soft limit protection)
  double min_position_ = -0.2;
  double max_position_ = 0.8;

  // Current updown position from /joint_states feedback
  double current_updown_position_ = 0.0;
  bool updown_position_valid_ = false;
  int prev_dir_ = 0;  // last axes[lift_axis_index_] direction (-1 / 0 / 1) for edge detection

  // Service clients
  rclcpp::Client<Trigger>::SharedPtr init_client_;
  rclcpp::Client<Trigger>::SharedPtr csp_mode_client_;
  rclcpp::Client<Trigger>::SharedPtr recover_client_;

  // Subscriber / publisher
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr xr_input_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_cmd_pub_;

  // Callback group for service clients
  rclcpp::CallbackGroup::SharedPtr service_cbg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CspInitNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}