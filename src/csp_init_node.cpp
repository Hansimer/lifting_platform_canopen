// CSP Initialization Node for lifting_platform_canopen
// Flow:
// 1. Wait for joint_states to get current motor position
// 2. Call init service (CANopen bus init → Operation Enabled)
// 3. Start publishing current position to forward_position_controller BEFORE CSP switch
// 4. Switch to CSP mode (cyclic_position_mode)
// 5. Continue publishing current position to stabilize

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;

class CspInitNode : public rclcpp::Node
{
public:
  CspInitNode()
  : rclcpp::Node("csp_init_node"),
    current_position_(0.0),
    position_received_(false),
    init_done_(false),
    mode_switched_(false),
    stabilization_active_(false),
    stabilization_count_(0)
  {
    // Parameters
    this->declare_parameter<std::string>("controller_name", "cia402_device_1_controller");
    this->declare_parameter<int>("stabilization_count", 500);  // 500 * 10ms = 5 seconds
    this->declare_parameter<int>("max_retries", 3);

    controller_name_ = this->get_parameter("controller_name").as_string();
    stabilization_count_ = this->get_parameter("stabilization_count").as_int();
    max_retries_ = this->get_parameter("max_retries").as_int();

    RCLCPP_INFO(this->get_logger(), "CSP Init Node started");
    RCLCPP_INFO(this->get_logger(), "  Controller: %s", controller_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Stabilization count: %d (x10ms)", stabilization_count_);
    RCLCPP_INFO(this->get_logger(), "  Max retries: %d", max_retries_);

    // Service names
    std::string init_srv = "/" + controller_name_ + "/init";
    std::string csp_mode_srv = "/" + controller_name_ + "/cyclic_position_mode";
    std::string recover_srv = "/" + controller_name_ + "/recover";

    // Create a separate callback group for service clients
    service_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Service clients
    init_client_ = this->create_client<Trigger>(init_srv, rmw_qos_profile_default, service_cbg_);
    csp_mode_client_ = this->create_client<Trigger>(csp_mode_srv, rmw_qos_profile_default, service_cbg_);
    recover_client_ = this->create_client<Trigger>(recover_srv, rmw_qos_profile_default, service_cbg_);

    // Publisher for forward_position_controller commands
    position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 10);

    // Subscribe to joint_states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->position.empty()) {
          current_position_ = msg->position[0];
          if (!position_received_) {
            position_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Current position received: %.6f", current_position_);
            start_init_sequence();
          }
          // Continuously publish position while stabilization is active
          if (stabilization_active_) {
            publish_position(current_position_);
          }
        }
      });

    RCLCPP_INFO(this->get_logger(), "CSP Init Node ready. Waiting for joint_states...");
  }

private:
  void publish_position(double position)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.push_back(position);
    position_pub_->publish(msg);
  }

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
    if (!client->wait_for_service(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Service %s not available!", name.c_str());
    }
  }

  void start_init_sequence()
  {
    if (init_done_) { return; }
    init_done_ = true;

    RCLCPP_INFO(this->get_logger(), "========== CSP Init Sequence Start ==========");

    // Step 1: Init (CANopen bus init)
    RCLCPP_INFO(this->get_logger(), "[Step 1/4] Waiting for init service...");
    wait_for_service(init_client_, "/" + controller_name_ + "/init");
    RCLCPP_INFO(this->get_logger(), "[Step 1/4] Initializing motor...");
    if (!call_trigger(init_client_, "init")) {
      RCLCPP_WARN(this->get_logger(), "Init failed, trying recover...");
      wait_for_service(recover_client_, "/" + controller_name_ + "/recover");
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
    RCLCPP_INFO(this->get_logger(), "[Step 1/4] Init completed.");

    rclcpp::sleep_for(500ms);

    // Step 2: Start publishing current position BEFORE CSP mode switch
    // This ensures the forward_position_controller has a valid target when CSP activates
    RCLCPP_INFO(this->get_logger(),
      "[Step 2/4] Pre-publishing current position: %.6f", current_position_);
    stabilization_active_ = true;
    stabilization_count_current_ = 0;

    // Publish several position commands before switching to CSP
    for (int i = 0; i < 50; i++) {
      publish_position(current_position_);
      rclcpp::sleep_for(10ms);
    }
    RCLCPP_INFO(this->get_logger(), "[Step 2/4] Pre-publish done (50 commands).");

    // Step 3: Switch to CSP mode
    RCLCPP_INFO(this->get_logger(), "[Step 3/4] Waiting for cyclic_position_mode service...");
    wait_for_service(csp_mode_client_, "/" + controller_name_ + "/cyclic_position_mode");
    switch_to_csp_mode(0);
  }

  void switch_to_csp_mode(int retry_count)
  {
    RCLCPP_INFO(this->get_logger(),
      "[Step 3/4] Switching to CSP mode (attempt %d/%d)...",
      retry_count + 1, max_retries_);

    if (!call_trigger(csp_mode_client_, "cyclic_position_mode")) {
      if (retry_count < max_retries_ - 1) {
        RCLCPP_WARN(this->get_logger(), "CSP mode switch failed, retrying in 1s...");
        rclcpp::sleep_for(1s);
        switch_to_csp_mode(retry_count + 1);
      } else {
        RCLCPP_ERROR(this->get_logger(), "CSP mode switch failed after %d attempts!", max_retries_);
        stabilization_active_ = false;
      }
      return;
    }

    on_csp_mode_activated();
  }

  void on_csp_mode_activated()
  {
    mode_switched_ = true;
    RCLCPP_INFO(this->get_logger(), "CSP mode activated!");

    // Step 4: Continue publishing current position to stabilize
    RCLCPP_INFO(this->get_logger(),
      "[Step 4/4] Stabilizing: publishing position for %d cycles (10ms each)...",
      stabilization_count_);

    // Timer to track stabilization completion
    stabilization_count_current_ = 0;
    hold_timer_ = this->create_wall_timer(10ms,
      [this]() {
        stabilization_count_current_++;
        if (stabilization_count_current_ >= stabilization_count_) {
          stabilization_active_ = false;
          hold_timer_->cancel();
          RCLCPP_INFO(this->get_logger(),
            "Stabilization complete. Sent %d position commands.", stabilization_count_current_);
          RCLCPP_INFO(this->get_logger(), "========== CSP Init Sequence Complete ==========");
          RCLCPP_INFO(this->get_logger(), "Motor in CSP mode at position: %.6f", current_position_);
        }
      });
  }

  // Member variables
  std::string controller_name_;
  int max_retries_;
  double current_position_;
  bool position_received_;
  bool init_done_;
  bool mode_switched_;
  bool stabilization_active_;
  int stabilization_count_;
  int stabilization_count_current_;

  // Service clients
  rclcpp::Client<Trigger>::SharedPtr init_client_;
  rclcpp::Client<Trigger>::SharedPtr csp_mode_client_;
  rclcpp::Client<Trigger>::SharedPtr recover_client_;

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr hold_timer_;

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