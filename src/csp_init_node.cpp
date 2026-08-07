// CSP Initialization Node for lifting_platform_canopen
// Flow:
// 1. Wait for joint_states to confirm motor communication
// 2. Call init service (CANopen bus init → Operation Enabled)
// 3. Switch to CSP mode

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
    joint_state_received_(false),
    init_done_(false)
  {
    // Parameters
    this->declare_parameter<std::string>("controller_name", "cia402_device_1_controller");
    this->declare_parameter<int>("max_retries", 3);

    controller_name_ = this->get_parameter("controller_name").as_string();
    max_retries_ = this->get_parameter("max_retries").as_int();

    RCLCPP_INFO(this->get_logger(), "CSP Init Node started");
    RCLCPP_INFO(this->get_logger(), "  Controller: %s", controller_name_.c_str());
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

    // Subscribe to joint_states to confirm motor communication is alive
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->position.empty() && !joint_state_received_) {
          joint_state_received_ = true;
          RCLCPP_INFO(this->get_logger(),
            "Joint state received, motor position: %d (raw)",
            static_cast<int32_t>(msg->position[0]));
          start_init_sequence();
        }
      });

    RCLCPP_INFO(this->get_logger(), "CSP Init Node ready. Waiting for joint_states...");
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

  void start_init_sequence()
  {
    if (init_done_) { return; }
    init_done_ = true;

    RCLCPP_INFO(this->get_logger(), "========== CSP Init Sequence Start ==========");

    // Step 1: Init CANopen bus
    RCLCPP_INFO(this->get_logger(), "[Step 1/2] Waiting for init service...");
    wait_for_service(init_client_, "/" + controller_name_ + "/init");
    RCLCPP_INFO(this->get_logger(), "[Step 1/2] Initializing CANopen motor...");
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
    RCLCPP_INFO(this->get_logger(), "[Step 1/2] Init completed.");

    rclcpp::sleep_for(500ms);

      //Step 2: Switch to CSP mode
    RCLCPP_INFO(this->get_logger(), "[Step 2/2] Waiting for cyclic_position_mode service...");
    wait_for_service(csp_mode_client_, "/" + controller_name_ + "/cyclic_position_mode");
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

  // Service clients
  rclcpp::Client<Trigger>::SharedPtr init_client_;
  rclcpp::Client<Trigger>::SharedPtr csp_mode_client_;
  rclcpp::Client<Trigger>::SharedPtr recover_client_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

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