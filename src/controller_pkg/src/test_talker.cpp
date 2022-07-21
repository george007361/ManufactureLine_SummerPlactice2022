#include <chrono>     // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory>     // Dynamic memory management
#include <string>     // String functions

#include "controller_interface_pkg/msg/lamp_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
class TestTalker : public rclcpp::Node {

public:
  TestTalker() : Node("test_talker_node"), count_(0) {
    publisher_ = this->create_publisher<controller_interface_pkg::msg::LampMsg>(
        "lamp_topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&TestTalker::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = controller_interface_pkg::msg::LampMsg();
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<controller_interface_pkg::msg::LampMsg>::SharedPtr
      publisher_;

  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TestTalker>());

  rclcpp::shutdown();
  return 0;
}