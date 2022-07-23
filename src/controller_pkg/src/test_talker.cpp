#include <chrono>     // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory>     // Dynamic memory management
#include <string>     // String functions

#include "LowLevelController.h"
#include "controller_interface_pkg/msg/angle_manipulator_msg.hpp"
#include "controller_interface_pkg/msg/lamp_msg.hpp"
#include "controller_interface_pkg/msg/palletizer_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TestTalker : public rclcpp::Node {

public:
  TestTalker() : Node("test_talker_node"), count_(0) {
    timer_slow = this->create_wall_timer(
        4000ms, std::bind(&TestTalker::timerslow_callback, this));
    timer_fast = this->create_wall_timer(
        250ms, std::bind(&TestTalker::timerfast_callback, this));

    //---------------------------------------
    publisher_lamp1 =
        this->create_publisher<controller_interface_pkg::msg::LampMsg>(
            "lamp_topic", 10);
    publisher_lamp2 =
        this->create_publisher<controller_interface_pkg::msg::LampMsg>(
            "lamp_palletizer_topic", 10);
    publisher_lamp3 =
        this->create_publisher<controller_interface_pkg::msg::LampMsg>(
            "lamp_angle_manipulator_topic", 10);
    publisher_p =
        this->create_publisher<controller_interface_pkg::msg::PalletizerMsg>(
            "palletizer_topic", 10);
    publisher_a = this->create_publisher<
        controller_interface_pkg::msg::AngleManipulatorMsg>(
        "angle_manipulator_topic", 10);

    //---------------------------------------
    for (int i = 0; i < 4; ++i)
      st[i] = 0;
    pt = 0;

    aa[0] = AngleManipulatorController::Position(110, 10, 75, 45);
    aa[1] = AngleManipulatorController::Position(110, 200, 75, 45);
    aa[2] = AngleManipulatorController::Position(210, 200, 75, 45);
    aa[3] = AngleManipulatorController::Position(210, 10, 75, 45);
    apt = 0;

    bb[0] = PalletizerController::Position(110, 10, 200);
    bb[1] = PalletizerController::Position(110, 110, 200);
    bb[2] = PalletizerController::Position(210, 110, 200);
    bb[3] = PalletizerController::Position(210, 10, 200);
    ppt = 0;
  }

private:
  void test_lamps() {
    st[pt] = !st[pt];
    pt++;
    if (pt > 3)
      pt = 0;

    auto message = controller_interface_pkg::msg::LampMsg();
    message.red = st[0];
    message.orange = st[1];
    message.green = st[2];
    message.blue = st[3];

    RCLCPP_INFO(this->get_logger(), " Lamp: %d%d%d%d", message.red,
                message.orange, message.green, message.blue);

    publisher_lamp1->publish(message);
    publisher_lamp2->publish(message);
    publisher_lamp3->publish(message);
  }

  void test_pallit() {
    auto message = controller_interface_pkg::msg::PalletizerMsg();

    message.x = bb[ppt].X;
    message.y = bb[ppt].Y;
    message.z = bb[ppt].Z;

    ppt++;
    if (apt > 3)
      ppt = 0;

    RCLCPP_INFO(this->get_logger(), "=== Palletizer: X:%d, Y:%d, Z:%d",
                message.x, message.y, message.z);

    publisher_p->publish(message);
  }

  void test_angle() {
    auto message = controller_interface_pkg::msg::AngleManipulatorMsg();

    message.x = aa[apt].X;
    message.y = aa[apt].Y;
    message.z = aa[apt].Z;
    message.angle = aa[apt].A;
    apt++;
    if (apt > 3)
      apt = 0;

    RCLCPP_INFO(this->get_logger(),
                "+++ AngleManipulator: X:%d, Y:%d, Z:%d, A:%d", message.x,
                message.y, message.z, message.angle);
    publisher_a->publish(message);
  }

private:
  void timerslow_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer slow callback");
    test_pallit();
    test_angle();
  }

  void timerfast_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer fast callback");
    test_lamps();
  }

private:
  bool st[4];
  int pt;
  AngleManipulatorController::Position aa[4];
  int apt;
  PalletizerController::Position bb[4];
  int ppt;

private:
  rclcpp::TimerBase::SharedPtr timer_slow;
  rclcpp::TimerBase::SharedPtr timer_fast;

private:
  rclcpp::Publisher<controller_interface_pkg::msg::LampMsg>::SharedPtr
      publisher_lamp1;
  rclcpp::Publisher<controller_interface_pkg::msg::LampMsg>::SharedPtr
      publisher_lamp2;
  rclcpp::Publisher<controller_interface_pkg::msg::LampMsg>::SharedPtr
      publisher_lamp3;
  rclcpp::Publisher<controller_interface_pkg::msg::PalletizerMsg>::SharedPtr
      publisher_p;
  rclcpp::Publisher<controller_interface_pkg::msg::AngleManipulatorMsg>::
      SharedPtr publisher_a;

  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TestTalker>());

  rclcpp::shutdown();
  return 0;
}