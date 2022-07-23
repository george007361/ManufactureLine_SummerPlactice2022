
#include <memory>
#include <string>

#include "LowLevelController.h"
#include "controller_interface_pkg/msg/angle_manipulator_msg.hpp"
#include "controller_interface_pkg/msg/lamp_msg.hpp"
#include "controller_interface_pkg/msg/palletizer_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//---------------------------------------
const int port = 8888;
const string my_ip = "192.168.1.50";
const string lamp_ip = "192.168.1.33";
const string lampPalletizer_ip = "192.168.1.31";
const string lampAngleManipulator_ip = "192.168.1.32";
const string palletizer_ip = "192.168.1.21";
const string angleManipulator_ip = "192.168.1.22";
//---------------------------------------

using namespace controller_interface_pkg::msg;
using std::placeholders::_1;

class Controller : public rclcpp::Node {
public:
  Controller()
      : Node("controller_node"),
        lamp(this->create_subscription<LampMsg>(
            "lamp_topic", 10, std::bind(&Controller::lampCallback, this, _1))),
        lampPalletizer(this->create_subscription<LampMsg>(
            "lamp_palletizer_topic", 10,
            std::bind(&Controller::lampPalletizerCallback, this, _1))),
        lampAngleManipulator(this->create_subscription<LampMsg>(
            "lamp_angle_manipulator_topic", 10,
            std::bind(&Controller::lampAngleManipulatorCallback, this, _1))),
        palletizer(this->create_subscription<PalletizerMsg>(
            "palletizer_topic", 10,
            std::bind(&Controller::palletizerCallback, this, _1))),
        angleManipulator(this->create_subscription<AngleManipulatorMsg>(
            "angle_manipulator_topic", 10,
            std::bind(&Controller::angleManipulatorCallback, this, _1))),
        lampController(std::make_shared<LampController>(lamp_ip, port)),
        lampPalletizerController(
            std::make_shared<LampController>(lampPalletizer_ip, port)),
        lampAngleManipulatorController(
            std::make_shared<LampController>(lampAngleManipulator_ip, port)),
        palletizerController(
            std::make_shared<PalletizerController>(palletizer_ip, port)),
        angleManipulatorController(std::make_shared<AngleManipulatorController>(
            angleManipulator_ip, port))

  {
    RCLCPP_INFO(this->get_logger(), "Conntoller ctor");
    initControllers();
    setAngleManipulatorZone();
    setPalletizerZone();
  }

private:
  void initControllers() {
    RCLCPP_INFO(this->get_logger(), "Conntoller init: %d%d%d%d%d",
                lampAngleManipulatorController.get()->init(),
                lampPalletizerController.get()->init(),
                lampController.get()->init(),

                palletizerController.get()->init(),
                angleManipulatorController.get()->init());
  }

  void setPalletizerZone() {

    palletizerController.get()->setZone(
        PalletizerController::Position(0, -300, 160),
        PalletizerController::Position(300, 300, 290));
  }

  void setAngleManipulatorZone() {
    angleManipulatorController.get()->setZone(
        AngleManipulatorController::Position(0, -300, 0, 0),
        AngleManipulatorController::Position(300, 300, 150, 90));
  }

private:
  void lampCallback(const LampMsg::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "lampCallback: get msg ROGB %d%d%d%d",
                msg->red, msg->orange, msg->green, msg->blue);
    lampController.get()->set((bool)msg->red, (bool)msg->orange,
                              (bool)msg->green, (bool)msg->blue);
  }

  void lampPalletizerCallback(const LampMsg::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(),
                "lampPalletizerCallback: get msg ROGB %d%d%d%d", msg->red,
                msg->orange, msg->green, msg->blue);
    lampPalletizerController.get()->set((bool)msg->red, (bool)msg->orange,
                                        (bool)msg->green, (bool)msg->blue);
  }

  void lampAngleManipulatorCallback(const LampMsg::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(),
                "lampAngleManipulatorCallback: get msg ROGB %d%d%d%d", msg->red,
                msg->orange, msg->green, msg->blue);
    lampAngleManipulatorController.get()->set(
        (bool)msg->red, (bool)msg->orange, (bool)msg->green, (bool)msg->blue);
  }

private:
  void palletizerCallback(const PalletizerMsg::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(),
                "palletizerCallback: get msg X:Y:Z:S %d %d %d %d", msg->x,
                msg->y, msg->z, msg->pomp);
    palletizerController.get()->changeState((int)msg->x, (int)msg->y,
                                            (int)msg->z, (bool)msg->pomp);
  }

  void
  angleManipulatorCallback(const AngleManipulatorMsg::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(),
                "angleManipulatorCallback: get msg X Y Z A S %d %d %d %d %d",
                msg->x, msg->y, msg->z, msg->angle, msg->pomp);
    angleManipulatorController.get()->changeState((int)msg->x, (int)msg->y,
                                                  (int)msg->z, (int)msg->angle,
                                                  (bool)msg->pomp);
  }

private:
  rclcpp::Subscription<LampMsg>::SharedPtr lamp;
  rclcpp::Subscription<LampMsg>::SharedPtr lampPalletizer;
  rclcpp::Subscription<LampMsg>::SharedPtr lampAngleManipulator;
  rclcpp::Subscription<PalletizerMsg>::SharedPtr palletizer;
  rclcpp::Subscription<AngleManipulatorMsg>::SharedPtr angleManipulator;

private:
  std::shared_ptr<LampController> lampController;
  std::shared_ptr<LampController> lampPalletizerController;
  std::shared_ptr<LampController> lampAngleManipulatorController;
  std::shared_ptr<PalletizerController> palletizerController;
  std::shared_ptr<AngleManipulatorController> angleManipulatorController;
};

int main(int argc, char *argv[]) {
  UDPSocket::setMyIp(my_ip);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();

  return 0;
}