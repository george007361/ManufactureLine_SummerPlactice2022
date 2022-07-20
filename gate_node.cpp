// #include "rclcpp/rclcpp.hpp"

// class GateNode : public rclcpp::Node
// {
// public:
//     MyNode() : Node("gate_node") {}
// private:
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<GateNode>();

//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include "LowLevelController/include/LowLevelController.h"

using namespace std;
void blink() {
  LampController l1("192.168.1.31", 8888);
  l1.init();

  LampController l2("192.168.1.32", 8888);
  l2.init();

  LampController l3("192.168.1.33", 8888);
  l3.init();
  while (1) {
    l1.set(1, 0, 0, 0);
    l2.set(0, 1, 0, 0);
    l3.set(0, 0, 1, 0);
    sleep(1);
    l1.set(0, 1, 0, 0);
    l2.set(0, 0, 1, 0);
    l3.set(0, 0, 0, 1);
    sleep(1);
    l1.set(0, 0, 1, 0);
    l2.set(0, 0, 0, 1);
    l3.set(1, 0, 0, 0);
    sleep(1);
    l1.set(0, 0, 0, 1);
    l2.set(1, 0, 0, 0);
    l3.set(0, 1, 0, 0);
    sleep(1);
  }
}

void pallet() {

  LampController l1("192.168.1.31", 8888);
  l1.init();

  PalletizerController p1("192.168.1.21", 8888);
  p1.init();
  p1.setZone(PalletizerController::Position(0, -300, 160),
             PalletizerController::Position(300, 300, 290));

  PalletizerController::Position a(20, -100, 170);
  PalletizerController::Position b(270, 270, 280);

  while (1) {
    p1.moveTo(a);
    l1.set(1, 0, 0, 0);
    sleep(6);

    p1.moveTo(b);
    l1.set(0, 0, 0, 1);
    sleep(6);
  }
}

void angle() {

  LampController l1("192.168.1.32", 8888);
  l1.init();

  AngleManipulatorController a1("192.168.1.22", 8888);
  a1.init();
  a1.setZone(AngleManipulatorController::Position(0, -300, 0, 0),
             AngleManipulatorController::Position(300, 300, 150, 90));

  AngleManipulatorController::Position a(100, -200, 30, 20);
  AngleManipulatorController::Position b(175, 0, 100, 50);
  AngleManipulatorController::Position c(250, 200, 135, 70);

  while (1) {
    a1.moveTo(a);
    l1.set(1, 0, 0, 0);
    sleep(6);

    a1.moveTo(b);
    l1.set(1, 1, 0, 0);
    sleep(6);

    a1.moveTo(c);
    l1.set(1, 1, 1, 0);
    sleep(6);
  }
}

int main() {

  blink();
  // angle();
  // pallet();

  return 0;
}