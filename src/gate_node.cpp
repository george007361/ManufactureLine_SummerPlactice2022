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

#include "LampController.hpp"
#include "ManipulatorController.hpp"

using namespace std;

int main() {
  LampController l1("192.168.1.32", 8888);
  AngleManipulatorController a1("192.168.1.22", 8888);
  while (1) {
    cout << "l1 on " << l1.Set(1, 1, 1, 1) << endl;
    cout << "a1 home " << a1.set(0, 0, 100, 0, 0) << endl;
    sleep(1);
    cout << "l1 off " << l1.Set(0, 0, 0, 0) << endl;
    cout << "a1 go " << a1.set(150, 150, 120, 45, 1) << endl;
    sleep(1);
  }

  return 0;
}