#include "LampController.h"

LampController::LampController(const string &ip, const int port)
    : UDPSocket(ip, port), redCurrState(0), blueCurrState(0), greenCurrState(0),
      orangeCurrState(0) {}

string LampController::createMsg(const bool redState, const bool blueState,
                                 const bool greenState,
                                 const bool orangeState) {
  string msg = "l:" + std::to_string(redState) + ":" +
               std::to_string(orangeState) + ":" + std::to_string(greenState) +
               ":" + std::to_string(blueState) + "#";

  return msg;
}

bool LampController::setRed(const bool state) {
  string msg = createMsg(state, blueCurrState, greenCurrState, orangeCurrState);
  if (sendMessage(msg)) {
    cout << "Lamp setRed error while sending\n" << endl;
    return false;
  }
  redCurrState = state;

  return true;
}

bool LampController::setGreen(const bool state) {
  string msg = createMsg(redCurrState, blueCurrState, state, orangeCurrState);
  if (sendMessage(msg)) {
    cout << "Lamp setGreen error while sending\n" << endl;
    return false;
  }
  greenCurrState = state;

  return true;
}

bool LampController::setBlue(const bool state) {

  string msg = createMsg(redCurrState, state, greenCurrState, orangeCurrState);
  if (sendMessage(msg)) {
    cout << "Lamp setBlue error while sending\n" << endl;
    return false;
  }
  blueCurrState = state;

  return true;
}

bool LampController::setOrange(const bool state) {

  string msg = createMsg(redCurrState, blueCurrState, greenCurrState, state);
  if (sendMessage(msg)) {
    cout << "Lamp setOrange error while sending\n" << endl;
    return false;
  }
  orangeCurrState = state;

  return true;
}

bool LampController::set(const bool redState, const bool orangeState,
                         const bool greenState, const bool blueState) {

  string msg = createMsg(redState, blueState, greenState, orangeState);
  if (sendMessage(msg)) {
    cout << "Lamp Set error while sending\n" << endl;
    return false;
  }
  redCurrState = redState;
  orangeCurrState = orangeState;
  blueCurrState = blueState;
  greenCurrState = greenState;

  return true;
}