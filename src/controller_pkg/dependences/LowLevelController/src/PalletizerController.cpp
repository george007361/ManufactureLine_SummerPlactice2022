#include "PalletizerController.h"

PalletizerController::PalletizerController(const string &ip, const int port)
    : UDPSocket(ip, port), prevPos(), prevState(0) {}

string PalletizerController::createMsg(const Position &pos, const int state) {

  string msg = "p:" + std::to_string(pos.X) + ":" + std::to_string(pos.Y) +
               ":" + std::to_string(pos.Z) + ":" + std::to_string(state) + "#";

  return msg;
}

bool PalletizerController::moveTo(const Position &pos) {
  if (!zone.setted()) {
    fprintf(stderr, "%s", "Set zone at first!\n");
    return false;
  }

  if (!zone.check(pos)) {
    fprintf(stderr, "%s", "Out Of Zone\n");
    return false;
  }

  string msg = createMsg(pos, prevState);
  if (sendMessage(msg)) {
    fprintf(stderr, "%s", "Palletizer moveTo error while sending\n");
    return false;
  }
  prevPos = pos;

  return true;
}

bool PalletizerController::moveTo(const int X, const int Y, const int Z) {
  Position pos(X, Y, Z);
  return moveTo(pos);
}

bool PalletizerController::setActive(const bool state) {
  string msg = createMsg(prevPos, state);
  if (sendMessage(msg)) {
    fprintf(stderr, "%s", "Palletizer setActive error while sending\n");
    return false;
  }
  prevState = state;

  return true;
}

bool PalletizerController::changeState(const int X, const int Y, const int Z,
                                       const bool state) {
  if (!zone.setted()) {
    fprintf(stderr, "%s", "Set zone at first!\n");
    return false;
  }

  Position newPos(X, Y, Z);

  if (!zone.check(newPos)) {
    fprintf(stderr, "%s", "Out Of Zone\n");
    return false;
  }

  string msg = createMsg(newPos, state);

  cout << msg << endl;
  if (sendMessage(msg)) {
    fprintf(stderr, "%s", "Palletizer set error while sending\n");
    return false;
  }
  prevPos = newPos;
  prevState = state;

  return true;
}