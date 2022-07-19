#pragma once

#include <string>

#include "UDPSocket.hpp"

using namespace std;

class ManipulatorController : public UDPSocket {
public:
  enum Type { ANGLE, PALLETIZER };

private:
  const int type;
  const string prefix;
  int pX, pY, pZ, pA, pS;

public:
  ManipulatorController() = delete;
  ManipulatorController(const ManipulatorController &) = delete;
  ManipulatorController(ManipulatorController &&) = delete;

  ManipulatorController(const string &prefix, const int t, const string &ip,
                        const int port);
  bool moveTo(const int X, const int Y, const int Z, const int angle);
  bool setActive(const bool state);
  bool set(const int X, const int Y, const int Z, const int angle,
           const bool state);

private:
  string createMsg(const int X, const int Y, const int Z, const int angle,
                   const int state);
};

class AngleManipulatorController : public ManipulatorController {
public:
  AngleManipulatorController(const string &ip, const int port)
      : ManipulatorController("g", ManipulatorController::Type::ANGLE, ip,
                              port) {}
};

class PalletizerController : public ManipulatorController {
public:
  PalletizerController(const string &ip, const int port)
      : ManipulatorController("p", ManipulatorController::Type::PALLETIZER, ip,
                              port) {}
};

ManipulatorController::ManipulatorController(const string &prefix, const int t,
                                             const string &ip, const int port)
    : UDPSocket(ip, port), type(t), pX(0), pY(0), pZ(0), pA(0), pS(0),
      prefix(prefix) {}

string ManipulatorController::createMsg(const int X, const int Y, const int Z,
                                        const int angle, const int state) {

  string msg = prefix + ":" + std::to_string(X) + ":" + std::to_string(Y) +
               ":" + std::to_string(Z) + ":" + std::to_string(angle) + ":" +
               std::to_string(state) + "#";

  return msg;
}

bool ManipulatorController::moveTo(const int X, const int Y, const int Z,
                                   const int angle) {
  // TODO check;
  string msg = createMsg(X, Y, Z, angle, pS);
  if (sendMessage(msg)) {
    perror("Manipulator moveTo error while sending\n");
    return false;
  }
  pX = X;
  pY = Y;
  pZ = Z;
  pA = angle;

  return true;
}

bool ManipulatorController::setActive(const bool state) {
  string msg = createMsg(pX, pY, pZ, pA, state);
  if (sendMessage(msg)) {
    perror("Manipulator setActive error while sending\n");
    return false;
  }
  pS = state;

  return true;
}

bool ManipulatorController::set(const int X, const int Y, const int Z,
                                const int angle, const bool state) {
  string msg = createMsg(X, Y, Z, angle, state);
  if (sendMessage(msg)) {
    perror("Manipulator set error while sending\n");
    return false;
  }
  pX = X;
  pY = Y;
  pZ = Z;
  pA = angle;
  pS = state;

  return true;
}