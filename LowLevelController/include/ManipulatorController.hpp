#pragma once

#include <string>

#include "UDPSocket.hpp"

using namespace std;

class ManipulatorController : public UDPSocket {
public:
  enum Type { ANGLE, PALLETIZER };
  struct Position {
    int X;
    int Y;
    int Z;
    int A;
    Position() : X(0), Y(0), Z(0), A(0) {}
    Position(const int X, const int Y, const int Z, const int A)
        : X(X), Y(Y), Z(Z), A(A) {}
    bool operator>(Position const &rhs) const {
      return X > rhs.X && Y > rhs.Y && Z > rhs.Z && A > rhs.A;
    }
    bool operator<(Position const &rhs) const {
      return X < rhs.X && Y < rhs.Y && Z < rhs.Z && A < rhs.A;
    }
  };
  struct Zone {
    Position min;
    Position max;
    bool minSetted;
    bool maxSetted;
    Zone() : min(), max(), minSetted(false), maxSetted(false) {}
    Zone(const Position &min, const Position &max)
        : min(min), max(max), minSetted(true), maxSetted(true) {}
    // void setMin(const int X, const int Y, const int Z, const int A) {
    //   min = Position(X, Y, Z, A);
    //   minSetted = true;
    // }
    // void setMax(const int X, const int Y, const int Z, const int A) {
    //   max = Position(X, Y, Z, A);
    //   maxSetted = true;
    // }
    bool check(const Position &pos) { return pos > min && pos < max; }
    bool setted() { return minSetted && maxSetted; }
  };

private:
  const int type;
  const string prefix;
  Position prevPos;
  int prevState;
  Zone zone;

public:
  ManipulatorController() = delete;
  ManipulatorController(const ManipulatorController &) = delete;
  ManipulatorController(ManipulatorController &&) = delete;

  ManipulatorController(const string &prefix, const int t, const string &ip,
                        const int port);
  bool moveTo(const int X, const int Y, const int Z, const int angle);
  bool setActive(const bool state);
  bool changeState(const int X, const int Y, const int Z, const int angle,
                   const bool state);
  void setZone(const Position &min, const Position &max) {
    zone = Zone(min, max);
  }

private:
  string createMsg(const Position &pos, const int state);
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
    : UDPSocket(ip, port), type(t), prefix(prefix), prevPos(), prevState(0) {}

string ManipulatorController::createMsg(const Position &pos, const int state) {

  string msg = prefix + ":" + std::to_string(pos.X) + ":" +
               std::to_string(pos.Y) + ":" + std::to_string(pos.Z) + ":" +
               std::to_string(pos.A) + ":" + std::to_string(state) + "#";

  return msg;
}

bool ManipulatorController::moveTo(const int X, const int Y, const int Z,
                                   const int angle) {
  if (!zone.setted()) {
    perror("Set zone at first!\n");
    return false;
  }

  Position newPos(X, Y, Z, angle);

  if (!zone.check(newPos)) {
    perror("Out Of Zone\n");
    return false;
  }

  string msg = createMsg(newPos, prevState);
  if (sendMessage(msg)) {
    perror("Manipulator moveTo error while sending\n");
    return false;
  }
  prevPos = newPos;

  return true;
}

bool ManipulatorController::setActive(const bool state) {
  string msg = createMsg(prevPos, state);
  if (sendMessage(msg)) {
    perror("Manipulator setActive error while sending\n");
    return false;
  }
  prevState = state;

  return true;
}

bool ManipulatorController::changeState(const int X, const int Y, const int Z,
                                        const int angle, const bool state) {
  if (!zone.setted()) {
    perror("Set zone at first!\n");
    return false;
  }

  Position newPos(X, Y, Z, angle);

  if (!zone.check(newPos)) {
    perror("Out Of Zone\n");
    return false;
  }

  string msg = createMsg(newPos, state);

  if (sendMessage(msg)) {
    perror("Manipulator set error while sending\n");
    return false;
  }
  prevPos = newPos;
  prevState = state;

  return true;
}