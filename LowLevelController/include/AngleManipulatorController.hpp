#pragma once

#include <string>

#include "UDPSocket.hpp"

using namespace std;

class AngleManipulatorController : public UDPSocket {
public:
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
    bool check(const Position &pos) { return pos > min && pos < max; }
    bool setted() { return minSetted && maxSetted; }
  };

private:
  Position prevPos;
  int prevState;
  Zone zone;

public:
  AngleManipulatorController() = delete;
  AngleManipulatorController(const AngleManipulatorController &) = delete;
  AngleManipulatorController(AngleManipulatorController &&) = delete;

  AngleManipulatorController(const string &ip, const int port);
  bool moveTo(const int X, const int Y, const int Z, const int angle);
  bool moveTo(const Position &pos);
  bool setActive(const bool state);
  bool changeState(const int X, const int Y, const int Z, const int angle,
                   const bool state);
  void setZone(const Position &min, const Position &max) {
    zone = Zone(min, max);
  }
  bool init() { return sendMessage("r"); }

private:
  string createMsg(const Position &pos, const int state);
};

AngleManipulatorController::AngleManipulatorController(const string &ip,
                                                       const int port)
    : UDPSocket(ip, port), prevPos(), prevState(0) {}

string AngleManipulatorController::createMsg(const Position &pos,
                                             const int state) {

  string msg = "g:" + std::to_string(pos.X) + ":" + std::to_string(pos.Y) +
               ":" + std::to_string(pos.A) + ":" + std::to_string(pos.Z) + ":" +
               std::to_string(state) + "#";

  return msg;
}

bool AngleManipulatorController::moveTo(const int X, const int Y, const int Z,
                                        const int angle) {
  Position pos(X, Y, Z, angle);
  return moveTo(pos);
}

bool AngleManipulatorController::moveTo(const Position &newPos) {

  if (!zone.setted()) {
    fprintf(stderr, "%s", "Set zone at first!\n");
    return false;
  }

  if (!zone.check(newPos)) {
    fprintf(stderr, "%s", "Out Of Zone\n");
    return false;
  }

  string msg = createMsg(newPos, prevState);
  if (sendMessage(msg)) {
    fprintf(stderr, "%s", "Angle Manipulator  moveTo error while sending\n");
    return false;
  }
  prevPos = newPos;

  return true;
}

bool AngleManipulatorController::setActive(const bool state) {
  string msg = createMsg(prevPos, state);
  if (sendMessage(msg)) {
    fprintf(stderr, "%s", "Angle Manipulator  setActive error while sending\n");
    return false;
  }
  prevState = state;

  return true;
}

bool AngleManipulatorController::changeState(const int X, const int Y,
                                             const int Z, const int angle,
                                             const bool state) {
  if (!zone.setted()) {
    fprintf(stderr, "%s", "Set zone at first!\n");
    return false;
  }

  Position newPos(X, Y, Z, angle);

  if (!zone.check(newPos)) {
    fprintf(stderr, "%s", "Out Of Zone\n");
    return false;
  }

  string msg = createMsg(newPos, state);

  cout << msg << endl;
  if (sendMessage(msg)) {
    fprintf(stderr, "%s", "Angle Manipulator  set error while sending\n");
    return false;
  }
  prevPos = newPos;
  prevState = state;

  return true;
}