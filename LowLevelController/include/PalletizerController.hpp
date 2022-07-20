#pragma once

#include <string>

#include "UDPSocket.hpp"

using namespace std;

class PalletizerController : public UDPSocket {
public:
  struct Position {
    int X;
    int Y;
    int Z;
    Position() : X(0), Y(0), Z(0) {}
    Position(const int X, const int Y, const int Z) : X(X), Y(Y), Z(Z) {}
    bool operator>(Position const &rhs) const {
      return X > rhs.X && Y > rhs.Y && Z > rhs.Z;
    }
    bool operator<(Position const &rhs) const {
      return X < rhs.X && Y < rhs.Y && Z < rhs.Z;
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
  PalletizerController() = delete;
  PalletizerController(const PalletizerController &) = delete;
  PalletizerController(PalletizerController &&) = delete;

  PalletizerController(const string &ip, const int port);
  bool moveTo(const int X, const int Y, const int Z);
  bool moveTo(const Position &pos);
  bool setActive(const bool state);
  bool changeState(const int X, const int Y, const int Z, const bool state);
  void setZone(const Position &min, const Position &max) {
    zone = Zone(min, max);
  }
  bool init() { return sendMessage("r"); }

private:
  string createMsg(const Position &pos, const int state);
};

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