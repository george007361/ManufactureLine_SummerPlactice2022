#pragma once

#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

using namespace std;

class UDPSocket {
private:
  struct sockaddr_in addr;
  int sockfd;
  static const int DEF_PORT = 8888;
  static const string DEF_ADDR;

public:
  UDPSocket() = delete;
  UDPSocket(const UDPSocket &) = delete;
  UDPSocket(UDPSocket &&) = delete;

  UDPSocket(const string &ip, const int port);
  ~UDPSocket() { close(sockfd); }
  int sendMessage(const string &msg);

private:
  bool validateIpAddress(const string &ipAddress);
};

const string UDPSocket::DEF_ADDR = "192.168.1.10";

UDPSocket::UDPSocket(const string &ip, const int port) : addr(), sockfd(0) {
  int _port = port; //!!
  string _ip = ip;

  if (port <= 0) {
    fprintf(stderr, "Invalid port. Using default: %d", DEF_PORT);
    _port = DEF_PORT;
  }
  if (!validateIpAddress(ip)) {
    fprintf(stderr, "Invalid addres. Using default: %s", DEF_ADDR.c_str());
    _ip = DEF_ADDR;
  }
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (0 > sockfd) {
    perror("Socket err\n");
    return;
  }
  addr.sin_family = AF_INET;
  addr.sin_port = htons(_port);
  addr.sin_addr.s_addr = inet_addr(_ip.c_str());
}

int UDPSocket::sendMessage(const string &msg) {
  int err = sendto(sockfd, msg.c_str(), msg.length() + 1, 0,
                   (struct sockaddr *)&addr, sizeof(addr));
  if (err <= 0) {
    perror("Sometthing went wrong\n");
    perror(strerror(errno));
  }
  return errno;
}

bool UDPSocket::validateIpAddress(const string &ipAddress) {
  struct sockaddr_in sa;
  int result = inet_pton(AF_INET, ipAddress.c_str(), &(sa.sin_addr));
  return result != 0;
}