#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <string>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

typedef struct sockaddr *saddrp;
using namespace std;

int main() {
  string ip = "192.168.1.2";
  string buf;
  int port = 8888;
  bool flag = true;
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  if (0 > sockfd) {
    perror("Socket err ");
    return -1;
  }

  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = inet_addr(ip.c_str());

  while (flag) {
    system("clear");
    cout << "Type\n	 \"addr\" to change addr\n	 \"port\" to change "
            "port\n	 \"c\" to "
            "run cmd\n	 \"exit\" to exit\n\n";

    cin >> buf;
    system("clear");

    if (buf == "addr") {
      cout << "Type ip: ";
      cin >> ip;
      addr.sin_addr.s_addr = inet_addr(ip.c_str());
    } else if (buf == "port") {
      cout << "Type port: ";
      cin >> port;
      addr.sin_port = htons(port);
    } else if (buf == "c") {
      cout << "Type command: ";
      cin >> buf;
      int err =
          sendto(sockfd, buf.c_str(), buf.length() + 1, 0, (saddrp)&addr, sizeof(addr));
      cout << " -> return code: " << err << " Strerr: " << strerror(errno)
           << endl;
    } else if (buf == "exit") {
      flag = 0;
    } else {
      cout << "Wrong command! \n";
    }
  }

  cout << endl;
  close(sockfd);
  return 0;
}