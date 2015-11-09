//
// Created by anastasia on 05.11.15.
//

#ifndef RGB_DEPTH_SYNC_SAVE_TO_SOCKET_H
#define RGB_DEPTH_SYNC_SAVE_TO_SOCKET_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string>
#include <vector>

namespace rgb_depth_sync {
class tcp_client {
public:
  tcp_client(std::string addr, int port);
  ~tcp_client();
  bool sendfilename(char* filename);
  bool sendpcddata(char* header, std::vector<float> pcd);
  bool sendlong(long long value);
private:
  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  bool senddata(void *buf, int buflen);
  bool sendheader(char* header);
  bool sendpcd(std::vector<float> pcd);
  bool sendbinary(float* buf, int buflen);
  };
}

#endif //RGB_DEPTH_SYNC_SAVE_TO_SOCKET_H
