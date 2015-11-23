/*
 * Establishes an TCP socket connection to a given sever socket.
 * Sends rgb point cloud data.
 */

#ifndef RGB_DEPTH_SYNC_TCP_CLIENT_H
#define RGB_DEPTH_SYNC_TCP_CLIENT_H

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
      int sockfd, portno;
      struct sockaddr_in serv_addr;
      struct hostent *server;
      bool senddata(void *buf, int buflen);
      bool sendheader(char* header);
      bool sendpcd(std::vector<float> pcd);
  };
}

#endif //RGB_DEPTH_SYNC_TCP_CLIENT_H
