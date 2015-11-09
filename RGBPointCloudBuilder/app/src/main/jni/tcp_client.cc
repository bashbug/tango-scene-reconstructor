//
// Created by anastasia on 05.11.15.
//

#include "rgb-depth-sync/tcp_client.h"
#include <tango-gl/util.h>
#include <vector>

namespace rgb_depth_sync {

  tcp_client::~tcp_client(){
    close(sockfd);
  }

  tcp_client::tcp_client(std::string addr, int port){

    portno = port;

    // SOCKET
    // AF_INET address domain of the socket
    // SOCK_STREAM stream socket in which characters are read in a continuous stream as if from a
    // file or pipe 0 is the default protocol TCP

    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0) {
      LOGE("ERROR opening socket");
    }

    // argv[1] is either a hostname,
    // or an IPv4 address in standard dot notation (as for inet_addr(3)),
    // or an IPv6 address in colon (and possibly dot) notation.
    server = gethostbyname(addr.c_str());

    if (server == NULL) {
      LOGE("ERROR, no such host");
      exit(0);
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;

    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);

    serv_addr.sin_port = htons(portno);

    // CONNECT TO SERVER
    // sockfd file descriptor
    // sockaddr host address
    if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
      LOGE("ERROR connecting");
    } else {
      LOGE("connected");
    }
  }

  bool tcp_client::senddata(void *buf, int buflen) {
    unsigned char *pbuf = (unsigned char *) buf; // converts to char array

    while (buflen > 0) {
      int num = send(sockfd, pbuf, buflen, 0);
      if (num <= 0) {
        LOGE("data send failed");
        return false;
      }
      pbuf += num;
      buflen -= num;
    }
    return true;
  }

  bool tcp_client::sendlong(long long value) {
    value = htonl(value);  //  converts the unsigned integer hostlong from host byte order to network byte order.
    return senddata(&value, sizeof(value));
  }

  bool tcp_client::sendfilename(char* filename) {
    long filenamesize = strlen(filename);

    if (filenamesize > 0) {
      int num = send(sockfd, filename, filenamesize, 0);
      if (num <= 0) {
        LOGE("data send failed");
        return false;
      }
    }
    return true;
  }

  bool tcp_client::sendpcddata(char* header, std::vector<float> pcd) {
    long datasize = (long)strlen(header) + (long)(pcd.size() * sizeof(float));

    if (datasize > 0) {
      sendheader(header);
      sendpcd(pcd);
    }
    return true;
  }

  bool tcp_client::sendheader(char* header) {

    int num = senddata(header, strlen(header));
    if (num <= 0) {
    LOGE("send header failed");
    return false;
    }
    return true;
  }

  bool tcp_client::sendpcd(std::vector<float> pcd) {

    if (!senddata(pcd.data(), (pcd.size() * sizeof(float)))) {
      LOGE("pcd data send failed");
      return false;
    }
    close(sockfd);
    return true;
  }
}