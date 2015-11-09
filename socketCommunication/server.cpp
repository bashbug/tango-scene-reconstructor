#include <stdio.h>
#include <string>
#include <cstring>
#include <fstream>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <arpa/inet.h>

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */


void * get_in_addr(struct sockaddr * sa)
{
	if(sa->sa_family == AF_INET)
	{
		return &(((struct sockaddr_in *)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6 *)sa)->sin6_addr);
}

void readlong(int socket, long long *value) {
	//printf("I want my fucking %i byte\n", sizeof(value));
  int n = recv(socket, value, sizeof(value), 0);
	//printf("Read %i byte for readlong\n", n);
  *value = ntohl(*value);
}

int main(int argc, char const *argv[]) {
  // Variables for writing a server.
  /*
  1. Getting the address data structure.
  2. Openning a new socket.
  3. Bind to the socket.
  4. Listen to the socket.
  5. Accept Connection.
  6. Receive Data.
  7. Close Connection.
  */
  int status;
  struct addrinfo hints, * res;
  int listner;

	if (argc < 2) {
			fprintf(stderr,"ERROR, no port provided\n");
			return 0;
	}

  // Before using hint you have to make sure that the data structure is empty
  std::memset(& hints, 0, sizeof hints);
  // Set the attribute for hint
  hints.ai_family = AF_UNSPEC; // We don't care V4 AF_INET or 6 AF_INET6
  hints.ai_socktype = SOCK_STREAM; // TCP Socket SOCK_DGRAM
  hints.ai_flags = AI_PASSIVE;

  // Fill the res data structure and make sure that the results make sense.
  status = getaddrinfo(NULL, argv[1] , &hints, &res);
  if(status != 0)
  {
    fprintf(stderr,"getaddrinfo error: %s\n",gai_strerror(status));
  } else {
		fprintf(stderr, "getaddrinfo for port: %s\n", argv[1]);
	}

  // Create Socket and check if error occured afterwards
  listner = socket(res->ai_family,res->ai_socktype, res->ai_protocol);
  if(listner < 0 )
  {
    fprintf(stderr,"socket error: %s\n",gai_strerror(status));
  }

  // Bind the socket to the address of my local machine and port number
  status = bind(listner, res->ai_addr, res->ai_addrlen);
  if(status < 0)
  {
    fprintf(stderr,"bind: %s\n",gai_strerror(status));
  }

  status = listen(listner, 10);
  if(status < 0)
  {
    fprintf(stderr,"listen: %s\n",gai_strerror(status));
  }

  // Free the res linked list after we are done with it
  freeaddrinfo(res);

  // We should wait now for a connection to accept
  int new_conn_fd;
  struct sockaddr_storage client_addr;
  socklen_t addr_size;
  char s[INET6_ADDRSTRLEN]; // an empty string

  // Calculate the size of the data structure
  addr_size = sizeof client_addr;

  printf(BOLDYELLOW "I am now accepting connections ...\n" RESET);

  int message_no = 0;

  while(1){
    // Accept a new connection and return back the socket desciptor
    new_conn_fd = accept(listner, (struct sockaddr *) & client_addr, &addr_size);
    if(new_conn_fd < 0)
    {
      fprintf(stderr,"accept: %s\n",gai_strerror(new_conn_fd));
      continue;
    }

    inet_ntop(client_addr.ss_family, get_in_addr((struct sockaddr *) &client_addr),s ,sizeof s);
    printf("I am now connected to " GREEN "%s \n" RESET,s);

    message_no++;
    printf( BOLDWHITE "Receiving message no: %i\n" RESET, message_no);

    long long timestamp_size;
    readlong(new_conn_fd, &timestamp_size);
    printf("Got timestamp length: %li\n", timestamp_size);

    long long filesize;
    readlong(new_conn_fd, &filesize);
    printf("Got filesize: %li\n", filesize);
    long c = 0;

		std::vector<char> tbuffer;
    tbuffer.resize(timestamp_size);

    int n = recv(new_conn_fd, &tbuffer[0], tbuffer.size(), 0);
		std::string timestamp(tbuffer.begin(),tbuffer.end());
    printf("Got timestamp: %s and read %i bytes\n", timestamp.c_str(), n);

    //std::string t("test.txt");
    std::string t(timestamp);
    std::string suffix(".pcd");
		std::string directory("./PCD/");
    printf("Write file: %s%s\n", t.c_str(), suffix.c_str());
		std::ofstream os ((directory + t + suffix).c_str(), std::ofstream::binary);
    while (filesize > c) {
			std::vector<char> buffer;
			int len;

			if (255 > filesize-c) {
				len = filesize-c;
			} else {
				len = 255;
			}
			buffer.resize(len);

      int b = recv(new_conn_fd, &buffer[0], buffer.size(), 0);
			//if (b > 0) {
			//	printf("Read %i bytes from socket (filesize=%i and c=%i)\n", b, filesize, c);
			//}

			os.write(&buffer[0], b);
    	c += b;
    }
    printf("\n=> File written\n");
		os.close();
  }

  // Close the socket before we finish
  close(new_conn_fd);

  return 0;
}
