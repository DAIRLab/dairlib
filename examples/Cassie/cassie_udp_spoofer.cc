#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <functional>
#include <iostream>
#include <chrono>
#include <mutex>

#include "polling_interface.h"
#include "datatypes/cassie_dispatch_types.h"
#include "cassie_udp_spoofer.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"



namespace dairlib {


struct sockaddr_in
CassieUdpSpoofer::make_sockaddr_in(const char *addr_str,
                 unsigned short port)
{
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, addr_str, &addr.sin_addr);
    addr.sin_port = htons(port);
    memset(&addr.sin_zero, 0, sizeof(addr.sin_zero));
    return addr;
}


// Create a UDP socket bound and connected to specific local/remote addresses
int CassieUdpSpoofer::udp_connect(const char *local_addr_str,
            const char *remote_addr_str, unsigned short lport, unsigned short rport)
{
    // Create address structs
    struct sockaddr_in local_addr = make_sockaddr_in(local_addr_str, lport);
    struct sockaddr_in remote_addr = make_sockaddr_in(remote_addr_str, rport);

    // Create socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sock) {
        perror("Error creating socket: ");
        return -1;
    }
    
    
    int optval = 1;
	setsockopt(sock, SOL_SOCKET, SO_REUSEADDR,
		   (const void *)&optval, sizeof(int));

	/*
	 * build the server's Internet address
	 */
	bzero((char *)&local_addr, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	local_addr.sin_port = htons((unsigned short)lport);

	/*
	 * bind: associate the parent socket with a port
	 */
	if (bind(sock, (struct sockaddr *)&local_addr,
		 sizeof(struct sockaddr)) < 0)
		perror("ERROR on binding");
    
    /*struct sockaddr * la = (struct sockaddr *) (&local_addr);
    // Bind to local address
    if (-1 == bind(sock,
                   la,
                   sizeof (struct sockaddr))) {
        perror("Error binding to local address: ");
        close(sock);
        return -1;
    }

    // Connect to remote address
    if (-1 == connect(sock,
                      (struct sockaddr *) &remote_addr,
                      sizeof (struct sockaddr))) {
        perror("Error connecting to remote address: ");
        close(sock);
        return -1;
    }*/

    return sock;
}

// Process packet header used to measure delay and skipped packets
void
CassieUdpSpoofer::process_packet_header(packet_header_info_t *info,
                      const char *header_in,
                      char *header_out)
{
    // Increment outgoing packet sequence number
    ++info->seq_num_out;

    // header_in[0]: sequence number of incoming packet
    // header_in[1]: sequence number of previous outgoing packet, looped back
    char seq_num_in = header_in[0];
    char loopback = header_in[1];

    // Compute round-trip delay and incoming sequence number diff
    info->delay = info->seq_num_out - loopback;
    info->seq_num_in_diff = seq_num_in - info->seq_num_in_last;
    info->seq_num_in_last = seq_num_in;

    // Write outgoing packet header
    header_out[0] = info->seq_num_out;
    header_out[1] = seq_num_in;
}

void CassieUdpSpoofer::SetupChannel()
{
  sock = udp_connect(this->local_address, this->remote_address, this->local_port, this->remote_port);
  if (-1 == sock)
      exit(EXIT_FAILURE);
 std::cout << "Cassie UDP Spoofer Waiting for Connection..." << std::endl;
 this->receive_message();
 std::cout << "Connection Established" << std::endl;
  // Make socket non-blocking
  fcntl(sock, O_NONBLOCK);
}

cassie_dispatch_robot_in_t CassieUdpSpoofer::receive_message()
{
  // Create cassie input/output structs
  cassie_user_in_t cassie_user_in;

  //hack for testing
  ssize_t des_len = 60;
  // Poll for a new packet of the correct length
  ssize_t nbytes = 0;
  /*do {
      // Wait if no packets are available
      struct pollfd fd = {.fd = sock, .events = POLLIN, .revents = 0};
      while (!poll(&fd, 1, 2000))
      {
        if(!continue_subscribing)
          return cassie_user_in;
      }

      // Get newest valid packet in RX buffer
      // Does not use sequence number for determining newest packet
      while (poll(&fd, 1, 0)) {
        ioctl(sock, FIONREAD, &nbytes);
        //std::cout <<  "ioctl: " << nbytes << std::endl;
        if (sizeof recvbuf == nbytes){
          nbytes = recv(sock, recvbuf, sizeof recvbuf, 0);
        }
        else {
          recv(sock, recvbuf, 0, 0); // Discard packet
        }
          //std::cout <<  "received: " << nbytes << std::endl;
      }
  } while (des_len != nbytes);*/
  
  int n = 0;
  socklen_t clientlen = sizeof(clientaddr);
  while (1) {

		/*
		 * recvfrom: receive a UDP datagram from a client
		 */
		//this->addrmux.lock();
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
		std::cout << "RECV" << std::endl;std::cout.flush();
		n = recvfrom(sock, recvbuf, sizeof(recvbuf), 0,
			     (struct sockaddr *)&clientaddr, &clientlen);
		//this->addrmux.unlock();
		if (n < 0)
			perror("ERROR in recvfrom");
		if (n == 0)
		  continue;
		//printf("received %d \n",n);
        if (n != 60)
            perror("Broken Packet!");
        break;
        std::cout << "No Message!" << std::endl;std::cout.flush();
		/*
		 * gethostbyaddr: determine who sent the datagram
		 */
  }
  

  // Split header and data
  const unsigned char * data_in = reinterpret_cast<const unsigned char *>(&recvbuf[2]);

  // Unpack received data into cassie output struct
  unpack_cassie_user_in_t(reinterpret_cast<const unsigned char *>(data_in), &cassie_user_in);
  //std::cout << "unpacked input" << std::endl; std::cout.flush();
  return cassie_user_in;
}

void CassieUdpSpoofer::subscription_function()
{
  //std::cout <<  "Polling UDP" << std::endl;
  cassie_dispatch_robot_in_t cassie_in;
  while(continue_subscribing)
  {
    cassie_in = receive_message();
    if (!continue_subscribing)
    {
      break;
    }
    this->handler(cassie_in);
  }
  //std::cout <<  "Terminate Polling UDP" << std::endl;
}


void CassieUdpSpoofer::SetSubscriptionHandler(std::function<void(cassie_dispatch_robot_in_t)> hand)
{
    this->handler = hand;
}


void CassieUdpSpoofer::StartSubscriber()
{
  continue_subscribing = true;
  // temp for testing!
  //Send(cassie_dispatch_robot_in_t());

  subscribing_thread = std::thread(&CassieUdpSpoofer::subscription_function, this);
}

void CassieUdpSpoofer::StopSubscriber()
{
  continue_subscribing = false;
  subscribing_thread.join();
}

void CassieUdpSpoofer::Publish(cassie_dispatch_robot_out_t robot_out)
{
  
  //cassie_out_t cassie_out = *(this->publishing_source.get());
  //std::cout <<  "Sending UDP command\n" << std::endl;
  const char *header_in = recvbuf;
  char *header_out = sendbuf;
  unsigned char *data_out = reinterpret_cast<unsigned char *>(&sendbuf[2]);
  pack_cassie_out_t(&robot_out, data_out);


  // Process incoming header and write outgoing header
  process_packet_header(&header_info, header_in, header_out);
  /*printf("[%6.3f] %d / %d\n",
         ((float) clock()) / CLOCKS_PER_SEC,
         header_info.delay, header_info.seq_num_in_diff);*/

  // Send response, retry if busy
  
  
  //while (-1 == send(sock, sendbuf, sizeof sendbuf, 0)) {}
  
  int n;
  //this->addrmux.lock();
  int clientlen = sizeof(clientaddr);
  struct sockaddr_in sendaddr = clientaddr;
  //this->addrmux.unlock();
  n = sendto(sock, sendbuf, sizeof(sendbuf), 0,
           (struct sockaddr *)&sendaddr, clientlen);
  if (n < 0)
    perror("ERROR in sendto");
  //printf("sent %d \n",n);
}


}
#pragma GCC diagnostic pop
