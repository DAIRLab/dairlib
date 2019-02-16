/* udpuser.c
 *
 * Sample program for faking the UDP messages from Cassie
 *
 * Copyright 2018 Agility Robotics
 */

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
#include "examples/Cassie/datatypes/cassie_out_t.h"
#include "examples/Cassie/datatypes/cassie_user_in_t.h"
#include "examples/Cassie/datatypes/cassie_networking.h"


// Data and results for processing packet header
typedef struct {
    char seq_num_out;
    char seq_num_in_last;
    char delay;
    char seq_num_in_diff;
} packet_header_info_t;


// Process packet header used to measure delay and skipped packets
static void
process_packet_header(packet_header_info_t *info,
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


// Dummy controller that outputs zeros
static void
null_dynamics(const cassie_out_t *out, cassie_user_in_t *in)
{
    memset(out, 'a', sizeof (cassie_out_t));
}


int main(int argc, char **argv)
{
	int sockfd;		/* socket */
	int portno;		/* port to listen on */
	int clientlen;		/* byte size of client's address */
	struct sockaddr_in serveraddr;	/* server's addr */
	//struct hostent *hostp;	/* client host info */
	//char *buf;		/* message buf */
	//char *hostaddrp;	/* dotted decimal host addr string */
	int optval;		/* flag value for setsockopt */
	int n;			/* message byte size */
  char recvbuf[2 + CASSIE_USER_IN_T_LEN];
  char sendbuf[2 + CASSIE_OUT_T_LEN];
  cassie_out_t cassie_out;
  cassie_user_in_t cassie_in;
  packet_header_info_t header_info = {0};
	/*
	 * check command line arguments
	 */
	portno = CASSIE_UDP_REMOTE_PORT;

	/*
	 * socket: create the parent socket
	 */
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0)
		perror("ERROR opening socket");

	/* setsockopt: Handy debugging trick that lets
	 * us rerun the server immediately after we kill it;
	 * otherwise we have to wait about 20 secs.
	 * Eliminates "ERROR on binding: Address already in use" error.
	 */
	optval = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,
		   (const void *)&optval, sizeof(int));

	/*
	 * build the server's Internet address
	 */
	bzero((char *)&serveraddr, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serveraddr.sin_port = htons((unsigned short)portno);


	while (1) {

		 const char *header_in = recvbuf;
     const char *data_in = &recvbuf[2];
     char *header_out = sendbuf;
     char *data_out = &sendbuf[2];
     // Pack cassie input struct into outgoing packet
     pack_cassie_out_t(&cassie_out, data_out);

		/*
		 * sendto: echo the input back to the client
		 */
		while(1){
      sleep(1);
      n = sendto(sockfd, sendbuf, sizeof(sendbuf), 0,
           (struct sockaddr *)&serveraddr, sizeof(serveraddr));
      if (n < 0)
        perror("ERROR in sendto");
      printf("sent %d \n",n);
    }
	}
}


