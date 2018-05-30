/* udpuser.c
 *
 * Sample program for running a controller on Cassie's User computer.
 * Requires UdpTarget.slx to be running on the Target computer.
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
#include "applications/cassie-dispatch/cassie_out_t.h"
#include "applications/cassie-dispatch/cassie_user_in_t.h"


// Construct an address struct given an address string and port number
static struct sockaddr_in
make_sockaddr_in(const char *addr_str,
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
static int
udp_connect(const char *local_addr_str,
            const char *remote_addr_str,
            unsigned short port)
{
    // Create address structs
    struct sockaddr_in local_addr = make_sockaddr_in(local_addr_str, port);
    struct sockaddr_in remote_addr = make_sockaddr_in(remote_addr_str, port);

    // Create socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sock) {
        perror("Error creating socket: ");
        return -1;
    }

    // Bind to local address
    if (-1 == bind(sock,
                   (struct sockaddr *) &local_addr,
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
    }

    return sock;
}


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
null_controller(const cassie_out_t *out, cassie_user_in_t *in)
{
    memset(in, 0, sizeof (cassie_user_in_t));
}


int
main()
{
    // Connect user computer to target computer
    int sock = udp_connect("10.10.10.100", "10.10.10.3", 25000);
    if (-1 == sock)
        exit(EXIT_FAILURE);

    // Make socket non-blocking
    fcntl(sock, O_NONBLOCK);

    // Create packet input/output buffers
    char recvbuf[2 + CASSIE_OUT_T_LEN];
    char sendbuf[2 + CASSIE_USER_IN_T_LEN];

    // Create cassie input/output structs
    cassie_out_t cassie_out;
    cassie_user_in_t cassie_in;

    // Create header information struct
    packet_header_info_t header_info = {0};

    // Listen/respond loop
    while (true) {
        // Poll for a new packet of the correct length
        ssize_t nbytes;
        do {
            // Wait if no packets are available
            struct pollfd fd = {.fd = sock, .events = POLLIN, .revents = 0};
            while (!poll(&fd, 1, 0)) {}

            // Get newest valid packet in RX buffer
            // Does not use sequence number for determining newest packet
            while (poll(&fd, 1, 0)) {
                ioctl(sock, FIONREAD, &nbytes);
                if (sizeof recvbuf == nbytes)
                    nbytes = recv(sock, recvbuf, sizeof recvbuf, 0);
                else
                    recv(sock, recvbuf, 0, 0); // Discard packet
            }
        } while (sizeof recvbuf != nbytes);

        // Split header and data
        const char *header_in = recvbuf;
        const char *data_in = &recvbuf[2];
        char *header_out = sendbuf;
        char *data_out = &sendbuf[2];

        // Process incoming header and write outgoing header
        process_packet_header(&header_info, header_in, header_out);
        printf("[%6.3f] %d / %d\n",
               ((float) clock()) / CLOCKS_PER_SEC,
               header_info.delay, header_info.seq_num_in_diff);

        // Unpack received data into cassie output struct
        unpack_cassie_out_t(data_in, &cassie_out);

        // Run controller
        null_controller(&cassie_out, &cassie_in);

        // Pack cassie input struct into outgoing packet
        pack_cassie_user_in_t(&cassie_in, data_out);

        // Send response, retry if busy
        while (-1 == send(sock, sendbuf, sizeof sendbuf, 0)) {}
    }
}
