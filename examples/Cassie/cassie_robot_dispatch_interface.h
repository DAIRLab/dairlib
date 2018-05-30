#ifndef CASSIE_ROBOT_DISPATCH_INTERFACE_H
#define CASSIE_ROBOT_DISPATCH_INTERFACE_H

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
#include <thread>

#include "polling_interface.h"
#include "datatypes/cassie_dispatch_types.h"
extern "C" {
#include "datatypes/cassie_out_t.h"
#include "datatypes/cassie_user_in_t.h"
}
typedef struct {
    char seq_num_out;
    char seq_num_in_last;
    char delay;
    char seq_num_in_diff;
} packet_header_info_t;

class CassieRobotDispatchInterface : public PollingInterface<cassie_dispatch_robot_in_t, cassie_dispatch_robot_out_t> {
  public:
    CassieRobotDispatchInterface(const char *local_addr,
            const char *remote_addr,
            unsigned short lport,
            unsigned short rport) : local_address(local_addr), remote_address(remote_addr), local_port(lport), remote_port(rport) {};
    virtual void SetupChannel();
    virtual void StartPolling(std::function<void(cassie_dispatch_robot_out_t)> handler);
    virtual void StopPolling();
    virtual void Send(cassie_dispatch_robot_in_t message);
  protected:
    virtual cassie_dispatch_robot_out_t Receive();

  private:
    static struct sockaddr_in make_sockaddr_in(const char *addr_str, unsigned short port);
    static int udp_connect(const char *local_addr_str, const char *remote_addr_str, unsigned short lport, unsigned short rport);
    static void process_packet_header(packet_header_info_t *info, const char *header_in, char *header_out);
    void polling_function(std::function<void(cassie_dispatch_robot_out_t)> handler);
    const char * local_address;
    const char * remote_address;
    unsigned short local_port;
    unsigned short remote_port;
    int sock;
    bool continue_running;

    // hack
    char recvbuf[2 + CASSIE_OUT_T_LEN];
    char sendbuf[2 + CASSIE_USER_IN_T_LEN];
    packet_header_info_t header_info = {0};
    std::thread polling_thread;
};
#endif
