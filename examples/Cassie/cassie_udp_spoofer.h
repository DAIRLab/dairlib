#ifndef CASSIE_UDP_SPOOFER_H
#define CASSIE_UDP_SPOOFER_H
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
#include <atomic>
#include <memory>

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

class CassieUdpSpoofer {
  public:
    CassieUdpSpoofer(const char *local_addr,
            const char *remote_addr,
            unsigned short lport,
            unsigned short rport) : local_address(local_addr), remote_address(remote_addr), local_port(lport), remote_port(rport) {};
    void SetupChannel();
    void SetSubscriptionHandler(std::function<void(cassie_dispatch_robot_in_t)> hand);
    void SetPublishingSource(std::shared_ptr<std::atomic<cassie_dispatch_robot_out_t>> ps);
    void SetPublishingPeriod(double dt);
    void StartSubscriber();
    void StopSubscriber();
    void StartPublisher();
    void StopPublisher();

  private:
    static struct sockaddr_in make_sockaddr_in(const char *addr_str, unsigned short port);
    static int udp_connect(const char *local_addr_str, const char *remote_addr_str, unsigned short lport, unsigned short rport);
    static void process_packet_header(packet_header_info_t *info, const char *header_in, char *header_out);
    void subscription_function();
    void publishing_function();
    cassie_dispatch_robot_in_t receive_message();
    void publish_message();
    void send_message();
    const char * local_address;
    const char * remote_address;
    unsigned short local_port;
    unsigned short remote_port;
    std::function<void(cassie_dispatch_robot_in_t)> handler;
    std::shared_ptr<std::atomic<cassie_dispatch_robot_out_t>> publishing_source; 
    int sock;
    bool continue_subscribing;
    bool continue_publishing;
    double publishing_period = 5e-4;

    char sendbuf[2 + CASSIE_OUT_T_LEN];
    char recvbuf[2 + CASSIE_USER_IN_T_LEN];
    packet_header_info_t header_info = {0};
    std::thread subscribing_thread;
    std::thread publishing_thread;
};
#endif
