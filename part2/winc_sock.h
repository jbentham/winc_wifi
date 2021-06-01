// ATWINC1500/1510 WiFi socket definitions for the Pi Pico
//
// Copyright (c) 2021 Jeremy P Bentham
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Socket definitions
#define UDP_PORTNUM     1025
#define TCP_PORTNUM     1025
#define UDP_SESSION     1
#define TCP_SESSION     1
#define MIN_SOCKET      0
#define MIN_TCP_SOCK    0
#define MAX_TCP_SOCK    7
#define MIN_UDP_SOCK    7
#define MAX_UDP_SOCK    10
#define MAX_SOCKETS     10
#define IP_FAMILY       2

#define STATE_CLOSED    0
#define STATE_BINDING   1
#define STATE_BOUND     2
#define STATE_ACCEPTED  3
#define STATE_CONNECTED 4

// Offsets of Tx data, from end of HIF header
#define UDP_DATA_OSET       68
#define TCP_DATA_OSET       80

// DHCP response message
typedef struct {
    uint32_t self, gate, dns,
             mask, lease;
} DHCP_RESP_MSG;

// Bind response message
typedef struct {
    uint8_t sock, status;
    uint16_t session;
} BIND_RESP_MSG;

// Listen response message
typedef struct {
    uint8_t sock, status;
    uint16_t session;
} LISTEN_RESP_MSG;

// Accept response message
typedef struct {
    SOCK_ADDR addr;
    uint8_t listen_sock, conn_sock;
    uint16_t oset;
} ACCEPT_RESP_MSG;

// Receive response message
typedef struct {
    SOCK_ADDR addr;
    int16_t dlen; // (status)
    uint16_t oset;
    uint8_t sock, x;
    uint16_t session;
} RECV_RESP_MSG;

// Response message union
typedef union {
    uint8_t data[16];
    int val;
    DHCP_RESP_MSG dhcp;
    BIND_RESP_MSG bind;
    LISTEN_RESP_MSG listen;
    ACCEPT_RESP_MSG accept;
    RECV_RESP_MSG recv;
} RESP_MSG;

// Storage for socket config
typedef struct {
    SOCK_ADDR addr;
    uint16_t localport, session;
    int state, conn_sock;
    uint32_t hif_data_addr;
    SOCK_HANDLER handler;
} SOCKET;

char *sock_err_str(int err);
int open_sock_server(int portnum, bool tcp, SOCK_HANDLER handler);
void interrupt_handler(void);
void sock_state(uint8_t sock, int news);
void check_sock(int fd, uint16_t gop, RESP_MSG *rmp);
bool put_sock_bind(int fd, uint8_t sock, uint16_t port);
bool put_sock_listen(int fd, uint8_t sock);
bool put_sock_recv(int fd, uint8_t sock);
bool put_sock_recvfrom(int fd, uint8_t sock);
bool put_sock_send(int fd, uint8_t sock, void *data, int len);
bool put_sock_sendto(int fd, uint8_t sock, void *data, int len);
bool put_sock_close(int fd, uint8_t sock);
bool get_sock_data(int fd, uint8_t sock, void *data, int len);
void tcp_echo_handler(int fd, uint8_t sock, int rxlen);
void udp_echo_handler(int fd, uint8_t sock, int rxlen);

// EOF

