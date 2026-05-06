// morseApp.h
#ifndef MORSEAPP_H
#define MORSEAPP_H

#define MAX_SYMBOLS    32
#define MORSE_UDP_PORT 0xF0B0
#define N_Ids          6
#define My_ID          1

#include "opendefs.h"

#ifndef OPENWSN_SOCK_H
#include "sock.h"
#endif

typedef struct {
    uint8_t sender_id;
    uint8_t receiver_id;
    uint8_t msg_len;
    char    msg[MAX_SYMBOLS + 1];
} morse_packet_t;

extern morse_packet_t received_pkt;
extern uint8_t        new_message;
extern sock_udp_t morse_sock;

void morseApp_init(void);
void morse_display_task(void);
void morseApp_poll_task(void);

#endif