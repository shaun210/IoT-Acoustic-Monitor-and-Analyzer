#ifndef CONNECT_TO_WIFI_H
#define CONNECT_TO_WIFI_H

#include <stdint.h>


extern uint8_t RemoteIP[4];

int Init_And_Connect_WiFi(void);

int Open_TCP_Connection(void);


#endif
