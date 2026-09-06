#include <stdatomic.h>
#include "common.h"

// NE2000 transmit hook: NULL = drop packets (no crash)
void (*_Atomic esp32_send_packet)(uint8_t *buf, int size) = NULL;

#include "common.h"

void wifi_main(const char *ssid, const char *pass)
{
    // WiFi not implemented for M5Stack Tab5
    // Network packets will be dropped
    (void)ssid;
    (void)pass;
}
