/* Byte links to the outside world, all carrying ATLAS frames (COBS + CRC16):
 *   LINK_CARRIER : Wi-Fi UDP to agent_bridge on the carrier; when docked also the dock DATA UART
 *   LINK_PI      : UART1 921600 to the Pi Zero 2 W vision module (Scout) */
#pragma once
#include <stddef.h>
#include <stdint.h>
typedef void (*links_frame_cb)(int link, uint8_t type, const uint8_t *payload, int n);
void links_init(links_frame_cb cb);
void links_send(int link, uint8_t type, const void *payload, size_t n);
