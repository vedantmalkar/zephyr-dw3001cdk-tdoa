#ifndef UWB_H
#define UWB_H

#include <stdint.h>
#include "deca_device_api.h"

#define SPEED_OF_LIGHT  299702547.0
#define UUS_TO_DWT_TIME 63898

/* default config: channel 9, 6.8 Mbps, 128 preamble, no STS */
extern dwt_config_t uwb_default_config;

/* init DW3000: probe, configure, set antenna delays */
int uwb_init(uint16_t ant_dly);

/* read 40-bit TX timestamp */
uint64_t uwb_get_tx_ts(void);

/* read 40-bit RX timestamp */
uint64_t uwb_get_rx_ts(void);

/* read 40-bit system time */
uint64_t uwb_get_sys_time(void);

/* send a frame immediately, blocks until TX done */
void uwb_tx(uint8_t *data, uint16_t len);

/* send a frame at a delayed time, blocks until TX done */
int uwb_tx_delayed(uint8_t *data, uint16_t len, uint32_t tx_time);

/* enable RX and wait for a frame or timeout/error.
 * returns 0 on success (frame received), -1 on timeout/error.
 * on success, frame is in rx_buf (len written to *rx_len). */
int uwb_rx(uint8_t *rx_buf, uint16_t *rx_len);

/* clear all RX/TX status flags */
void uwb_clear_status(void);

/* pack a 40-bit timestamp into 5 bytes (little-endian) */
void uwb_pack_ts(uint64_t ts, uint8_t *buf);

/* unpack 5 bytes (little-endian) into a 40-bit timestamp */
uint64_t uwb_unpack_ts(const uint8_t *buf);

#endif
