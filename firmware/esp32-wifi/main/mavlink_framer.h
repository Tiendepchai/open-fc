#ifndef OPENFC_MAVLINK_FRAMER_H
#define OPENFC_MAVLINK_FRAMER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MAVLink2 max packet: 10 header + 255 payload + 2 CRC + 13 signature = 280 bytes */
#define OPENFC_MAVLINK_MAX_PACKET_LEN 280U
#define OPENFC_MAVLINK_STX_V1 0xFEU
#define OPENFC_MAVLINK_STX_V2 0xFDU
#define OPENFC_MAVLINK_V2_IFLAG_SIGNED 0x01U

typedef struct {
    uint32_t packets_v1;
    uint32_t packets_v2;
    uint32_t bytes_dropped_resync;
    uint32_t framing_resets;
} openfc_mavlink_framer_stats_t;

typedef struct {
    uint8_t buf[OPENFC_MAVLINK_MAX_PACKET_LEN];
    size_t pos;
    size_t expected_len;
    openfc_mavlink_framer_stats_t stats;
} openfc_mavlink_framer_t;

void openfc_mavlink_framer_init(openfc_mavlink_framer_t *f);
void openfc_mavlink_framer_reset(openfc_mavlink_framer_t *f);

/**
 * Feed one byte into the transparent MAVLink1/MAVLink2 stream framer.
 * Returns true when a complete packet is ready in f->buf[0 .. *out_len - 1].
 * Never allocates memory and never injects synthetic frames.
 */
bool openfc_mavlink_framer_feed(openfc_mavlink_framer_t *f, uint8_t byte, size_t *out_len);

/**
 * Validate that a standalone UDP datagram begins with a bounded MAVLink1/2 packet
 * before forwarding to FC UART (rejects garbage UDP noise at the trust boundary).
 */
bool openfc_mavlink_validate_datagram(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* OPENFC_MAVLINK_FRAMER_H */
