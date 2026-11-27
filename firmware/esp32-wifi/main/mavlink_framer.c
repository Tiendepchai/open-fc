#include "mavlink_framer.h"

#include <string.h>

void openfc_mavlink_framer_init(openfc_mavlink_framer_t *f) {
    if (f == NULL) {
        return;
    }
    memset(f, 0, sizeof(*f));
}

void openfc_mavlink_framer_reset(openfc_mavlink_framer_t *f) {
    if (f == NULL) {
        return;
    }
    if (f->pos > 0U) {
        f->stats.framing_resets++;
        f->stats.bytes_dropped_resync += (uint32_t)f->pos;
    }
    f->pos = 0U;
    f->expected_len = 0U;
}

static size_t compute_v1_len(uint8_t payload_len) {
    /* STX(1) + LEN(1) + SEQ(1) + SYS(1) + COMP(1) + MSGID(1) + PAYLOAD(N) + CRC(2) */
    return 8U + (size_t)payload_len;
}

static size_t compute_v2_len(uint8_t payload_len, uint8_t incompat_flags) {
    /* STX(1) + LEN(1) + INC(1) + CMP(1) + SEQ(1) + SYS(1) + COMP(1) + MSGID(3) + PAYLOAD(N) + CRC(2) [+ SIG(13)] */
    size_t sig = ((incompat_flags & OPENFC_MAVLINK_V2_IFLAG_SIGNED) != 0U) ? 13U : 0U;
    return 12U + (size_t)payload_len + sig;
}

/*
 * When a candidate MAVLink2 header at buf[0..2] fails incompat_flags validation,
 * drop buf[0] and non-recursively rescan buf[1..2] so an immediately following
 * valid STX (e.g. [0xFD, 0xFD, len, ...] or [0xFD, noise, 0xFD, ...]) is not lost.
 */
static void rescan_rejected_v2_header(openfc_mavlink_framer_t *f) {
    uint8_t b1 = f->buf[1];
    uint8_t b2 = f->buf[2];
    f->stats.framing_resets++;
    f->stats.bytes_dropped_resync++; /* drop buf[0] */
    f->pos = 0U;
    f->expected_len = 0U;

    if (b1 == OPENFC_MAVLINK_STX_V1) {
        f->buf[0] = b1;
        f->buf[1] = b2;
        f->pos = 2U;
        f->expected_len = compute_v1_len(b2);
    } else if (b1 == OPENFC_MAVLINK_STX_V2) {
        f->buf[0] = b1;
        f->buf[1] = b2;
        f->pos = 2U;
    } else {
        f->stats.bytes_dropped_resync++; /* drop b1 */
        if (b2 == OPENFC_MAVLINK_STX_V1 || b2 == OPENFC_MAVLINK_STX_V2) {
            f->buf[0] = b2;
            f->pos = 1U;
        } else {
            f->stats.bytes_dropped_resync++; /* drop b2 */
        }
    }
}

bool openfc_mavlink_framer_feed(openfc_mavlink_framer_t *f, uint8_t byte, size_t *out_len) {
    if (out_len != NULL) {
        *out_len = 0U;
    }
    if (f == NULL) {
        return false;
    }

    /* Defensive recovery if caller/memory left pos or buf[0] in an inconsistent state */
    if (f->pos >= OPENFC_MAVLINK_MAX_PACKET_LEN ||
        (f->pos > 0U && f->buf[0] != OPENFC_MAVLINK_STX_V1 && f->buf[0] != OPENFC_MAVLINK_STX_V2)) {
        openfc_mavlink_framer_reset(f);
    }

    if (f->pos == 0U) {
        if (byte == OPENFC_MAVLINK_STX_V2 || byte == OPENFC_MAVLINK_STX_V1) {
            f->buf[0] = byte;
            f->pos = 1U;
            f->expected_len = 0U;
        } else {
            f->stats.bytes_dropped_resync++;
        }
        return false;
    }

    f->buf[f->pos++] = byte;

    if (f->buf[0] == OPENFC_MAVLINK_STX_V1) {
        if (f->pos == 2U) {
            f->expected_len = compute_v1_len(f->buf[1]);
        }
    } else {
        /* MAVLink2: incompat_flags at index 2 determines whether 13-byte signature follows CRC */
        if (f->pos == 3U) {
            uint8_t incompat = f->buf[2];
            /* Reject unknown incompatibility flags (only bit 0 = signed is standard) */
            if ((incompat & (uint8_t)(~OPENFC_MAVLINK_V2_IFLAG_SIGNED)) != 0U) {
                rescan_rejected_v2_header(f);
                return false;
            }
            f->expected_len = compute_v2_len(f->buf[1], incompat);
        }
    }

    if (f->expected_len > OPENFC_MAVLINK_MAX_PACKET_LEN) {
        openfc_mavlink_framer_reset(f);
        return false;
    }

    if (f->expected_len > 0U && f->pos == f->expected_len) {
        size_t pkt_len = f->expected_len;
        if (f->buf[0] == OPENFC_MAVLINK_STX_V2) {
            f->stats.packets_v2++;
        } else {
            f->stats.packets_v1++;
        }
        f->pos = 0U;
        f->expected_len = 0U;
        if (out_len != NULL) {
            *out_len = pkt_len;
        }
        return true;
    }

    return false;
}

bool openfc_mavlink_validate_datagram(const uint8_t *data, size_t len) {
    if (data == NULL || len == 0U) {
        return false;
    }
    size_t offset = 0U;
    while (offset < len) {
        uint8_t stx = data[offset];
        size_t pkt_len = 0U;
        if (stx == OPENFC_MAVLINK_STX_V1) {
            if (len - offset < 8U) {
                return false;
            }
            pkt_len = compute_v1_len(data[offset + 1U]);
        } else if (stx == OPENFC_MAVLINK_STX_V2) {
            if (len - offset < 12U) {
                return false;
            }
            uint8_t incompat = data[offset + 2U];
            if ((incompat & (uint8_t)(~OPENFC_MAVLINK_V2_IFLAG_SIGNED)) != 0U) {
                return false;
            }
            pkt_len = compute_v2_len(data[offset + 1U], incompat);
        } else {
            return false;
        }
        if (pkt_len == 0U || pkt_len > OPENFC_MAVLINK_MAX_PACKET_LEN || pkt_len > (len - offset)) {
            return false;
        }
        offset += pkt_len;
    }
    return offset == len;
}
