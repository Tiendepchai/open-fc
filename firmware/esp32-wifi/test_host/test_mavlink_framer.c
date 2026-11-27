#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../main/mavlink_framer.h"

static bool feed_packet(openfc_mavlink_framer_t *f, const uint8_t *pkt, size_t len, size_t *out_len) {
    bool done = false;
    for (size_t i = 0U; i < len; ++i) {
        done = openfc_mavlink_framer_feed(f, pkt[i], out_len);
        if (i + 1U < len) {
            assert(!done);
            assert(*out_len == 0U);
        }
    }
    return done;
}

static void test_null_and_corrupt_state_guards(void) {
    openfc_mavlink_framer_init(NULL);
    openfc_mavlink_framer_reset(NULL);

    size_t out_len = 99U;
    assert(!openfc_mavlink_framer_feed(NULL, OPENFC_MAVLINK_STX_V2, &out_len));
    assert(out_len == 0U);

    assert(!openfc_mavlink_validate_datagram(NULL, 10U));
    uint8_t dummy[4] = {0xFD, 0x00, 0x00, 0x00};
    assert(!openfc_mavlink_validate_datagram(dummy, 0U));

    /* Corrupted internal state (pos >= MAX_PACKET_LEN) must reset and accept incoming STX */
    openfc_mavlink_framer_t f;
    openfc_mavlink_framer_init(&f);
    f.pos = OPENFC_MAVLINK_MAX_PACKET_LEN;
    f.buf[0] = 0xAAU;
    assert(!openfc_mavlink_framer_feed(&f, OPENFC_MAVLINK_STX_V1, &out_len));
    assert(f.pos == 1U && f.buf[0] == OPENFC_MAVLINK_STX_V1);
    assert(f.stats.framing_resets == 1U);
    assert(f.stats.bytes_dropped_resync == OPENFC_MAVLINK_MAX_PACKET_LEN);

    /* Corrupted buf[0] with pos > 0 must also self-recover */
    f.pos = 3U;
    f.buf[0] = 0x12U;
    assert(!openfc_mavlink_framer_feed(&f, OPENFC_MAVLINK_STX_V2, NULL));
    assert(f.pos == 1U && f.buf[0] == OPENFC_MAVLINK_STX_V2);
    assert(f.stats.framing_resets == 2U);
}

static void test_v1_and_v2_size_boundaries(void) {
    openfc_mavlink_framer_t f;
    openfc_mavlink_framer_init(&f);
    size_t out_len = 0U;

    /* MAVLink1 min length: payload_len = 0 -> total 8 bytes */
    uint8_t v1_min[8];
    memset(v1_min, 0x22, sizeof(v1_min));
    v1_min[0] = OPENFC_MAVLINK_STX_V1;
    v1_min[1] = 0U;
    assert(feed_packet(&f, v1_min, sizeof(v1_min), &out_len));
    assert(out_len == 8U && memcmp(f.buf, v1_min, sizeof(v1_min)) == 0);
    assert(f.stats.packets_v1 == 1U);

    /* MAVLink1 max length: payload_len = 255 -> total 263 bytes (with embedded 0xFD/0xFE in payload) */
    uint8_t v1_max[8U + 255U];
    memset(v1_max, 0xFDU, sizeof(v1_max));
    v1_max[0] = OPENFC_MAVLINK_STX_V1;
    v1_max[1] = 255U;
    v1_max[10] = OPENFC_MAVLINK_STX_V1;
    assert(feed_packet(&f, v1_max, sizeof(v1_max), &out_len));
    assert(out_len == 263U && memcmp(f.buf, v1_max, sizeof(v1_max)) == 0);
    assert(f.stats.packets_v1 == 2U);

    /* MAVLink2 min length: payload_len = 0, unsigned -> total 12 bytes */
    uint8_t v2_min[12];
    memset(v2_min, 0x33, sizeof(v2_min));
    v2_min[0] = OPENFC_MAVLINK_STX_V2;
    v2_min[1] = 0U;
    v2_min[2] = 0U;
    assert(feed_packet(&f, v2_min, sizeof(v2_min), &out_len));
    assert(out_len == 12U && memcmp(f.buf, v2_min, sizeof(v2_min)) == 0);
    assert(f.stats.packets_v2 == 1U);

    /* MAVLink2 max unsigned length: payload_len = 255, incompat = 0 -> total 267 bytes */
    uint8_t v2_max_u[12U + 255U];
    memset(v2_max_u, 0xFEU, sizeof(v2_max_u));
    v2_max_u[0] = OPENFC_MAVLINK_STX_V2;
    v2_max_u[1] = 255U;
    v2_max_u[2] = 0U;
    assert(feed_packet(&f, v2_max_u, sizeof(v2_max_u), &out_len));
    assert(out_len == 267U && memcmp(f.buf, v2_max_u, sizeof(v2_max_u)) == 0);
    assert(f.stats.packets_v2 == 2U);

    /* MAVLink2 max signed length: payload_len = 255, incompat = 0x01 -> total 280 bytes */
    uint8_t v2_max_s[OPENFC_MAVLINK_MAX_PACKET_LEN];
    memset(v2_max_s, 0xFDU, sizeof(v2_max_s));
    v2_max_s[0] = OPENFC_MAVLINK_STX_V2;
    v2_max_s[1] = 255U;
    v2_max_s[2] = OPENFC_MAVLINK_V2_IFLAG_SIGNED;
    assert(feed_packet(&f, v2_max_s, sizeof(v2_max_s), &out_len));
    assert(out_len == OPENFC_MAVLINK_MAX_PACKET_LEN);
    assert(memcmp(f.buf, v2_max_s, sizeof(v2_max_s)) == 0);
    assert(f.stats.packets_v2 == 3U && f.pos == 0U && f.expected_len == 0U);
}

static void test_resync_and_embedded_stx(void) {
    /* Golden MAVLink2 HEARTBEAT (#0, len=9, total=21 bytes) verified via pymavlink */
    const uint8_t hb_v2[21] = {
        0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0xBF, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x12, 0x08, 0x00, 0x04, 0x03, 0xAE, 0xC6
    };
    const uint8_t pkt_v1[12] = {
        0xFE, 0x04, 0x01, 0x01, 0x01, 0x00, 0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34
    };

    openfc_mavlink_framer_t f;
    openfc_mavlink_framer_init(&f);
    size_t out_len = 0U;

    /* 1. Noise prefix followed by valid MAVLink2 packet */
    assert(!openfc_mavlink_framer_feed(&f, 0xAAU, &out_len));
    assert(!openfc_mavlink_framer_feed(&f, 0x55U, &out_len));
    assert(f.stats.bytes_dropped_resync == 2U);
    assert(feed_packet(&f, hb_v2, sizeof(hb_v2), &out_len));
    assert(out_len == sizeof(hb_v2) && memcmp(f.buf, hb_v2, sizeof(hb_v2)) == 0);
    assert(f.stats.packets_v2 == 1U);

    /* 2. Invalid MAVLink2 incompat_flags with non-STX bytes resets cleanly */
    uint32_t drops_before = f.stats.bytes_dropped_resync;
    assert(!openfc_mavlink_framer_feed(&f, OPENFC_MAVLINK_STX_V2, &out_len));
    assert(!openfc_mavlink_framer_feed(&f, 9U, &out_len));
    assert(!openfc_mavlink_framer_feed(&f, 0x80U, &out_len));
    assert(f.pos == 0U && f.stats.framing_resets == 1U);
    assert(f.stats.bytes_dropped_resync == drops_before + 3U);

    /* 3. Spurious 0xFD immediately preceding valid MAVLink2: [0xFD] + hb_v2 (0xFD, 0x09, 0x00...) */
    drops_before = f.stats.bytes_dropped_resync;
    assert(!openfc_mavlink_framer_feed(&f, OPENFC_MAVLINK_STX_V2, &out_len));
    assert(feed_packet(&f, hb_v2, sizeof(hb_v2), &out_len));
    assert(out_len == sizeof(hb_v2) && memcmp(f.buf, hb_v2, sizeof(hb_v2)) == 0);
    assert(f.stats.bytes_dropped_resync == drops_before + 1U);

    /* 4. Double spurious [0xFD, 0xFD] immediately preceding valid MAVLink2 */
    drops_before = f.stats.bytes_dropped_resync;
    assert(!openfc_mavlink_framer_feed(&f, OPENFC_MAVLINK_STX_V2, &out_len));
    assert(!openfc_mavlink_framer_feed(&f, OPENFC_MAVLINK_STX_V2, &out_len));
    assert(feed_packet(&f, hb_v2, sizeof(hb_v2), &out_len));
    assert(out_len == sizeof(hb_v2) && memcmp(f.buf, hb_v2, sizeof(hb_v2)) == 0);
    assert(f.stats.bytes_dropped_resync == drops_before + 2U);

    /* 5. Spurious [0xFD, 0x55] immediately preceding valid MAVLink2 (real STX lands at buf[2]) */
    drops_before = f.stats.bytes_dropped_resync;
    assert(!openfc_mavlink_framer_feed(&f, OPENFC_MAVLINK_STX_V2, &out_len));
    assert(!openfc_mavlink_framer_feed(&f, 0x55U, &out_len));
    assert(feed_packet(&f, hb_v2, sizeof(hb_v2), &out_len));
    assert(out_len == sizeof(hb_v2) && memcmp(f.buf, hb_v2, sizeof(hb_v2)) == 0);
    assert(f.stats.bytes_dropped_resync == drops_before + 2U);

    /* 6. Spurious 0xFD immediately preceding valid MAVLink1: [0xFD] + pkt_v1 (0xFE, 0x04, ...) */
    drops_before = f.stats.bytes_dropped_resync;
    assert(!openfc_mavlink_framer_feed(&f, OPENFC_MAVLINK_STX_V2, &out_len));
    assert(feed_packet(&f, pkt_v1, sizeof(pkt_v1), &out_len));
    assert(out_len == sizeof(pkt_v1) && memcmp(f.buf, pkt_v1, sizeof(pkt_v1)) == 0);
    assert(f.stats.bytes_dropped_resync == drops_before + 1U);

    /* 7. Spurious [0xFD, 0x55] immediately preceding valid MAVLink1 (real 0xFE lands at buf[2]) */
    drops_before = f.stats.bytes_dropped_resync;
    assert(!openfc_mavlink_framer_feed(&f, OPENFC_MAVLINK_STX_V2, &out_len));
    assert(!openfc_mavlink_framer_feed(&f, 0x55U, &out_len));
    assert(feed_packet(&f, pkt_v1, sizeof(pkt_v1), &out_len));
    assert(out_len == sizeof(pkt_v1) && memcmp(f.buf, pkt_v1, sizeof(pkt_v1)) == 0);
    assert(f.stats.bytes_dropped_resync == drops_before + 2U);

    /* 8. Truncated partial frame cleared by explicit reset (UART idle / frame error) */
    drops_before = f.stats.bytes_dropped_resync;
    uint32_t resets_before = f.stats.framing_resets;
    for (size_t i = 0U; i < 7U; ++i) {
        assert(!openfc_mavlink_framer_feed(&f, hb_v2[i], &out_len));
    }
    openfc_mavlink_framer_reset(&f);
    assert(f.pos == 0U && f.expected_len == 0U);
    assert(f.stats.framing_resets == resets_before + 1U);
    assert(f.stats.bytes_dropped_resync == drops_before + 7U);
    /* Resetting an already-idle framer must be a no-op */
    openfc_mavlink_framer_reset(&f);
    assert(f.stats.framing_resets == resets_before + 1U);
    assert(feed_packet(&f, hb_v2, sizeof(hb_v2), &out_len));
    assert(out_len == sizeof(hb_v2));
}

static void test_udp_datagram_validation(void) {
    const uint8_t hb_v2[21] = {
        0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0xBF, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x12, 0x08, 0x00, 0x04, 0x03, 0xAE, 0xC6
    };
    uint8_t v1_pkt[12];
    memset(v1_pkt, 0x44, sizeof(v1_pkt));
    v1_pkt[0] = OPENFC_MAVLINK_STX_V1;
    v1_pkt[1] = 4U;

    uint8_t v2_signed[12U + 4U + 13U];
    memset(v2_signed, 0x11, sizeof(v2_signed));
    v2_signed[0] = OPENFC_MAVLINK_STX_V2;
    v2_signed[1] = 4U;
    v2_signed[2] = OPENFC_MAVLINK_V2_IFLAG_SIGNED;

    uint8_t v2_max_signed[OPENFC_MAVLINK_MAX_PACKET_LEN];
    memset(v2_max_signed, 0x77, sizeof(v2_max_signed));
    v2_max_signed[0] = OPENFC_MAVLINK_STX_V2;
    v2_max_signed[1] = 255U;
    v2_max_signed[2] = OPENFC_MAVLINK_V2_IFLAG_SIGNED;

    /* Valid singles */
    assert(openfc_mavlink_validate_datagram(hb_v2, sizeof(hb_v2)));
    assert(openfc_mavlink_validate_datagram(v1_pkt, sizeof(v1_pkt)));
    assert(openfc_mavlink_validate_datagram(v2_signed, sizeof(v2_signed)));
    assert(openfc_mavlink_validate_datagram(v2_max_signed, sizeof(v2_max_signed)));

    /* Valid coalesced multi-packet datagram: V1 + V2 + V2_signed + 4x max 280B = 1182 B (> 1024 B) */
    uint8_t multi[sizeof(v1_pkt) + sizeof(hb_v2) + sizeof(v2_signed) + (4U * OPENFC_MAVLINK_MAX_PACKET_LEN)];
    size_t off = 0U;
    memcpy(multi + off, v1_pkt, sizeof(v1_pkt));
    off += sizeof(v1_pkt);
    memcpy(multi + off, hb_v2, sizeof(hb_v2));
    off += sizeof(hb_v2);
    memcpy(multi + off, v2_signed, sizeof(v2_signed));
    off += sizeof(v2_signed);
    for (size_t k = 0U; k < 4U; ++k) {
        memcpy(multi + off, v2_max_signed, sizeof(v2_max_signed));
        off += sizeof(v2_max_signed);
    }
    assert(off == sizeof(multi));
    assert(openfc_mavlink_validate_datagram(multi, sizeof(multi)));

    /* Truncated multi-packet datagram (UDP fragmentation / MTU clip) MUST be rejected */
    assert(!openfc_mavlink_validate_datagram(multi, sizeof(multi) - 1U));
    assert(!openfc_mavlink_validate_datagram(hb_v2, sizeof(hb_v2) - 1U));
    assert(!openfc_mavlink_validate_datagram(hb_v2, 11U));
    assert(!openfc_mavlink_validate_datagram(v1_pkt, 7U));
    assert(!openfc_mavlink_validate_datagram(v2_signed, sizeof(v2_signed) - 1U));

    /* Trailing garbage byte after valid packet MUST be rejected */
    uint8_t trailing[sizeof(hb_v2) + 1U];
    memcpy(trailing, hb_v2, sizeof(hb_v2));
    trailing[sizeof(hb_v2)] = 0x00U;
    assert(!openfc_mavlink_validate_datagram(trailing, sizeof(trailing)));

    /* Unknown incompat_flags MUST be rejected */
    uint8_t bad_iflag[sizeof(hb_v2)];
    memcpy(bad_iflag, hb_v2, sizeof(hb_v2));
    bad_iflag[2] = 0x02U;
    assert(!openfc_mavlink_validate_datagram(bad_iflag, sizeof(bad_iflag)));
    bad_iflag[2] = 0xFFU;
    assert(!openfc_mavlink_validate_datagram(bad_iflag, sizeof(bad_iflag)));

    /* Pure garbage MUST be rejected */
    assert(!openfc_mavlink_validate_datagram((const uint8_t *)"garbage", 7U));
}

int main(void) {
    test_null_and_corrupt_state_guards();
    test_v1_and_v2_size_boundaries();
    test_resync_and_embedded_stx();
    test_udp_datagram_validation();

    puts("test_mavlink_framer: PASS");
    return 0;
}
