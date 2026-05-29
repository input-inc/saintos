/**
 * Host-runnable test for the shared OTA streamer.
 *
 * The streamer's job is to take arbitrary-sized HTTP body chunks and
 * turn them into platform-write-aligned flash writes WITHOUT changing
 * the CRC32 the downloader sees. Bugs here are silent corruption,
 * mid-flash failure, or a CRC mismatch that fails an otherwise good
 * OTA — so the assertions here are tight.
 *
 * Cases:
 *   - random chunk sizes pass through correctly
 *   - intermediate flushes are exactly page_size
 *   - non-page-aligned tails get padded with 0xFF up to pad_granularity
 *   - CRC32 matches over the un-padded stream (not over what hit flash)
 *   - oversized stream rejected via OTA_STREAM_ERR_OVERFLOW
 *   - pad_granularity=256 (RP2040 case) pads the tail to a full page
 *   - feed-after-finalize rejected
 *   - write_fn failure propagates
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

/* Pull in the units under test (and their dep). */
#include "../src/crc32.c"
#include "../src/ota_streamer.c"

/* ── Captured-write sink ──────────────────────────────────────────── */

#define SINK_CAP (16 * 1024)
static uint8_t  sink[SINK_CAP];
static uint32_t sink_len = 0;
static int      fail_next_write = -1;  /* -1 = never fail */

static void sink_reset(void)
{
    memset(sink, 0, sizeof(sink));
    sink_len = 0;
    fail_next_write = -1;
}

static int sink_write(void* ctx, uint32_t off, const uint8_t* data, uint32_t len)
{
    (void)ctx;
    if (fail_next_write == 0) {
        fail_next_write = -1;
        return -42;
    }
    if (fail_next_write > 0) fail_next_write--;
    /* Each write must always land contiguously after the prior one
     * (the streamer never seeks backwards). */
    if (off != sink_len) {
        fprintf(stderr, "  WRITE_OFFSET_MISMATCH: got %u expected %u\n",
                (unsigned)off, (unsigned)sink_len);
        return -99;
    }
    if (sink_len + len > sizeof(sink)) return -1;
    memcpy(sink + sink_len, data, len);
    sink_len += len;
    return 0;
}

static int fail_count = 0;
#define EXPECT(cond, label) do {                                                \
    if (cond) printf("  ok   %s\n", label);                                     \
    else { printf("  FAIL %s\n", label); fail_count++; }                        \
} while (0)

/* ── Cases ────────────────────────────────────────────────────────── */

static void case_aligned_full_pages(void)
{
    printf("case_aligned_full_pages\n");
    sink_reset();

    uint8_t page_buf[256];
    ota_streamer_t s;
    ota_streamer_init(&s, page_buf, sizeof(page_buf), 4, 8192,
                       sink_write, NULL);

    /* Feed exactly two full pages in one call. */
    uint8_t src[512];
    for (size_t i = 0; i < sizeof(src); i++) src[i] = (uint8_t)i;
    EXPECT(ota_streamer_feed(&s, src, sizeof(src)) == OTA_STREAM_OK,
           "feed 512 bytes");
    EXPECT(ota_streamer_finalize(&s) == OTA_STREAM_OK,
           "finalize");
    EXPECT(sink_len == sizeof(src),  "sink has 512 bytes");
    EXPECT(memcmp(sink, src, sizeof(src)) == 0, "bytes round-trip");
    EXPECT(s.bytes_streamed == 512,  "bytes_streamed == 512");
}

static void case_unaligned_tail(void)
{
    printf("case_unaligned_tail\n");
    sink_reset();

    uint8_t page_buf[256];
    ota_streamer_t s;
    ota_streamer_init(&s, page_buf, sizeof(page_buf), 4, 8192,
                       sink_write, NULL);

    /* 300 bytes = one full page (256) + a 44-byte tail (44 % 4 == 0,
     * so no padding). */
    uint8_t src[300];
    for (size_t i = 0; i < sizeof(src); i++) src[i] = (uint8_t)i;
    EXPECT(ota_streamer_feed(&s, src, sizeof(src)) == OTA_STREAM_OK,
           "feed 300 bytes");
    EXPECT(ota_streamer_finalize(&s) == OTA_STREAM_OK,
           "finalize");
    EXPECT(sink_len == 300,                   "sink has 300 bytes, no padding");
    EXPECT(memcmp(sink, src, sizeof(src)) == 0, "bytes intact");
}

static void case_non_word_aligned_tail_pads(void)
{
    printf("case_non_word_aligned_tail_pads\n");
    sink_reset();

    uint8_t page_buf[256];
    ota_streamer_t s;
    ota_streamer_init(&s, page_buf, sizeof(page_buf), 4, 8192,
                       sink_write, NULL);

    /* 301 bytes: 256 page + 45-byte tail (45 % 4 == 1, needs 3 bytes
     * of padding to reach 48). Padding bytes must be 0xFF. */
    uint8_t src[301];
    for (size_t i = 0; i < sizeof(src); i++) src[i] = (uint8_t)i;
    EXPECT(ota_streamer_feed(&s, src, sizeof(src)) == OTA_STREAM_OK,
           "feed 301 bytes");
    EXPECT(ota_streamer_finalize(&s) == OTA_STREAM_OK,
           "finalize");
    EXPECT(sink_len == 304,
           "sink has 304 bytes (301 + 3 pad)");
    EXPECT(memcmp(sink, src, 301) == 0,
           "first 301 bytes match input");
    EXPECT(sink[301] == 0xFF && sink[302] == 0xFF && sink[303] == 0xFF,
           "tail padded with 0xFF");
    EXPECT(s.bytes_streamed == 301,
           "bytes_streamed excludes pad");
}

static void case_crc_over_unpadded_stream(void)
{
    printf("case_crc_over_unpadded_stream\n");
    sink_reset();

    uint8_t page_buf[256];
    ota_streamer_t s;
    ota_streamer_init(&s, page_buf, sizeof(page_buf), 4, 8192,
                       sink_write, NULL);

    /* Use a length that forces a non-zero pad so a CRC computed over
     * the padded sink would diverge from a CRC over the input. */
    uint8_t src[1023];
    for (size_t i = 0; i < sizeof(src); i++) src[i] = (uint8_t)(i * 31u);

    EXPECT(ota_streamer_feed(&s, src, sizeof(src)) == OTA_STREAM_OK,
           "feed 1023 bytes");
    EXPECT(ota_streamer_finalize(&s) == OTA_STREAM_OK, "finalize");

    uint32_t expected_crc = saint_crc32_update(0, src, sizeof(src));
    EXPECT(s.crc32 == expected_crc,
           "streamer CRC matches input (NOT the padded sink)");

    uint32_t sink_crc = saint_crc32_update(0, sink, sink_len);
    EXPECT(sink_crc != expected_crc,
           "sink CRC differs from input CRC (padding present)");
}

static void case_random_chunk_sizes(void)
{
    printf("case_random_chunk_sizes\n");
    sink_reset();

    uint8_t page_buf[256];
    ota_streamer_t s;
    ota_streamer_init(&s, page_buf, sizeof(page_buf), 4, 8192,
                       sink_write, NULL);

    /* Source: 4 KB of pseudo-random bytes. Feed in chunks of varying
     * size — including 1, 3, 73, 511, 1024 — to exercise the buffer
     * accumulation across recv-boundary configurations the http
     * client really produces. */
    uint8_t src[4096];
    for (size_t i = 0; i < sizeof(src); i++) src[i] = (uint8_t)((i * 7) ^ (i >> 3));
    const size_t splits[] = { 1, 3, 4, 73, 200, 511, 256, 1024, 1024, 1024 };

    size_t off = 0;
    for (size_t i = 0; i < sizeof(splits) / sizeof(splits[0]); i++) {
        size_t take = splits[i];
        if (off + take > sizeof(src)) take = sizeof(src) - off;
        EXPECT(ota_streamer_feed(&s, src + off, take) == OTA_STREAM_OK,
               "feed chunk");
        off += take;
    }
    EXPECT(ota_streamer_finalize(&s) == OTA_STREAM_OK, "finalize");
    EXPECT(s.bytes_streamed == 4096,                  "bytes_streamed == 4096");
    EXPECT(memcmp(sink, src, 4096) == 0,              "round-trip OK");
    EXPECT(s.crc32 == saint_crc32_update(0, src, sizeof(src)),
           "CRC over random-chunked feed matches");
}

static void case_overflow_rejected(void)
{
    printf("case_overflow_rejected\n");
    sink_reset();

    uint8_t page_buf[256];
    ota_streamer_t s;
    ota_streamer_init(&s, page_buf, sizeof(page_buf), 4, /*max=*/100,
                       sink_write, NULL);

    uint8_t src[150];
    memset(src, 0xAB, sizeof(src));
    /* Feed within budget first. */
    EXPECT(ota_streamer_feed(&s, src, 100) == OTA_STREAM_OK, "feed 100/100");
    /* Now one byte too many — must be rejected. */
    EXPECT(ota_streamer_feed(&s, src, 1) == OTA_STREAM_ERR_OVERFLOW,
           "+1 over budget rejected");
}

static void case_rp2040_pad_granularity(void)
{
    printf("case_rp2040_pad_granularity\n");
    sink_reset();

    /* pad_granularity = 256 (RP2040 flash_range_program wants whole
     * pages). Feed 300 bytes — sink should land 256 + 256 = 512 with
     * the second page mostly 0xFF padding. */
    uint8_t page_buf[256];
    ota_streamer_t s;
    ota_streamer_init(&s, page_buf, sizeof(page_buf), /*pad=*/256, 8192,
                       sink_write, NULL);

    uint8_t src[300];
    for (size_t i = 0; i < sizeof(src); i++) src[i] = (uint8_t)i;
    EXPECT(ota_streamer_feed(&s, src, sizeof(src)) == OTA_STREAM_OK,
           "feed 300 bytes");
    EXPECT(ota_streamer_finalize(&s) == OTA_STREAM_OK, "finalize");
    EXPECT(sink_len == 512,  "sink padded out to two full pages");
    EXPECT(memcmp(sink, src, 300) == 0, "first 300 bytes intact");
    bool all_ff = true;
    for (size_t i = 300; i < 512; i++) {
        if (sink[i] != 0xFF) { all_ff = false; break; }
    }
    EXPECT(all_ff, "remaining 212 bytes all 0xFF");
}

static void case_feed_after_finalize_rejected(void)
{
    printf("case_feed_after_finalize_rejected\n");
    sink_reset();

    uint8_t page_buf[256];
    ota_streamer_t s;
    ota_streamer_init(&s, page_buf, sizeof(page_buf), 4, 1024,
                       sink_write, NULL);

    uint8_t b = 0;
    ota_streamer_feed(&s, &b, 1);
    EXPECT(ota_streamer_finalize(&s) == OTA_STREAM_OK, "finalize");
    EXPECT(ota_streamer_feed(&s, &b, 1) == OTA_STREAM_ERR_INVAL,
           "feed after finalize rejected");
    EXPECT(ota_streamer_finalize(&s) == OTA_STREAM_ERR_INVAL,
           "double finalize rejected");
}

static void case_write_failure_propagates(void)
{
    printf("case_write_failure_propagates\n");
    sink_reset();
    fail_next_write = 0;  /* Fail the very first sink_write call. */

    uint8_t page_buf[256];
    ota_streamer_t s;
    ota_streamer_init(&s, page_buf, sizeof(page_buf), 4, 8192,
                       sink_write, NULL);

    /* Feed enough bytes to force a flush. */
    uint8_t buf[256];
    memset(buf, 0xCC, sizeof(buf));
    EXPECT(ota_streamer_feed(&s, buf, sizeof(buf)) == OTA_STREAM_ERR_WRITE,
           "feed propagates write failure");
}

int main(void)
{
    case_aligned_full_pages();
    case_unaligned_tail();
    case_non_word_aligned_tail_pads();
    case_crc_over_unpadded_stream();
    case_random_chunk_sizes();
    case_overflow_rejected();
    case_rp2040_pad_granularity();
    case_feed_after_finalize_rejected();
    case_write_failure_propagates();

    if (fail_count) {
        printf("\ntest_ota_streamer: %d failure(s)\n", fail_count);
        return 1;
    }
    printf("\ntest_ota_streamer: OK\n");
    return 0;
}
