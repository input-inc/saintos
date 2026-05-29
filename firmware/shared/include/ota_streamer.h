/**
 * SAINT.OS Firmware - OTA download streamer (shared)
 *
 * Both the RP2040 OTA bootloader and the Teensy in-app OTA path do the
 * same thing while a firmware image is downloading: accumulate
 * arbitrary-sized HTTP body chunks into a fixed page buffer, flush to
 * flash in whole pages, accumulate a CRC32 over the un-padded byte
 * stream, and pad the residual tail when the final chunk lands mid-
 * page. This module owns that bookkeeping so the platform-specific
 * code is just the flash-write callback.
 *
 * Why bother extracting:
 *   - flash_write_block on iMXRT1062 (Teensy) rejects any call with
 *     count % 4 != 0. http_client.c hands the body in whatever chunks
 *     the TCP recv returned, which is almost never aligned. The
 *     buffer-then-flush pattern is the canonical fix on both
 *     platforms; keeping one implementation means a regression there
 *     can only happen in one place.
 *   - CRC32 is computed over the raw stream BEFORE padding, so the
 *     server-side CRC and the in-firmware CRC compare apples-to-
 *     apples regardless of how the recv chunks fell.
 *
 * Usage:
 *   ota_streamer_t s;
 *   ota_streamer_init(&s, page_buf, page_size, max_bytes, write_fn, write_ctx);
 *   // for each HTTP body chunk:
 *   if (ota_streamer_feed(&s, data, len) != OTA_STREAM_OK) bail();
 *   // at end-of-body:
 *   if (ota_streamer_finalize(&s) != OTA_STREAM_OK) bail();
 *   // verify size + CRC:
 *   if (s.bytes_streamed != expected_size) bail();
 *   if (s.crc32 != expected_crc) bail();
 *
 * page_buf must outlive the streamer. page_size must be a multiple of
 * the platform's flash write granularity (4 on iMXRT; 256 / FLASH_PAGE_SIZE
 * on RP2040 — the streamer just needs it to be a multiple of 4 so the
 * tail-pad math works out).
 */

#ifndef SAINT_OTA_STREAMER_H
#define SAINT_OTA_STREAMER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OTA_STREAM_OK            =  0,
    OTA_STREAM_ERR_OVERFLOW  = -1,  /* fed more than max_bytes               */
    OTA_STREAM_ERR_WRITE     = -2,  /* write_fn returned non-zero            */
    OTA_STREAM_ERR_INVAL     = -3,  /* bad arguments to init/feed/finalize   */
} ota_stream_status_t;

/** Platform-supplied flash write hook. Called with a buffer whose
 *  length is always a multiple of 4. write_offset is the offset from
 *  the streamer's logical zero (0 on first call, growing by `len` on
 *  each subsequent call). Return 0 on success, non-zero to abort. */
typedef int (*ota_stream_write_fn)(void*    write_ctx,
                                    uint32_t write_offset,
                                    const uint8_t* data,
                                    uint32_t len);

typedef struct ota_streamer {
    /* Configuration — set by ota_streamer_init, then read-only. */
    uint8_t*  page_buf;
    uint32_t  page_size;       /* size of page_buf in bytes — multiple of 4 */
    uint32_t  pad_granularity; /* tail padded up to a multiple of this      */
    uint32_t  max_bytes;       /* refuse to accept more than this many bytes */
    ota_stream_write_fn write_fn;
    void*     write_ctx;

    /* Running state. */
    uint32_t  page_len;        /* bytes currently sitting in page_buf       */
    uint32_t  write_offset;    /* total bytes already flushed to write_fn   */
    uint32_t  bytes_streamed;  /* total bytes received (pre-padding)        */
    uint32_t  crc32;           /* running CRC32 over the un-padded stream   */
    bool      finalized;
} ota_streamer_t;

/* pad_granularity: the platform-write minimum unit. Teensy iMXRT must
 * pass multiples of 4 to flash_write_block, so 4 is the right value.
 * RP2040 flash_range_program requires whole FLASH_PAGE_SIZE (256-byte)
 * pages, so 256 is the right value there. Must divide page_size. */
void ota_streamer_init(ota_streamer_t* s,
                        uint8_t* page_buf, uint32_t page_size,
                        uint32_t pad_granularity,
                        uint32_t max_bytes,
                        ota_stream_write_fn write_fn, void* write_ctx);

/** Feed one HTTP body chunk through the streamer. The page buffer is
 *  flushed (in write_fn calls of page_size each) every time it fills.
 *  CRC32 is updated over `data` before any buffering. */
ota_stream_status_t ota_streamer_feed(ota_streamer_t* s,
                                       const uint8_t* data, size_t len);

/** Call once at end-of-body. Flushes whatever's left in the page
 *  buffer, padding the tail to a 4-byte boundary with 0xFF (erased-
 *  flash value) so the platform write_fn — which on iMXRT must take
 *  multiples of 4 — accepts it. The padding bytes are OUTSIDE the
 *  bytes_streamed window the CRC was computed over, so verification
 *  is unaffected. */
ota_stream_status_t ota_streamer_finalize(ota_streamer_t* s);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_OTA_STREAMER_H */
