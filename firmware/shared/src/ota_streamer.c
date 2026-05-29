#include "ota_streamer.h"
#include "crc32.h"

#include <string.h>

void ota_streamer_init(ota_streamer_t* s,
                        uint8_t* page_buf, uint32_t page_size,
                        uint32_t pad_granularity,
                        uint32_t max_bytes,
                        ota_stream_write_fn write_fn, void* write_ctx)
{
    memset(s, 0, sizeof(*s));
    s->page_buf        = page_buf;
    s->page_size       = page_size;
    s->pad_granularity = pad_granularity ? pad_granularity : 4u;
    s->max_bytes       = max_bytes;
    s->write_fn        = write_fn;
    s->write_ctx       = write_ctx;
}

/* Flush whatever's in page_buf. If `pad_tail` is true (end-of-body),
 * the residual is padded up to pad_granularity (e.g. 4 on Teensy, 256
 * on RP2040) with 0xFF — outside the bytes_streamed CRC window. */
static ota_stream_status_t flush_page(ota_streamer_t* s, bool pad_tail)
{
    if (s->page_len == 0) return OTA_STREAM_OK;

    uint32_t write_len = s->page_len;
    if (pad_tail) {
        uint32_t g = s->pad_granularity;
        uint32_t pad = (g - (write_len % g)) % g;
        if (pad) {
            memset(s->page_buf + write_len, 0xFF, pad);
            write_len += pad;
        }
    }
    /* If !pad_tail (page filled exactly), write_len == page_size which
     * is itself a multiple of pad_granularity — see init contract. */

    if (s->write_fn(s->write_ctx, s->write_offset, s->page_buf, write_len) != 0) {
        return OTA_STREAM_ERR_WRITE;
    }
    s->write_offset += write_len;
    s->page_len      = 0;
    return OTA_STREAM_OK;
}

ota_stream_status_t ota_streamer_feed(ota_streamer_t* s,
                                       const uint8_t* data, size_t len)
{
    if (!s || !data || s->finalized) return OTA_STREAM_ERR_INVAL;
    if (len == 0) return OTA_STREAM_OK;

    /* Bounds check: refuse to accept more than the staging area can
     * hold. The +s->page_len term accounts for bytes already sitting
     * in the page buffer waiting on the next flush — without it we'd
     * silently over-write the area past max_bytes. */
    if (s->bytes_streamed + len > s->max_bytes) {
        return OTA_STREAM_ERR_OVERFLOW;
    }

    /* CRC is taken over the as-received stream BEFORE any padding so
     * the server-side CRC compares apples-to-apples. */
    s->crc32 = saint_crc32_update(s->crc32, data, len);
    s->bytes_streamed += (uint32_t)len;

    /* Append into page_buf, flushing whenever it fills. */
    while (len > 0) {
        uint32_t free_in_page = s->page_size - s->page_len;
        uint32_t take = (len < (size_t)free_in_page) ? (uint32_t)len : free_in_page;
        memcpy(s->page_buf + s->page_len, data, take);
        s->page_len += take;
        data        += take;
        len         -= take;
        if (s->page_len == s->page_size) {
            ota_stream_status_t st = flush_page(s, false);
            if (st != OTA_STREAM_OK) return st;
        }
    }
    return OTA_STREAM_OK;
}

ota_stream_status_t ota_streamer_finalize(ota_streamer_t* s)
{
    if (!s || s->finalized) return OTA_STREAM_ERR_INVAL;
    ota_stream_status_t st = flush_page(s, true);
    s->finalized = true;
    return st;
}
