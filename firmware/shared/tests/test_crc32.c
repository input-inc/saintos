/**
 * Host-runnable test for shared CRC32.
 *
 * The shared saint_crc32_update is what gen_imghdr.py, the server's
 * firmware-info computation, and the RP2040/Teensy in-app OTA paths all
 * trust to agree byte-for-byte. A divergence here would silently
 * corrupt a downloaded image without tripping any error. Test against
 * the well-known IEEE 802.3 / zlib values to lock in the polynomial,
 * the init value, and the xorout — those are the three knobs that
 * would let a "looks reasonable" implementation drift.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "../src/crc32.c"

static int fail_count = 0;

#define EXPECT_EQ_U32(actual, expected, label)                                 \
    do {                                                                        \
        uint32_t a = (uint32_t)(actual);                                        \
        uint32_t e = (uint32_t)(expected);                                      \
        if (a != e) {                                                           \
            printf("  FAIL %s: got 0x%08X expected 0x%08X\n", label, a, e);     \
            fail_count++;                                                       \
        } else {                                                                \
            printf("  ok   %s = 0x%08X\n", label, a);                           \
        }                                                                       \
    } while (0)

int main(void)
{
    printf("test_crc32\n");

    /* Empty input: CRC32(empty) is the documented identity value 0. */
    EXPECT_EQ_U32(saint_crc32_update(0, (const uint8_t*)"", 0), 0u, "empty");

    /* The standard "check" string from the CRC-32/ISO-HDLC reference
     * vector — every IEEE 802.3 CRC32 must produce 0xCBF43926 here. */
    EXPECT_EQ_U32(saint_crc32_update(0, (const uint8_t*)"123456789", 9),
                  0xCBF43926u, "\"123456789\"");

    /* 'a' alone: zlib's classic single-byte vector. */
    EXPECT_EQ_U32(saint_crc32_update(0, (const uint8_t*)"a", 1),
                  0xE8B7BE43u, "\"a\"");

    /* Incremental update must equal one-shot — the running CRC has to
     * be a true intermediate state, not a bookkeeping value the
     * implementation re-zeroes between calls. */
    {
        const uint8_t* msg = (const uint8_t*)"123456789";
        uint32_t c = saint_crc32_update(0, msg, 4);
        c = saint_crc32_update(c, msg + 4, 5);
        EXPECT_EQ_U32(c, 0xCBF43926u, "incremental(4+5) == one-shot(9)");
    }

    /* Many small chunks vs one big chunk — byte-at-a-time accumulation
     * is the form the OTA body callback actually uses. */
    {
        const char* msg = "The quick brown fox jumps over the lazy dog";
        size_t len = strlen(msg);
        uint32_t one_shot = saint_crc32_update(0, (const uint8_t*)msg, len);
        uint32_t streamed = 0;
        for (size_t i = 0; i < len; i++) {
            streamed = saint_crc32_update(streamed,
                                           (const uint8_t*)msg + i, 1);
        }
        EXPECT_EQ_U32(streamed, one_shot, "byte-by-byte == one-shot");
        /* Well-known CRC for this exact string. */
        EXPECT_EQ_U32(one_shot, 0x414FA339u, "\"quick brown fox\"");
    }

    /* Large buffer (4 KiB of 0xAA) — exercises the polynomial enough
     * times that a wrong shift direction or polynomial constant would
     * diverge sharply. Value pre-computed with python -c
     * "import zlib; print(hex(zlib.crc32(b'\\xaa' * 4096)))". */
    {
        uint8_t buf[4096];
        memset(buf, 0xAA, sizeof(buf));
        EXPECT_EQ_U32(saint_crc32_update(0, buf, sizeof(buf)),
                      0x55BCB83Cu, "4 KiB of 0xAA");
    }

    if (fail_count) {
        printf("\ntest_crc32: %d failure(s)\n", fail_count);
        return 1;
    }
    printf("\ntest_crc32: OK\n");
    return 0;
}
