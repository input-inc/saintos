#include "crc32.h"

uint32_t saint_crc32_update(uint32_t crc, const uint8_t* buf, size_t len)
{
    crc ^= 0xFFFFFFFFu;
    while (len--) {
        crc ^= *buf++;
        for (int i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1));
        }
    }
    return crc ^ 0xFFFFFFFFu;
}
