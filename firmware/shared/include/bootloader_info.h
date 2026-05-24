/**
 * bootloader_info.h — fixed-address descriptor the OTA bootloader
 * stamps into its own flash so the app can read which bootloader is
 * running.
 *
 * Why this exists: the bootloader is NOT OTA-updatable (it's the thing
 * doing updates). Once a board is BOOTSEL-flashed, its bootloader is
 * effectively frozen until the next manual reflash. Without a way to
 * read it back, we have no idea which boards in the field have the
 * current bootloader vs. an older one missing a fix (e.g. the DHCP
 * fix in 80b42b5, the failure-reason fix in 057f389).
 *
 * Mechanism: the bootloader places a const `bootloader_info_t` in a
 * dedicated `.bl_info` section at a fixed flash address near the end
 * of its flash region (see bootloader_shell.ld). The app reads from
 * the same absolute address at boot. The `magic` field gates trust —
 * if the bootloader was built without this descriptor, the app sees
 * an arbitrary flash byte pattern and reports "unknown".
 */

#ifndef SAINT_BOOTLOADER_INFO_H
#define SAINT_BOOTLOADER_INFO_H

#include <stddef.h>
#include <stdint.h>

/* Identifies a valid bootloader-info slot. 0x534F4254 = ASCII "SOBT". */
#define BOOTLOADER_INFO_MAGIC  0x534F4254u

/* Fixed flash address the bootloader places the descriptor at. Last
 * 256 bytes of the bootloader's FLASH region (360k from 0x10000000),
 * sitting just before FLASH_IMGHDR at 0x1005A000. The address is part
 * of the ABI between the bootloader and the app, so don't move it
 * without bumping `struct_version` and updating both sides. */
#define BOOTLOADER_INFO_ADDR   0x10059F00u

/* struct_version bumps on any layout-breaking change. Readers that
 * understand a higher version may still read fields they recognize;
 * readers that see a HIGHER number than they were built against
 * should fall back to "unknown" rather than guessing. */
#define BOOTLOADER_INFO_STRUCT_VERSION  1u

typedef struct {
    uint32_t magic;                 /* BOOTLOADER_INFO_MAGIC                 */
    uint32_t struct_version;        /* BOOTLOADER_INFO_STRUCT_VERSION        */
    char     version_string[48];    /* e.g. "1.0.0-1779153570" — null-term  */
    char     build_timestamp[20];   /* "YYYY-MM-DD HH:MM:SS" — null-term    */
    char     git_hash[16];          /* short git hash — null-term           */
    uint32_t build_unix;            /* unix epoch seconds                    */
    uint32_t _reserved[3];          /* zero — future fields                  */
} bootloader_info_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Returns a pointer to the bootloader's info descriptor, or NULL when
 * the magic doesn't match (older bootloader, or no bootloader at all
 * e.g. simulation / standalone builds). The returned pointer is into
 * memory-mapped flash; do not free or modify. */
static inline const bootloader_info_t* saint_bootloader_info(void)
{
    const bootloader_info_t* info =
        (const bootloader_info_t*)BOOTLOADER_INFO_ADDR;
    if (info->magic != BOOTLOADER_INFO_MAGIC) return NULL;
    return info;
}

#ifdef __cplusplus
}
#endif

#endif /* SAINT_BOOTLOADER_INFO_H */
