/**
 * SAINT.OS Firmware - Minimal HTTP/1.0 GET client (shared)
 *
 * Platform-agnostic GET implementation used by both the RP2040 OTA
 * bootloader (W5500 ioLibrary transport) and the Teensy 4.1 FlasherX
 * integration (NativeEthernet transport). The caller plugs in a
 * transport via function pointers and supplies a body callback that
 * gets each chunk of the response body as it arrives — that callback
 * is where flash writes / CRC accumulation / staging happen.
 *
 * Scope (deliberately tiny):
 *   - HTTP/1.0 GET, Connection: close
 *   - Reads Content-Length and treats it as authoritative
 *   - No chunked transfer-encoding (server must send Content-Length)
 *   - No TLS, no redirects, no keep-alive, no auth
 *
 * If/when we need HTTPS or fancier semantics, this is where to grow
 * (or swap in Mongoose).
 */

#ifndef SAINT_HTTP_CLIENT_H
#define SAINT_HTTP_CLIENT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Transport plug. All callbacks receive the same opaque `ctx` pointer. */
typedef struct http_transport {
    /** Open a TCP connection. Return 0 on success, < 0 on error. */
    int  (*open) (void* ctx, const char* host, uint16_t port);
    /** Send `len` bytes. Return bytes written, or < 0 on error. */
    int  (*send) (void* ctx, const uint8_t* buf, size_t len);
    /** Receive up to `len` bytes. Return bytes read, 0 if peer closed,
     *  or < 0 on error. Should block briefly when no data is available. */
    int  (*recv) (void* ctx, uint8_t* buf, size_t len);
    /** Close the connection. Always called once per successful open(). */
    void (*close)(void* ctx);
    /** Per-instance state for the transport implementation. */
    void* ctx;
} http_transport_t;

/** Body chunk callback. Called once per chunk as the body streams in.
 *  Return 0 to continue, non-zero to abort the download. */
typedef int (*http_body_cb)(void* user, const uint8_t* data, size_t len);

typedef struct http_get_request {
    const char* host;            /* e.g. "192.168.10.1"                  */
    uint16_t    port;            /* e.g. 80                              */
    const char* path;            /* e.g. "/api/firmware/rp2040/saint_node.bin" */
    http_transport_t* transport; /* connection plug                      */
    http_body_cb      body_cb;   /* called with each chunk of the body   */
    void*             user;      /* opaque, passed to body_cb            */
} http_get_request_t;

typedef enum http_status {
    HTTP_OK              =  0,   /* request completed, status 2xx        */
    HTTP_ERR_OPEN        = -1,   /* transport open() failed              */
    HTTP_ERR_SEND        = -2,   /* short/failed send while writing req  */
    HTTP_ERR_RECV        = -3,   /* read error or premature close        */
    HTTP_ERR_HEADERS     = -4,   /* malformed response head              */
    HTTP_ERR_STATUS      = -5,   /* server returned non-2xx              */
    HTTP_ERR_NO_LENGTH   = -6,   /* missing Content-Length               */
    HTTP_ERR_TRUNCATED   = -7,   /* fewer body bytes than Content-Length */
    HTTP_ERR_CALLBACK    = -8,   /* body_cb returned non-zero            */
    HTTP_ERR_INVAL       = -9,   /* bad arguments                        */
} http_status_t;

typedef struct http_get_result {
    http_status_t status;        /* terminal outcome                     */
    int           http_code;     /* HTTP status code if reached (e.g. 200, 404) */
    size_t        content_length;/* declared body size                   */
    size_t        bytes_received;/* body bytes actually streamed         */
} http_get_result_t;

/** Perform a single GET. Streams response body through req->body_cb.
 *  Always closes the transport before returning, on any path. */
http_status_t http_get(const http_get_request_t* req, http_get_result_t* result);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_HTTP_CLIENT_H */
