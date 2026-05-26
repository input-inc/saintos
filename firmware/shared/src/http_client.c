/**
 * SAINT.OS Firmware - Minimal HTTP/1.0 GET client (shared)
 *
 * See http_client.h for the API contract. This implementation:
 *   1. transport->open(host, port)
 *   2. send a fixed GET request (HTTP/1.0 + Connection: close + Host)
 *   3. recv into a small line-buffer; parse status line + headers
 *      looking for Content-Length
 *   4. stream the remaining body to body_cb in ~512-byte chunks
 *   5. transport->close()
 *
 * Memory: ~640 bytes of stack (line buffer 256 + recv chunk 512 + locals).
 * No heap.
 */

#include "http_client.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HC_LINE_BUF_SIZE   256   /* enough for any reasonable header line */
#define HC_RECV_BUF_SIZE   512   /* body chunk size — bigger = fewer syscalls */
#define HC_RECV_TIMEOUT_TRIES 2000  /* recv() retries before giving up */

/* Case-insensitive prefix match. */
static int hc_iprefix(const char* s, const char* prefix)
{
    while (*prefix) {
        int a = *s++;
        int b = *prefix++;
        if (a >= 'A' && a <= 'Z') a += 32;
        if (b >= 'A' && b <= 'Z') b += 32;
        if (a != b) return 0;
    }
    return 1;
}

/* Read one full byte from the transport with light retry on transient
 * zero returns. Returns 1 on byte read, 0 on EOF, -1 on error. */
static int hc_recv_one(http_transport_t* t, uint8_t* out)
{
    int retries = HC_RECV_TIMEOUT_TRIES;
    while (retries-- > 0) {
        int n = t->recv(t->ctx, out, 1);
        if (n > 0) return 1;
        if (n == 0) {
            /* transient: nothing available right now */
            continue;
        }
        return -1;
    }
    return 0;  /* deadline */
}

/* Read up to `cap-1` bytes until '\n', null-terminate, strip trailing CR.
 * Returns length (0 for empty line) or -1 on read error. */
static int hc_read_line(http_transport_t* t, char* buf, size_t cap)
{
    size_t n = 0;
    while (n < cap - 1) {
        uint8_t b;
        int r = hc_recv_one(t, &b);
        if (r < 0) return -1;
        if (r == 0) {
            if (n == 0) return -1;  /* premature close */
            break;
        }
        if (b == '\n') break;
        buf[n++] = (char)b;
    }
    /* Strip trailing CR */
    if (n > 0 && buf[n - 1] == '\r') n--;
    buf[n] = '\0';
    return (int)n;
}

/* Robust send of the entire buffer. Returns 0 on success, < 0 on failure. */
static int hc_send_all(http_transport_t* t, const uint8_t* buf, size_t len)
{
    while (len > 0) {
        int n = t->send(t->ctx, buf, len);
        if (n <= 0) return -1;
        buf += n;
        len -= (size_t)n;
    }
    return 0;
}

http_status_t http_get(const http_get_request_t* req, http_get_result_t* result)
{
    if (!req || !req->host || !req->path || !req->transport || !req->body_cb) {
        if (result) {
            result->status = HTTP_ERR_INVAL;
            result->http_code = 0;
            result->content_length = 0;
            result->bytes_received = 0;
        }
        return HTTP_ERR_INVAL;
    }

    http_transport_t* t = req->transport;
    http_get_result_t r = { HTTP_OK, 0, 0, 0 };

    if (t->open(t->ctx, req->host, req->port) < 0) {
        r.status = HTTP_ERR_OPEN;
        if (result) *result = r;
        return r.status;
    }

    /* Build and send the request. */
    char req_buf[HC_LINE_BUF_SIZE];
    int req_len = snprintf(req_buf, sizeof(req_buf),
        "GET %s HTTP/1.0\r\n"
        "Host: %s\r\n"
        "User-Agent: saintos-ota/1\r\n"
        "Connection: close\r\n"
        "\r\n",
        req->path, req->host);
    if (req_len <= 0 || (size_t)req_len >= sizeof(req_buf)) {
        r.status = HTTP_ERR_INVAL;
        goto done;
    }
    if (hc_send_all(t, (const uint8_t*)req_buf, (size_t)req_len) < 0) {
        r.status = HTTP_ERR_SEND;
        goto done;
    }

    /* Parse status line: "HTTP/1.x CODE TEXT" */
    char line[HC_LINE_BUF_SIZE];
    int line_len = hc_read_line(t, line, sizeof(line));
    if (line_len < 0) {
        r.status = HTTP_ERR_RECV;
        goto done;
    }
    if (!hc_iprefix(line, "HTTP/1.")) {
        r.status = HTTP_ERR_HEADERS;
        goto done;
    }
    const char* sp = strchr(line, ' ');
    if (!sp) {
        r.status = HTTP_ERR_HEADERS;
        goto done;
    }
    r.http_code = atoi(sp + 1);
    if (r.http_code < 200 || r.http_code >= 300) {
        r.status = HTTP_ERR_STATUS;
        goto done;
    }

    /* Read header lines until blank line. Extract Content-Length. */
    bool have_content_length = false;
    for (;;) {
        line_len = hc_read_line(t, line, sizeof(line));
        if (line_len < 0) {
            r.status = HTTP_ERR_RECV;
            goto done;
        }
        if (line_len == 0) break;  /* end of headers */
        if (hc_iprefix(line, "Content-Length:")) {
            const char* v = line + strlen("Content-Length:");
            while (*v == ' ' || *v == '\t') v++;
            r.content_length = (size_t)strtoul(v, NULL, 10);
            have_content_length = true;
        }
    }
    if (!have_content_length) {
        r.status = HTTP_ERR_NO_LENGTH;
        goto done;
    }

    /* Stream the body. Each chunk goes through body_cb. */
    uint8_t chunk[HC_RECV_BUF_SIZE];
    while (r.bytes_received < r.content_length) {
        size_t want = r.content_length - r.bytes_received;
        if (want > sizeof(chunk)) want = sizeof(chunk);
        int n = t->recv(t->ctx, chunk, want);
        if (n < 0) {
            r.status = HTTP_ERR_RECV;
            goto done;
        }
        if (n == 0) {
            /* peer closed before we got everything */
            r.status = HTTP_ERR_TRUNCATED;
            goto done;
        }
        if (req->body_cb(req->user, chunk, (size_t)n) != 0) {
            r.status = HTTP_ERR_CALLBACK;
            goto done;
        }
        r.bytes_received += (size_t)n;
    }

    r.status = HTTP_OK;

done:
    t->close(t->ctx);
    if (result) *result = r;
    return r.status;
}
