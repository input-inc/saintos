/**
 * Host-runnable test for shared saint_log.
 *
 * Covers the bug-prone bits:
 *   - pre-ros calls enqueue, post-ros calls publish
 *   - drain replays in order, only once we're ros_ready
 *   - drain is idempotent
 *   - JSON envelope escapes quotes, backslash, \n, \r, \t, and other
 *     control bytes (the embedded-newline bug that bit the RoboClaw
 *     version log line — server's json.loads would bail with "Invalid
 *     control character at line 1 column N" if any escape regresses)
 *   - saint_log_emit_local always fires regardless of ros_ready
 *
 * The per-platform hooks (saint_log_emit_local, saint_log_emit_ros,
 * saint_log_uptime_ms) are stubbed below so we can observe what the
 * shared layer hands them.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

/* ── Per-platform hook stubs (the shared module calls into these) ─── */

#define MAX_CAPTURED 64
#define CAP_LINE_LEN 512

static char captured_local[MAX_CAPTURED][CAP_LINE_LEN];
static int  captured_local_count = 0;

static char captured_ros[MAX_CAPTURED][CAP_LINE_LEN];
static int  captured_ros_count = 0;

static uint32_t fake_uptime_ms = 1234;

void saint_log_emit_local(const char* level, const char* text)
{
    if (captured_local_count >= MAX_CAPTURED) return;
    snprintf(captured_local[captured_local_count],
             sizeof(captured_local[0]),
             "[%s] %s", level, text);
    captured_local_count++;
}

bool saint_log_emit_ros(const char* json, size_t len)
{
    if (captured_ros_count >= MAX_CAPTURED) return true;
    size_t n = len < sizeof(captured_ros[0]) - 1
               ? len : sizeof(captured_ros[0]) - 1;
    memcpy(captured_ros[captured_ros_count], json, n);
    captured_ros[captured_ros_count][n] = '\0';
    captured_ros_count++;
    return true;
}

uint32_t saint_log_uptime_ms(void)
{
    return fake_uptime_ms;
}

/* Now include the unit under test so its file-scope statics get
 * defined inside our translation unit. */
#include "../src/saint_log.c"

/* ── Test plumbing ────────────────────────────────────────────────── */

static int fail_count = 0;
#define EXPECT(cond, label)                                                    \
    do {                                                                        \
        if (cond) { printf("  ok   %s\n", label); }                             \
        else { printf("  FAIL %s\n", label); fail_count++; }                    \
    } while (0)

static void reset_capture(void)
{
    captured_local_count = 0;
    captured_ros_count   = 0;
    memset(captured_local, 0, sizeof(captured_local));
    memset(captured_ros,   0, sizeof(captured_ros));
}

/* Force the shared module back to a clean state between cases. The
 * statics live in saint_log.c — we reach in (same TU via the #include).
 *
 * The boot-log model is now a ring buffer (g_pending_head/tail/dropped)
 * drained one entry per call with a pacer, NOT the old
 * count+flushed-flag. Reset the ring indices, the ready flag, and the
 * pacer timestamp so each case starts fresh. */
static void reset_shared_state(void)
{
    g_pending_head    = 0;
    g_pending_tail    = 0;
    g_pending_dropped = 0;
    g_ros_ready       = false;
    g_last_publish_ms = 0;
    fake_uptime_ms    = 1000;   /* nonzero base so the pacer math is clean */
}

/* Drain the whole pending queue.
 *
 * The shared drain (saint_log_drain_pending) pops at most ONE entry per
 * call and enforces SAINT_LOG_MIN_PUBLISH_INTERVAL_MS between publishes,
 * gated on the stubbed saint_log_uptime_ms() (== fake_uptime_ms). We
 * advance the fake clock past the interval before each attempt so the
 * pacer never blocks, and stop when a call pops nothing (queue empty or
 * not ros-ready). Returns how many entries this drain emitted. */
static int drain_all(void)
{
    int before = captured_ros_count;
    for (int guard = 0; guard < BOOT_LOG_MAX * 2 + 8; guard++) {
        fake_uptime_ms += SAINT_LOG_MIN_PUBLISH_INTERVAL_MS;
        if (!saint_log_drain_pending()) break;
    }
    return captured_ros_count - before;
}

/* ── Cases ────────────────────────────────────────────────────────── */

static void case_local_always_fires(void)
{
    printf("case_local_always_fires\n");
    reset_capture(); reset_shared_state();

    saint_log_publish("info", "hello");
    EXPECT(captured_local_count == 1, "local fired");
    EXPECT(strcmp(captured_local[0], "[info] hello") == 0,
           "local has '[info] hello'");
    EXPECT(captured_ros_count == 0, "no ros publish (not ready)");
}

static void case_drain_replays_in_order(void)
{
    printf("case_drain_replays_in_order\n");
    reset_capture(); reset_shared_state();

    saint_log_publish("info",  "first");
    saint_log_publish("warn",  "second");
    saint_log_publish("error", "third");
    EXPECT(captured_ros_count == 0, "no ros publish before ready");

    saint_log_set_ros_ready(true);

    /* set_ros_ready alone doesn't drain — the announce gate is the
     * caller's responsibility. */
    EXPECT(captured_ros_count == 0, "still no ros publish before drain");

    EXPECT(drain_all() == 3, "3 entries drained");
    EXPECT(strstr(captured_ros[0], "\"first\"")  != NULL, "first  in slot 0");
    EXPECT(strstr(captured_ros[1], "\"second\"") != NULL, "second in slot 1");
    EXPECT(strstr(captured_ros[2], "\"third\"")  != NULL, "third  in slot 2");
}

static void case_drain_is_idempotent(void)
{
    printf("case_drain_is_idempotent\n");
    reset_capture(); reset_shared_state();

    saint_log_publish("info", "only");
    saint_log_set_ros_ready(true);
    EXPECT(drain_all() == 1, "first drain emits");
    EXPECT(drain_all() == 0, "second drain is no-op (queue empty)");
    EXPECT(captured_ros_count == 1, "still just the one entry");
}

static void case_publish_always_enqueues_even_when_ready(void)
{
    printf("case_publish_always_enqueues_even_when_ready\n");
    reset_capture(); reset_shared_state();

    /* The old "if ros_ready, publish inline" fast path was REMOVED — it
     * published from callback context and produced empty BEST_EFFORT
     * frames. saint_log_publish now always enqueues; the main-loop
     * drain is the only thing that emits. So even when ready, a publish
     * does NOT emit inline — it waits for the drain. */
    saint_log_set_ros_ready(true);
    saint_log_publish("info", "direct");
    EXPECT(captured_ros_count == 0, "publish enqueues, does not emit inline");
    EXPECT(drain_all() == 1, "drain emits the enqueued entry");
    EXPECT(strstr(captured_ros[0], "\"direct\"") != NULL, "envelope has text");
}

static void case_uptime_lands_in_envelope(void)
{
    printf("case_uptime_lands_in_envelope\n");
    reset_capture(); reset_shared_state();

    saint_log_set_ros_ready(true);
    saint_log_publish("info", "x");
    /* uptime in the envelope is stamped at PUBLISH (drain) time, not
     * enqueue time. drain_all advances the fake clock past the pacer, so
     * pin the clock to a known value right before the (single) drain. */
    fake_uptime_ms = 4242 - SAINT_LOG_MIN_PUBLISH_INTERVAL_MS;
    EXPECT(drain_all() == 1, "one entry drained");
    EXPECT(strstr(captured_ros[0], "\"uptime_ms\":4242") != NULL,
           "uptime_ms field stamped at publish time");
}

static void case_json_escapes(void)
{
    printf("case_json_escapes\n");
    reset_capture(); reset_shared_state();

    saint_log_set_ros_ready(true);

    /* Embedded newline → \n. Real-world: the RoboClaw GETVERSION
     * response ends with a literal \n; without escaping the server's
     * json.loads bails with "Invalid control character at line 1 col N". */
    saint_log_publish("info", "v4.4.8\n");
    drain_all();   /* publish enqueues; drain emits + escapes */
    EXPECT(strstr(captured_ros[0], "\\n")  != NULL, "newline -> \\n");
    EXPECT(strchr(captured_ros[0], '\n')   == NULL, "no literal newline in envelope");

    /* Quote and backslash: standard JSON requirements. */
    reset_capture();
    saint_log_publish("info", "quoted \"x\" and a \\back");
    drain_all();
    EXPECT(strstr(captured_ros[0], "\\\"x\\\"") != NULL, "quote escaped");
    EXPECT(strstr(captured_ros[0], "\\\\back")  != NULL, "backslash escaped");

    /* Tab + CR */
    reset_capture();
    saint_log_publish("info", "tab:\there\rcr");
    drain_all();
    EXPECT(strstr(captured_ros[0], "\\t")  != NULL, "tab -> \\t");
    EXPECT(strstr(captured_ros[0], "\\r")  != NULL, "CR  -> \\r");

    /* SOH (0x01): not in the common-escape set, must come out as
     * . Rare in practice but the implementation's catch-all
     * path needs verification — otherwise an exotic byte breaks JSON. */
    reset_capture();
    char raw[8] = { 'a', 0x01, 'b', '\0', 0, 0, 0, 0 };
    saint_log_publish("info", raw);
    drain_all();
    EXPECT(strstr(captured_ros[0], "\\u0001") != NULL, "0x01 -> \\u0001");
}

static void case_buffer_is_bounded(void)
{
    printf("case_buffer_is_bounded\n");
    reset_capture(); reset_shared_state();

    /* Ring buffer: push more than it holds. When full it drops the
     * OLDEST entry (keeps the most recent context — see enqueue_pending
     * in saint_log.c). Usable capacity is BOOT_LOG_MAX-1 (head==tail is
     * the empty sentinel). So the newest lines survive and the earliest
     * are dropped — the opposite of the old reject-when-full model. */
    const int pushed = BOOT_LOG_MAX + 5;
    for (int i = 0; i < pushed; i++) {
        saint_log_publish("info", "line %d", i);
    }
    saint_log_set_ros_ready(true);
    int drained = drain_all();
    EXPECT(drained == BOOT_LOG_MAX - 1,
           "drain yields ring capacity (BOOT_LOG_MAX-1)");
    EXPECT(g_pending_dropped == (uint32_t)(pushed - (BOOT_LOG_MAX - 1)),
           "dropped count = overflow amount");
    /* Newest survived, oldest evicted. */
    char newest[32];
    snprintf(newest, sizeof(newest), "\"line %d\"", pushed - 1);
    EXPECT(strstr(captured_ros[drained - 1], newest) != NULL,
           "newest line survived (last drained)");
    EXPECT(strstr(captured_ros[0], "\"line 0\"") == NULL,
           "earliest line was dropped");
}

static void case_boot_queue_helper(void)
{
    printf("case_boot_queue_helper\n");
    reset_capture(); reset_shared_state();

    /* saint_log_boot_queue must ALWAYS buffer, even if ros is already
     * ready — that's the contract the explicit pre-init call sites in
     * main.c depend on (they want the line replayed AFTER announce
     * so the server's subscription is established). */
    saint_log_set_ros_ready(true);
    saint_log_boot_queue("info", "deferred");
    EXPECT(captured_ros_count == 0, "boot_queue does not auto-publish");

    EXPECT(drain_all() == 1, "drain replays the deferred line");
    EXPECT(strstr(captured_ros[0], "\"deferred\"") != NULL, "text round-trips");
}

int main(void)
{
    case_local_always_fires();
    case_drain_replays_in_order();
    case_drain_is_idempotent();
    case_publish_always_enqueues_even_when_ready();
    case_uptime_lands_in_envelope();
    case_json_escapes();
    case_buffer_is_bounded();
    case_boot_queue_helper();

    if (fail_count) {
        printf("\ntest_saint_log: %d failure(s)\n", fail_count);
        return 1;
    }
    printf("\ntest_saint_log: OK\n");
    return 0;
}
