/**
 * Drag-scroll composable — Vue equivalent of the Angular
 * `DragScrollService`.
 *
 * Browser native pan-scroll only fires for `pointerType: "touch"`, but
 * on Steam Deck Game Mode gamescope's touch-to-cursor emulation makes
 * every screen touch arrive at WebKit as `pointerType: "mouse"`. This
 * module installs document-level pointer listeners that emulate
 * pan-scroll regardless of pointer type:
 *   - pointerdown on something inside a scrollable container starts
 *     a drag candidate
 *   - pointermove past a small jitter threshold actually scrolls
 *   - taps on buttons / inputs / links short-circuit cleanly so we
 *     don't hijack click semantics
 *
 * Call `installDragScroll()` once at app bootstrap (from `main.ts` or
 * `App.vue` mounted). Subsequent calls are no-ops — the listeners are
 * idempotent.
 *
 * No reactive state — this is a pure DOM side-effect, not something
 * components need to observe. Hence no `useDragScroll()` composable;
 * just a single install function.
 */

let scrollEl: HTMLElement | null = null;
let startY = 0;
let startScrollTop = 0;
let startX = 0;
let startScrollLeft = 0;
let pointerId: number | null = null;
let movedPastThreshold = false;

const DRAG_THRESHOLD_PX = 5;

const INTERACTIVE_SELECTOR =
    'button, input, select, textarea, a, ' +
    '[role="button"], [role="slider"], [role="switch"], [contenteditable="true"]';

function onDown(e: PointerEvent): void {
    const target = e.target as HTMLElement | null;
    if (!target) return;
    // Don't capture drags that start on a control. Buttons and form
    // fields handle their own pointer semantics; hijacking them
    // breaks taps, focus, and (for sliders) live tracking.
    if (target.closest(INTERACTIVE_SELECTOR)) return;

    const scrollable = findScrollableAncestor(target);
    if (!scrollable) return;

    scrollEl = scrollable;
    startY = e.clientY;
    startX = e.clientX;
    startScrollTop = scrollable.scrollTop;
    startScrollLeft = scrollable.scrollLeft;
    pointerId = e.pointerId;
    movedPastThreshold = false;
}

function onMove(e: PointerEvent): void {
    if (scrollEl === null) return;
    if (e.pointerId !== pointerId) return;

    const dy = e.clientY - startY;
    const dx = e.clientX - startX;

    if (!movedPastThreshold) {
        // Below threshold: don't scroll yet. This is what lets a
        // button INSIDE a scroll container still receive its click —
        // small finger jitter between down and up doesn't become
        // a scroll.
        if (Math.abs(dy) < DRAG_THRESHOLD_PX && Math.abs(dx) < DRAG_THRESHOLD_PX) {
            return;
        }
        movedPastThreshold = true;
    }

    scrollEl.scrollTop = startScrollTop - dy;
    scrollEl.scrollLeft = startScrollLeft - dx;
    e.preventDefault();
}

function onUp(e: PointerEvent): void {
    if (e.pointerId !== pointerId) return;
    scrollEl = null;
    pointerId = null;
    movedPastThreshold = false;
}

/** Walk from `el` up the parent chain looking for the first ancestor
 *  that's actually scrollable on either axis. "Scrollable" = computed
 *  overflow is auto/scroll AND content exceeds the client size on
 *  that axis. */
function findScrollableAncestor(el: HTMLElement | null): HTMLElement | null {
    while (el && el !== document.body) {
        const style = getComputedStyle(el);
        const overflowY = style.overflowY;
        const overflowX = style.overflowX;
        if ((overflowY === 'auto' || overflowY === 'scroll') &&
            el.scrollHeight > el.clientHeight) {
            return el;
        }
        if ((overflowX === 'auto' || overflowX === 'scroll') &&
            el.scrollWidth > el.clientWidth) {
            return el;
        }
        el = el.parentElement;
    }
    return null;
}

let installed = false;

export function installDragScroll(): void {
    if (installed) return;
    installed = true;
    document.addEventListener('pointerdown', onDown, { passive: true });
    document.addEventListener('pointermove', onMove, { passive: false });
    document.addEventListener('pointerup', onUp);
    document.addEventListener('pointercancel', onUp);
}
