import { Injectable, NgZone } from '@angular/core';

/**
 * Global pointer-based drag-scroll behaviour.
 *
 * Browser native pan-scroll only fires for ``pointerType: "touch"`` —
 * but on Steam Deck Game Mode, gamescope's touch-to-cursor emulation
 * makes every screen touch arrive at WebKit as ``pointerType: "mouse"``.
 * WebKit will let the operator drag the scrollbar (mouse path), but
 * dragging directly on the content does nothing. The scrollbar trick
 * is the workaround operators have been using; it stinks.
 *
 * This service installs document-level pointer listeners that emulate
 * pan-scroll regardless of pointer type:
 *   - pointerdown on something inside a scrollable container starts
 *     a drag candidate
 *   - pointermove past a small jitter threshold actually scrolls
 *   - taps on buttons / inputs / links short-circuit cleanly so we
 *     don't hijack click semantics
 *
 * Listening at the document level means we cover every scroll
 * container in the app without each component opting in. Angular
 * dynamically swaps content as the operator navigates; this handler
 * doesn't care, because it walks up from the actual touched element
 * to find a scrollable ancestor every time.
 *
 * Coexists with native scrolling — when the browser DOES handle
 * pan natively (desktop builds, real touch events), our threshold
 * check makes us a no-op for trivial moves, and our scrollTop sets
 * happen to match what the native scroller would have done. We
 * never call preventDefault on the initial pointerdown so click
 * events on buttons inside scroll containers still fire correctly.
 */
@Injectable({ providedIn: 'root' })
export class DragScrollService {
  private scrollEl: HTMLElement | null = null;
  private startY = 0;
  private startScrollTop = 0;
  private startX = 0;
  private startScrollLeft = 0;
  private pointerId: number | null = null;
  private movedPastThreshold = false;

  /** Minimum pointer travel (in CSS px) before we treat the gesture
   *  as a drag-scroll. Smaller deltas pass through as taps so a
   *  finger that lands on a button doesn't accidentally start a
   *  scroll. 5 px is the same threshold most touch-scroll libs use. */
  private readonly DRAG_THRESHOLD_PX = 5;

  /** CSS selector for elements whose clicks we must NEVER hijack. If
   *  the touch lands inside any of these, we don't even start the
   *  drag candidate. Keep tight — false positives here block
   *  scrolling on common content. */
  private readonly INTERACTIVE_SELECTOR =
    'button, input, select, textarea, a, ' +
    '[role="button"], [role="slider"], [role="switch"], [contenteditable="true"]';

  constructor(private zone: NgZone) {}

  /** Call once at app bootstrap. */
  install(): void {
    // Outside Angular's zone — pointer events fire at high frequency
    // and we don't want each one to trigger change detection. The
    // service mutates DOM scroll positions directly; Angular state
    // isn't touched.
    this.zone.runOutsideAngular(() => {
      document.addEventListener('pointerdown', this.onDown, { passive: true });
      document.addEventListener('pointermove', this.onMove, { passive: false });
      document.addEventListener('pointerup', this.onUp);
      document.addEventListener('pointercancel', this.onUp);
    });
  }

  private onDown = (e: PointerEvent): void => {
    const target = e.target as HTMLElement | null;
    if (!target) return;
    // Don't capture drags that start on a control. Buttons and
    // form fields handle their own pointer semantics; hijacking
    // them breaks taps, focus, and (for sliders) live tracking.
    if (target.closest(this.INTERACTIVE_SELECTOR)) return;

    const scrollable = this.findScrollableAncestor(target);
    if (!scrollable) return;

    this.scrollEl = scrollable;
    this.startY = e.clientY;
    this.startX = e.clientX;
    this.startScrollTop = scrollable.scrollTop;
    this.startScrollLeft = scrollable.scrollLeft;
    this.pointerId = e.pointerId;
    this.movedPastThreshold = false;
  };

  private onMove = (e: PointerEvent): void => {
    if (this.scrollEl === null) return;
    if (e.pointerId !== this.pointerId) return;

    const dy = e.clientY - this.startY;
    const dx = e.clientX - this.startX;

    if (!this.movedPastThreshold) {
      // Below threshold: don't start scrolling yet. This is what
      // lets a button INSIDE a scroll container still receive its
      // click — small finger jitter between down and up doesn't
      // become a scroll.
      if (Math.abs(dy) < this.DRAG_THRESHOLD_PX &&
          Math.abs(dx) < this.DRAG_THRESHOLD_PX) {
        return;
      }
      this.movedPastThreshold = true;
    }

    // Past the threshold: scroll AND swallow the event so it doesn't
    // bubble into native scroll handlers (which would either fight
    // us or, on desktop, double-apply).
    this.scrollEl.scrollTop = this.startScrollTop - dy;
    this.scrollEl.scrollLeft = this.startScrollLeft - dx;
    e.preventDefault();
  };

  private onUp = (e: PointerEvent): void => {
    if (e.pointerId !== this.pointerId) return;
    this.scrollEl = null;
    this.pointerId = null;
    this.movedPastThreshold = false;
  };

  /** Walk from `el` up the parent chain looking for the first
   *  ancestor that's actually scrollable on either axis. "Scrollable"
   *  = computed overflow is auto/scroll AND content exceeds the
   *  client size on that axis. Returns null when no such ancestor
   *  exists (e.g. drags inside fully-visible content). */
  private findScrollableAncestor(el: HTMLElement | null): HTMLElement | null {
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
}
