import { Injectable, signal } from '@angular/core';

/**
 * Service to manage the built-in virtual keyboard.
 *
 * Shows an on-screen keyboard when text inputs are focused,
 * optimized for gamepad/touch input on Steam Deck.
 */
/** localStorage key that persists the operator's choice between the
 *  in-app virtual keyboard and the platform OSK (Steam OSK in Game
 *  Mode, KDE virtual keyboard in Desktop Mode, etc.). Survives app
 *  restart; settings panel reads + writes through setEnabled below. */
const STORAGE_KEY = 'saint-controller-osk-enabled';

@Injectable({
  providedIn: 'root'
})
export class KeyboardService {
  // Keyboard visibility state
  private _isVisible = signal(false);
  isVisible = this._isVisible.asReadonly();

  /** Whether the in-app virtual keyboard is enabled. Default OFF:
   *  on Steam Deck Game Mode the Steam OSK (Steam + X) is more
   *  discoverable and integrates with Steam Input. In Desktop Mode
   *  the user typically has a physical or KDE virtual keyboard. When
   *  this is false:
   *    - Focus-in events on text fields do NOT open the in-app keyboard
   *    - `show()` is a no-op
   *    - If the keyboard happens to be visible when toggled off, we
   *      hide it immediately so the operator isn't stuck.
   *  The setting persists in localStorage so the choice survives an
   *  app restart, and the settings panel can toggle it from a switch. */
  private _enabled = signal(this.loadEnabled());
  enabled = this._enabled.asReadonly();

  // The currently focused input element
  private activeInput: HTMLInputElement | HTMLTextAreaElement | null = null;

  // Callback for when value is submitted
  private onSubmitCallback: ((value: string) => void) | null = null;

  constructor() {
    console.log('[KeyboardService] Initializing keyboard service (enabled=' + this._enabled() + ')');
    this.setupGlobalListeners();
  }

  /** Toggle the in-app keyboard on/off. Persists to localStorage so
   *  the next session starts in the same state. Used from the
   *  settings panel; if you flip from on→off while a keyboard is
   *  open, the open instance is closed so the operator doesn't end
   *  up with a stuck overlay. */
  setEnabled(value: boolean): void {
    this._enabled.set(value);
    try {
      localStorage.setItem(STORAGE_KEY, value ? '1' : '0');
    } catch {
      // Ignore — localStorage in some sandboxed contexts can throw;
      // the in-memory signal still works for the session.
    }
    if (!value && this._isVisible()) {
      this.hide();
    }
  }

  private loadEnabled(): boolean {
    try {
      const raw = localStorage.getItem(STORAGE_KEY);
      if (raw === '1') return true;
      if (raw === '0') return false;
    } catch {
      // localStorage unavailable — fall through to default.
    }
    return false; // Default off: prefer platform OSK.
  }

  /**
   * Show the virtual keyboard for a specific input element. No-op
   * when the in-app keyboard is disabled — the operator's relying on
   * Steam OSK / KDE virtual keyboard / physical keyboard instead.
   */
  show(input?: HTMLInputElement | HTMLTextAreaElement): void {
    if (!this._enabled()) {
      return;
    }
    console.log('[KeyboardService] show called');

    if (input) {
      this.activeInput = input;
    }

    this._isVisible.set(true);
  }

  /**
   * Hide the virtual keyboard without submitting.
   */
  hide(): void {
    console.log('[KeyboardService] hide called');
    this._isVisible.set(false);
    this.activeInput = null;
    this.onSubmitCallback = null;
  }

  /**
   * Submit the keyboard value and hide.
   */
  submitValue(value: string): void {
    console.log('[KeyboardService] submitValue:', value);

    // Update the active input if we have one
    if (this.activeInput) {
      this.activeInput.value = value;
      // Dispatch input event so Angular forms detect the change
      this.activeInput.dispatchEvent(new Event('input', { bubbles: true }));
      this.activeInput.dispatchEvent(new Event('change', { bubbles: true }));
    }

    // Call the submit callback if set
    if (this.onSubmitCallback) {
      this.onSubmitCallback(value);
    }

    this.hide();
  }

  /**
   * Get the current value from the active input.
   */
  getCurrentInputValue(): string {
    return this.activeInput?.value ?? '';
  }

  /**
   * Set a callback for when the keyboard submits a value.
   */
  onSubmit(callback: (value: string) => void): void {
    this.onSubmitCallback = callback;
  }

  /**
   * Check if a text input is currently focused.
   * Used to determine if button presses should be passed through for keyboard use.
   */
  isTextFieldFocused(): boolean {
    return this.isTextInput(document.activeElement as HTMLElement);
  }

  /**
   * Set up global focus/blur listeners for all text inputs.
   */
  private setupGlobalListeners(): void {
    console.log('[KeyboardService] Setting up global focus/blur listeners');

    // Listen for focus events on text inputs. We only OPEN the
    // in-app keyboard when it's enabled (the show() call below
    // early-returns when disabled, but we also skip the activeInput
    // bookkeeping so a later toggle-on doesn't suddenly grab a
    // stale focus target). The focusin listener stays registered
    // unconditionally so a runtime toggle-on takes effect on the
    // NEXT focus event without a page reload.
    document.addEventListener('focusin', (event) => {
      const target = event.target as HTMLElement;
      if (!this._enabled()) return;
      console.log('[KeyboardService] focusin event:', target.tagName, target.getAttribute('type'), target.className);

      if (this.isTextInput(target)) {
        console.log('[KeyboardService] Text input focused, showing keyboard');
        this.activeInput = target as HTMLInputElement | HTMLTextAreaElement;
        this.show();
      }
    });

    // Listen for blur events - but don't auto-hide since user might click keyboard
    document.addEventListener('focusout', (event) => {
      const target = event.target as HTMLElement;
      console.log('[KeyboardService] focusout event:', target.tagName);

      // Don't hide keyboard on focusout - the keyboard component handles its own dismissal
      // This prevents the keyboard from closing when you click on it
    });

    console.log('[KeyboardService] Listeners registered');
  }

  /**
   * Check if an element is a text input that would benefit from a keyboard.
   */
  private isTextInput(element: HTMLElement | null): boolean {
    if (!element) return false;

    // Check for input elements
    if (element.tagName === 'INPUT') {
      const inputType = (element as HTMLInputElement).type.toLowerCase();
      // These input types would use a keyboard
      const textTypes = ['text', 'password', 'email', 'search', 'tel', 'url', 'number'];
      return textTypes.includes(inputType);
    }

    // Check for textarea
    if (element.tagName === 'TEXTAREA') {
      return true;
    }

    // Check for contenteditable
    if (element.isContentEditable) {
      return true;
    }

    return false;
  }
}
