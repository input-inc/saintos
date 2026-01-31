import { Injectable, signal } from '@angular/core';

/**
 * Service to manage the built-in virtual keyboard.
 *
 * Shows an on-screen keyboard when text inputs are focused,
 * optimized for gamepad/touch input on Steam Deck.
 */
@Injectable({
  providedIn: 'root'
})
export class KeyboardService {
  // Keyboard visibility state
  private _isVisible = signal(false);
  isVisible = this._isVisible.asReadonly();

  // The currently focused input element
  private activeInput: HTMLInputElement | HTMLTextAreaElement | null = null;

  // Callback for when value is submitted
  private onSubmitCallback: ((value: string) => void) | null = null;

  constructor() {
    console.log('[KeyboardService] Initializing keyboard service');
    this.setupGlobalListeners();
  }

  /**
   * Show the virtual keyboard for a specific input element.
   */
  show(input?: HTMLInputElement | HTMLTextAreaElement): void {
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

    // Listen for focus events on text inputs
    document.addEventListener('focusin', (event) => {
      const target = event.target as HTMLElement;
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
