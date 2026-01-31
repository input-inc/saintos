import { Injectable } from '@angular/core';
import { invoke } from '@tauri-apps/api/core';

/**
 * Service to manage the Steam virtual keyboard on Steam Deck.
 *
 * On Linux (Steam Deck), this invokes the Steam keyboard when text inputs are focused.
 * On other platforms, it's a no-op.
 */
@Injectable({
  providedIn: 'root'
})
export class KeyboardService {
  constructor() {
    console.log('[KeyboardService] Initializing keyboard service');
    // Set up global focus/blur listeners for input fields
    this.setupGlobalListeners();
  }

  /**
   * Show the virtual keyboard.
   * Always invokes the command - the keyboard may have been dismissed by the user.
   */
  async showKeyboard(): Promise<void> {
    console.log('[KeyboardService] showKeyboard called');

    try {
      console.log('[KeyboardService] Invoking show_keyboard command...');
      const result = await invoke('show_keyboard');
      console.log('[KeyboardService] show_keyboard result:', result);
    } catch (err) {
      console.error('[KeyboardService] Failed to show keyboard:', err);
    }
  }

  /**
   * Hide the virtual keyboard.
   */
  async hideKeyboard(): Promise<void> {
    console.log('[KeyboardService] hideKeyboard called');

    try {
      console.log('[KeyboardService] Invoking hide_keyboard command...');
      const result = await invoke('hide_keyboard');
      console.log('[KeyboardService] hide_keyboard result:', result);
    } catch (err) {
      console.error('[KeyboardService] Failed to hide keyboard:', err);
    }
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
        this.showKeyboard();
      } else {
        console.log('[KeyboardService] Not a text input, ignoring');
      }
    });

    // Listen for blur events to hide keyboard
    document.addEventListener('focusout', (event) => {
      const target = event.target as HTMLElement;
      console.log('[KeyboardService] focusout event:', target.tagName, target.getAttribute('type'));

      if (this.isTextInput(target)) {
        // Small delay to allow for focus to move to another input
        setTimeout(() => {
          const activeElement = document.activeElement as HTMLElement;
          console.log('[KeyboardService] After delay, activeElement:', activeElement?.tagName, activeElement?.getAttribute('type'));

          if (!this.isTextInput(activeElement)) {
            console.log('[KeyboardService] Focus left text inputs, hiding keyboard');
            this.hideKeyboard();
          } else {
            console.log('[KeyboardService] Focus moved to another text input, keeping keyboard');
          }
        }, 100);
      }
    });

    console.log('[KeyboardService] Listeners registered');
  }

  /**
   * Check if a text input is currently focused.
   * Used to determine if button presses should be passed through for keyboard use.
   */
  isTextFieldFocused(): boolean {
    return this.isTextInput(document.activeElement as HTMLElement);
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
      const isText = textTypes.includes(inputType);
      console.log('[KeyboardService] isTextInput check: INPUT type=', inputType, 'isText=', isText);
      return isText;
    }

    // Check for textarea
    if (element.tagName === 'TEXTAREA') {
      console.log('[KeyboardService] isTextInput check: TEXTAREA = true');
      return true;
    }

    // Check for contenteditable
    if (element.isContentEditable) {
      console.log('[KeyboardService] isTextInput check: contentEditable = true');
      return true;
    }

    return false;
  }
}
