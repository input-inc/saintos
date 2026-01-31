import {
  Component,
  OnInit,
  OnDestroy,
  ElementRef,
  ViewChild,
  AfterViewInit,
  signal,
  effect,
  inject,
  NgZone,
} from '@angular/core';
import { CommonModule } from '@angular/common';
import Keyboard from 'simple-keyboard';
import { KeyboardService } from '../../../core/services/keyboard.service';

type KeyboardLayout = 'default' | 'shift' | 'symbols' | 'symbols-shift';

@Component({
  selector: 'app-virtual-keyboard',
  standalone: true,
  imports: [CommonModule],
  template: `
    @if (keyboardService.isVisible()) {
      <div class="keyboard-overlay" (click)="onOverlayClick($event)">
        <div class="keyboard-container" (click)="$event.stopPropagation()">
          <!-- Input preview -->
          <div class="input-preview">
            <span class="preview-text">{{ inputValue() }}</span>
            <span class="cursor">|</span>
          </div>

          <!-- Keyboard -->
          <div #keyboardContainer class="keyboard-wrapper"></div>

          <!-- Bottom toolbar -->
          <div class="keyboard-toolbar">
            <button class="toolbar-btn" (click)="onCancel()">
              <span class="material-icons">close</span>
              Cancel
            </button>
            <div class="toolbar-spacer"></div>
            <button class="toolbar-btn" (click)="onBackspace()">
              <span class="material-icons">backspace</span>
            </button>
            <button class="toolbar-btn" (click)="onSpace()">
              <span class="material-icons">space_bar</span>
              Space
            </button>
            <button class="toolbar-btn primary" (click)="onDone()">
              <span class="material-icons">check</span>
              Done
            </button>
          </div>
        </div>
      </div>
    }
  `,
  styles: [`
    .keyboard-overlay {
      position: fixed;
      inset: 0;
      background: rgba(15, 23, 42, 0.9);
      backdrop-filter: blur(4px);
      display: flex;
      align-items: flex-end;
      justify-content: center;
      z-index: 9999;
      padding-bottom: 1rem;
    }

    .keyboard-container {
      width: 100%;
      max-width: 900px;
      background: #1e293b;
      border-radius: 1rem 1rem 0 0;
      border: 1px solid #334155;
      border-bottom: none;
      overflow: hidden;
      box-shadow: 0 -4px 20px rgba(0, 0, 0, 0.5);
    }

    .input-preview {
      padding: 1rem 1.5rem;
      background: #0f172a;
      border-bottom: 1px solid #334155;
      font-family: 'Inter', sans-serif;
      font-size: 1.25rem;
      color: #f1f5f9;
      min-height: 3.5rem;
      display: flex;
      align-items: center;
    }

    .preview-text {
      white-space: pre-wrap;
      word-break: break-all;
    }

    .cursor {
      animation: blink 1s infinite;
      color: #3b82f6;
      margin-left: 1px;
    }

    @keyframes blink {
      0%, 50% { opacity: 1; }
      51%, 100% { opacity: 0; }
    }

    .keyboard-wrapper {
      padding: 0.75rem;
      touch-action: manipulation;
    }

    .keyboard-toolbar {
      display: flex;
      align-items: center;
      gap: 0.5rem;
      padding: 0.75rem 1rem;
      background: #0f172a;
      border-top: 1px solid #334155;
    }

    .toolbar-spacer {
      flex: 1;
    }

    .toolbar-btn {
      display: flex;
      align-items: center;
      gap: 0.5rem;
      padding: 0.75rem 1.25rem;
      background: #334155;
      border: 1px solid #475569;
      border-radius: 0.5rem;
      color: #f1f5f9;
      font-family: 'Inter', sans-serif;
      font-size: 1rem;
      font-weight: 500;
      cursor: pointer;
      transition: all 0.15s ease;
    }

    .toolbar-btn:hover {
      background: #475569;
    }

    .toolbar-btn:active {
      background: #3b82f6;
      transform: translateY(1px);
    }

    .toolbar-btn.primary {
      background: #3b82f6;
      border-color: #60a5fa;
    }

    .toolbar-btn.primary:hover {
      background: #2563eb;
    }

    .toolbar-btn .material-icons {
      font-size: 1.25rem;
    }
  `]
})
export class VirtualKeyboardComponent implements OnInit, AfterViewInit, OnDestroy {
  @ViewChild('keyboardContainer') keyboardContainer!: ElementRef<HTMLDivElement>;

  keyboardService = inject(KeyboardService);
  private ngZone = inject(NgZone);

  private keyboard: Keyboard | null = null;
  private currentLayout = signal<KeyboardLayout>('default');

  inputValue = signal('');

  // Track visibility to init/destroy keyboard
  private visibilityEffect = effect(() => {
    const visible = this.keyboardService.isVisible();
    console.log('[VirtualKeyboard] visibility changed:', visible);
    if (visible) {
      // Get the current input value when keyboard becomes visible
      const initialValue = this.keyboardService.getCurrentInputValue();
      console.log('[VirtualKeyboard] initial value:', initialValue);
      this.inputValue.set(initialValue);
      // Small delay to ensure DOM is ready
      setTimeout(() => this.initKeyboard(), 50);
    } else {
      this.destroyKeyboard();
    }
  });

  ngOnInit(): void {
    // Initial value is now set in the visibility effect
  }

  ngAfterViewInit(): void {
    // Keyboard will be initialized by effect when visible
  }

  ngOnDestroy(): void {
    this.destroyKeyboard();
  }

  private initKeyboard(): void {
    if (this.keyboard || !this.keyboardContainer?.nativeElement) return;

    console.log('[VirtualKeyboard] Initializing keyboard...');

    this.keyboard = new Keyboard(this.keyboardContainer.nativeElement, {
      onChange: (input) => {
        console.log('[VirtualKeyboard] onChange callback:', input);
        this.ngZone.run(() => this.onInputChange(input));
      },
      onKeyPress: (button) => {
        console.log('[VirtualKeyboard] onKeyPress callback:', button);
        this.ngZone.run(() => this.onKeyPress(button));
      },
      layout: {
        default: [
          '1 2 3 4 5 6 7 8 9 0',
          'q w e r t y u i o p',
          'a s d f g h j k l',
          '{shift} z x c v b n m {backspace}',
          '{symbols} {space} . {enter}'
        ],
        shift: [
          '1 2 3 4 5 6 7 8 9 0',
          'Q W E R T Y U I O P',
          'A S D F G H J K L',
          '{shift} Z X C V B N M {backspace}',
          '{symbols} {space} . {enter}'
        ],
        symbols: [
          '! @ # $ % ^ & * ( )',
          '- _ = + [ ] { } |',
          '; : \' " , . < > ?',
          '{shift} / \\ ~ ` {backspace}',
          '{abc} {space} . {enter}'
        ],
        'symbols-shift': [
          '1 2 3 4 5 6 7 8 9 0',
          '€ £ ¥ © ® ™ § ¶ •',
          '° ± × ÷ ≠ ≈ ∞ µ',
          '{shift} … – — « » {backspace}',
          '{abc} {space} . {enter}'
        ]
      },
      display: {
        '{backspace}': '⌫',
        '{enter}': '↵',
        '{shift}': '⇧',
        '{space}': ' ',
        '{symbols}': '?123',
        '{abc}': 'ABC'
      },
      theme: 'simple-keyboard hg-theme-default',
      useButtonTag: true,  // Use <button> elements for better touch support
      disableButtonHold: true,  // Disable hold behavior for simpler touch handling
      useTouchEvents: true,  // Use touch events instead of mouse events
      useMouseEvents: true,  // Also keep mouse events for desktop testing
    });

    // Debug: Add a test click listener to verify events reach the container
    this.keyboardContainer.nativeElement.addEventListener('click', (e) => {
      const target = e.target as HTMLElement;
      console.log('[VirtualKeyboard] Container click event:', target.tagName, target.className, target.textContent?.trim());
    });
    this.keyboardContainer.nativeElement.addEventListener('touchend', (e) => {
      const target = e.target as HTMLElement;
      console.log('[VirtualKeyboard] Container touchend event:', target.tagName, target.className, target.textContent?.trim());
    });

    // Set initial value
    this.keyboard.setInput(this.inputValue());
    console.log('[VirtualKeyboard] Keyboard initialized');
  }

  private destroyKeyboard(): void {
    if (this.keyboard) {
      this.keyboard.destroy();
      this.keyboard = null;
    }
  }

  private onInputChange(input: string): void {
    console.log('[VirtualKeyboard] onChange:', input);
    this.inputValue.set(input);
  }

  private onKeyPress(button: string): void {
    if (button === '{shift}') {
      this.toggleShift();
    } else if (button === '{symbols}') {
      this.currentLayout.set('symbols');
      this.keyboard?.setOptions({ layoutName: 'symbols' });
    } else if (button === '{abc}') {
      this.currentLayout.set('default');
      this.keyboard?.setOptions({ layoutName: 'default' });
    } else if (button === '{enter}') {
      this.onDone();
    }
  }

  private toggleShift(): void {
    const layout = this.currentLayout();
    if (layout === 'default') {
      this.currentLayout.set('shift');
      this.keyboard?.setOptions({ layoutName: 'shift' });
    } else if (layout === 'shift') {
      this.currentLayout.set('default');
      this.keyboard?.setOptions({ layoutName: 'default' });
    } else if (layout === 'symbols') {
      this.currentLayout.set('symbols-shift');
      this.keyboard?.setOptions({ layoutName: 'symbols-shift' });
    } else {
      this.currentLayout.set('symbols');
      this.keyboard?.setOptions({ layoutName: 'symbols' });
    }
  }

  onOverlayClick(event: Event): void {
    // Click on overlay (outside keyboard) = cancel
    this.onCancel();
  }

  onBackspace(): void {
    const current = this.inputValue();
    if (current.length > 0) {
      const newValue = current.slice(0, -1);
      this.inputValue.set(newValue);
      this.keyboard?.setInput(newValue);
    }
  }

  onSpace(): void {
    const newValue = this.inputValue() + ' ';
    this.inputValue.set(newValue);
    this.keyboard?.setInput(newValue);
  }

  onCancel(): void {
    this.keyboardService.hide();
  }

  onDone(): void {
    this.keyboardService.submitValue(this.inputValue());
  }
}
