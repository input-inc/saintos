import {
  Component,
  ElementRef,
  EventEmitter,
  HostListener,
  Input,
  Output,
  signal,
  ViewChild,
  computed,
} from '@angular/core';
import { CommonModule } from '@angular/common';

export interface JoystickPosition {
  x: number; // -1 to 1
  y: number; // -1 to 1
}

@Component({
  selector: 'app-virtual-joystick',
  standalone: true,
  imports: [CommonModule],
  template: `
    <div
      #joystickBase
      class="relative bg-saint-background rounded-full border-2 select-none touch-none"
      [class.border-saint-success]="hasExternalInput()"
      [class.border-saint-surface-light]="!hasExternalInput()"
      [style.width.px]="size"
      [style.height.px]="size"
      (mousedown)="onStart($event)"
      (touchstart)="onTouchStart($event)">
      <div
        class="absolute rounded-full cursor-pointer transition-colors shadow-lg"
        [class.bg-saint-primary]="isDragging()"
        [class.bg-saint-success]="hasExternalInput() && !isDragging()"
        [class.bg-saint-surface-light]="!hasExternalInput() && !isDragging()"
        [style.width.px]="knobSize"
        [style.height.px]="knobSize"
        [style.left.px]="knobLeft()"
        [style.top.px]="knobTop()">
      </div>
    </div>
  `,
})
export class VirtualJoystickComponent {
  @Input() size = 80;
  @Input() knobSize = 40;
  @Input() externalPosition: JoystickPosition | null = null;

  @Output() positionChange = new EventEmitter<JoystickPosition>();

  @ViewChild('joystickBase', { static: true }) joystickBase!: ElementRef<HTMLDivElement>;

  isDragging = signal(false);
  private localPosition = signal<JoystickPosition>({ x: 0, y: 0 });

  // Use external position if provided and not dragging, otherwise use local
  hasExternalInput = computed(() => {
    return this.externalPosition !== null &&
           (Math.abs(this.externalPosition.x) > 0.05 || Math.abs(this.externalPosition.y) > 0.05);
  });

  private get position(): JoystickPosition {
    if (this.isDragging()) {
      return this.localPosition();
    }
    if (this.externalPosition && this.hasExternalInput()) {
      return this.externalPosition;
    }
    return this.localPosition();
  }

  knobLeft(): number {
    const maxOffset = (this.size - this.knobSize) / 2;
    return (this.size - this.knobSize) / 2 + this.position.x * maxOffset;
  }

  knobTop(): number {
    const maxOffset = (this.size - this.knobSize) / 2;
    return (this.size - this.knobSize) / 2 - this.position.y * maxOffset;
  }

  onStart(event: MouseEvent): void {
    event.preventDefault();
    this.isDragging.set(true);
    this.updatePosition(event.clientX, event.clientY);
  }

  onTouchStart(event: TouchEvent): void {
    event.preventDefault();
    if (event.touches.length > 0) {
      this.isDragging.set(true);
      this.updatePosition(event.touches[0].clientX, event.touches[0].clientY);
    }
  }

  @HostListener('window:mousemove', ['$event'])
  onMouseMove(event: MouseEvent): void {
    if (this.isDragging()) {
      this.updatePosition(event.clientX, event.clientY);
    }
  }

  @HostListener('window:touchmove', ['$event'])
  onTouchMove(event: TouchEvent): void {
    if (this.isDragging() && event.touches.length > 0) {
      this.updatePosition(event.touches[0].clientX, event.touches[0].clientY);
    }
  }

  @HostListener('window:mouseup')
  @HostListener('window:touchend')
  @HostListener('window:touchcancel')
  onEnd(): void {
    if (this.isDragging()) {
      this.isDragging.set(false);
      this.localPosition.set({ x: 0, y: 0 });
      this.positionChange.emit({ x: 0, y: 0 });
    }
  }

  private updatePosition(clientX: number, clientY: number): void {
    const rect = this.joystickBase.nativeElement.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;

    const maxRadius = (this.size - this.knobSize) / 2;

    let deltaX = clientX - centerX;
    let deltaY = centerY - clientY; // Invert Y so up is positive

    // Calculate distance from center
    const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    // Clamp to max radius
    if (distance > maxRadius) {
      deltaX = (deltaX / distance) * maxRadius;
      deltaY = (deltaY / distance) * maxRadius;
    }

    // Normalize to -1 to 1
    const normalizedX = deltaX / maxRadius;
    const normalizedY = deltaY / maxRadius;

    this.localPosition.set({ x: normalizedX, y: normalizedY });
    this.positionChange.emit({ x: normalizedX, y: normalizedY });
  }
}
