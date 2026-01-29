import { Component, computed, inject } from '@angular/core';
import { CommonModule } from '@angular/common';
import { BindingsService, Preset, PresetPanel } from '../../../core/services/bindings.service';

@Component({
  selector: 'app-preset-panel',
  standalone: true,
  imports: [CommonModule],
  template: `
    @if (panel(); as p) {
      <div class="fixed inset-0 z-50 flex items-center justify-center bg-black/60 backdrop-blur-sm"
           (click)="onBackdropClick($event)">
        <div class="preset-panel w-full max-w-2xl mx-4 rounded-2xl overflow-hidden shadow-2xl"
             [style.--panel-color]="p.color">
          <!-- Header -->
          <div class="panel-header px-6 py-4 flex items-center justify-between"
               [style.background]="p.color">
            <div class="flex items-center gap-3">
              <span class="material-symbols-outlined text-2xl text-white">{{ p.icon }}</span>
              <h2 class="text-xl font-semibold text-white">{{ p.name }}</h2>
            </div>
            <div class="text-white/80 text-sm">
              Page {{ currentPage() + 1 }}/{{ totalPages() }}
            </div>
          </div>

          <!-- Grid Content -->
          <div class="panel-content bg-saint-surface p-4">
            <div class="grid gap-3"
                 [style.grid-template-columns]="'repeat(' + p.columns + ', 1fr)'">
              @for (preset of visiblePresets(); track preset.id; let i = $index) {
                <button class="preset-item group relative flex flex-col items-center justify-center
                               p-4 rounded-xl transition-all duration-150"
                        [class.preset-selected]="isSelected(i)"
                        [style.--preset-color]="preset.color || p.color"
                        (click)="selectPreset(preset)">
                  <!-- Icon -->
                  <span class="material-symbols-outlined text-3xl mb-2 transition-transform
                               group-hover:scale-110"
                        [style.color]="preset.color || p.color">
                    {{ preset.icon || 'radio_button_unchecked' }}
                  </span>
                  <!-- Name -->
                  <span class="text-sm font-medium text-saint-text text-center">
                    {{ preset.name }}
                  </span>
                  <!-- Selection indicator -->
                  @if (isSelected(i)) {
                    <div class="absolute inset-0 rounded-xl border-2 pointer-events-none"
                         [style.border-color]="preset.color || p.color"></div>
                  }
                </button>
              }
            </div>

            <!-- Empty state -->
            @if (visiblePresets().length === 0) {
              <div class="text-center py-12 text-saint-text-muted">
                <span class="material-symbols-outlined text-4xl mb-2">folder_open</span>
                <p>No presets in this panel</p>
              </div>
            }
          </div>

          <!-- Footer with controls hint -->
          <div class="panel-footer px-6 py-3 bg-saint-surface-light border-t border-saint-border
                      flex items-center justify-between text-sm text-saint-text-muted">
            <div class="flex items-center gap-4">
              <span class="flex items-center gap-1">
                <span class="inline-flex items-center justify-center w-6 h-6 rounded bg-saint-surface
                             text-xs font-bold">D</span>
                Navigate
              </span>
              <span class="flex items-center gap-1">
                <span class="inline-flex items-center justify-center w-6 h-6 rounded bg-green-600
                             text-white text-xs font-bold">A</span>
                Select
              </span>
              <span class="flex items-center gap-1">
                <span class="inline-flex items-center justify-center w-6 h-6 rounded bg-red-500
                             text-white text-xs font-bold">B</span>
                Close
              </span>
            </div>
            <div class="flex items-center gap-2">
              @if (totalPages() > 1) {
                <button class="p-1.5 rounded hover:bg-saint-surface transition-colors"
                        [disabled]="currentPage() === 0"
                        [class.opacity-50]="currentPage() === 0"
                        (click)="prevPage()">
                  <span class="material-symbols-outlined text-lg">chevron_left</span>
                </button>
                <button class="p-1.5 rounded hover:bg-saint-surface transition-colors"
                        [disabled]="currentPage() >= totalPages() - 1"
                        [class.opacity-50]="currentPage() >= totalPages() - 1"
                        (click)="nextPage()">
                  <span class="material-symbols-outlined text-lg">chevron_right</span>
                </button>
              }
            </div>
          </div>
        </div>
      </div>
    }
  `,
  styles: [`
    .preset-panel {
      animation: slideUp 0.2s ease-out;
    }

    @keyframes slideUp {
      from {
        opacity: 0;
        transform: translateY(20px);
      }
      to {
        opacity: 1;
        transform: translateY(0);
      }
    }

    .preset-item {
      background: var(--saint-surface-light, #2a2a2a);
      border: 1px solid transparent;
    }

    .preset-item:hover {
      background: var(--saint-surface, #1a1a1a);
      border-color: var(--preset-color);
    }

    .preset-selected {
      background: color-mix(in srgb, var(--preset-color) 15%, var(--saint-surface-light, #2a2a2a));
    }
  `]
})
export class PresetPanelComponent {
  private bindingsService = inject(BindingsService);

  readonly panel = computed(() => this.bindingsService.activePanel());
  readonly panelState = computed(() => this.bindingsService.activePanelState());

  readonly currentPage = computed(() => this.panelState().currentPage);
  readonly selectedIndex = computed(() => this.panelState().selectedIndex);

  readonly totalPages = computed(() => {
    const p = this.panel();
    if (!p) return 0;
    return Math.ceil(p.presets.length / p.itemsPerPage);
  });

  readonly visiblePresets = computed(() => {
    const p = this.panel();
    if (!p) return [];
    const start = this.currentPage() * p.itemsPerPage;
    return p.presets.slice(start, start + p.itemsPerPage);
  });

  isSelected(pageIndex: number): boolean {
    const p = this.panel();
    if (!p) return false;
    const globalIndex = this.currentPage() * p.itemsPerPage + pageIndex;
    return globalIndex === this.selectedIndex();
  }

  selectPreset(preset: Preset): void {
    this.bindingsService.activatePreset(preset.id);
    this.bindingsService.hidePanel();
  }

  onBackdropClick(event: MouseEvent): void {
    if ((event.target as HTMLElement).classList.contains('fixed')) {
      this.bindingsService.hidePanel();
    }
  }

  prevPage(): void {
    this.bindingsService.navigatePanel('prev_page');
  }

  nextPage(): void {
    this.bindingsService.navigatePanel('next_page');
  }
}
