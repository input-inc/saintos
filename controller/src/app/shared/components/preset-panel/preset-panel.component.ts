import { Component, computed, inject } from '@angular/core';
import { CommonModule } from '@angular/common';
import { BindingsService, Preset } from '../../../core/services/bindings.service';

@Component({
  selector: 'app-preset-panel',
  standalone: true,
  imports: [CommonModule],
  template: `
    @if (panel(); as p) {
      <div class="h-full flex flex-col" [style.--panel-color]="p.color">
        <!-- Header -->
        <div class="panel-header px-6 py-4 flex items-center justify-between"
             [style.background]="p.color">
          <div class="flex items-center gap-3">
            <span class="material-symbols-outlined text-2xl text-white">{{ p.icon }}</span>
            <h2 class="text-xl font-semibold text-white">{{ p.name }}</h2>
          </div>
          <div class="flex items-center gap-4">
            <div class="text-white/80 text-sm">
              Page {{ currentPage() + 1 }}/{{ totalPages() }}
            </div>
            <button class="p-1.5 rounded hover:bg-white/20 transition-colors text-white"
                    (click)="close()">
              <span class="material-symbols-outlined">close</span>
            </button>
          </div>
        </div>

        <!-- Grid Content -->
        <div class="panel-content flex-1 bg-saint-background p-6 overflow-auto">
          <div class="grid gap-4 max-w-4xl mx-auto"
               [style.grid-template-columns]="'repeat(' + p.columns + ', 1fr)'">
            @for (preset of visiblePresets(); track preset.id; let i = $index) {
              <button class="preset-item group relative flex flex-col items-center justify-center
                             p-6 rounded-xl transition-all duration-150"
                      [class.preset-selected]="isSelected(i)"
                      [style.--preset-color]="preset.color || p.color"
                      (click)="selectPreset(preset)">
                <!-- Icon -->
                <span class="material-symbols-outlined text-4xl mb-3 transition-transform
                             group-hover:scale-110"
                      [style.color]="preset.color || p.color">
                  {{ preset.icon || 'radio_button_unchecked' }}
                </span>
                <!-- Name -->
                <span class="text-base font-medium text-saint-text text-center">
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
        <div class="panel-footer px-6 py-3 bg-saint-surface border-t border-saint-surface-light
                    flex items-center justify-between text-sm text-saint-text-muted">
          <div class="flex items-center gap-4">
            <span class="flex items-center gap-1">
              <span class="inline-flex items-center justify-center w-6 h-6 rounded bg-saint-surface-light text-xs font-bold">D</span>
              Navigate
            </span>
            <span class="flex items-center gap-1">
              <span class="inline-flex items-center justify-center w-6 h-6 rounded bg-green-600 text-white text-xs font-bold">A</span>
              Select
            </span>
            <span class="flex items-center gap-1">
              <span class="inline-flex items-center justify-center w-6 h-6 rounded bg-red-500 text-white text-xs font-bold">B</span>
              Close
            </span>
            @if (totalPages() > 1) {
              <span class="flex items-center gap-1 ml-2 pl-2 border-l border-saint-surface-light">
                <span class="inline-flex items-center justify-center px-2 h-6 rounded bg-saint-surface-light text-xs font-bold">LB</span>
                <span class="inline-flex items-center justify-center px-2 h-6 rounded bg-saint-surface-light text-xs font-bold">RB</span>
                Page
              </span>
            }
          </div>
          <div class="flex items-center gap-2">
            @if (totalPages() > 1) {
              <button class="p-1.5 rounded hover:bg-saint-surface-light transition-colors" [disabled]="currentPage() === 0" [class.opacity-50]="currentPage() === 0" (click)="prevPage()">
                <span class="material-symbols-outlined text-lg">chevron_left</span>
              </button>
              <span class="text-saint-text">{{ currentPage() + 1 }} / {{ totalPages() }}</span>
              <button class="p-1.5 rounded hover:bg-saint-surface-light transition-colors" [disabled]="currentPage() >= totalPages() - 1" [class.opacity-50]="currentPage() >= totalPages() - 1" (click)="nextPage()">
                <span class="material-symbols-outlined text-lg">chevron_right</span>
              </button>
            }
          </div>
        </div>
      </div>
    }
  `,
  styles: [`
    :host {
      display: block;
      height: 100%;
    }

    .preset-item {
      background: var(--saint-surface, #1a1a1a);
      border: 1px solid var(--saint-surface-light, #2a2a2a);
    }

    .preset-item:hover {
      background: var(--saint-surface-light, #2a2a2a);
      border-color: var(--preset-color);
    }

    .preset-selected {
      background: color-mix(in srgb, var(--preset-color) 15%, var(--saint-surface, #1a1a1a));
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

  close(): void {
    this.bindingsService.hidePanel();
  }

  prevPage(): void {
    this.bindingsService.navigatePanel('prev_page');
  }

  nextPage(): void {
    this.bindingsService.navigatePanel('next_page');
  }
}
