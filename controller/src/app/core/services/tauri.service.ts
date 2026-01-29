import { Injectable, NgZone } from '@angular/core';
import { invoke } from '@tauri-apps/api/core';
import { listen, UnlistenFn } from '@tauri-apps/api/event';
import { Observable, Subject } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class TauriService {
  private unlisteners: UnlistenFn[] = [];

  constructor(private ngZone: NgZone) {}

  async invoke<T>(cmd: string, args?: Record<string, unknown>): Promise<T> {
    return invoke<T>(cmd, args);
  }

  listen<T>(event: string): Observable<T> {
    const subject = new Subject<T>();

    listen<T>(event, (ev) => {
      this.ngZone.run(() => {
        subject.next(ev.payload);
      });
    }).then(unlisten => {
      this.unlisteners.push(unlisten);
    });

    return subject.asObservable();
  }

  cleanup(): void {
    this.unlisteners.forEach(unlisten => unlisten());
    this.unlisteners = [];
  }
}
