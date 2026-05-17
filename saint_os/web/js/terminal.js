/**
 * SAINT.OS web terminal frontend.
 *
 * Wires xterm.js to the server's PTY-backed shell session over the
 * existing WebSocket. The server forwards keystrokes through
 * `terminal.input` and pushes shell output as binary WebSocket frames
 * that we feed directly into xterm.js.
 *
 * Lifecycle:
 *   - openSession() runs when the user navigates to the Terminal page
 *   - closeSession() runs when they leave the page (or on disconnect)
 *
 * Resize: we observe the mount div and send `terminal.resize` on every
 * size change, debounced. The PTY ioctl on the server side makes vim/
 * htop/etc. reflow correctly.
 */

class TerminalManager {
    constructor() {
        this.term = null;
        this.fitAddon = null;
        this.opened = false;
        this._binaryHandler = null;
        this._inputDisposable = null;
        this._resizeObserver = null;
        this._resizeTimeout = null;
        this._exitHandler = null;
    }

    /**
     * Lazily build the xterm.Terminal instance. Mount + DOM are not yet
     * required at construction; we only call open() once we're on the
     * Terminal page so the container has real dimensions.
     */
    _ensureTerminal() {
        if (this.term) return;
        // The xterm.js global lives under window.Terminal.
        this.term = new window.Terminal({
            cursorBlink: true,
            fontSize: 13,
            fontFamily: '"JetBrains Mono", "Menlo", "Consolas", monospace',
            scrollback: 5000,
            theme: {
                background: '#000000',
                foreground: '#e2e8f0',
                cursor: '#22d3ee',
                selectionBackground: '#334155',
            },
            convertEol: false,
        });
        this.fitAddon = new window.FitAddon.FitAddon();
        this.term.loadAddon(this.fitAddon);
    }

    /**
     * Mount the terminal into #terminal-mount and start a shell session.
     * Safe to call multiple times — second call is a no-op.
     */
    async openSession() {
        const mount = document.getElementById('terminal-mount');
        if (!mount) return;

        this._ensureTerminal();

        // Mount once. Subsequent navigations reuse the same DOM.
        if (!this.term.element || !mount.contains(this.term.element)) {
            mount.innerHTML = '';
            this.term.open(mount);
        }

        if (this.opened) {
            this.fitAddon.fit();
            return;
        }

        const ws = window.saintWS;
        if (!ws || !ws.connected) {
            this.setStatus('Server not connected');
            return;
        }

        // Size the terminal to the container *before* asking the server
        // to start the shell so the initial TIOCSWINSZ is correct.
        this.fitAddon.fit();
        const { cols, rows } = this.term;

        this.setStatus('Starting shell...');
        try {
            await ws.management('terminal.open', { cols, rows });
        } catch (err) {
            this.setStatus(`Failed to start shell: ${err.message}`);
            this.term.writeln(`\r\n\x1b[31m[saint-os] ${err.message}\x1b[0m`);
            return;
        }
        this.setStatus(`Live  (${cols}×${rows})`);
        this.showRestartButton(true);

        // Binary output → xterm
        this._binaryHandler = (buf) => {
            this.term.write(new Uint8Array(buf));
        };
        ws.on('binary', this._binaryHandler);

        // Shell exit event → flag in UI, allow restart
        this._exitHandler = (msg) => {
            if (msg.node !== 'terminal') return;
            const code = msg.data?.code;
            this.term.writeln(`\r\n\x1b[33m[saint-os] shell exited (code=${code})\x1b[0m`);
            this.setStatus('Shell exited');
            this.opened = false;
        };
        ws.on('state', this._exitHandler);

        // Keyboard → server
        this._inputDisposable = this.term.onData((data) => {
            ws.management('terminal.input', { data }).catch((err) => {
                console.warn('terminal.input failed:', err);
            });
        });

        // Send resize on container size changes (debounced 100ms).
        this._resizeObserver = new ResizeObserver(() => {
            clearTimeout(this._resizeTimeout);
            this._resizeTimeout = setTimeout(() => this._sendResize(), 100);
        });
        this._resizeObserver.observe(mount);

        this.opened = true;
        this.term.focus();
    }

    _sendResize() {
        if (!this.opened || !this.term) return;
        try {
            this.fitAddon.fit();
        } catch (e) { /* container probably hidden */ return; }
        const { cols, rows } = this.term;
        const ws = window.saintWS;
        if (!ws) return;
        ws.management('terminal.resize', { cols, rows }).catch(() => {});
        this.setStatus(`Live  (${cols}×${rows})`);
    }

    /**
     * Tear down the session and detach event handlers. Keeps the xterm
     * DOM mounted so navigating back is instant.
     */
    async closeSession() {
        const ws = window.saintWS;
        if (this._binaryHandler && ws) {
            ws.off('binary', this._binaryHandler);
            this._binaryHandler = null;
        }
        if (this._exitHandler && ws) {
            ws.off('state', this._exitHandler);
            this._exitHandler = null;
        }
        if (this._inputDisposable) {
            this._inputDisposable.dispose();
            this._inputDisposable = null;
        }
        if (this._resizeObserver) {
            this._resizeObserver.disconnect();
            this._resizeObserver = null;
        }
        clearTimeout(this._resizeTimeout);

        if (this.opened && ws) {
            try {
                await ws.management('terminal.close', {});
            } catch (err) {
                console.warn('terminal.close failed:', err);
            }
        }
        this.opened = false;
        this.setStatus('Not connected');
        this.showRestartButton(false);
    }

    async restart() {
        await this.closeSession();
        if (this.term) {
            this.term.clear();
            this.term.reset();
        }
        await this.openSession();
    }

    setStatus(text) {
        const el = document.getElementById('terminal-status');
        if (el) el.textContent = text;
    }

    showRestartButton(visible) {
        const btn = document.getElementById('terminal-restart-btn');
        if (!btn) return;
        if (visible) btn.classList.remove('hidden');
        else btn.classList.add('hidden');
    }
}

window.terminalManager = new TerminalManager();

// Wire the restart button once the DOM is ready.
document.addEventListener('DOMContentLoaded', () => {
    const btn = document.getElementById('terminal-restart-btn');
    if (btn) {
        btn.addEventListener('click', () => window.terminalManager.restart());
    }
});
