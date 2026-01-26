/**
 * SAINT.OS LiveLink Manager
 *
 * Handles LiveLink Face data visualization in the web UI.
 */

class LiveLinkManager {
    constructor() {
        this.paused = false;
        this.lastBlendShapes = null;
        this.connected = false;
        this.subscribed = false;
        this._blendShapesInitialized = false;
    }

    /**
     * Initialize LiveLink manager.
     */
    init() {
        // Listen for livelink state updates
        const ws = window.saintWS;
        if (ws) {
            ws.on('state', (message) => {
                if (message.node === 'livelink') {
                    this.handleStatusUpdate(message.data);
                } else if (message.node === 'livelink/blend_shapes') {
                    this.handleBlendShapesUpdate(message.data);
                }
            });
        }
    }

    /**
     * Subscribe to LiveLink blend shapes.
     */
    async subscribe() {
        if (this.subscribed) return;

        const ws = window.saintWS;
        if (ws && ws.connected) {
            await ws.subscribe(['livelink', 'livelink/blend_shapes']);
            this.subscribed = true;
            console.log('Subscribed to LiveLink data');
        }
    }

    /**
     * Unsubscribe from LiveLink blend shapes.
     */
    async unsubscribe() {
        if (!this.subscribed) return;

        const ws = window.saintWS;
        if (ws && ws.connected) {
            await ws.unsubscribe(['livelink/blend_shapes']);
            this.subscribed = false;
            console.log('Unsubscribed from LiveLink blend shapes');
        }
    }

    /**
     * Handle LiveLink status update.
     */
    handleStatusUpdate(data) {
        if (!data) return;

        const receiver = data.receiver || {};
        this.connected = receiver.connected || false;

        // Update inputs page summary
        this.updateInputsSummary(data);

        // Update detail page if visible
        this.updateDetailPage(data);
    }

    /**
     * Update inputs page LiveLink summary.
     */
    updateInputsSummary(data) {
        const receiver = data.receiver || {};
        const router = data.router || {};

        // Status badge
        const badge = document.getElementById('livelink-status-badge');
        if (badge) {
            if (receiver.connected) {
                badge.className = 'px-2 py-1 text-xs font-medium rounded-full bg-emerald-500/20 text-emerald-400';
                badge.textContent = 'Connected';
            } else if (receiver.running) {
                badge.className = 'px-2 py-1 text-xs font-medium rounded-full bg-amber-500/20 text-amber-400';
                badge.textContent = 'Listening';
            } else {
                badge.className = 'px-2 py-1 text-xs font-medium rounded-full bg-slate-500/20 text-slate-400';
                badge.textContent = 'Disconnected';
            }
        }

        // Status text
        const statusEl = document.getElementById('livelink-receiver-status');
        if (statusEl) {
            statusEl.textContent = receiver.connected ? 'Receiving data' : (receiver.running ? 'Waiting for connection' : 'Not running');
        }

        // Port
        const portEl = document.getElementById('livelink-port');
        if (portEl && receiver.port) {
            portEl.textContent = receiver.port.toString();
        }

        // Packet count
        const packetsEl = document.getElementById('livelink-packet-count');
        if (packetsEl) {
            packetsEl.textContent = (receiver.packet_count || 0).toLocaleString();
        }
    }

    /**
     * Update LiveLink detail page.
     */
    updateDetailPage(data) {
        const receiver = data.receiver || {};

        // Connection status
        const statusEl = document.getElementById('livelink-connection-status');
        if (statusEl) {
            const dot = statusEl.querySelector('span:first-child');
            const text = statusEl.querySelector('span:last-child');

            if (receiver.connected) {
                dot.className = 'w-2 h-2 rounded-full bg-emerald-500';
                text.textContent = 'Connected';
                text.className = 'text-sm text-emerald-400';
            } else if (receiver.running) {
                dot.className = 'w-2 h-2 rounded-full bg-amber-500 animate-pulse';
                text.textContent = 'Listening';
                text.className = 'text-sm text-amber-400';
            } else {
                dot.className = 'w-2 h-2 rounded-full bg-slate-500';
                text.textContent = 'Disconnected';
                text.className = 'text-sm text-slate-400';
            }
        }

        // Stats
        const portEl = document.getElementById('livelink-detail-port');
        if (portEl && receiver.port) {
            portEl.textContent = receiver.port.toString();
        }

        const packetsEl = document.getElementById('livelink-detail-packets');
        if (packetsEl) {
            packetsEl.textContent = (receiver.packet_count || 0).toLocaleString();
        }

        const errorsEl = document.getElementById('livelink-detail-errors');
        if (errorsEl) {
            errorsEl.textContent = (receiver.error_count || 0).toString();
        }

        const lastUpdateEl = document.getElementById('livelink-detail-last-update');
        if (lastUpdateEl && receiver.last_packet_time) {
            const elapsed = Date.now() / 1000 - receiver.last_packet_time;
            if (elapsed < 1) {
                lastUpdateEl.textContent = 'Just now';
            } else if (elapsed < 60) {
                lastUpdateEl.textContent = `${Math.floor(elapsed)}s ago`;
            } else {
                lastUpdateEl.textContent = `${Math.floor(elapsed / 60)}m ago`;
            }
        }
    }

    /**
     * Handle blend shapes data update.
     */
    handleBlendShapesUpdate(data) {
        if (this.paused || !data) return;

        this.lastBlendShapes = data;

        // Update blend shapes grid
        this.renderBlendShapes(data);

        // Update eye visualization
        this.updateEyeVisualization(data);

        // Update head pose visualization
        this.updateHeadPose(data);
    }

    /**
     * Render blend shapes grid.
     * Uses efficient DOM updates to avoid flickering.
     */
    renderBlendShapes(data) {
        const container = document.getElementById('livelink-blend-shapes');
        if (!container) return;

        // List of key blend shapes to display
        const keyShapes = [
            // Head rotation (extended data)
            'HeadYaw', 'HeadPitch', 'HeadRoll',
            // Eyes
            'EyeBlinkLeft', 'EyeBlinkRight',
            'EyeLookUpLeft', 'EyeLookDownLeft', 'EyeLookInLeft', 'EyeLookOutLeft',
            'EyeLookUpRight', 'EyeLookDownRight', 'EyeLookInRight', 'EyeLookOutRight',
            'EyeSquintLeft', 'EyeSquintRight', 'EyeWideLeft', 'EyeWideRight',
            // Jaw
            'JawOpen', 'JawLeft', 'JawRight', 'JawForward',
            // Mouth
            'MouthSmileLeft', 'MouthSmileRight', 'MouthFrownLeft', 'MouthFrownRight',
            'MouthPucker', 'MouthFunnel',
            // Brows
            'BrowDownLeft', 'BrowDownRight', 'BrowInnerUp', 'BrowOuterUpLeft', 'BrowOuterUpRight',
            // Cheeks
            'CheekPuff', 'CheekSquintLeft', 'CheekSquintRight',
            // Nose
            'NoseSneerLeft', 'NoseSneerRight',
            // Tongue
            'TongueOut'
        ];

        // Check if we need to create the initial structure
        if (!this._blendShapesInitialized || container.children.length !== keyShapes.length) {
            let html = '';
            for (const name of keyShapes) {
                html += `
                    <div class="blend-shape-item p-2 bg-slate-900/50 rounded-lg border border-slate-700/50" data-shape="${name}">
                        <div class="flex items-center justify-between mb-1">
                            <span class="text-xs text-slate-400 truncate" title="${name}">${this.formatShapeName(name)}</span>
                            <span class="blend-value text-xs font-mono text-slate-500">0.00</span>
                        </div>
                        <div class="h-1.5 bg-slate-700 rounded-full overflow-hidden">
                            <div class="blend-bar h-full ${this.getBarColor(name)}" style="width: 0%; transition: width 0.1s ease-out"></div>
                        </div>
                    </div>
                `;
            }
            container.innerHTML = html;
            this._blendShapesInitialized = true;
        }

        // Update values efficiently
        for (const name of keyShapes) {
            const item = container.querySelector(`[data-shape="${name}"]`);
            if (!item) continue;

            const value = data[name] || 0;

            // Head rotation values are -1 to 1, map to 0-100% with 50% as center
            // Other blend shapes are 0 to 1
            let percent;
            if (name.includes('Head')) {
                // Map -1 to 1 → 0% to 100%, with 0 at 50%
                percent = Math.min(100, Math.max(0, (value + 1) * 50));
            } else {
                percent = Math.min(100, Math.max(0, value * 100));
            }

            // Update value text
            const valueEl = item.querySelector('.blend-value');
            if (valueEl) {
                valueEl.textContent = value.toFixed(2);
                valueEl.className = `blend-value text-xs font-mono ${this.getBlendShapeColor(name, value)}`;
            }

            // Update bar width
            const barEl = item.querySelector('.blend-bar');
            if (barEl) {
                barEl.style.width = `${percent}%`;
            }
        }
    }

    /**
     * Format blend shape name for display.
     */
    formatShapeName(name) {
        // Remove common prefixes and add spaces
        return name
            .replace(/^Eye/, '')
            .replace(/^Mouth/, '')
            .replace(/^Brow/, '')
            .replace(/^Jaw/, '')
            .replace(/^Cheek/, '')
            .replace(/^Nose/, '')
            .replace(/Left$/, ' L')
            .replace(/Right$/, ' R')
            .replace(/([A-Z])/g, ' $1')
            .trim();
    }

    /**
     * Get color class for blend shape value.
     */
    getBlendShapeColor(name, value) {
        // For head rotation, use absolute value since they can be negative
        const absValue = Math.abs(value);
        if (name.includes('Head')) {
            if (absValue > 0.5) return 'text-rose-400';
            if (absValue > 0.2) return 'text-slate-300';
            return 'text-slate-500';
        }
        if (value > 0.7) return 'text-cyan-400';
        if (value > 0.3) return 'text-slate-300';
        return 'text-slate-500';
    }

    /**
     * Get bar color class for blend shape type.
     */
    getBarColor(name) {
        if (name.includes('Head')) return 'bg-rose-500';
        if (name.includes('Eye')) return 'bg-cyan-500';
        if (name.includes('Brow')) return 'bg-violet-500';
        if (name.includes('Mouth') || name.includes('Jaw')) return 'bg-amber-500';
        if (name.includes('Cheek') || name.includes('Nose')) return 'bg-emerald-500';
        return 'bg-slate-500';
    }

    /**
     * Update eye visualization.
     */
    updateEyeVisualization(data) {
        // Left eye
        const leftPupil = document.getElementById('eye-left-pupil');
        if (leftPupil) {
            const h = (data.eye_look_horizontal_left || 0) * 30; // -30 to 30 pixels
            const v = -(data.eye_look_vertical_left || 0) * 30; // -30 to 30 pixels (inverted)
            leftPupil.style.left = `calc(50% + ${h}px)`;
            leftPupil.style.top = `calc(50% + ${v}px)`;

            // Blink effect - shrink pupil vertically
            const blink = data.EyeBlinkLeft || 0;
            leftPupil.style.transform = `translate(-50%, -50%) scaleY(${1 - blink * 0.9})`;
        }

        const leftBlinkEl = document.getElementById('eye-left-blink');
        if (leftBlinkEl) {
            leftBlinkEl.textContent = `Blink: ${Math.round((data.EyeBlinkLeft || 0) * 100)}%`;
        }

        // Right eye
        const rightPupil = document.getElementById('eye-right-pupil');
        if (rightPupil) {
            const h = (data.eye_look_horizontal_right || 0) * 30;
            const v = -(data.eye_look_vertical_right || 0) * 30;
            rightPupil.style.left = `calc(50% + ${h}px)`;
            rightPupil.style.top = `calc(50% + ${v}px)`;

            const blink = data.EyeBlinkRight || 0;
            rightPupil.style.transform = `translate(-50%, -50%) scaleY(${1 - blink * 0.9})`;
        }

        const rightBlinkEl = document.getElementById('eye-right-blink');
        if (rightBlinkEl) {
            rightBlinkEl.textContent = `Blink: ${Math.round((data.EyeBlinkRight || 0) * 100)}%`;
        }
    }

    /**
     * Update head pose visualization.
     */
    updateHeadPose(data) {
        // Get head rotation values (in radians, typically -1 to 1)
        const yaw = data.HeadYaw || 0;      // Left/right turn
        const pitch = data.HeadPitch || 0;  // Up/down tilt
        const roll = data.HeadRoll || 0;    // Side tilt

        // Update the head shape rotation
        const headShape = document.getElementById('head-shape');
        if (headShape) {
            // Convert radians to degrees for CSS transform
            // Yaw rotates the top-down view, roll tilts it
            // Pitch affects the vertical scale (simulating looking up/down)
            const yawDeg = yaw * 57.3;  // radians to degrees (180/PI ≈ 57.3)
            const rollDeg = roll * 57.3;
            const pitchScale = 1 - Math.abs(pitch) * 0.3;  // Compress when looking up/down

            headShape.style.transform = `rotate(${yawDeg}deg) scaleY(${pitchScale})`;
        }

        // Update value displays
        const yawEl = document.getElementById('head-yaw-value');
        const pitchEl = document.getElementById('head-pitch-value');
        const rollEl = document.getElementById('head-roll-value');

        if (yawEl) {
            yawEl.textContent = yaw.toFixed(2);
            // Color based on direction
            yawEl.className = `font-mono ${yaw < -0.1 ? 'text-cyan-400' : yaw > 0.1 ? 'text-cyan-400' : 'text-slate-400'}`;
        }
        if (pitchEl) {
            pitchEl.textContent = pitch.toFixed(2);
            pitchEl.className = `font-mono ${Math.abs(pitch) > 0.1 ? 'text-violet-400' : 'text-slate-400'}`;
        }
        if (rollEl) {
            rollEl.textContent = roll.toFixed(2);
            rollEl.className = `font-mono ${Math.abs(roll) > 0.1 ? 'text-amber-400' : 'text-slate-400'}`;
        }
    }

    /**
     * Toggle pause state.
     */
    togglePause() {
        this.paused = !this.paused;

        const icon = document.getElementById('livelink-pause-icon');
        const text = document.getElementById('livelink-pause-text');

        if (this.paused) {
            if (icon) {
                icon.textContent = 'play_circle';
            }
            if (text) {
                text.textContent = 'Resume';
            }
        } else {
            if (icon) {
                icon.textContent = 'pause_circle';
            }
            if (text) {
                text.textContent = 'Pause';
            }
        }
    }

    /**
     * Get LiveLink status.
     */
    async getStatus() {
        const ws = window.saintWS;
        if (!ws || !ws.connected) return null;

        try {
            const result = await ws.send({
                type: 'livelink',
                action: 'get_status'
            });
            return result;
        } catch (e) {
            console.error('Failed to get LiveLink status:', e);
            return null;
        }
    }
}

// Create global instance
window.liveLinkManager = new LiveLinkManager();

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.liveLinkManager.init();
});
