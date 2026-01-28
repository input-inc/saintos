/**
 * SAINT.OS Moods Manager
 *
 * Manages mood data assets for head animation control.
 * Loads .uasset files from server, displays properties,
 * and applies moods to the head node.
 */

console.log('moods.js: Script loading...');

class MoodsManager {
    constructor() {
        this.moods = [];           // List of mood files
        this.selectedMood = null;  // Currently selected mood
        this.parsedMood = null;    // Parsed mood data
        this.headNodeId = null;    // Head node ID for applying moods
    }

    /**
     * Initialize the moods manager.
     */
    init() {
        console.log('MoodsManager: Initialized');
    }

    /**
     * Load moods when tab becomes active.
     */
    async load() {
        console.log('MoodsManager: Loading moods...');
        await this.refresh();
        await this.findHeadNode();
    }

    /**
     * Find the head node to apply moods to.
     */
    async findHeadNode() {
        const ws = window.saintWS;
        if (!ws || !ws.connected) return;

        try {
            const result = await ws.management('list_adopted');
            const nodes = result?.nodes || [];

            // Find node with head role
            const headNode = nodes.find(n =>
                n.role?.toLowerCase().includes('head') ||
                n.display_name?.toLowerCase().includes('head')
            );

            if (headNode) {
                this.headNodeId = headNode.node_id;
                console.log('MoodsManager: Found head node:', this.headNodeId);
            } else {
                console.log('MoodsManager: No head node found');
            }

            this.updateApplyButton();
        } catch (e) {
            console.error('MoodsManager: Failed to find head node:', e);
        }
    }

    /**
     * Refresh the moods list from server.
     */
    async refresh() {
        const listContainer = document.getElementById('moods-list');
        if (!listContainer) return;

        const ws = window.saintWS;
        if (!ws || !ws.connected) {
            listContainer.innerHTML = `
                <p class="text-slate-400 text-sm">Not connected to server.</p>
            `;
            return;
        }

        try {
            // List mood files from server
            // Note: WebSocket client extracts 'data' from response, so result is the data object
            const result = await ws.send('file', 'list', {
                pattern: 'DA_*.uasset'
            });

            this.moods = result?.files || [];
            console.log('MoodsManager: Found', this.moods.length, 'mood files');
            this.renderMoodsList();

        } catch (e) {
            console.error('MoodsManager: Failed to load moods:', e);
            listContainer.innerHTML = `
                <p class="text-red-400 text-sm">Failed to load moods: ${e.message}</p>
            `;
        }
    }

    /**
     * Render the moods list.
     */
    renderMoodsList() {
        const listContainer = document.getElementById('moods-list');
        if (!listContainer) return;

        if (this.moods.length === 0) {
            listContainer.innerHTML = `
                <div class="flex flex-col items-center py-4 text-center">
                    <span class="material-icons text-slate-600 mb-2" style="font-size: 32px;">sentiment_neutral</span>
                    <p class="text-slate-500 text-sm">No mood files found</p>
                    <p class="text-slate-600 text-xs mt-1">Upload DA_*.uasset files below</p>
                </div>
            `;
            return;
        }

        let html = '';
        for (const mood of this.moods) {
            const name = this.getMoodName(mood.name);
            const isSelected = this.selectedMood === mood.path;
            const sizeKB = (mood.size / 1024).toFixed(1);

            html += `
                <div class="mood-item ${isSelected ? 'selected' : ''}"
                     onclick="moodsManager.selectMood('${mood.path}')"
                     data-path="${mood.path}">
                    <div>
                        <span class="text-white font-medium">${name}</span>
                        <span class="text-slate-500 text-xs ml-2">${sizeKB} KB</span>
                    </div>
                    <span class="material-icons md-18 text-slate-500">chevron_right</span>
                </div>
            `;
        }

        listContainer.innerHTML = html;
    }

    /**
     * Extract mood name from filename.
     */
    getMoodName(filename) {
        // DA_Angry.uasset -> Angry
        let name = filename.replace(/^DA_/, '').replace(/\.uasset$/, '');
        // Capitalize first letter
        return name.charAt(0).toUpperCase() + name.slice(1);
    }

    /**
     * Select a mood and load its properties.
     */
    async selectMood(path) {
        console.log('MoodsManager: Selecting mood:', path);
        this.selectedMood = path;
        this.parsedMood = null;

        // Update list selection visual
        this.renderMoodsList();

        // Load and display mood details
        await this.loadMoodDetails(path);
    }

    /**
     * Load mood details from server.
     */
    async loadMoodDetails(path) {
        const detailsContainer = document.getElementById('mood-details');
        if (!detailsContainer) return;

        detailsContainer.innerHTML = `
            <div class="flex items-center justify-center py-8">
                <span class="material-icons animate-spin text-cyan-500">refresh</span>
                <span class="text-slate-400 ml-2">Loading mood data...</span>
            </div>
        `;

        const ws = window.saintWS;
        if (!ws || !ws.connected) {
            detailsContainer.innerHTML = `
                <p class="text-red-400 text-sm">Not connected to server.</p>
            `;
            return;
        }

        try {
            // Download the file
            const filename = path.split('/').pop();
            const category = path.includes('/') ? path.split('/')[0] : null;

            // Note: WebSocket client extracts 'data' from response
            const downloadResult = await ws.send('file', 'download', {
                filename: filename,
                category: category
            });

            if (!downloadResult?.content) {
                throw new Error('No content received');
            }

            // Parse the file by re-uploading with parse action
            const parseResult = await ws.send('file', 'parse', {
                filename: filename,
                content: downloadResult.content,
                category: category || 'moods'
            });

            if (parseResult?.parsed) {
                this.parsedMood = parseResult.parsed;
                this.renderMoodDetails();
            } else {
                throw new Error('Failed to parse mood data');
            }

        } catch (e) {
            console.error('MoodsManager: Failed to load mood details:', e);
            detailsContainer.innerHTML = `
                <div class="flex items-center gap-3 p-3 bg-red-900/20 rounded-lg border border-red-800/50">
                    <span class="material-icons md-20 text-red-400">error</span>
                    <div>
                        <p class="text-sm font-medium text-red-400">Failed to Load Mood</p>
                        <p class="text-xs text-red-300/70">${e.message || 'Unknown error'}</p>
                    </div>
                </div>
            `;
        }
    }

    /**
     * Render mood details/properties.
     */
    renderMoodDetails() {
        const detailsContainer = document.getElementById('mood-details');
        if (!detailsContainer || !this.parsedMood) return;

        const props = this.parsedMood.properties || {};
        const propNames = Object.keys(props).sort();

        if (propNames.length === 0) {
            detailsContainer.innerHTML = `
                <p class="text-slate-400 text-sm">No properties found in this mood.</p>
            `;
            this.updateApplyButton();
            return;
        }

        // Group properties by category
        const groups = {
            'Speed': [],
            'Frequency': [],
            'Cycle': [],
            'Rotation': [],
            'Mirror': [],
            'Other': []
        };

        for (const name of propNames) {
            const value = props[name];
            let grouped = false;

            for (const groupName of Object.keys(groups)) {
                if (groupName !== 'Other' && name.includes(groupName)) {
                    groups[groupName].push({ name, value });
                    grouped = true;
                    break;
                }
            }

            if (!grouped) {
                groups['Other'].push({ name, value });
            }
        }

        let html = `
            <div class="mb-4">
                <span class="text-lg font-medium text-white">${this.parsedMood.name}</span>
                <span class="text-sm text-slate-500 ml-2">${this.parsedMood.class}</span>
            </div>
        `;

        for (const [groupName, items] of Object.entries(groups)) {
            if (items.length === 0) continue;

            html += `
                <div class="mb-4">
                    <h4 class="text-sm font-medium text-slate-400 mb-2">${groupName}</h4>
                    <div class="bg-slate-800/50 rounded-lg p-3">
            `;

            for (const { name, value } of items) {
                const displayValue = this.formatValue(value);
                html += `
                    <div class="mood-property">
                        <span class="text-slate-300">${name}</span>
                        <span class="text-cyan-400 font-mono">${displayValue}</span>
                    </div>
                `;
            }

            html += `
                    </div>
                </div>
            `;
        }

        detailsContainer.innerHTML = html;
        this.updateApplyButton();
    }

    /**
     * Format a property value for display.
     */
    formatValue(value) {
        if (typeof value === 'boolean') {
            return value ? 'Yes' : 'No';
        }
        if (typeof value === 'number') {
            return value.toFixed(2);
        }
        if (typeof value === 'object') {
            return JSON.stringify(value);
        }
        return String(value);
    }

    /**
     * Update the apply button state.
     */
    updateApplyButton() {
        const btn = document.getElementById('btn-apply-mood');
        if (!btn) return;

        const canApply = this.parsedMood && this.headNodeId;
        btn.disabled = !canApply;

        if (!this.headNodeId) {
            btn.title = 'No head node found';
        } else if (!this.parsedMood) {
            btn.title = 'Select a mood first';
        } else {
            btn.title = `Apply to ${this.headNodeId}`;
        }
    }

    /**
     * Apply the selected mood to the head node.
     */
    async applyMood() {
        if (!this.parsedMood || !this.headNodeId) {
            console.warn('MoodsManager: Cannot apply - no mood or head node');
            return;
        }

        console.log('MoodsManager: Applying mood to head:', this.headNodeId);

        const ws = window.saintWS;
        if (!ws || !ws.connected) {
            alert('Not connected to server');
            return;
        }

        try {
            // Send mood properties to the head node via ROS
            // The ROS bridge should route this to the head state topic
            await ws.send('ros', 'publish', {
                endpoint: '/saint/head',
                data: {
                    mood: this.parsedMood.name,
                    properties: this.parsedMood.properties
                }
            });

            // If we get here, the request succeeded (WebSocket client throws on error)
            // Show success feedback
            if (typeof app !== 'undefined') {
                app.addActivityLogEntry({
                    text: `Applied mood "${this.parsedMood.name}" to head`,
                    level: 'info'
                });
            }

        } catch (e) {
            console.error('MoodsManager: Failed to apply mood:', e);
            alert(`Failed to apply mood: ${e.message}`);
        }
    }

    /**
     * Handle file selection for upload.
     */
    async handleFileSelect(event) {
        const file = event.target.files[0];
        if (!file) return;

        const statusEl = document.getElementById('mood-upload-status');
        if (statusEl) {
            statusEl.textContent = `Uploading ${file.name}...`;
            statusEl.className = 'text-sm text-slate-400';
        }

        try {
            // Read file as base64
            const content = await this.readFileAsBase64(file);

            const ws = window.saintWS;
            if (!ws || !ws.connected) {
                throw new Error('Not connected to server');
            }

            // Upload and parse
            // Note: WebSocket client extracts 'data' from response
            const result = await ws.send('file', 'parse', {
                filename: file.name,
                content: content,
                category: 'moods'
            });

            // If we get here, upload succeeded
            if (statusEl) {
                statusEl.textContent = `Uploaded ${file.name}`;
                statusEl.className = 'text-sm text-emerald-400';
            }

            // Refresh the moods list
            await this.refresh();

            // Select the newly uploaded mood
            if (result?.path) {
                await this.selectMood(result.path);
            }

        } catch (e) {
            console.error('MoodsManager: Upload failed:', e);
            if (statusEl) {
                statusEl.textContent = `Failed: ${e.message}`;
                statusEl.className = 'text-sm text-red-400';
            }
        }

        // Reset file input
        event.target.value = '';
    }

    /**
     * Read a file as base64.
     */
    readFileAsBase64(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = () => {
                // Remove data URL prefix
                const base64 = reader.result.split(',')[1];
                resolve(base64);
            };
            reader.onerror = () => reject(reader.error);
            reader.readAsDataURL(file);
        });
    }

    /**
     * Cleanup when leaving tab.
     */
    cleanup() {
        // Nothing to cleanup currently
    }
}

// Global instance
console.log('moods.js: Creating MoodsManager instance...');
const moodsManager = new MoodsManager();

// Make it globally accessible
window.moodsManager = moodsManager;

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    console.log('moods.js: DOMContentLoaded, initializing...');
    moodsManager.init();
});
