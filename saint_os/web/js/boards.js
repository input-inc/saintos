/**
 * SAINT.OS Settings → Boards panel
 *
 * CRUD over chip + board YAML files on the server. Built-in boards
 * (ones shipped with SAINT.OS) are read-only — the modal shows them
 * but disables the Save button. Operator-authored boards have full
 * view / edit / delete.
 *
 * Backed by these WebSocket actions:
 *   list_chips                   — chip families
 *   list_boards                  — all boards
 *   get_board_yaml(board_id)     — raw YAML for one board
 *   save_board(yaml)             — write/replace an operator board
 *   delete_board(board_id)       — remove an operator board
 */

class BoardsManager {
    constructor() {
        this._editingBoardId = null;   // null = new board
        this._editingBuiltin = false;
    }

    init() {
        const addBtn = document.getElementById('boards-add-btn');
        if (addBtn) addBtn.addEventListener('click', () => this.openNewBoardEditor());
    }

    /** Called when the Settings → Boards tab is shown. */
    async activate() {
        await this.refreshList();
    }

    async refreshList() {
        const list = document.getElementById('boards-list');
        if (!list) return;
        try {
            const result = await window.saintWS.management('list_boards', {});
            const boards = (result?.boards || []).slice()
                .sort((a, b) => (b.builtin ? 1 : 0) - (a.builtin ? 1 : 0)
                              || a.board_id.localeCompare(b.board_id));
            if (boards.length === 0) {
                list.innerHTML = '<p class="text-slate-500 italic text-sm">No boards configured.</p>';
                return;
            }
            list.innerHTML = boards.map(b => this._renderRow(b)).join('');
        } catch (e) {
            console.error('list_boards failed:', e);
            list.innerHTML = `<p class="text-red-400 text-sm">Failed to load boards: ${escapeHtml(String(e))}</p>`;
        }
    }

    _renderRow(b) {
        const pinCount = (b.available_pins || []).length;
        const reservedCount = (b.reserved_pins || []).length;
        const builtinPeripheralIds = (b.builtin_peripherals || []).map(p => p.id).join(', ');
        const editBtn = b.builtin
            ? `<button class="btn-secondary text-xs" onclick="boardsManager.openEditor('${escapeAttr(b.board_id)}')">View</button>`
            : `<button class="btn-secondary text-xs" onclick="boardsManager.openEditor('${escapeAttr(b.board_id)}')">Edit</button>`;
        const deleteBtn = b.builtin
            ? ''
            : `<button class="btn-secondary text-xs hover:bg-red-900/40 hover:border-red-700"
                       onclick="boardsManager.deleteBoard('${escapeAttr(b.board_id)}')">Delete</button>`;
        const badge = b.builtin
            ? '<span class="px-2 py-0.5 rounded-full text-xs bg-slate-700 text-slate-300 ml-2">Built-in</span>'
            : '<span class="px-2 py-0.5 rounded-full text-xs bg-cyan-700/40 text-cyan-200 ml-2">Custom</span>';
        return `
            <div class="border border-slate-700 bg-slate-900/40 rounded p-3">
                <div class="flex items-center justify-between gap-2">
                    <div class="min-w-0">
                        <div class="font-semibold text-white truncate">
                            ${escapeHtml(b.display_name)}
                            <span class="text-xs text-slate-500 ml-2 font-mono">${escapeHtml(b.board_id)}</span>
                            ${badge}
                        </div>
                        <div class="text-xs text-slate-400 mt-1">
                            chip: <span class="font-mono">${escapeHtml(b.chip_family)}</span>
                            · ${pinCount} available pin${pinCount === 1 ? '' : 's'}
                            · ${reservedCount} reserved
                            ${builtinPeripheralIds ? ` · built-in: ${escapeHtml(builtinPeripheralIds)}` : ''}
                        </div>
                    </div>
                    <div class="flex items-center gap-1 shrink-0">
                        ${editBtn}
                        ${deleteBtn}
                    </div>
                </div>
            </div>`;
    }

    async openEditor(boardId) {
        try {
            const result = await window.saintWS.management('get_board_yaml', { board_id: boardId });
            if (!result || !result.yaml) {
                alert('Failed to load board YAML');
                return;
            }
            this._editingBoardId = boardId;
            // Look up whether this is built-in (Save will be disabled)
            const list = await window.saintWS.management('list_boards', {});
            const b = (list?.boards || []).find(x => x.board_id === boardId);
            this._editingBuiltin = !!(b && b.builtin);
            this._showEditor(
                this._editingBuiltin ? `${boardId} (read-only — built-in)` : `Edit ${boardId}`,
                result.yaml,
                this._editingBuiltin
            );
        } catch (e) {
            alert('Failed to load board: ' + (e.message || e));
        }
    }

    openNewBoardEditor() {
        this._editingBoardId = null;
        this._editingBuiltin = false;
        const template =
`board_id: my_custom_board
display_name: "My Custom Board"
chip_family: rp2040
builtin: false

available_pins:
  - {gpio:  5, name: "D5"}
  - {gpio: 26, name: "A0"}

reserved_pins: []

builtin_peripherals: []
`;
        this._showEditor('New Board', template, false);
    }

    _showEditor(title, yamlText, readOnly) {
        const modal = document.getElementById('board-editor-modal');
        const titleEl = document.getElementById('board-editor-title');
        const ta = document.getElementById('board-editor-yaml');
        const saveBtn = document.getElementById('board-editor-save-btn');
        const errEl = document.getElementById('board-editor-error');
        if (!modal || !ta) return;
        titleEl.textContent = title;
        ta.value = yamlText;
        ta.readOnly = readOnly;
        if (errEl) { errEl.textContent = ''; errEl.classList.add('hidden'); }
        if (saveBtn) {
            saveBtn.disabled = readOnly;
            saveBtn.classList.toggle('opacity-50', readOnly);
            saveBtn.classList.toggle('cursor-not-allowed', readOnly);
        }
        modal.classList.remove('hidden');
    }

    closeEditor() {
        const modal = document.getElementById('board-editor-modal');
        if (modal) modal.classList.add('hidden');
    }

    async saveEditor() {
        if (this._editingBuiltin) return;
        const ta = document.getElementById('board-editor-yaml');
        const errEl = document.getElementById('board-editor-error');
        const yamlText = ta?.value || '';
        try {
            const result = await window.saintWS.management('save_board', { yaml: yamlText });
            if (!result || result.success === false) {
                if (errEl) {
                    errEl.textContent = result?.message || 'Save failed';
                    errEl.classList.remove('hidden');
                }
                return;
            }
            this.closeEditor();
            await this.refreshList();
        } catch (e) {
            if (errEl) {
                errEl.textContent = e.message || String(e);
                errEl.classList.remove('hidden');
            }
        }
    }

    async deleteBoard(boardId) {
        if (!confirm(`Delete board "${boardId}"? Nodes currently assigned to it will need to be reassigned.`)) return;
        try {
            const result = await window.saintWS.management('delete_board', { board_id: boardId });
            if (!result || result.success === false) {
                alert(result?.message || 'Delete failed');
                return;
            }
            await this.refreshList();
        } catch (e) {
            alert('Delete failed: ' + (e.message || e));
        }
    }
}

const boardsManager = new BoardsManager();
window.boardsManager = boardsManager;

document.addEventListener('DOMContentLoaded', () => boardsManager.init());
