// ArduFlite Web Configuration UI
(function() {
    'use strict';
    
    const API = {
        config: '/api/config',
        telemetry: '/api/telemetry',
        status: '/api/system/status',
        calibrate: '/api/system/calibrate',
        flash: '/api/flash',
        reboot: '/api/config/reboot',
        reset: '/api/config/reset',
        export: '/api/config/export',
        import: '/api/config/import'
    };
    
    // Known prefixes for tabs (Other = everything else)
    const KNOWN_PREFIXES = ['rate.', 'att.', 'mix.', 'servo.', 'imu.', 'failsafe.', 'crsf.', 'web.', 'sys.'];
    
    let configData = [];
    let pendingChanges = new Map(); // key -> value
    let currentPattern = 'rate.*';
    let currentTab = 'rate';
    let telemetryInterval = null;
    
    // DOM elements
    const container = document.getElementById('config-container');
    const tabs = document.querySelectorAll('.tab');
    const search = document.getElementById('search');
    const toast = document.getElementById('toast');
    const btnReboot = document.getElementById('btn-reboot');
    const btnRebootNow = document.getElementById('btn-reboot-now');
    const btnCalibrate = document.getElementById('btn-calibrate');
    const btnExport = document.getElementById('btn-export');
    const btnImport = document.getElementById('btn-import');
    const btnFactoryReset = document.getElementById('btn-factory-reset');
    const fileImport = document.getElementById('file-import');
    const logsContainer = document.getElementById('logs-container');
    const backupContainer = document.getElementById('backup-container');
    const logsList = document.getElementById('logs-list');
    const logsSpace = document.getElementById('logs-space');
    const modal = document.getElementById('modal');
    
    // Dashboard elements
    const dashRoll = document.getElementById('dash-roll');
    const dashPitch = document.getElementById('dash-pitch');
    const dashYaw = document.getElementById('dash-yaw');
    const dashAlt = document.getElementById('dash-alt');
    const dashMode = document.getElementById('dash-mode');
    const dashArmed = document.getElementById('dash-armed');
    
    // --- Telemetry ---
    async function pollTelemetry() {
        try {
            const res = await fetch(API.telemetry);
            if (!res.ok) return;
            const data = await res.json();
            
            dashRoll.textContent = data.roll.toFixed(1) + '°';
            dashPitch.textContent = data.pitch.toFixed(1) + '°';
            dashYaw.textContent = data.yaw.toFixed(0) + '°';
            dashAlt.textContent = data.altitude.toFixed(1) + 'm';
            
            const modes = ['Attitude', 'Rate', 'Manual'];
            const modeClasses = ['mode-attitude', 'mode-rate', 'mode-manual'];
            const modeName = modes[data.mode] || 'Unknown';
            const modeClass = modeClasses[data.mode] || 'mode-unknown';
            dashMode.textContent = modeName;
            dashMode.className = 'dash-mode ' + modeClass;
            
            dashArmed.textContent = data.armed ? 'ARMED' : 'DISARMED';
            dashArmed.className = 'dash-armed ' + (data.armed ? 'armed' : 'disarmed');
        } catch (err) {
            // Silently fail - device may be offline
        }
    }
    
    function startTelemetry() {
        pollTelemetry();
        telemetryInterval = setInterval(pollTelemetry, 1000);
    }
    
    // --- Configuration ---
    async function loadConfig(pattern = '*') {
        try {
            container.innerHTML = '<div id="loading">Loading...</div>';
            const res = await fetch(`${API.config}?pattern=${encodeURIComponent(pattern)}`);
            let data = await res.json();
            
            // For "Other" tab, filter out known prefixes
            if (currentTab === 'other') {
                data = data.filter(p => !KNOWN_PREFIXES.some(prefix => p.key.startsWith(prefix)));
            }
            
            configData = data;
            pendingChanges.clear();
            renderConfig();
            updateRebootButton();
        } catch (err) {
            console.error('Failed to load config:', err);
            container.innerHTML = '<div id="loading">Error loading configuration</div>';
        }
    }
    
    function renderConfig() {
        const filter = search.value.toLowerCase().trim();
        let filtered = configData;
        
        if (filter) {
            filtered = configData.filter(p => 
                p.key.toLowerCase().includes(filter) ||
                p.desc.toLowerCase().includes(filter)
            );
        }
        
        if (filtered.length === 0) {
            container.innerHTML = '<div id="loading">No parameters found</div>';
            return;
        }
        
        container.innerHTML = filtered.map(p => renderCard(p)).join('');
        attachInputListeners();
    }
    
    function renderCard(p) {
        const hasPending = pendingChanges.has(p.key);
        const pendingValue = pendingChanges.get(p.key);
        
        const badges = [];
        if (p.reboot) badges.push('<span class="badge reboot">Reboot</span>');
        if (p.dirty) badges.push('<span class="badge dirty">Saved</span>');
        
        const classes = ['config-card'];
        if (hasPending) classes.push('dirty');
        
        let inputHtml = '';
        const displayValue = hasPending ? pendingValue : p.value;
        
        if (p.type === 3) { // BOOL
            const checked = hasPending ? pendingValue : p.value;
            inputHtml = `
                <div class="toggle-container">
                    <label class="toggle">
                        <input type="checkbox" data-key="${p.key}" ${checked ? 'checked' : ''}>
                        <span class="toggle-slider"></span>
                    </label>
                    <span class="toggle-label">${checked ? 'Enabled' : 'Disabled'}</span>
                </div>
                <button class="btn-apply ${hasPending ? '' : 'hidden'}" data-key="${p.key}">Apply</button>
            `;
        } else if (p.type === 4) { // STRING
            inputHtml = `
                <input type="text" class="${hasPending ? 'pending' : ''}" 
                    data-key="${p.key}" value="${escapeHtml(displayValue || '')}">
                <button class="btn-apply ${hasPending ? '' : 'hidden'}" data-key="${p.key}">Apply</button>
            `;
        } else { // FLOAT, INT32, UINT8
            const step = p.type === 0 ? '0.001' : '1';
            inputHtml = `
                <input type="number" class="${hasPending ? 'pending' : ''}"
                    data-key="${p.key}" value="${displayValue}" 
                    min="${p.min}" max="${p.max}" step="${step}">
                <button class="btn-apply ${hasPending ? '' : 'hidden'}" data-key="${p.key}">Apply</button>
            `;
        }
        
        const rangeHtml = (p.type < 3) ? 
            `<div class="config-range">Range: ${p.min} – ${p.max} (default: ${p.default})</div>` : 
            (p.type === 4 ? `<div class="config-range">Default: "${p.default}"</div>` : '');
        
        return `
            <div class="${classes.join(' ')}">
                <div class="config-header">
                    <span class="config-key">${p.key}</span>
                    <div class="config-badges">${badges.join('')}</div>
                </div>
                <div class="config-desc">${escapeHtml(p.desc)}</div>
                <div class="config-input-row">${inputHtml}</div>
                ${rangeHtml}
            </div>
        `;
    }
    
    function attachInputListeners() {
        container.querySelectorAll('input').forEach(input => {
            input.addEventListener('input', handleInputChange);
            input.addEventListener('change', handleInputChange);
        });
        
        container.querySelectorAll('.btn-apply').forEach(btn => {
            btn.addEventListener('click', handleApply);
        });
    }
    
    function handleInputChange(e) {
        const input = e.target;
        const key = input.dataset.key;
        const param = configData.find(p => p.key === key);
        if (!param) return;
        
        let value;
        if (input.type === 'checkbox') {
            value = input.checked;
        } else if (input.type === 'number') {
            value = parseFloat(input.value);
        } else {
            value = input.value;
        }
        
        // Check if value differs from saved
        const isDifferent = value !== param.value;
        
        if (isDifferent) {
            pendingChanges.set(key, value);
        } else {
            pendingChanges.delete(key);
        }
        
        // Update UI
        const card = input.closest('.config-card');
        const applyBtn = card.querySelector('.btn-apply');
        
        if (isDifferent) {
            card.classList.add('dirty');
            input.classList.add('pending');
            applyBtn.classList.remove('hidden');
        } else {
            card.classList.remove('dirty');
            input.classList.remove('pending');
            applyBtn.classList.add('hidden');
        }
        
        // Update toggle label
        if (input.type === 'checkbox') {
            const label = card.querySelector('.toggle-label');
            if (label) label.textContent = value ? 'Enabled' : 'Disabled';
        }
    }
    
    async function handleApply(e) {
        const btn = e.target;
        const key = btn.dataset.key;
        const value = pendingChanges.get(key);
        
        if (value === undefined) return;
        
        btn.disabled = true;
        btn.textContent = '...';
        
        try {
            const res = await fetch(`${API.config}/${encodeURIComponent(key)}`, {
                method: 'PUT',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ value })
            });
            
            if (!res.ok) {
                const err = await res.json();
                showToast(`Error: ${err.error}`, 'error');
                btn.disabled = false;
                btn.textContent = 'Apply';
                return;
            }
            
            // Update local data
            const param = configData.find(p => p.key === key);
            if (param) {
                param.value = value;
                param.dirty = true;
            }
            
            pendingChanges.delete(key);
            showToast(`${key} saved`, 'success');
            renderConfig();
            updateRebootButton();
        } catch (err) {
            console.error('Failed to update:', err);
            showToast('Failed to save parameter', 'error');
            btn.disabled = false;
            btn.textContent = 'Apply';
        }
    }
    
    function updateRebootButton() {
        const needsReboot = configData.some(p => p.reboot && p.dirty);
        btnReboot.classList.toggle('hidden', !needsReboot);
    }
    
    // --- Flash Logs ---
    async function loadFlashLogs() {
        try {
            const res = await fetch(API.flash);
            const data = await res.json();
            
            if (data.files && data.files.length > 0) {
                const usedKB = Math.round(data.used / 1024);
                const totalKB = Math.round(data.total / 1024);
                logsSpace.textContent = `${usedKB} / ${totalKB} KB used`;
                
                logsList.innerHTML = data.files.map(log => `
                    <div class="log-item">
                        <div class="log-info">
                            <span class="log-name">${log.name}</span>
                            <span class="log-size">${formatBytes(log.size)}</span>
                        </div>
                        <div class="log-actions">
                            <button class="btn-secondary" onclick="downloadLog('${log.name}')">⬇ Download</button>
                            <button class="btn-danger" onclick="deleteLog('${log.name}')">✕ Delete</button>
                        </div>
                    </div>
                `).join('');
            } else {
                logsSpace.textContent = 'No files';
                logsList.innerHTML = '<div style="color: var(--text-dim); text-align: center; padding: 1rem;">No flight logs recorded</div>';
            }
        } catch (err) {
            logsList.innerHTML = '<div style="color: var(--danger)">Error loading logs</div>';
        }
    }
    
    window.downloadLog = function(name) {
        window.location.href = `${API.flash}/${name}`;
    };
    
    window.deleteLog = async function(name) {
        if (!confirm(`Delete ${name}?`)) return;
        
        try {
            await fetch(`${API.flash}/${name}`, { method: 'DELETE' });
            showToast('Log deleted', 'success');
            loadFlashLogs();
        } catch (err) {
            showToast('Delete failed', 'error');
        }
    };
    
    // --- Tab Handling ---
    tabs.forEach(tab => {
        tab.addEventListener('click', () => {
            tabs.forEach(t => t.classList.remove('active'));
            tab.classList.add('active');
            
            currentPattern = tab.dataset.pattern;
            currentTab = currentPattern.replace(/[.*_]/g, '');
            
            // Toggle visibility of sections
            const isLogs = currentPattern === '__logs__';
            const isBackup = currentPattern === '__backup__';
            const isConfig = !isLogs && !isBackup;
            
            logsContainer.classList.toggle('hidden', !isLogs);
            backupContainer.classList.toggle('hidden', !isBackup);
            container.classList.toggle('hidden', !isConfig);
            document.getElementById('toolbar').classList.toggle('hidden', !isConfig);
            
            if (isLogs) {
                loadFlashLogs();
            } else if (isBackup) {
                // Nothing to load for backup
            } else {
                // For 'Other' tab, load all params and filter client-side
                const pattern = currentPattern === '__other__' ? '*' : currentPattern;
                loadConfig(pattern);
            }
        });
    });
    
    // --- Search ---
    let searchDebounce = null;
    search.addEventListener('input', () => {
        clearTimeout(searchDebounce);
        searchDebounce = setTimeout(renderConfig, 200);
    });
    
    // --- Action Buttons ---
    btnCalibrate.addEventListener('click', async () => {
        btnCalibrate.disabled = true;
        btnCalibrate.textContent = 'Calibrating...';
        
        try {
            const res = await fetch(API.calibrate, { method: 'POST' });
            const data = await res.json();
            
            if (data.ok) {
                showToast('IMU calibration started', 'success');
            } else {
                showToast('Calibration failed', 'error');
            }
        } catch (err) {
            showToast('Connection error', 'error');
        }
        
        setTimeout(() => {
            btnCalibrate.disabled = false;
            btnCalibrate.textContent = 'Calibrate IMU';
        }, 3000);
    });
    
    btnFactoryReset.addEventListener('click', () => {
        showModal(
            'Reset to Defaults',
            'Reset ALL parameters to factory defaults? This cannot be undone.',
            async () => {
                await fetch(API.reset, { method: 'POST' });
                showToast('Configuration reset to defaults', 'success');
                loadConfig(currentPattern);
            }
        );
    });
    
    btnRebootNow.addEventListener('click', () => {
        showModal(
            'Reboot ESP32',
            'Reboot the flight controller now?',
            async () => {
                await fetch(API.reboot, { method: 'POST' });
                showToast('Rebooting...', 'success');
            }
        );
    });
    
    btnReboot.addEventListener('click', () => {
        showModal(
            'Reboot Required',
            'Some changes require a reboot. Reboot now?',
            async () => {
                await fetch(API.reboot, { method: 'POST' });
                showToast('Rebooting...', 'success');
            }
        );
    });
    
    btnExport.addEventListener('click', async () => {
        try {
            const res = await fetch(API.export);
            const data = await res.text();
            
            const blob = new Blob([data], { type: 'application/json' });
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = 'arduflite-config.json';
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            URL.revokeObjectURL(url);
            
            showToast('Configuration exported', 'success');
        } catch (err) {
            showToast('Export failed', 'error');
        }
    });
    
    btnImport.addEventListener('click', () => {
        fileImport.click();
    });
    
    fileImport.addEventListener('change', async (e) => {
        const file = e.target.files[0];
        if (!file) return;
        
        try {
            const text = await file.text();
            const res = await fetch(API.import, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: text
            });
            
            if (res.ok) {
                showToast('Configuration imported', 'success');
                loadConfig(currentPattern);
            } else {
                showToast('Import failed - invalid JSON', 'error');
            }
        } catch (err) {
            showToast('Import failed', 'error');
        }
        
        // Reset input so same file can be imported again
        fileImport.value = '';
    });
    
    // --- Modal ---
    function showModal(title, message, onConfirm) {
        const titleEl = modal.querySelector('h3');
        const msgEl = modal.querySelector('p');
        const confirmBtn = document.getElementById('modal-confirm');
        const cancelBtn = document.getElementById('modal-cancel');
        
        titleEl.textContent = title;
        msgEl.textContent = message;
        modal.classList.remove('hidden');
        
        const cleanup = () => {
            modal.classList.add('hidden');
            confirmBtn.onclick = null;
            cancelBtn.onclick = null;
        };
        
        confirmBtn.onclick = async () => {
            cleanup();
            if (onConfirm) await onConfirm();
        };
        
        cancelBtn.onclick = cleanup;
    }
    
    // --- Toast ---
    let toastTimeout = null;
    function showToast(message, type = 'success') {
        clearTimeout(toastTimeout);
        toast.textContent = message;
        toast.className = 'toast ' + type;
        toast.classList.remove('hidden');
        
        toastTimeout = setTimeout(() => {
            toast.classList.add('hidden');
        }, 3000);
    }
    
    // --- Helpers ---
    function escapeHtml(str) {
        const div = document.createElement('div');
        div.textContent = str;
        return div.innerHTML;
    }
    
    function formatBytes(bytes) {
        if (bytes < 1024) return bytes + ' B';
        if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB';
        return (bytes / (1024 * 1024)).toFixed(1) + ' MB';
    }
    
    // --- Initialize ---
    loadConfig(currentPattern);
    startTelemetry();
    
})();
