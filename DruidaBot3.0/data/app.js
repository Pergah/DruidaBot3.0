// =========================
// Druida BOT — frontend
// =========================
function byId(id) { return document.getElementById(id); }
window.druidaToast = showToast;

// =========================
// Toast
// =========================
function showToast(msg, isErr = false) {
  const t = byId('actionToast');
  if (!t) return;
  t.textContent = msg || '';
  t.classList.remove('err');
  if (isErr) t.classList.add('err');
  t.classList.add('show');
  clearTimeout(window.__toastTimer);
  window.__toastTimer = setTimeout(() => t.classList.remove('show'), 1800);
}

// =========================
// Navegación SPA + tabs
// =========================
function setView(mode) {
  const views = ['homeView','devicesView','paramsView'];
  views.forEach(id => {
    const el = byId(id);
    if (el) el.classList.add('preloaded-hidden');
  });
  const target = byId(mode);
  if (target) target.classList.remove('preloaded-hidden');

  document.querySelectorAll('.tab').forEach(t => {
    t.classList.toggle('active', t.dataset.tab === mode);
  });

  window.scrollTo({ top: 0, behavior: 'instant' in window ? 'instant' : 'auto' });
}

function showPreloadedHome()    { setView('homeView'); }
function showPreloadedDevices() { setView('devicesView'); renderDevices(); }
function showPreloadedParams()  { setView('paramsView');  renderParams(); }

// alias legacy (por si el firmware aún los referencia)
function showPreloadedControl() { showPreloadedDevices(); }
function showPreloadedConfig()  { showPreloadedParams(); }
function showPreloadedControlR1(){ showPreloadedDevices(); }
function showPreloadedControlR2(){ showPreloadedDevices(); }
function showPreloadedControlR3(){ showPreloadedDevices(); }
function showPreloadedControlR4(){ showPreloadedDevices(); }
function showPreloadedControlR5(){ showPreloadedDevices(); }
function showPreloadedControlR6(){ showPreloadedDevices(); }
function showPreloadedConfigR1(){ showPreloadedParams(); }
function showPreloadedConfigR2(){ showPreloadedParams(); }
function showPreloadedConfigR3(){ showPreloadedParams(); }
function showPreloadedConfigR4(){ showPreloadedParams(); }
function showPreloadedConfigR5(){ showPreloadedParams(); }
function showPreloadedConfigR6(){ showPreloadedParams(); }
function showPreloadedConfigWiFi(){ showPreloadedParams(); }

// =========================
// Quick action / submit
// =========================
async function quickAction(url, label) {
  showToast(label || 'Procesando...');
  try {
    if (!url.includes('ajax=1')) url += (url.includes('?') ? '&' : '?') + 'ajax=1';
    const res = await fetch(url, { method: 'POST', headers: { 'X-Requested-With': 'XMLHttpRequest' } });
    if (!res.ok) throw new Error();
    const txt = await res.text();
    showToast(txt || (label || '') + ' OK');
    loadState();
  } catch {
    showToast('Error en acción', true);
  }
}

async function submitFormConfig(formId, msg) {
  const f = byId(formId);
  if (!f) return false;
  showToast(msg || 'Guardando...');
  try {
    let url = f.action || window.location.pathname;
    if (!url.includes('ajax=1')) url += (url.includes('?') ? '&' : '?') + 'ajax=1';
    const res = await fetch(url, {
      method: 'POST',
      body: new FormData(f),
      headers: { 'X-Requested-With': 'XMLHttpRequest' }
    });
    const txt = (await res.text()).trim();
    showToast(txt || 'Configuración guardada');
  } catch {
    showToast('Error al guardar', true);
  }
  return false;
}

// =========================
// State client-side (cache + fallback)
// =========================
const PARAMS_KEY  = 'druida_params_v1';
const RELAYCFG_KEY = 'druida_relaycfg_v1';

const DEFAULT_PARAMS = {
  tempHeaterOn: 18, tempHeaterOff: 22, tempAcOff: 25, tempAcOn: 28,
  humHumidOn: 40,  humHumidOff: 55,  humDehumOff: 65, humDehumOn: 75,
  acRelay: 5, heaterRelay: 6, humidRelay: 1, dehumidRelay: 2
};

const QUAD_CFG = {
  temp: { min: 15, max: 35, step: 0.5, gapOuter: 3,  gapInner: 1,  fmt: v => v.toFixed(1) + ' °C' },
  hum:  { min: 30, max: 90, step: 1,   gapOuter: 10, gapInner: 5,  fmt: v => Math.round(v) + ' %'  }
};
const QUAD_KEYS = {
  temp: ['tempHeaterOn', 'tempHeaterOff', 'tempAcOff', 'tempAcOn'],
  hum:  ['humHumidOn',  'humHumidOff',  'humDehumOff', 'humDehumOn']
};

const RELAY_DEFAULT_NAMES = {
  1: 'Humidificación',
  2: 'Extracción',
  3: 'Irrigación 1',
  4: 'Iluminación 1',
  5: 'Aire Acondicionado',
  6: 'Calefacción',
  7: 'Iluminación 2',
  8: 'Irrigación 2'
};

const DEFAULT_RELAY_CFG = () => {
  const out = {};
  for (let i = 1; i <= 8; i++) {
    out['r' + i] = {
      mode: 'auto', on: false, onTime: '06:00', offTime: '22:00',
      horaOn: 8, minOn: 0, horaOff: 22, minOff: 0,
      pulsoVal: 30, pulsoUnit: 1, pausaVal: 10, pausaUnit: 60, ciclos: 4,
      diasRiego: [false, true, true, true, true, true, false]
    };
  }
  return out;
};

function loadParams() {
  try {
    const raw = localStorage.getItem(PARAMS_KEY);
    if (raw) return Object.assign({}, DEFAULT_PARAMS, JSON.parse(raw));
  } catch {}
  return Object.assign({}, DEFAULT_PARAMS);
}
function saveParamsLocal(p) {
  try { localStorage.setItem(PARAMS_KEY, JSON.stringify(p)); } catch {}
}
function loadRelayCfg() {
  try {
    const raw = localStorage.getItem(RELAYCFG_KEY);
    if (raw) return Object.assign(DEFAULT_RELAY_CFG(), JSON.parse(raw));
  } catch {}
  return DEFAULT_RELAY_CFG();
}
function saveRelayCfgLocal(c) {
  try { localStorage.setItem(RELAYCFG_KEY, JSON.stringify(c)); } catch {}
}

let lastState = null;
let currentParams = loadParams();
let currentRelayCfg = loadRelayCfg();
let dirtyParams = false;
const dirtyRelayIds = new Set();

// Si el firmware envía params/configs en /api/state, los preferimos.
function ingestServerState(s) {
  if (s && s.params && typeof s.params === 'object') {
    if (!dirtyParams) {
      currentParams = Object.assign({}, DEFAULT_PARAMS, s.params);
      saveParamsLocal(currentParams);
    }
  }
  if (s && s.relays && typeof s.relays === 'object') {
    let touched = false;
    for (let i = 1; i <= 8; i++) {
      const r = s.relays['r' + i];
      if (!r) continue;
      const cfg = currentRelayCfg['r' + i] || {};
      if (dirtyRelayIds.has(i)) {
        if (r.on !== undefined && cfg.on == null) cfg.on = !!r.on;
        currentRelayCfg['r' + i] = cfg;
        continue;
      }
      // El firmware puede enviar modeKind ("auto"/"manual"/"timer") o mode numérico.
      if (r.modeKind) { cfg.mode = r.modeKind; touched = true; }
      else if (r.mode !== undefined) {
        const k = numericModeToKind(r.mode);
        if (k) { cfg.mode = k; touched = true; }
      }
      if (r.onTime)  { cfg.onTime  = r.onTime;  touched = true; }
      if (r.offTime) { cfg.offTime = r.offTime; touched = true; }
      if (r.horaOn  !== undefined) { cfg.horaOn  = parseInt(r.horaOn,  10); touched = true; }
      if (r.minOn   !== undefined) { cfg.minOn   = parseInt(r.minOn,   10); touched = true; }
      if (r.horaOff !== undefined) { cfg.horaOff = parseInt(r.horaOff, 10); touched = true; }
      if (r.minOff  !== undefined) { cfg.minOff  = parseInt(r.minOff,  10); touched = true; }
      if (IRRIGATION_RELAYS.has(i)) {
        if (r.pulsoVal  !== undefined) { cfg.pulsoVal  = +r.pulsoVal;  touched = true; }
        if (r.pulsoUnit !== undefined) { cfg.pulsoUnit = +r.pulsoUnit; touched = true; }
        if (r.pausaVal  !== undefined) { cfg.pausaVal  = +r.pausaVal;  touched = true; }
        if (r.pausaUnit !== undefined) { cfg.pausaUnit = +r.pausaUnit; touched = true; }
        if (r.ciclos    !== undefined) { cfg.ciclos    = +r.ciclos;    touched = true; }
        if (Array.isArray(r.diasRiego)) { cfg.diasRiego = r.diasRiego.map(Boolean); touched = true; }
      }
      if (r.on !== undefined) cfg.on = !!r.on;
      currentRelayCfg['r' + i] = cfg;
    }
    if (touched) saveRelayCfgLocal(currentRelayCfg);
  }
}

function numericModeToKind(m) {
  const s = String(m);
  if (s === '1') return 'manual';
  if (s === '2') return 'auto';
  if (s === '6') return 'timer';
  return null;
}
function kindToNumericMode(k) {
  if (k === 'manual') return 1;
  if (k === 'auto')   return 2;
  if (k === 'timer')  return 6;
  return 0;
}

// =========================
// Iconos / etiquetas
// =========================
const RELAY_ICONS = {
  1: '💦', // Humidificación
  2: '💨', // Extracción
  3: '🚿', // Irrigación 1
  4: '☀️', // Iluminación 1
  5: '❄️', // A/C
  6: '🔥', // Calefacción
  7: '💡', // Iluminación 2
  8: '🚿'  // Irrigación 2
};
const TIMER_LABEL_RELAYS = new Set([3, 4, 7, 8]);
const ILLUMINATION_RELAYS = new Set([4, 7]);
const IRRIGATION_RELAYS   = new Set([3, 8]);

const MODE_TEXT = {
  '0': 'Apagado', '1': 'Manual', '2': 'Auto',
  '4': 'Ciclo',   '6': 'Timer',  '9': 'Auto VPD', '13': 'Superciclo',
  'auto': 'Auto', 'manual': 'Manual', 'timer': 'Timer'
};

function modeToText(mode) {
  if (mode == null || mode === '' || mode === '--') return '--';
  const s = String(mode).trim();
  return MODE_TEXT[s] || s;
}

function reasonForRelay(r, params) {
  if (r && r.reason) return String(r.reason);
  const kind = r && r.modeKind ? r.modeKind : numericModeToKind(r && r.mode);
  if (kind === 'manual') return r.on ? 'Manual · encendido' : 'Manual · apagado';
  if (kind === 'timer')  {
    const cfg = (currentRelayCfg && currentRelayCfg['r' + r.idx]) || {};
    return `Timer · ${cfg.onTime || '--:--'} → ${cfg.offTime || '--:--'}`;
  }
  if (kind === 'auto') {
    if (!params) return r.on ? 'Auto · activo' : 'Auto · en espera';
    const i = r.idx;
    if (params.acRelay      == i) return r.on ? `Auto · T > ${params.tempAcOn}° → OFF a ${params.tempAcOff}°` : `Auto · enciende si T > ${params.tempAcOn}°`;
    if (params.heaterRelay  == i) return r.on ? `Auto · T < ${params.tempHeaterOn}° → OFF a ${params.tempHeaterOff}°` : `Auto · enciende si T < ${params.tempHeaterOn}°`;
    if (params.humidRelay   == i) return r.on ? `Auto · HR < ${params.humHumidOn}% → OFF a ${params.humHumidOff}%` : `Auto · enciende si HR < ${params.humHumidOn}%`;
    if (params.dehumidRelay == i) return r.on ? `Auto · HR > ${params.humDehumOn}% → OFF a ${params.humDehumOff}%` : `Auto · enciende si HR > ${params.humDehumOn}%`;
    return r.on ? 'Auto · activo' : 'Auto · sin asignación';
  }
  return modeToText(r && r.mode);
}

function relayPills(r) {
  const out = [];
  const kind = r && r.modeKind ? r.modeKind : numericModeToKind(r && r.mode);
  if (kind === 'auto')   out.push({ text: 'Automático', kind: 'blue' });
  if (kind === 'timer')  out.push({ text: 'Por horario', kind: 'purple' });
  if (kind === 'manual') out.push({ text: 'Manual',     kind: 'amber' });
  if (r && r.on === true)  out.push({ text: 'Encendido', kind: 'green' });
  if (r && r.on === false) out.push({ text: 'Apagado',   kind: '' });
  return out;
}

function escapeHtml(s) {
  return String(s == null ? '' : s)
    .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;').replace(/'/g, '&#39;');
}

// =========================
// Render dashboard de relés (grid 3×3)
// =========================
function renderDashRelays(state) {
  const host = byId('dashRelays');
  if (!host) return;
  const relays = (state && state.relays) || {};

  let html = '';
  for (let i = 1; i <= 8; i++) {
    const r = relays['r' + i] || {};
    const name = r.name || RELAY_DEFAULT_NAMES[i] || ('R' + i);
    const icon = RELAY_ICONS[i] || '⚡';
    const cfg  = currentRelayCfg['r' + i] || {};
    const kind = cfg.mode || numericModeToKind(r.mode) || 'auto';
    const modeLabel = kind === 'auto' ? 'auto' : kind === 'timer' ? 'timer' : 'manual';
    html += `
      <div class="relay-btn ${r.on ? 'is-on' : ''}">
        <span class="relay-btn-dot ${r.on ? 'on' : ''}"></span>
        <span class="relay-btn-icon">${icon}</span>
        <span class="relay-btn-name">${escapeHtml(name)}</span>
        <span class="relay-btn-mode">${modeLabel}</span>
      </div>`;
  }
  // 9ª celda vacía (para completar la grilla 3×3)
  html += '<div class="relay-btn" style="opacity:0;pointer-events:none"></div>';
  host.innerHTML = html;

  // mantener legacy ids
  for (let i = 1; i <= 8; i++) {
    const r = relays['r' + i] || {};
    setText(`nameR${i}`, r.name || RELAY_DEFAULT_NAMES[i] || `R${i}`);
    setText(`modeR${i}Badge`, modeToText(r.mode));
    const dot = byId(`dotR${i}`);
    if (dot) { dot.classList.remove('on', 'off'); dot.classList.add(r.on ? 'on' : 'off'); }
  }
}

// =========================
// Render DISPOSITIVOS (modo por relé)
// =========================
function renderDevices() {
  const host = byId('devicesList');
  if (!host) return;
  const relays = (lastState && lastState.relays) || {};

  const cards = [];
  for (let i = 1; i <= 8; i++) {
    const r = relays['r' + i] || {};
    const cfg = currentRelayCfg['r' + i] || { mode: 'auto', on: false, onTime: '06:00', offTime: '22:00' };
    const name = r.name || RELAY_DEFAULT_NAMES[i] || ('R' + i);
    const icon = RELAY_ICONS[i] || '⚡';

    const useTimerLabel = TIMER_LABEL_RELAYS.has(i);
    const modes = useTimerLabel ? ['auto','manual'] : ['auto','manual','timer'];
    const activeMode = useTimerLabel && cfg.mode === 'timer' ? 'auto' : cfg.mode;
    const modeBtns = modes.map(k => `
      <button type="button" class="mode-btn ${activeMode === k ? 'sel' : ''}"
              onclick="setRelayMode(${i}, '${k}')">
        ${k === 'auto' ? (useTimerLabel ? 'Timer' : 'Auto') : k === 'manual' ? 'Manual' : 'Timer'}
      </button>
    `).join('');

    let extra = '';
    if (activeMode === 'auto' && IRRIGATION_RELAYS.has(i)) {
      const pad2 = n => String(n == null ? 0 : n).padStart(2, '0');
      const pVal  = cfg.pulsoVal  ?? 30;
      const pUnit = cfg.pulsoUnit ?? 1;
      const paVal = cfg.pausaVal  ?? 10;
      const paUnit= cfg.pausaUnit ?? 60;
      const cics  = cfg.ciclos    ?? 4;
      const dias  = Array.isArray(cfg.diasRiego) ? cfg.diasRiego : [false,true,true,true,true,true,false];
      const { h: offH, m: offM } = calcIrrigOff(cfg);
      extra = `
        <div class="mode-extra">
          <div class="timer-grid">
            <div class="timer-box">
              <div class="timer-label">Hora inicio</div>
              <div class="hm-inputs">
                <input type="number" class="hm-input" min="0" max="23" value="${pad2(cfg.horaOn ?? 8)}"
                       onchange="setIrrigField(${i},'horaOn',+this.value)">
                <span class="hm-sep">h</span>
                <input type="number" class="hm-input" min="0" max="59" value="${pad2(cfg.minOn ?? 0)}"
                       onchange="setIrrigField(${i},'minOn',+this.value)">
                <span class="hm-sep">m</span>
              </div>
            </div>
            <div class="timer-box">
              <div class="timer-label">Fin (calculado)</div>
              <div class="hm-inputs">
                <span class="hm-calc" id="irrig-off-${i}">${pad2(offH)}:${pad2(offM)}</span>
              </div>
            </div>
          </div>
          <div class="irrig-param-row">
            <span class="timer-label">Pulso</span>
            <div class="irrig-num-unit">
              <input type="number" class="hm-input irrig-num" min="1" value="${pVal}"
                     onchange="setIrrigField(${i},'pulsoVal',+this.value)">
              <select class="cfg-select irrig-unit-sel" onchange="setIrrigField(${i},'pulsoUnit',+this.value)">
                <option value="1"    ${pUnit==1?   'selected':''}>seg</option>
                <option value="60"   ${pUnit==60?  'selected':''}>min</option>
                <option value="3600" ${pUnit==3600?'selected':''}>h</option>
              </select>
            </div>
          </div>
          <div class="irrig-param-row">
            <span class="timer-label">Pausa</span>
            <div class="irrig-num-unit">
              <input type="number" class="hm-input irrig-num" min="0" value="${paVal}"
                     onchange="setIrrigField(${i},'pausaVal',+this.value)">
              <select class="cfg-select irrig-unit-sel" onchange="setIrrigField(${i},'pausaUnit',+this.value)">
                <option value="1"    ${paUnit==1?   'selected':''}>seg</option>
                <option value="60"   ${paUnit==60?  'selected':''}>min</option>
                <option value="3600" ${paUnit==3600?'selected':''}>h</option>
              </select>
            </div>
          </div>
          <div class="irrig-param-row">
            <span class="timer-label">Ciclos/día</span>
            <input type="number" class="hm-input irrig-num" min="1" max="99" value="${cics}"
                   onchange="setIrrigField(${i},'ciclos',+this.value)">
          </div>
          <div class="irrig-dias">
            <div class="timer-label" style="margin-top:8px;margin-bottom:4px">Días de riego</div>
            <div class="irrig-dias-grid">
              ${['D','L','M','M','J','V','S'].map((d, idx) => `
                <div class="irrig-dia">
                  <span>${d}</span>
                  <input type="checkbox" ${dias[idx] ? 'checked' : ''}
                         onchange="setIrrigDia(${i},${idx},this.checked)">
                </div>`).join('')}
            </div>
          </div>
        </div>`;
    } else if (activeMode === 'auto' && ILLUMINATION_RELAYS.has(i)) {
      const pad = n => String(n == null ? 0 : n).padStart(2, '0');
      extra = `
        <div class="mode-extra">
          <div class="timer-grid">
            <div class="timer-box">
              <div class="timer-label">Encendido</div>
              <div class="hm-inputs">
                <input type="number" class="hm-input" min="0" max="23" value="${pad(cfg.horaOn)}"
                       onchange="setIllumTime(${i},'horaOn',+this.value)">
                <span class="hm-sep">h</span>
                <input type="number" class="hm-input" min="0" max="59" value="${pad(cfg.minOn)}"
                       onchange="setIllumTime(${i},'minOn',+this.value)">
                <span class="hm-sep">m</span>
              </div>
            </div>
            <div class="timer-box">
              <div class="timer-label">Apagado</div>
              <div class="hm-inputs">
                <input type="number" class="hm-input" min="0" max="23" value="${pad(cfg.horaOff)}"
                       onchange="setIllumTime(${i},'horaOff',+this.value)">
                <span class="hm-sep">h</span>
                <input type="number" class="hm-input" min="0" max="59" value="${pad(cfg.minOff)}"
                       onchange="setIllumTime(${i},'minOff',+this.value)">
                <span class="hm-sep">m</span>
              </div>
            </div>
          </div>
        </div>`;
    } else if (activeMode === 'auto') {
      extra = `<div class="mode-extra"><div class="hint">${escapeHtml(autoHintFor(i))}</div></div>`;
    } else if (activeMode === 'manual') {
      extra = `
        <div class="mode-extra">
          <div class="manual-actions">
            <button type="button" class="btn-on"  onclick="setManualState(${i}, true)">ENCENDER</button>
            <button type="button" class="btn-off" onclick="setManualState(${i}, false)">APAGAR</button>
          </div>
        </div>`;
    } else if (activeMode === 'timer') {
      extra = `
        <div class="mode-extra">
          <div class="timer-grid">
            <div class="timer-box">
              <div class="timer-label">Encendido</div>
              <input type="time" value="${escapeHtml(cfg.onTime || '06:00')}"
                     onchange="setTimerTime(${i}, 'onTime', this.value)">
            </div>
            <div class="timer-box">
              <div class="timer-label">Apagado</div>
              <input type="time" value="${escapeHtml(cfg.offTime || '22:00')}"
                     onchange="setTimerTime(${i}, 'offTime', this.value)">
            </div>
          </div>
        </div>`;
    }

    cards.push(`
      <div class="relay-card ${r.on ? 'is-on' : ''}">
        <div class="relay-header">
          <div class="relay-left">
            <div class="relay-icon">${icon}</div>
            <div class="relay-text">
              <div class="relay-name">${escapeHtml(name)} <span style="opacity:.5;font-weight:500">· R${i}</span></div>
              <div class="relay-status">${escapeHtml(reasonForRelay({...r, idx:i, modeKind: cfg.mode}, currentParams))}</div>
            </div>
          </div>
          <span class="relay-state-dot ${r.on ? 'on' : ''}"></span>
        </div>
        <div class="mode-selector">${modeBtns}</div>
        ${extra}
      </div>
    `);
  }
  host.innerHTML = cards.join('');
  IRRIGATION_RELAYS.forEach(idx => updateIrrigOff(idx));
}

function autoHintFor(i) {
  const p = currentParams;
  if (p.acRelay      == i) return `Encenderá cuando T > ${p.tempAcOn}°. Se apagará al llegar a ${p.tempAcOff}°.`;
  if (p.heaterRelay  == i) return `Encenderá cuando T < ${p.tempHeaterOn}°. Se apagará al llegar a ${p.tempHeaterOff}°.`;
  if (p.humidRelay   == i) return `Encenderá cuando HR < ${p.humHumidOn}%. Se apagará al llegar a ${p.humHumidOff}%.`;
  if (p.dehumidRelay == i) return `Encenderá cuando HR > ${p.humDehumOn}%. Se apagará al llegar a ${p.humDehumOff}%.`;
  return `Sin asignación en Parámetros: este relé no se activará en automático. Asignalo en la pestaña Parámetros.`;
}

// handlers Dispositivos
function setRelayMode(i, kind) {
  const cfg = currentRelayCfg['r' + i] || {};
  cfg.mode = kind;
  currentRelayCfg['r' + i] = cfg;
  dirtyRelayIds.add(i);
  saveRelayCfgLocal(currentRelayCfg);
  renderDevices();
}
function calcIrrigOff(cfg) {
  const startSec = (cfg.horaOn ?? 8) * 3600 + (cfg.minOn ?? 0) * 60;
  const ciclos   = Math.max(1, cfg.ciclos    ?? 4);
  const pulsoSec = (cfg.pulsoVal ?? 30) * (cfg.pulsoUnit ?? 1);
  const pausaSec = (cfg.pausaVal ?? 10) * (cfg.pausaUnit ?? 60);
  const totalSec = ciclos * pulsoSec + Math.max(0, ciclos - 1) * pausaSec;
  const offSec   = (startSec + totalSec) % 86400;
  return { h: Math.floor(offSec / 3600), m: Math.floor((offSec % 3600) / 60) };
}

function updateIrrigOff(i) {
  const el = document.getElementById('irrig-off-' + i);
  if (!el) return;
  const cfg = currentRelayCfg['r' + i] || {};
  const { h, m } = calcIrrigOff(cfg);
  el.textContent = String(h).padStart(2,'0') + ':' + String(m).padStart(2,'0');
}

function setIrrigField(i, key, val) {
  const cfg = currentRelayCfg['r' + i] || {};
  cfg[key] = val;
  currentRelayCfg['r' + i] = cfg;
  dirtyRelayIds.add(i);
  saveRelayCfgLocal(currentRelayCfg);
  updateIrrigOff(i);
}

function setIrrigDia(i, dayIdx, checked) {
  const cfg = currentRelayCfg['r' + i] || {};
  if (!Array.isArray(cfg.diasRiego)) cfg.diasRiego = [false,true,true,true,true,true,false];
  cfg.diasRiego[dayIdx] = checked;
  currentRelayCfg['r' + i] = cfg;
  dirtyRelayIds.add(i);
  saveRelayCfgLocal(currentRelayCfg);
}

function setIllumTime(i, key, val) {
  const cfg = currentRelayCfg['r' + i] || {};
  cfg[key] = val;
  currentRelayCfg['r' + i] = cfg;
  dirtyRelayIds.add(i);
  saveRelayCfgLocal(currentRelayCfg);
}
function setTimerTime(i, key, val) {
  const cfg = currentRelayCfg['r' + i] || {};
  cfg[key] = val;
  currentRelayCfg['r' + i] = cfg;
  dirtyRelayIds.add(i);
  saveRelayCfgLocal(currentRelayCfg);
}
function setManualState(i, on) {
  const cfg = currentRelayCfg['r' + i] || {};
  cfg.on = !!on;
  currentRelayCfg['r' + i] = cfg;
  saveRelayCfgLocal(currentRelayCfg);
  // intenta enviar al firmware
  postRelay(i).then(() => loadState());
}

async function saveAllRelays() {
  showToast('Guardando dispositivos…');
  try {
    for (let i = 1; i <= 8; i++) await postRelay(i);
    showToast('Dispositivos guardados');
    dirtyRelayIds.clear();
    loadState();
  } catch {
    showToast('Guardado localmente (firmware sin /api/relay)', true);
  }
}

async function postRelay(i) {
  const cfg = currentRelayCfg['r' + i] || {};
  const body = new URLSearchParams({
    n: String(i),
    mode: cfg.mode || 'auto',
    modeNum: String(kindToNumericMode(cfg.mode || 'auto')),
    on: cfg.on ? '1' : '0',
    onTime: cfg.onTime || '',
    offTime: cfg.offTime || '',
    horaOn:  String(cfg.horaOn  ?? ''),
    minOn:   String(cfg.minOn   ?? ''),
    horaOff: String(cfg.horaOff ?? ''),
    minOff:  String(cfg.minOff  ?? '')
  });
  if (IRRIGATION_RELAYS.has(i)) {
    const { h: offH, m: offM } = calcIrrigOff(cfg);
    body.append('pulsoVal',  String(cfg.pulsoVal  ?? 30));
    body.append('pulsoUnit', String(cfg.pulsoUnit ?? 1));
    body.append('pausaVal',  String(cfg.pausaVal  ?? 10));
    body.append('pausaUnit', String(cfg.pausaUnit ?? 60));
    body.append('ciclos',    String(cfg.ciclos    ?? 4));
    body.append('horaOffCalc', String(offH));
    body.append('minOffCalc',  String(offM));
    const dias = cfg.diasRiego || [false,true,true,true,true,true,false];
    dias.forEach((d, idx) => body.append('diaRiego' + idx, d ? '1' : '0'));
  }
  const res = await fetch('/api/relay', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded', 'X-Requested-With': 'XMLHttpRequest' },
    body
  });
  if (!res.ok) throw new Error('relay save failed');
  return res;
}

// =========================
// Render PARÁMETROS
// =========================
function renderParams() {
  const p = currentParams;
  const tempVals = normalizeQuadValues('temp', [p.tempHeaterOn, p.tempHeaterOff, p.tempAcOff, p.tempAcOn]);
  const humVals  = normalizeQuadValues('hum',  [p.humHumidOn,  p.humHumidOff,  p.humDehumOff, p.humDehumOn]);
  QUAD_KEYS.temp.forEach((k, i) => { currentParams[k] = tempVals[i]; });
  QUAD_KEYS.hum.forEach((k, i) => { currentParams[k] = humVals[i]; });
  setQuadRange('temp', ...tempVals);
  setQuadRange('hum',  ...humVals);

  // selects de asignación: 6 relés + ninguno
  ['acRelay','heaterRelay','humidRelay','dehumidRelay'].forEach(key => {
    const sel = byId('a-' + key);
    if (!sel) return;
    const current = currentParams[key];
    const opts = ['<option value="0">— Ninguno —</option>'];
    for (let i = 1; i <= 8; i++) {
      const name = (lastState && lastState.relays && lastState.relays['r' + i] && lastState.relays['r' + i].name) || RELAY_DEFAULT_NAMES[i] || ('Relé ' + i);
      opts.push(`<option value="${i}">R${i} · ${escapeHtml(name)}</option>`);
    }
    sel.innerHTML = opts.join('');
    sel.value = String(current == null ? 0 : current);
  });
}

function normalizeQuadValues(key, values) {
  const cfg = QUAD_CFG[key];
  const go = cfg.gapOuter, gi = cfg.gapInner;
  let [v1, v2, v3, v4] = values.map(v => parseFloat(v));
  if (![v1, v2, v3, v4].every(isFinite)) {
    [v1, v2, v3, v4] = key === 'temp' ? [18, 22, 25, 28] : [40, 55, 65, 75];
  }
  v1 = Math.max(cfg.min, Math.min(cfg.max, v1));
  v4 = Math.max(cfg.min, Math.min(cfg.max, v4));
  v2 = Math.max(v1 + go, Math.min(v3 - gi, v2));
  v3 = Math.max(v2 + gi, Math.min(v4 - go, v3));
  v2 = Math.max(cfg.min, Math.min(cfg.max, v2));
  v3 = Math.max(cfg.min, Math.min(cfg.max, v3));
  return [v1, v2, v3, v4];
}

function setQuadRange(key, v1, v2, v3, v4) {
  [v1, v2, v3, v4].forEach((val, i) => {
    const el = byId('qr-' + key + '-' + (i + 1));
    if (el) el.value = val;
  });
  updateQuadFill(key, v1, v2, v3, v4);
  updateQuadLabels(key, v1, v2, v3, v4);
}

function onQuadRange(key, which) {
  for (let i = 1; i <= 4; i++) {
    const el = byId('qr-' + key + '-' + i);
    const thumb = byId('qt-' + key + '-' + i);
    if (thumb) thumb.classList.toggle('is-active', i === which);
  }
  const cfg = QUAD_CFG[key];
  const go = cfg.gapOuter, gi = cfg.gapInner;
  const vals = [1,2,3,4].map(i => parseFloat(byId('qr-' + key + '-' + i).value));
  let [v1, v2, v3, v4] = vals;

  if (which === 1) v1 = Math.min(v1, v2 - go);
  if (which === 2) { v2 = Math.max(v2, v1 + go); v2 = Math.min(v2, v3 - gi); }
  if (which === 3) { v3 = Math.max(v3, v2 + gi); v3 = Math.min(v3, v4 - go); }
  if (which === 4) v4 = Math.max(v4, v3 + go);

  v1 = Math.max(cfg.min, v1);
  v4 = Math.min(cfg.max, v4);
  v2 = Math.max(v1 + go, Math.min(v3 - gi, v2));
  v3 = Math.max(v2 + gi, Math.min(v4 - go, v3));

  [v1, v2, v3, v4].forEach((val, i) => {
    const el = byId('qr-' + key + '-' + (i + 1));
    if (el) el.value = val;
    currentParams[QUAD_KEYS[key][i]] = val;
  });
  dirtyParams = true;
  saveParamsLocal(currentParams);
  updateQuadFill(key, v1, v2, v3, v4);
  updateQuadLabels(key, v1, v2, v3, v4);
}

function updateQuadFill(key, v1, v2, v3, v4) {
  const cfg = QUAD_CFG[key];
  const span = cfg.max - cfg.min;
  const pct = v => (v - cfg.min) / span * 100;
  const cold    = byId('qf-' + key + '-cold');
  const comfort = byId('qf-' + key + '-comfort');
  const hot     = byId('qf-' + key + '-hot');
  if (cold) { cold.style.left = pct(v1) + '%'; cold.style.width = (pct(v2) - pct(v1)) + '%'; }
  if (comfort) { comfort.style.left = pct(v2) + '%'; comfort.style.width = (pct(v3) - pct(v2)) + '%'; }
  if (hot)  { hot.style.left = pct(v3) + '%'; hot.style.width = (pct(v4) - pct(v3)) + '%'; }
}

function applySetpoint(key) {
  const el = byId('sp-' + key);
  if (!el) return;
  let S = parseFloat(el.value);
  if (!isFinite(S)) return;
  const { min: absMin, max: absMax, step, gapOuter: go, gapInner: gi } = QUAD_CFG[key];
  const roundToStep = v => Math.round(v / step) * step;
  S = Math.max(absMin, Math.min(absMax, roundToStep(S)));
  const halfInner = Math.ceil((gi / 2) / step) * step;
  let v2 = roundToStep(S - halfInner);
  let v3 = roundToStep(S + halfInner);
  let v1 = v2 - go;
  let v4 = v3 + go;
  // clamp to absolute range
  v1 = Math.max(absMin, v1);
  v4 = Math.min(absMax, v4);
  v2 = Math.max(v1 + go, Math.min(v3 - gi, v2));
  v3 = Math.max(v2 + gi, Math.min(v4 - go, v3));
  QUAD_KEYS[key].forEach((k, i) => { currentParams[k] = [v1, v2, v3, v4][i]; });
  dirtyParams = true;
  saveParamsLocal(currentParams);
  setQuadRange(key, v1, v2, v3, v4);
}

function updateQuadLabels(key, v1, v2, v3, v4) {
  const fmt = QUAD_CFG[key].fmt;
  [v1, v2, v3, v4].forEach((val, i) => {
    const el = byId('qv-' + key + '-' + (i + 1));
    if (el) el.textContent = fmt(val);
  });
  updateQuadScale(key, v1, v2, v3, v4);
}

function updateQuadScale(key, v1, v2, v3, v4) {
  const cfg = QUAD_CFG[key];
  const span = cfg.max - cfg.min;
  const pct = v => (v - cfg.min) / span * 100;
  const pointText = v => key === 'temp' ? v.toFixed(1) : String(Math.round(v));
  [v1, v2, v3, v4].forEach((val, i) => {
    const el = byId('qp-' + key + '-' + (i + 1));
    const thumb = byId('qt-' + key + '-' + (i + 1));
    const left = pct(val) + '%';
    if (el) {
      el.style.left = left;
      el.textContent = pointText(val);
    }
    if (thumb) thumb.style.left = left;
  });

  const center = (v2 + v3) / 2;
  const marker = byId('qc-' + key);
  const markerValue = byId('qcv-' + key);
  const setpoint = byId('sp-' + key);
  const icon = key === 'temp' ? '🌡️' : '💧';
  if (marker) marker.style.left = pct(center) + '%';
  if (markerValue) markerValue.innerHTML = `<span class="quad-setpoint-icon">${icon}</span>${pointText(center)}`;
  if (setpoint) setpoint.value = key === 'temp' ? center.toFixed(1) : String(Math.round(center));
}

function formatParam(key, val) {
  const v = parseFloat(val);
  if (!isFinite(v)) return '--';
  if (key === 'tempMin' || key === 'tempMax') return v.toFixed(1) + ' °C';
  return Math.round(v) + ' %';
}

function onAssignChange(key, val) {
  currentParams[key] = parseInt(val, 10) || 0;
  dirtyParams = true;
  saveParamsLocal(currentParams);
}

async function saveParams() {
  showToast('Guardando parámetros…');
  try {
    const body = new URLSearchParams(Object.entries(currentParams).reduce((acc, [k, v]) => {
      acc[k] = String(v);
      return acc;
    }, {}));
    const res = await fetch('/api/params', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded', 'X-Requested-With': 'XMLHttpRequest' },
      body
    });
    if (!res.ok) throw new Error();
    showToast('Parámetros guardados');
    dirtyParams = false;
    loadState();
  } catch {
    showToast('Guardado localmente (firmware sin /api/params)', true);
  }
}

// =========================
// Histórico 24h (localStorage)
// =========================
const HIST_KEY = 'druida_history_v1';
const HIST_HOURS = 24;
const SAMPLE_INTERVAL_MS = 60 * 1000;

function loadHistory() {
  try {
    const raw = localStorage.getItem(HIST_KEY);
    if (!raw) return [];
    const arr = JSON.parse(raw);
    if (!Array.isArray(arr)) return [];
    const cutoff = Date.now() - HIST_HOURS * 3600 * 1000;
    return arr.filter(p => p && p.t >= cutoff);
  } catch { return []; }
}
function saveHistory(arr) {
  try { localStorage.setItem(HIST_KEY, JSON.stringify(arr)); } catch {}
}
function pushHistory(temp, hum) {
  const t = parseFloat(temp), h = parseFloat(hum);
  if (!isFinite(t) && !isFinite(h)) return;
  const arr = loadHistory();
  const last = arr[arr.length - 1];
  if (last && Date.now() - last.t < SAMPLE_INTERVAL_MS) return;
  arr.push({ t: Date.now(), temp: isFinite(t) ? t : null, hum: isFinite(h) ? h : null });
  saveHistory(arr);
}
function fmtTime(ts) {
  const d = new Date(ts);
  const pad = n => String(n).padStart(2, '0');
  return pad(d.getHours()) + ':' + pad(d.getMinutes());
}
function computeExtremes(arr, key) {
  let min = null, max = null, minTs = null, maxTs = null;
  arr.forEach(p => {
    const v = p[key];
    if (v == null || !isFinite(v)) return;
    if (min == null || v < min) { min = v; minTs = p.t; }
    if (max == null || v > max) { max = v; maxTs = p.t; }
  });
  return { min, max, minTs, maxTs };
}
function updateExtremes() {
  const arr = loadHistory();
  const t = computeExtremes(arr, 'temp');
  const h = computeExtremes(arr, 'hum');
  setText('tempMin', t.min != null ? t.min.toFixed(1) + '°' : '--');
  setText('tempMax', t.max != null ? t.max.toFixed(1) + '°' : '--');
  setText('tempMinHora', t.minTs ? fmtTime(t.minTs) : '--:--');
  setText('tempMaxHora', t.maxTs ? fmtTime(t.maxTs) : '--:--');
  setText('humMin', h.min != null ? Math.round(h.min) + '%' : '--');
  setText('humMax', h.max != null ? Math.round(h.max) + '%' : '--');
  setText('humMinHora', h.minTs ? fmtTime(h.minTs) : '--:--');
  setText('humMaxHora', h.maxTs ? fmtTime(h.maxTs) : '--:--');
}

// =========================
// Chart canvas
// =========================
function drawChart() {
  const cv = byId('historyChart');
  const empty = byId('chartEmpty');
  if (!cv) return;
  const arr = loadHistory();
  if (arr.length < 2) {
    cv.classList.add('hidden');
    if (empty) empty.classList.remove('hidden');
    return;
  }
  cv.classList.remove('hidden');
  if (empty) empty.classList.add('hidden');

  const dpr = window.devicePixelRatio || 1;
  const cssW = cv.clientWidth || 600, cssH = cv.clientHeight || 220;
  cv.width = Math.round(cssW * dpr);
  cv.height = Math.round(cssH * dpr);
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cssW, cssH);

  const padL = 30, padR = 30, padT = 12, padB = 22;
  const w = cssW - padL - padR, h = cssH - padT - padB;
  const now = Date.now();
  const tStart = now - HIST_HOURS * 3600 * 1000;
  const xs = t => padL + ((t - tStart) / (now - tStart)) * w;
  const tMin = 5, tMax = 40;
  const yT = v => padT + h - ((v - tMin) / (tMax - tMin)) * h;
  const yH = v => padT + h - (v / 100) * h;

  ctx.strokeStyle = 'rgba(255,255,255,0.06)';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const yy = padT + (h * i / 4);
    ctx.beginPath(); ctx.moveTo(padL, yy); ctx.lineTo(padL + w, yy); ctx.stroke();
  }
  ctx.fillStyle = '#565c70';
  ctx.font = '10px -apple-system, sans-serif';
  ctx.textAlign = 'center';
  for (let h6 = 0; h6 <= 4; h6++) {
    const ts = tStart + (now - tStart) * (h6 / 4);
    const x = padL + (w * h6 / 4);
    ctx.fillText(fmtTime(ts), x, cssH - 6);
  }
  ctx.textAlign = 'right'; ctx.fillStyle = '#f87171';
  ctx.fillText(tMax + '°', padL - 4, padT + 4);
  ctx.fillText(tMin + '°', padL - 4, padT + h);
  ctx.textAlign = 'left'; ctx.fillStyle = '#60a5fa';
  ctx.fillText('100%', padL + w + 4, padT + 4);
  ctx.fillText('0%',   padL + w + 4, padT + h);

  drawSeries(ctx, arr, 'temp', xs, yT, '#f87171');
  drawSeries(ctx, arr, 'hum',  xs, yH, '#60a5fa');

  const tEx = computeExtremes(arr, 'temp');
  if (tEx.minTs) markPoint(ctx, xs(tEx.minTs), yT(tEx.min), '#60a5fa');
  if (tEx.maxTs) markPoint(ctx, xs(tEx.maxTs), yT(tEx.max), '#fbbf24');
}
function drawSeries(ctx, arr, key, xs, ys, color) {
  ctx.strokeStyle = color; ctx.lineWidth = 1.8;
  ctx.beginPath();
  let started = false;
  arr.forEach(p => {
    const v = p[key];
    if (v == null || !isFinite(v)) return;
    const x = xs(p.t), y = ys(v);
    if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
  });
  ctx.stroke();
}
function markPoint(ctx, x, y, color) {
  ctx.fillStyle = color;
  ctx.beginPath(); ctx.arc(x, y, 3.5, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = '#0f1117'; ctx.lineWidth = 1.5; ctx.stroke();
}

// =========================
// VPD / online / utils
// =========================
function updateVpdBadge(vpdRaw) {
  const el = byId('vpdBadge');
  if (!el) return;
  const v = parseFloat(String(vpdRaw).replace(',', '.'));
  if (!isFinite(v)) { el.textContent = '--'; el.className = 'vpd-badge'; return; }
  let label = 'Óptimo', cls = 'optimal';
  if (v < 0.4)       { label = 'Muy bajo'; cls = 'low'; }
  else if (v < 0.8)  { label = 'Bajo';     cls = 'moderate'; }
  else if (v <= 1.2) { label = 'Óptimo';   cls = 'optimal'; }
  else if (v <= 1.6) { label = 'Moderado'; cls = 'moderate'; }
  else               { label = 'Alto';     cls = 'high'; }
  el.textContent = label;
  el.className = 'vpd-badge ' + cls;
}
function setOnline(ok) {
  const b = byId('onlineBadge'); const l = byId('onlineLabel');
  if (!b) return;
  b.classList.toggle('offline', !ok);
  if (l) l.textContent = ok ? 'En línea' : 'Sin conexión';
}
function setText(id, val) { const el = byId(id); if (el) el.textContent = val; }
function getLocalFechaHora() {
  const d   = new Date();
  const pad = n => String(n).padStart(2, '0');
  return `${d.getDate()}/${d.getMonth()+1}/${d.getFullYear()} ${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
}
function formatNum(raw, suffix, decimals) {
  if (raw == null || raw === '' || raw === '--') return '--';
  const s = String(raw);
  const n = parseFloat(s.replace(',', '.'));
  if (!isFinite(n)) return s;
  const d = decimals != null ? decimals : 1;
  return n.toFixed(d) + (suffix || '');
}

// =========================
// loadState
// =========================
async function loadState() {
  try {
    const res = await fetch('/api/state');
    if (!res.ok) throw new Error('state-' + res.status);
    // El ESP32 puede enviar NaN o Infinity — no son JSON válido, los sanitizamos
    const raw = await res.text();
    const s   = JSON.parse(raw.replace(/:[ \t]*NaN\b/g, ': null').replace(/:[ \t]*[-]?Infinity\b/g, ': null'));
    lastState = s;

    setText('temp',      formatNum(s.temp, '°'));
    setText('hum',       formatNum(s.hum, '%', 0));
    setText('dpv',       formatNum(s.dpv, ' kPa'));
    setText('fechaHora', s.fechaHora || '');

    if (s.diaVege  !== undefined) setText('diaVege',  'DIA VEGE: '   + s.diaVege);
    if (s.diaFlora !== undefined) setText('diaFlora', 'DIA FLORA: '  + s.diaFlora);
    if (s.diaSuper !== undefined) {
      setText('diaSuper', 'SUPERCICLOS: ' + s.diaSuper);
      const el = byId('diaSuper'); if (el) el.classList.remove('hidden');
    }

    ingestServerState(s);
    updateVpdBadge(s.dpv);
    renderDashRelays(s);

    pushHistory(s.temp, s.hum);
    updateExtremes();
    drawChart();

    if (!byId('devicesView').classList.contains('preloaded-hidden')) renderDevices();
    if (!byId('paramsView').classList.contains('preloaded-hidden'))  renderParams();

    setOnline(true);
  } catch {
    // Fallback: si /api/state falla, intentar mostrar datos desde /api/sensors
    setOnline(await fetchSensorFallback());
  }
}

// Fallback: muestra datos del sensor ambiente en el dashboard cuando /api/state falla
async function fetchSensorFallback() {
  try {
    const res = await fetch('/api/sensors');
    if (!res.ok) return false;
    const raw = await res.text();
    const d   = JSON.parse(raw.replace(/:[ \t]*NaN\b/g, ': null').replace(/:[ \t]*[-]?Infinity\b/g, ': null'));
    if (!d || !Array.isArray(d.sensors)) return false;
    // Buscar el primer sensor ambiente habilitado con datos
    const ambient = d.sensors.find(s => s.enabled !== false && s.type !== 'soil' && s.temp != null);
    if (!ambient) return false;
    setText('temp', formatNum(ambient.temp, '°'));
    if (ambient.hum != null) setText('hum', formatNum(ambient.hum, '%', 0));
    if (ambient.dpv != null) {
      setText('dpv', formatNum(ambient.dpv, ' kPa'));
      updateVpdBadge(ambient.dpv);
    }
    // Mostrar la hora local del navegador como referencia
    setText('fechaHora', getLocalFechaHora());

    pushHistory(ambient.temp, ambient.hum);
    updateExtremes();
    drawChart();
    return true; // hay datos → mostrar "En línea"
  } catch {
    return false;
  }
}

// =========================
// Panel views
// =========================
const ALL_VIEWS = [
  'homeView','devicesView','paramsView',
  'panelView','sensorsView','sensorConfigView','wifiView','alertsView','updatesView'
];

function setViewEx(id) {
  ALL_VIEWS.forEach(v => {
    const el = byId(v);
    if (el) el.classList.add('preloaded-hidden');
  });
  const target = byId(id);
  if (target) target.classList.remove('preloaded-hidden');

  const tabMap = { homeView:'homeView', devicesView:'devicesView', paramsView:'paramsView' };
  document.querySelectorAll('.tab').forEach(t => {
    t.classList.toggle('active', t.dataset.tab === (tabMap[id] || '__none__'));
  });
  window.scrollTo({ top: 0, behavior: 'instant' in window ? 'instant' : 'auto' });
}

// Patch setView to use extended list
const _setViewOrig = setView;
// Override global setView so existing calls still work
function setView(id) {
  ALL_VIEWS.forEach(v => {
    const el = byId(v);
    if (el) el.classList.add('preloaded-hidden');
  });
  const target = byId(id);
  if (target) target.classList.remove('preloaded-hidden');
  const tabMap = { homeView:'homeView', devicesView:'devicesView', paramsView:'paramsView' };
  document.querySelectorAll('.tab').forEach(t => {
    t.classList.toggle('active', t.dataset.tab === (tabMap[id] || '__none__'));
  });
  window.scrollTo({ top: 0, behavior: 'instant' in window ? 'instant' : 'auto' });
}

function showPreloadedPanel()  { setView('panelView');  loadInfo(); }
function showPreloadedSensors(){ setView('sensorsView'); loadSensors(); }
function showPreloadedWifi()   { setView('wifiView'); }
function showPreloadedAlerts() { setView('alertsView'); loadAlerts(); }
function showPreloadedUpdates() {
  setView('updatesView');
}
// =========================
// Info del dispositivo (/api/info)
// =========================
async function loadInfo() {
  try {
    const r = await fetch('/api/info');
    if (!r.ok) throw new Error();
    const d = await r.json();
    setText('pVersion', d.version || '--');
    setText('pPlan',    d.plan    || '--');
    setText('pUptime',  d.uptime  || '--');
    setText('pWifiSsid', d.ssid   || '--');
  } catch {
    setText('pVersion', '3.0');
    setText('pPlan',    'Estándar');
    setText('pUptime',  '--');
  }
}

// =========================
// Sensores (/api/sensors)
// =========================
const SENSOR_CFG_KEY = 'druida_sensors_v2';

const SENSOR_DEFAULTS = [
  { id:'s1', name:'Ambiente 1', type:'ambient', addr:'1', enabled:true  },
  { id:'s2', name:'Ambiente 2', type:'ambient', addr:'2', enabled:false },
  { id:'s3', name:'Ambiente 3', type:'ambient', addr:'3', enabled:false },
  { id:'s4', name:'Ambiente 4', type:'ambient', addr:'4', enabled:false },
  { id:'s5', name:'Suelo 1',    type:'soil',    addr:'5', enabled:false },
  { id:'s6', name:'Suelo 2',    type:'soil',    addr:'6', enabled:false },
];

function loadSensorCfg() {
  try {
    const raw = localStorage.getItem(SENSOR_CFG_KEY);
    if (raw) {
      const saved = JSON.parse(raw);
      return SENSOR_DEFAULTS.map(d => Object.assign({}, d, saved.find(s => s.id === d.id) || {}));
    }
  } catch {}
  return SENSOR_DEFAULTS.map(d => Object.assign({}, d));
}
function saveSensorCfgLocal(arr) {
  try { localStorage.setItem(SENSOR_CFG_KEY, JSON.stringify(arr)); } catch {}
}

let currentSensors = loadSensorCfg();
let editingSensorId = null;

async function loadSensors() {
  try {
    const r = await fetch('/api/sensors');
    if (!r.ok) throw new Error();
    const d = await r.json();
    if (d.sensors && Array.isArray(d.sensors)) {
      // merge server data into local cfg
      currentSensors = currentSensors.map(cfg => {
        const srv = d.sensors.find(s => s.id === cfg.id || s.addr == cfg.addr);
        if (!srv) return cfg;
        return Object.assign({}, cfg, {
          enabled: srv.enabled !== undefined ? !!srv.enabled : cfg.enabled,
          connected: srv.connected,
          temp: numberOrNull(srv.temp),
          hum: numberOrNull(srv.hum),
          dpv: numberOrNull(srv.dpv),
          ec: numberOrNull(srv.ec),
          moisture: numberOrNull(srv.moisture ?? srv.hum),
          addr: srv.addr != null ? String(srv.addr) : cfg.addr,
          type: srv.type || cfg.type,
          name: srv.name || cfg.name
        });
      });
    }
  } catch { /* sin /api/sensors — mostrar config local */ }
  renderSensors();
}

function renderSensors() {
  const ambient = currentSensors.filter(s => s.type === 'ambient' && s.enabled);
  const soil    = currentSensors.filter(s => s.type === 'soil'    && s.enabled);
  const canAddAmbient = ambient.length < 4;
  const canAddSoil    = soil.length < 2;

  byId('sensorsAmbient').innerHTML =
    (ambient.length ? ambient.map(sensorCardHtml).join('') : `<div class="sensor-empty">Sensor ambiente ID 1 incluido de fabrica.</div>`) +
    (canAddAmbient ? `<button class="add-sensor-btn" onclick="addSensor('ambient')">+ Agregar sensor de ambiente</button>` : '');

  byId('sensorsSoil').innerHTML =
    (soil.length ? soil.map(sensorCardHtml).join('') : `<div class="sensor-empty">No hay sensores de suelo agregados.</div>`) +
    (canAddSoil ? `<button class="add-sensor-btn" onclick="addSensor('soil')">+ Agregar sensor de suelo</button>` : '');
}

function addSensor(type) {
  const next = currentSensors.find(s => s.type === type && !s.enabled);
  if (!next) return;
  next.enabled = true;
  saveSensorCfgLocal(currentSensors);
  showSensorConfig(next.id);
}

function removeSensor(id) {
  const s = currentSensors.find(x => x.id === id);
  if (!s) return;
  if (s.id === 's1') {
    showToast('El sensor ambiente 1 viene de fabrica');
    return;
  }
  s.enabled = false;
  saveSensorCfgLocal(currentSensors);
  saveSensorEnabledToDevice(s, false);
  showToast('Sensor eliminado');
  showPreloadedSensors();
}

function sensorCardHtml(s) {
  const conn = s.connected === true;
  const conn_unknown = s.connected === undefined;
  const isOn = conn || conn_unknown;

  let reading = '';
  if (s.type === 'ambient') {
    if (s.temp != null) reading += s.temp.toFixed(1) + '°C';
    if (s.hum  != null) reading += (reading ? '<br>' : '') + Math.round(s.hum) + '% HR';
    if (s.dpv  != null) reading += (reading ? '<br>' : '') + 'DPV ' + s.dpv.toFixed(1);
  } else {
    if (s.moisture != null) reading += 'Hum: ' + Math.round(s.moisture) + '%';
    if (s.ec != null)       reading += (reading ? '<br>' : '') + 'EC ' + Math.round(s.ec);
    if (s.temp != null)     reading += (reading ? '<br>' : '') + s.temp.toFixed(1) + '°C';
  }
  if (!reading) reading = conn_unknown ? '<span style="color:var(--text-3)">sin datos</span>' : '';

  const badge = conn_unknown
    ? `<span class="sensor-badge" style="background:var(--amber-bg);color:var(--amber-text);border-color:rgba(245,158,11,.2)">Sin verificar</span>`
    : conn
      ? `<span class="sensor-badge">Conectado</span>`
      : `<span class="sensor-badge off">Sin señal</span>`;

  return `
    <div class="sensor-card ${conn ? 'connected' : ''}" onclick="showSensorConfig('${s.id}')">
      <span class="sensor-dot ${isOn ? 'on' : ''}"></span>
      <div class="sensor-info">
        <div class="sensor-name">${escapeHtml(s.name)}</div>
        <div class="sensor-meta">Dirección: ${escapeHtml(s.addr)} · ${s.type === 'ambient' ? 'Ambiente' : 'Suelo'}</div>
      </div>
      <div style="display:flex;flex-direction:column;align-items:flex-end;gap:5px">
        ${badge}
        ${reading ? `<div class="sensor-reading">${reading}</div>` : ''}
      </div>
    </div>`;
}

function showSensorConfig(id) {
  editingSensorId = id;
  const s = currentSensors.find(x => x.id === id);
  if (!s) return;
  const card = byId('sensorConfigCard');
  if (!card) return;
  card.innerHTML = `
    <div class="card-title-row">
      <span>${s.type === 'ambient' ? '🌡️' : '🌱'}</span>
      <div class="card-title">Configurar sensor</div>
    </div>
    <label class="cfg-label">Nombre</label>
    <input type="text" id="scName" class="cfg-input" value="${escapeHtml(s.name)}">
    <label class="cfg-label">Dirección / ID</label>
    <input type="number" id="scAddr" class="cfg-input" value="${escapeHtml(s.addr)}" min="${s.type === 'ambient' ? 1 : 5}" max="${s.type === 'ambient' ? 4 : 6}" step="1"
           placeholder="Ej: 1  ·  0x40  ·  28FF3A...">
    <div class="param-hint" style="margin-top:6px">
      Usá el mismo identificador que configuraste en el firmware
      (número Modbus, dirección I²C en hex, o ROM de sensor 1-Wire).
    </div>
    <label class="cfg-label">Tipo</label>
    <select id="scType" class="cfg-select" disabled>
      <option value="ambient" ${s.type==='ambient'?'selected':''}>Ambiente (T° y HR)</option>
      <option value="soil"    ${s.type==='soil'   ?'selected':''}>Suelo (humedad)</option>
    </select>
    ${s.id === 's1' ? '' : `<button type="button" class="btn-remove-sensor" onclick="removeSensor('${s.id}')">Eliminar sensor</button>`}`;
  setView('sensorConfigView');
}

function saveSensorConfig() {
  const s = currentSensors.find(x => x.id === editingSensorId);
  if (!s) return;
  s.name = (byId('scName').value || s.name).trim();
  s.type = byId('scType').value;
  const addr = normalizeSensorAddress(s.type, byId('scAddr').value || s.addr);
  if (!addr) {
    showToast(s.type === 'ambient' ? 'ID ambiente valido: 1 a 4' : 'ID suelo valido: 5 o 6', true);
    return;
  }
  s.addr = String(addr);
  s.enabled = true;
  saveSensorCfgLocal(currentSensors);
  const body = new URLSearchParams({ id: s.id, name: s.name, addr: s.addr, sensor: s.addr, type: s.type, enabled: '1' });
  fetch('/api/sensor', {
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded','X-Requested-With':'XMLHttpRequest'},
    body
  }).catch(() => {});
  showToast('Sensor guardado');
  showPreloadedSensors();
}

function numberOrNull(v) {
  if (v === null || v === undefined || v === '--') return null;
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

function normalizeSensorAddress(type, raw) {
  const n = parseInt(String(raw).replace(/[^\d]/g, ''), 10);
  if (type === 'ambient') return n >= 1 && n <= 4 ? n : 0;
  return n >= 5 && n <= 6 ? n : 0;
}

function saveSensorEnabledToDevice(s, enabled) {
  const body = new URLSearchParams({ sensor: s.addr, enabled: enabled ? '1' : '0' });
  fetch('/api/sensor', {
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded','X-Requested-With':'XMLHttpRequest'},
    body
  }).catch(() => {});
}

// =========================
// WiFi (panel)
// =========================
async function scanWifiPanel() {
  const sel = byId('wifiScanSel');
  if (!sel) return;
  sel.classList.remove('hidden');
  sel.innerHTML = '<option>Buscando…</option>';
  try {
    const r = await fetch('/scanWiFi');
    const j = await r.json();
    sel.innerHTML = '';
    if (!j.networks || !j.networks.length) {
      sel.innerHTML = '<option>Sin redes</option>'; return;
    }
    j.networks.forEach(n => {
      const o = document.createElement('option');
      o.value = n; o.textContent = n; sel.appendChild(o);
    });
    sel.onchange = () => { byId('wifiSsid').value = sel.value; };
  } catch { sel.innerHTML = '<option>Error</option>'; }
}

async function saveWifi() {
  const ssid = (byId('wifiSsid') || {}).value || '';
  const pass = (byId('wifiPass') || {}).value || '';
  if (!ssid) { showToast('Ingresá el nombre de la red', true); return; }
  showToast('Guardando WiFi…');
  try {
    const body = new URLSearchParams({ ssid, pass });
    const r = await fetch('/saveWiFi', {
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded','X-Requested-With':'XMLHttpRequest'},
      body
    });
    if (!r.ok) throw new Error();
    showToast('WiFi guardado — reiniciando…');
    setText('pWifiSsid', ssid);
  } catch { showToast('Error al guardar WiFi', true); }
}

async function syncTimePanel() {
  const msg = byId('wifiSyncMsg');
  if (msg) msg.textContent = 'Sincronizando…';
  try {
    const d = new Date();
    const body = new URLSearchParams({
      y: d.getFullYear(), mo: d.getMonth()+1, d: d.getDate(),
      h: d.getHours(), mi: d.getMinutes(), s: d.getSeconds(), w: d.getDay()
    });
    const r = await fetch('/syncPhoneTime', {
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded','X-Requested-With':'XMLHttpRequest'},
      body
    });
    const j = await r.json().catch(() => null);
    if (msg) msg.textContent = (r.ok && j && j.ok) ? '✅ Hora sincronizada' : '❌ Error';
  } catch { if (msg) msg.textContent = '❌ Error'; }
}

// =========================
// Alertas
// =========================
const ALERTS_KEY = 'druida_alerts_v1';
function loadAlerts() {
  try {
    const a = JSON.parse(localStorage.getItem(ALERTS_KEY) || '{}');
    const t = id => { const el = byId(id); if (el) el.classList.toggle('on', !!a[id]); };
    t('alertSensor'); t('alertTemp'); t('alertHum');
  } catch {}
}
function saveAlerts() {
  const val = id => !!(byId(id) && byId(id).classList.contains('on'));
  const a = { alertSensor: val('alertSensor'), alertTemp: val('alertTemp'), alertHum: val('alertHum') };
  try { localStorage.setItem(ALERTS_KEY, JSON.stringify(a)); } catch {}
  const body = new URLSearchParams(Object.entries(a).map(([k,v]) => [k, v?'1':'0']));
  fetch('/api/alerts', { method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded','X-Requested-With':'XMLHttpRequest'}, body
  }).catch(() => {});
  showToast('Alertas guardadas');
}


// =========================
// Actualizaciones OTA
// =========================
async function postOTAUpdate(url, label) {
  const msg = byId('updateMsg');

  const ok = confirm(
    label + '\n\n' +
    'El equipo puede reiniciarse durante el proceso.\n' +
    'No cortes la alimentación.\n\n' +
    '¿Continuar?'
  );

  if (!ok) return;

  if (msg) msg.textContent = 'Iniciando ' + label + '…';
  showToast('Iniciando ' + label + '…');

  try {
    const r = await fetch(url, {
      method: 'POST',
      headers: {
        'X-Requested-With': 'XMLHttpRequest'
      }
    });

    const txt = await r.text();

    if (!r.ok) {
      if (msg) msg.textContent = '❌ ' + txt;
      showToast(txt || 'Error en actualización', true);
      return;
    }

    if (msg) msg.textContent = '✅ ' + (txt || label + ' iniciada');
    showToast(txt || label + ' iniciada');

  } catch (e) {
    if (msg) msg.textContent = '❌ Error de conexión';
    showToast('Error de conexión', true);
  }
}

function updateBackend() {
  postOTAUpdate('/updateBackend', 'actualización Backend');
}

function updateFrontend() {
  postOTAUpdate('/updateFrontend', 'actualización Frontend');
}

function updateGeneral() {
  postOTAUpdate('/updateGeneral', 'actualización General');
}

// =========================
// Init
// =========================
document.addEventListener('DOMContentLoaded', () => {
  showPreloadedHome();
  loadState();
  setInterval(loadState, 5000);
  window.addEventListener('resize', () => drawChart());
});
