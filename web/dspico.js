// ===========================================================================
// DSPico EQ configurator — logic, AutoEQ import, response curve, WebUSB.
// The wire protocol constants here MUST match src/config_usb.c on the device.
// ===========================================================================
'use strict';

// Bumped on every configurator change and printed in the Ready line, so a
// stale/cached deployment is immediately visible in any pasted log.
const APP_REV = 'r9';

// --- Shared protocol contract (keep in sync with the firmware) --------------
const PROTO = {
  ITF_VENDOR: 2,            // vendor interface number in the composite device
  REQ_INFO:       0x01,     // IN  : device info (magic/version/maxbands/rate)
  REQ_GET_STATE:  0x02,     // IN  : pre-gain + all bands
  REQ_SET_PREGAIN:0x03,     // OUT : f32 dB
  REQ_SET_BAND:   0x04,     // OUT : wIndex=idx, 16-byte band record
  REQ_COMMIT:     0x05,     // OUT : persist to flash
  REQ_RESET:      0x06,     // OUT : flat/defaults
  REQ_GET_LOG:    0x07,     // IN  : drain the device's diagnostic log text
  MAGIC: 0x51505344,        // 'DSPQ' little-endian
};

// Band types — values match peq_type_t in dsp_peq.h.
const TYPE = { PEAKING:0, LOWSHELF:1, HIGHSHELF:2, LOWPASS:3, HIGHPASS:4 };
const TYPE_NAME = ['Peaking','Low shelf','High shelf','Low pass','High pass'];

// Allowed ranges — match dsp_peq.h.
const LIMIT = { FC_MIN:1, FC_MAX:20000, GAIN_MIN:-12, GAIN_MAX:12, Q_MIN:0.1, Q_MAX:10 };
let MAX_BANDS = 16;         // updated from device INFO when connected
let SAMPLE_RATE = 48000;    // updated from device INFO when connected

// --- Model ------------------------------------------------------------------
const state = { preGainDb: 0, bands: [] };

function newBand(o={}) {
  return {
    enabled: o.enabled ?? true,
    type:    o.type ?? TYPE.PEAKING,
    fc:      o.fc ?? 1000,
    gain:    o.gain ?? 0,
    q:       o.q ?? 1.0,
  };
}
function clamp(v, lo, hi){ return v < lo ? lo : (v > hi ? hi : v); }
function clampBand(b){
  b.fc   = clamp(+b.fc,   LIMIT.FC_MIN,  LIMIT.FC_MAX);
  b.gain = clamp(+b.gain, LIMIT.GAIN_MIN,LIMIT.GAIN_MAX);
  b.q    = clamp(+b.q,    LIMIT.Q_MIN,   LIMIT.Q_MAX);
  return b;
}

// ===========================================================================
// AutoEQ import
// ===========================================================================
// AutoEQ's ParametricEQ.txt looks like:
//   Preamp: -6.8 dB
//   Filter 1: ON PK Fc 105 Hz Gain 5.5 dB Q 0.70
//   Filter 2: ON LSC Fc 105 Hz Gain -1.2 dB Q 0.70
//   Filter 3: ON LPQ Fc 20000 Hz Q 0.70          (no Gain for LP/HP)
const AUTOEQ_TYPE = {
  PK:TYPE.PEAKING, PEQ:TYPE.PEAKING,
  LS:TYPE.LOWSHELF,  LSC:TYPE.LOWSHELF,  LSHELF:TYPE.LOWSHELF,
  HS:TYPE.HIGHSHELF, HSC:TYPE.HIGHSHELF, HSHELF:TYPE.HIGHSHELF,
  LP:TYPE.LOWPASS,  LPQ:TYPE.LOWPASS,
  HP:TYPE.HIGHPASS, HPQ:TYPE.HIGHPASS,
};

function parseAutoEq(text, maxBands = MAX_BANDS) {
  const out = { preampDb: 0, bands: [], warnings: [] };
  const lines = String(text).split(/\r?\n/);

  for (const raw of lines) {
    const line = raw.trim();
    if (!line) continue;

    const pre = line.match(/^Preamp:\s*(-?\d+(?:\.\d+)?)\s*dB/i);
    if (pre) { out.preampDb = parseFloat(pre[1]); continue; }

    const f = line.match(
      /^Filter\s+\d+:\s*(ON|OFF)\s+([A-Za-z]+)\s+Fc\s+(\d+(?:\.\d+)?)\s*Hz(?:\s+Gain\s+(-?\d+(?:\.\d+)?)\s*dB)?(?:\s+Q\s+(\d+(?:\.\d+)?))?/i
    );
    if (!f) {
      if (/^Filter/i.test(line)) out.warnings.push('Unparsed: ' + line);
      continue;
    }
    const [, onoff, tok, fc, gain, q] = f;
    const type = AUTOEQ_TYPE[tok.toUpperCase()];
    if (type === undefined) {
      out.warnings.push(`Unsupported filter type "${tok}" — skipped`);
      continue;
    }
    const band = newBand({
      enabled: onoff.toUpperCase() === 'ON',
      type,
      fc: parseFloat(fc),
      gain: gain !== undefined ? parseFloat(gain) : 0,
      q: q !== undefined ? parseFloat(q) : 0.707,
    });
    // Note any clamping so the user knows the device won't reproduce it exactly.
    const before = { fc: band.fc, gain: band.gain, q: band.q };
    clampBand(band);
    if (before.fc !== band.fc)   out.warnings.push(`Fc ${before.fc}Hz clamped to ${band.fc}Hz`);
    if (before.gain !== band.gain) out.warnings.push(`Gain ${before.gain}dB clamped to ${band.gain}dB`);
    if (before.q !== band.q)     out.warnings.push(`Q ${before.q} clamped to ${band.q}`);
    out.bands.push(band);
  }

  if (out.bands.length > maxBands) {
    out.warnings.push(`Preset has ${out.bands.length} filters; device holds ${maxBands}. Extra bands dropped.`);
    out.bands = out.bands.slice(0, maxBands);
  }
  return out;
}

// ===========================================================================
// Response curve — RBJ biquad magnitude (matches the device's per-band design
// closely, and matches what AutoEQ assumes). DISPLAY-ONLY approximation: the
// device itself runs a TPT state-variable filter (src/dsp_peq.c), which is
// audibly identical but stays accurate at frequency extremes.
// ===========================================================================
function rbjCoeffs(band, fs) {
  const A = Math.pow(10, band.gain / 40);
  const w0 = 2 * Math.PI * band.fc / fs;
  const cw = Math.cos(w0), sw = Math.sin(w0);
  const alpha = sw / (2 * band.q);
  let b0, b1, b2, a0, a1, a2;
  switch (band.type) {
    case TYPE.PEAKING:
      b0=1+alpha*A; b1=-2*cw; b2=1-alpha*A; a0=1+alpha/A; a1=-2*cw; a2=1-alpha/A; break;
    case TYPE.LOWSHELF: {
      const s=2*Math.sqrt(A)*alpha;
      b0=A*((A+1)-(A-1)*cw+s); b1=2*A*((A-1)-(A+1)*cw); b2=A*((A+1)-(A-1)*cw-s);
      a0=(A+1)+(A-1)*cw+s; a1=-2*((A-1)+(A+1)*cw); a2=(A+1)+(A-1)*cw-s; break; }
    case TYPE.HIGHSHELF: {
      const s=2*Math.sqrt(A)*alpha;
      b0=A*((A+1)+(A-1)*cw+s); b1=-2*A*((A-1)+(A+1)*cw); b2=A*((A+1)+(A-1)*cw-s);
      a0=(A+1)-(A-1)*cw+s; a1=2*((A-1)-(A+1)*cw); a2=(A+1)-(A-1)*cw-s; break; }
    case TYPE.LOWPASS:
      b0=(1-cw)/2; b1=1-cw; b2=(1-cw)/2; a0=1+alpha; a1=-2*cw; a2=1-alpha; break;
    case TYPE.HIGHPASS:
      b0=(1+cw)/2; b1=-(1+cw); b2=(1+cw)/2; a0=1+alpha; a1=-2*cw; a2=1-alpha; break;
    default: return {b0:1,b1:0,b2:0,a1:0,a2:0};
  }
  return { b0:b0/a0, b1:b1/a0, b2:b2/a0, a1:a1/a0, a2:a2/a0 };
}

function biquadMagDb(c, w) {
  const cw = Math.cos(w), sw = Math.sin(w), c2 = Math.cos(2*w), s2 = Math.sin(2*w);
  const nr = c.b0 + c.b1*cw + c.b2*c2, ni = -(c.b1*sw + c.b2*s2);
  const dr = 1 + c.a1*cw + c.a2*c2,    di = -(c.a1*sw + c.a2*s2);
  const num = Math.hypot(nr, ni), den = Math.hypot(dr, di);
  return 20 * Math.log10(num / den);
}

// Total response (dB) at frequency f for the whole EQ (pre-gain + enabled bands).
function responseDb(model, f, fs = SAMPLE_RATE) {
  let db = model.preGainDb;
  const w = 2 * Math.PI * f / fs;
  for (const b of model.bands) {
    if (b.enabled) db += biquadMagDb(rbjCoeffs(b, fs), w);
  }
  return db;
}

// ---- Node export for unit testing (ignored in the browser) -----------------
if (typeof module !== 'undefined' && module.exports) {
  module.exports = { parseAutoEq, clampBand, responseDb, rbjCoeffs, biquadMagDb,
                     TYPE, LIMIT, newBand };
}

// ===========================================================================
// Everything below is browser-only (DOM + WebUSB).
// ===========================================================================
if (typeof document !== 'undefined') {

let usbDevice = null;
const $ = (id) => document.getElementById(id);
function log(msg, cls){ const el=$('log'); const t=new Date().toLocaleTimeString();
  el.textContent += `[${t}] ${msg}\n`; el.scrollTop = el.scrollHeight;
  if (cls) console.log(msg); }

// ---- Rendering -------------------------------------------------------------
function renderBands() {
  const tb = $('bands').querySelector('tbody');
  tb.innerHTML = '';
  state.bands.forEach((b, i) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td><input type="checkbox" ${b.enabled?'checked':''} data-i="${i}" data-k="enabled"></td>
      <td><select data-i="${i}" data-k="type">${
        TYPE_NAME.map((n,ti)=>`<option value="${ti}" ${b.type===ti?'selected':''}>${n}</option>`).join('')
      }</select></td>
      <td><input class="num" type="number" step="1"   min="1" max="20000" value="${b.fc}"   data-i="${i}" data-k="fc"></td>
      <td><input class="num" type="number" step="0.1" min="-12" max="12"  value="${b.gain}" data-i="${i}" data-k="gain"></td>
      <td><input class="num" type="number" step="0.05" min="0.1" max="10" value="${b.q}"    data-i="${i}" data-k="q"></td>
      <td><button data-i="${i}" data-k="del">✕</button></td>`;
    tb.appendChild(tr);
  });
  $('bandCount').textContent = `${state.bands.length} / ${MAX_BANDS} bands`;
  $('btnAddBand').disabled = state.bands.length >= MAX_BANDS;
  $('preGain').value = state.preGainDb;
  updateAutoHint();
  drawGraph();
}

function updateAutoHint(){
  let maxBoost = 0;
  for (const b of state.bands)
    if (b.enabled && b.gain > maxBoost &&
        (b.type===TYPE.PEAKING||b.type===TYPE.LOWSHELF||b.type===TYPE.HIGHSHELF))
      maxBoost = b.gain;
  // Heuristic: covers the single largest boost; overlapping boosts can sum higher.
  $('autoHint').textContent = maxBoost>0 ? `suggested (max band): ${(-maxBoost).toFixed(1)} dB` : 'no boosts';
}

// ---- Frequency-response graph ---------------------------------------------
function drawGraph() {
  const cv = $('graph'), ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height, padL=44, padR=12, padT=10, padB=24;
  const fMin=20, fMax=20000, dbMax=15, dbMin=-15;
  const x = f => padL + (Math.log10(f)-Math.log10(fMin))/(Math.log10(fMax)-Math.log10(fMin))*(W-padL-padR);
  const y = db => padT + (dbMax-db)/(dbMax-dbMin)*(H-padT-padB);

  ctx.clearRect(0,0,W,H);
  ctx.fillStyle='#1e242c'; ctx.fillRect(0,0,W,H);
  // grid
  ctx.strokeStyle='#232b34'; ctx.fillStyle='#93a1b0'; ctx.font='10px system-ui'; ctx.lineWidth=1;
  for (const db of [-12,-9,-6,-3,0,3,6,9,12]) {
    ctx.beginPath(); ctx.moveTo(padL,y(db)); ctx.lineTo(W-padR,y(db)); ctx.stroke();
    ctx.fillText(db>0?'+'+db:''+db, 6, y(db)+3);
  }
  for (const f of [20,50,100,200,500,1000,2000,5000,10000,20000]) {
    ctx.beginPath(); ctx.moveTo(x(f),padT); ctx.lineTo(x(f),H-padB); ctx.stroke();
    ctx.fillText(f>=1000?(f/1000)+'k':''+f, x(f)-8, H-8);
  }
  // 0 dB line
  ctx.strokeStyle='#3a444f'; ctx.beginPath(); ctx.moveTo(padL,y(0)); ctx.lineTo(W-padR,y(0)); ctx.stroke();
  // curve
  ctx.strokeStyle='#4ea1ff'; ctx.lineWidth=2; ctx.beginPath();
  const N=400;
  for (let i=0;i<=N;i++){
    const f = fMin*Math.pow(fMax/fMin, i/N);
    const db = clamp(responseDb(state, f), dbMin, dbMax);
    const px=x(f), py=y(db);
    i?ctx.lineTo(px,py):ctx.moveTo(px,py);
  }
  ctx.stroke();
}

// ---- Edits -----------------------------------------------------------------
function onTableInput(e){
  const t=e.target, i=+t.dataset.i, k=t.dataset.k;
  if (k==='del'){ state.bands.splice(i,1); renderBands(); pushBand(); return; }
  if (i>=state.bands.length) return;
  const b=state.bands[i];
  if (k==='enabled') b.enabled=t.checked;
  else if (k==='type') b.type=+t.value;
  else b[k]=+t.value;
  clampBand(b);
  drawGraph(); updateAutoHint();
  pushOneBand(i);
}

// ===========================================================================
// WebUSB
// ===========================================================================
// Map a failed connect step + error onto something actionable. The usual
// culprits are OS driver/permission issues, not the device itself.
function connectHint(step, err){
  const lines = [`Connect failed at ${step}: ${err.name}: ${err.message}`];
  if (step === 'open' || step === 'claimInterface') {
    lines.push('Hints:');
    lines.push(' • Windows: the WinUSB driver may not be bound to the config interface.');
    lines.push('   Unplug/replug the device (this firmware bumps bcdDevice so Windows');
    lines.push('   re-reads the driver descriptors). If it still fails: Device Manager →');
    lines.push('   find the DSPico entry → Uninstall device (tick "delete driver") → replug.');
    lines.push(' • Linux: your user needs rw access to the USB node. Install the udev rule:');
    lines.push('   sudo cp docs/99-dspico.rules /etc/udev/rules.d/ && sudo udevadm control --reload');
    lines.push('   then replug the device.');
    lines.push(' • Close other tabs/apps that may hold the device open.');
  } else if (step === 'readInfo') {
    lines.push('The interface was claimed but the device did not answer the INFO request —');
    lines.push('likely an old firmware on the device. Reflash the current dspico.uf2.');
  }
  if (err.name === 'TimeoutError') {
    lines.push('A USB operation hung — usually the OS re-enumerating the device mid-call.');
    lines.push('Unplug the DSPico, wait 3 seconds, replug, and let the page auto-connect.');
  }
  return lines.join('\n');
}

const sleep = (ms) => new Promise(r => setTimeout(r, ms));
const DSPICO_FILTER = { vendorId: 0x1209, productId: 0xD590 };

// Every USB await is time-boxed: a hung promise (device re-enumerating mid
// operation) must fail loudly and release the connect guard, never wedge the
// page into a state where clicking Connect does nothing.
function withTimeout(promise, ms, what){
  let t;
  const killer = new Promise((_, rej) => {
    t = setTimeout(() => rej(Object.assign(
      new Error(`${what} timed out after ${ms} ms — the device stopped responding (replug and retry)`),
      { name: 'TimeoutError' })), ms);
  });
  return Promise.race([promise, killer]).finally(() => clearTimeout(t));
}

// A device object already granted permission (no chooser needed), or null.
async function getPermittedDevice(){
  const devs = await navigator.usb.getDevices();
  return devs.find(d => d.vendorId === DSPICO_FILTER.vendorId &&
                        d.productId === DSPICO_FILTER.productId) || null;
}

// Open with recovery. "An operation that changes interface state is in
// progress" / "device disconnected" mean a stale handle or an OS
// re-enumeration mid-flight: back off, fetch a FRESH device object from
// getDevices() (the old object can be permanently poisoned), and try again.
async function openWithRetry(dev){
  for (let attempt = 0; ; attempt++) {
    try { await withTimeout(dev.open(), 3000, 'open'); return dev; }
    catch (e) {
      const transient = e.name === 'InvalidStateError' || e.name === 'NotFoundError' ||
                        e.name === 'TimeoutError' || /disconnected|in progress/i.test(e.message);
      if (attempt >= 4 || !transient) throw e;
      log(`open busy (${e.name}), retry ${attempt + 1}/4…`);
      try { await withTimeout(dev.close(), 1000, 'close'); } catch (_) {}
      await sleep(700);
      const fresh = await getPermittedDevice();
      if (fresh) dev = fresh;
    }
  }
}

// interactive=true shows the chooser; false silently uses an already
// permitted device (auto-connect on load / on replug).
let connectInFlight = false;
async function connect(interactive = true){
  if (!('usb' in navigator)) { log('WebUSB not available — use Chrome/Edge over https or localhost.', 'e'); return; }
  if (connectInFlight) { log('(connect already in progress — ignored)'); return; }
  if (usbDevice)       { log('(already connected)'); return; }
  connectInFlight = true;
  let step = 'requestDevice';
  try {
    // After a replug, Windows can spend several seconds re-binding drivers,
    // during which the device is invisible to the browser. Auto-connect polls
    // patiently; the manual path checks once then falls back to the chooser.
    let dev = await getPermittedDevice();
    for (let i = 0; !dev && !interactive && i < 8; i++) {
      await sleep(1200);
      dev = await getPermittedDevice();
    }
    if (!dev) {
      if (!interactive) {
        log('DSPico not visible to the browser (drivers still installing?) — click "Connect device" to retry.');
        return;
      }
      dev = await navigator.usb.requestDevice({ filters: [DSPICO_FILTER] });
    }
    step = 'open';           usbDevice = await openWithRetry(dev);
    step = 'selectConfiguration';
    if (usbDevice.configuration === null)
      await withTimeout(usbDevice.selectConfiguration(1), 3000, 'selectConfiguration');
    step = 'claimInterface';
    await withTimeout(usbDevice.claimInterface(PROTO.ITF_VENDOR), 3000, 'claimInterface');
    step = 'readInfo';
    await withTimeout(readInfo(), 3000, 'readInfo');
    setConnected(true);
    log('Connected.');
  } catch (err) {
    log(connectHint(step, err), 'e');
    setConnected(false);
    try { if (usbDevice) await withTimeout(usbDevice.close(), 1000, 'close'); } catch (_) {}
    usbDevice = null;
  } finally {
    connectInFlight = false;
  }
}

// Poll the device's diagnostic log (host-side USB events: DAC attach, format
// candidates, setup steps, heartbeats) so bring-up is debuggable in-browser.
let logTimer = null;
async function pollDeviceLog(){
  if (!usbDevice) return;
  try {
    const d = await ctrlIn(PROTO.REQ_GET_LOG, 255);
    if (d && d.byteLength) {
      const text = new TextDecoder().decode(
        d.buffer.slice(d.byteOffset, d.byteOffset + d.byteLength));
      text.split('\n').filter(s => s.trim()).forEach(s => log('[device] ' + s));
    }
  } catch (_) { /* transient — next poll retries */ }
}

function setConnected(on){
  $('dot').classList.toggle('on', on);
  ['btnLoadDev','btnPushDev','btnCommit'].forEach(id=>$(id).disabled=!on);
  $('btnConnect').textContent = on ? 'Disconnect' : 'Connect device';
  if (on && !logTimer) logTimer = setInterval(pollDeviceLog, 800);
  if (!on && logTimer) { clearInterval(logTimer); logTimer = null; }
}

async function ctrlIn(request, length, index=PROTO.ITF_VENDOR){
  const r = await usbDevice.controlTransferIn(
    { requestType:'vendor', recipient:'interface', request, value:0, index }, length);
  if (r.status !== 'ok') throw new Error('controlIn status '+r.status);
  return r.data;
}
async function ctrlOut(request, data=new ArrayBuffer(0), index=PROTO.ITF_VENDOR){
  const r = await usbDevice.controlTransferOut(
    { requestType:'vendor', recipient:'interface', request, value:0, index }, data);
  if (r.status !== 'ok') throw new Error('controlOut status '+r.status);
}

async function readInfo(){
  const d = await ctrlIn(PROTO.REQ_INFO, 12);
  const magic = d.getUint32(0, true);
  if (magic !== PROTO.MAGIC) { log('Warning: unexpected device magic 0x'+magic.toString(16), 'e'); }
  const version = d.getUint16(4, true);
  MAX_BANDS   = d.getUint8(6);
  SAMPLE_RATE = d.getUint32(8, true);
  $('devinfo').textContent = `DSPico v${version} · ${MAX_BANDS} bands · ${SAMPLE_RATE/1000|0}kHz`;
}

function packBand(b){
  const buf = new ArrayBuffer(16), v = new DataView(buf);
  v.setUint8(0, b.type); v.setUint8(1, b.enabled?1:0);
  v.setFloat32(4, b.fc, true); v.setFloat32(8, b.gain, true); v.setFloat32(12, b.q, true);
  return buf;
}

async function loadFromDevice(){
  try {
    const d = await ctrlIn(PROTO.REQ_GET_STATE, 4 + 4 + MAX_BANDS*16);
    state.preGainDb = Math.round(d.getFloat32(0, true)*10)/10;
    const n = d.getUint8(4);
    state.bands = [];
    for (let i=0;i<n;i++){
      const o = 8 + i*16;
      state.bands.push(newBand({
        type: d.getUint8(o), enabled: d.getUint8(o+1)!==0,
        fc: d.getFloat32(o+4,true), gain: d.getFloat32(o+8,true), q: d.getFloat32(o+12,true),
      }));
    }
    renderBands();
    log(`Loaded ${n} bands from device.`);
  } catch (err) { log('Load failed: '+err.message, 'e'); }
}

async function pushOneBand(i){
  if (!usbDevice) return;
  try {
    await usbDevice.controlTransferOut(
      { requestType:'vendor', recipient:'interface', request:PROTO.REQ_SET_BAND, value:0, index:(i<<8)|PROTO.ITF_VENDOR },
      packBand(state.bands[i]));
  } catch (err) { log('Push band '+i+' failed: '+err.message, 'e'); }
}

async function pushBand(){ /* re-push everything (used after add/remove) */
  if (!usbDevice) return;
  await pushPreGain();
  for (let i=0;i<MAX_BANDS;i++){
    const b = i<state.bands.length ? state.bands[i] : newBand({enabled:false});
    await usbDevice.controlTransferOut(
      { requestType:'vendor', recipient:'interface', request:PROTO.REQ_SET_BAND, value:0, index:(i<<8)|PROTO.ITF_VENDOR },
      packBand(b));
  }
  log('Pushed all bands to device.');
}

async function pushPreGain(){
  if (!usbDevice) return;
  const buf = new ArrayBuffer(4); new DataView(buf).setFloat32(0, state.preGainDb, true);
  await ctrlOut(PROTO.REQ_SET_PREGAIN, buf);
}

// ===========================================================================
// Wire up UI
// ===========================================================================
$('bands').addEventListener('input', onTableInput);
$('bands').addEventListener('click', (e)=>{ if(e.target.dataset.k==='del') onTableInput(e); });

$('btnAddBand').onclick = ()=>{ if(state.bands.length<MAX_BANDS){ state.bands.push(newBand()); renderBands(); pushBand(); } };
$('preGain').oninput = ()=>{ state.preGainDb=+$('preGain').value; drawGraph(); pushPreGain(); };
$('btnAuto').onclick = ()=>{
  let maxBoost=0; for(const b of state.bands) if(b.enabled&&b.gain>maxBoost) maxBoost=b.gain;
  state.preGainDb = -Math.max(0,maxBoost); renderBands(); pushPreGain();
};

// Toggle connect/disconnect. Guards against double-clicks and always AWAITS
// close() — an un-awaited close left "an operation in progress" that made the
// next open() fail.
let busy = false;
$('btnConnect').onclick = async ()=>{
  if (busy) return;
  busy = true; $('btnConnect').disabled = true;
  try {
    if (usbDevice) {
      const d = usbDevice; usbDevice = null;
      setConnected(false);
      try { await d.close(); } catch (_) {}
      log('Disconnected.');
    } else {
      await connect();
    }
  } finally { busy = false; $('btnConnect').disabled = false; }
};
$('btnLoadDev').onclick = loadFromDevice;
$('btnPushDev').onclick = pushBand;
$('btnCommit').onclick  = async ()=>{ try{ await ctrlOut(PROTO.REQ_COMMIT); log('Saved to flash.'); }catch(e){ log('Commit failed: '+e.message,'e'); } };

function applyImport(res, sourceLabel){
  state.preGainDb = Math.round(res.preampDb*10)/10;
  state.bands = res.bands.map(newBand);
  renderBands(); pushBand();
  const msg = `Imported ${res.bands.length} bands from ${sourceLabel}` +
              (res.warnings.length ? ` · ${res.warnings.length} note(s)` : '');
  $('importMsg').textContent = msg;
  $('importMsg').className = res.warnings.length ? 'warnbox' : 'muted';
  log(msg); res.warnings.forEach(w=>log('  • '+w));
}

$('btnParsePaste').onclick = ()=> applyImport(parseAutoEq($('autoEqText').value, MAX_BANDS), 'pasted text');
$('fileAutoEq').onchange = (e)=>{ const f=e.target.files[0]; if(!f) return;
  const r=new FileReader(); r.onload=()=>applyImport(parseAutoEq(r.result, MAX_BANDS), f.name); r.readAsText(f); };
$('fileJson').onchange = (e)=>{ const f=e.target.files[0]; if(!f) return;
  const r=new FileReader(); r.onload=()=>{ try{
    const j=JSON.parse(r.result);
    state.preGainDb = +j.preGainDb || 0;
    state.bands = (j.bands||[]).slice(0,MAX_BANDS).map(newBand).map(b=>(clampBand(b),b));
    renderBands(); pushBand(); log('Loaded JSON preset '+f.name);
  }catch(err){ log('Bad JSON: '+err.message,'e'); } }; r.readAsText(f); };

$('btnExport').onclick = ()=>{
  const blob = new Blob([JSON.stringify({ format:'dspico-eq', version:1,
    preGainDb: state.preGainDb, bands: state.bands }, null, 2)], {type:'application/json'});
  const a=document.createElement('a'); a.href=URL.createObjectURL(blob);
  a.download='dspico-preset.json'; a.click(); URL.revokeObjectURL(a.href);
};

// ---- Log utilities -----------------------------------------------------
$('btnCopyLog').onclick = async ()=>{
  try { await navigator.clipboard.writeText($('log').textContent); log('Log copied to clipboard.'); }
  catch (e) { log('Copy failed: ' + e.message, 'e'); }
};
$('btnClearLog').onclick = ()=>{ $('log').textContent = ''; };

// ---- Plug/unplug awareness + auto-connect --------------------------------
if ('usb' in navigator) {
  navigator.usb.addEventListener('disconnect', (e)=>{
    if (usbDevice && e.device === usbDevice) {
      usbDevice = null;
      setConnected(false);
      log('Device unplugged.');
    }
  });
  navigator.usb.addEventListener('connect', (e)=>{
    if (!usbDevice &&
        e.device.vendorId === DSPICO_FILTER.vendorId &&
        e.device.productId === DSPICO_FILTER.productId) {
      log('DSPico plugged in — connecting…');
      connect(false);            // polls while the OS finishes driver binding
    }
  });
}

// Seed with a couple of example bands so the graph isn't empty.
state.bands = [ newBand({type:TYPE.LOWSHELF, fc:105, gain:4, q:0.7}),
                newBand({type:TYPE.PEAKING, fc:3000, gain:-3, q:1.5}) ];
renderBands();
{
  const chrome = (navigator.userAgent.match(/(Chrome|Edg)\/[\d.]+/g) || []).join(' ');
  log(`Ready — configurator ${APP_REV} · ${navigator.platform || '?'} · ${chrome || navigator.userAgent.slice(0, 40)}`);
  log('Design offline, or Connect a DSPico to push live.');
}

// Reconnect silently if this browser already has permission for a DSPico.
getPermittedDevice().then(d => { if (d) { log('Found permitted DSPico — auto-connecting…'); connect(false); } })
                    .catch(()=>{});

} // end browser-only block
