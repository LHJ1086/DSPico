// ---------------------------------------------------------------------------
// DSPico — Node unit tests for the configurator's pure logic (web/dspico.js).
//
// Covers the AutoEQ importer, parameter clamping, and the response-curve math —
// everything that runs with no browser, no DOM and no device. dspico.js only
// exports these when required under Node (the DOM/WebUSB half is guarded).
//
// Run:  node tests/web_test.js       (or via tests/run.sh)
// Exits non-zero if any assertion fails.
// ---------------------------------------------------------------------------
'use strict';
const assert = require('assert');
const path = require('path');

const {
  parseAutoEq, clampBand, responseDb, TYPE, LIMIT, newBand,
} = require(path.join(__dirname, '..', 'web', 'dspico.js'));

let pass = 0;
function test(name, fn) {
  try { fn(); pass++; }
  catch (e) { console.error('  FAIL: ' + name + '\n         ' + e.message); process.exitCode = 1; }
}
const approx = (a, b, tol = 1e-6) => Math.abs(a - b) <= tol;

console.log('=== DSPico web configurator tests ===');

// --- clampBand --------------------------------------------------------------
test('clampBand pulls out-of-range values into the allowed ranges', () => {
  const b = clampBand(newBand({ fc: 1e9, gain: 99, q: 99 }));
  assert.strictEqual(b.fc, LIMIT.FC_MAX);
  assert.strictEqual(b.gain, LIMIT.GAIN_MAX);
  assert.strictEqual(b.q, LIMIT.Q_MAX);
  const c = clampBand(newBand({ fc: 0.001, gain: -99, q: 0.0001 }));
  assert.strictEqual(c.fc, LIMIT.FC_MIN);
  assert.strictEqual(c.gain, LIMIT.GAIN_MIN);
  assert.strictEqual(c.q, LIMIT.Q_MIN);
});

test('clampBand leaves in-range values untouched', () => {
  const b = clampBand(newBand({ fc: 1000, gain: 3, q: 1.4 }));
  assert.strictEqual(b.fc, 1000);
  assert.strictEqual(b.gain, 3);
  assert.strictEqual(b.q, 1.4);
});

// --- parseAutoEq ------------------------------------------------------------
test('parseAutoEq reads preamp and the standard filter types', () => {
  const txt = [
    'Preamp: -6.5 dB',
    'Filter 1: ON PK Fc 105 Hz Gain 5.5 dB Q 0.70',
    'Filter 2: ON LSC Fc 200 Hz Gain -1.2 dB Q 0.71',
    'Filter 3: ON HSC Fc 8000 Hz Gain 2.0 dB Q 0.72',
    'Filter 4: ON LPQ Fc 20000 Hz Q 0.70',
    'Filter 5: ON HPQ Fc 30 Hz Q 0.70',
  ].join('\n');
  const r = parseAutoEq(txt, 16);
  assert.strictEqual(r.preampDb, -6.5);
  assert.strictEqual(r.bands.length, 5);
  assert.strictEqual(r.bands[0].type, TYPE.PEAKING);
  assert.ok(approx(r.bands[0].fc, 105) && approx(r.bands[0].gain, 5.5) && approx(r.bands[0].q, 0.70));
  assert.strictEqual(r.bands[1].type, TYPE.LOWSHELF);
  assert.strictEqual(r.bands[2].type, TYPE.HIGHSHELF);
  assert.strictEqual(r.bands[3].type, TYPE.LOWPASS);
  assert.strictEqual(r.bands[4].type, TYPE.HIGHPASS);
});

test('parseAutoEq honours ON/OFF', () => {
  const r = parseAutoEq('Filter 1: OFF PK Fc 1000 Hz Gain 3 dB Q 1', 16);
  assert.strictEqual(r.bands.length, 1);
  assert.strictEqual(r.bands[0].enabled, false);
});

test('parseAutoEq clamps out-of-range values and warns', () => {
  const r = parseAutoEq('Filter 1: ON PK Fc 60000 Hz Gain 30 dB Q 50', 16);
  assert.strictEqual(r.bands[0].fc, LIMIT.FC_MAX);
  assert.strictEqual(r.bands[0].gain, LIMIT.GAIN_MAX);
  assert.strictEqual(r.bands[0].q, LIMIT.Q_MAX);
  assert.ok(r.warnings.length >= 3, 'expected a warning per clamped field');
});

test('parseAutoEq skips unsupported filter types with a note', () => {
  const r = parseAutoEq('Filter 1: ON NO Fc 1000 Hz Gain 0 dB Q 3', 16);   // notch
  assert.strictEqual(r.bands.length, 0);
  assert.ok(r.warnings.some(w => /Unsupported/.test(w)));
});

test('parseAutoEq truncates presets longer than the band count with a note', () => {
  const lines = [];
  for (let i = 1; i <= 20; i++) lines.push(`Filter ${i}: ON PK Fc ${100 + i} Hz Gain 1 dB Q 1`);
  const r = parseAutoEq(lines.join('\n'), 16);
  assert.strictEqual(r.bands.length, 16);
  assert.ok(r.warnings.some(w => /device holds 16/.test(w)));
});

test('parseAutoEq defaults Q for LP/HP filters that omit it', () => {
  const r = parseAutoEq('Filter 1: ON LPQ Fc 20000 Hz Q 0.70\nFilter 2: ON HP Fc 30 Hz', 16);
  assert.ok(approx(r.bands[1].q, 0.707), 'missing Q should default to ~0.707');
});

// --- responseDb -------------------------------------------------------------
test('responseDb is preamp-only when no bands are enabled', () => {
  assert.ok(approx(responseDb({ preGainDb: -4, bands: [] }, 1000, 48000), -4, 1e-9));
});

test('responseDb hits the peaking gain at the centre frequency', () => {
  const model = { preGainDb: 0, bands: [newBand({ type: TYPE.PEAKING, fc: 1000, gain: 6, q: 1 })] };
  assert.ok(approx(responseDb(model, 1000, 48000), 6, 0.15), 'peak ~ +6 dB at Fc');
  assert.ok(approx(responseDb(model, 50, 48000), 0, 0.2), 'flat well below Fc');
});

test('responseDb sums pre-gain and band gain', () => {
  const model = { preGainDb: -6, bands: [newBand({ type: TYPE.PEAKING, fc: 1000, gain: 6, q: 1 })] };
  assert.ok(approx(responseDb(model, 1000, 48000), 0, 0.15), 'preamp -6 + band +6 ~ 0 dB');
});

if (!process.exitCode) console.log(`\n${pass} tests passed.\nOK`);
else console.log(`\n${pass} tests passed, some FAILED`);
