// Executes the real inline JS from frontend/index.html against a minimal
// DOM/Leaflet stub and the REAL running dev server, so the "Refresh cache
// stats" button can be tested without a browser.
//
// Not a general-purpose harness - just enough stubbing to let the page's
// script run to completion, then fire the button's click handler and
// inspect what it wrote into #cacheStats.
//
// Run (with dev_server.py running):  node scripts/test_frontend_cache_button.js
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const HTML = fs.readFileSync(path.join(__dirname, '..', 'frontend', 'index.html'), 'utf8');
const script = HTML.match(/<script>([\s\S]*)<\/script>/)[1];

// ---- minimal DOM stub -------------------------------------------------
const elements = new Map();
function makeEl(id) {
  return {
    id,
    textContent: '',
    innerHTML: '',
    value: id === 'kValue' ? '1' : (id === 'conditionKind' ? 'closed' : '3'),
    checked: false,
    disabled: false,
    style: {},
    _handlers: {},
    addEventListener(evt, fn) { (this._handlers[evt] = this._handlers[evt] || []).push(fn); },
    async click() { for (const fn of (this._handlers.click || [])) await fn({}); },
  };
}
function getEl(id) {
  if (!elements.has(id)) elements.set(id, makeEl(id));
  return elements.get(id);
}

const layerStub = () => ({
  addTo() { return this; }, clearLayers() {}, getBounds() { return {}; },
});
const L = {
  map: () => ({ setView() { return this; }, on() {}, fitBounds() {}, removeLayer() {} }),
  tileLayer: () => ({ addTo() { return this; } }),
  layerGroup: layerStub, marker: layerStub, polyline: layerStub, circleMarker: layerStub,
  latLngBounds: () => ({}),
};

const sandbox = {
  L,
  document: { getElementById: getEl },
  fetch: (url, opts) => fetch(url.startsWith('http') ? url : `http://localhost:8765${url}`, opts),
  setTimeout, clearTimeout, console, URLSearchParams, JSON, Math, Date, Promise,
};
sandbox.window = sandbox;

const settle = (ms) => new Promise((r) => setTimeout(r, ms));
const indent = (s) => s.split('\n').map((l) => '     ' + l).join('\n');

let loadError = null;
try {
  vm.createContext(sandbox);
  vm.runInContext(script, sandbox, { filename: 'index.html:<script>' });
} catch (err) {
  loadError = err;
}

(async () => {
  console.log('=== a. page script executed without a runtime error? ===');
  console.log(loadError ? `   RUNTIME ERROR: ${loadError.message}` : '   ok');

  const btn = getEl('refreshCacheStats');
  const handlers = (btn._handlers.click || []).length;
  console.log('\n=== b. click handler registered on #refreshCacheStats? ===');
  console.log(`   ${handlers} handler(s) -> ${handlers > 0 ? 'WIRED' : 'NOT WIRED'}`);

  await settle(700);
  console.log('\n=== c. #cacheStats after page load ===');
  const afterLoad = getEl('cacheStats').textContent;
  console.log(afterLoad ? indent(afterLoad) : '     (empty)');

  // Change server-side state so a refresh has something new to report.
  console.log('\n=== d. performing a NEW compute so the stats must change ===');
  const r = await fetch('http://localhost:8765/compute?start_lat=20.2960&start_lon=85.8240'
    + '&end_lat=20.3010&end_lon=85.8290');
  const body = await r.json();
  console.log(`     compute returned cached=${body.cached} in ${body.elapsed_ms} ms`);

  console.log('\n=== e. clicking "Refresh cache stats" ===');
  getEl('cacheStats').textContent = '<<CLEARED BEFORE CLICK>>';
  await btn.click();
  await settle(700);
  const afterClick = getEl('cacheStats').textContent;

  if (afterClick === '<<CLEARED BEFORE CLICK>>') {
    console.log('     FAIL - the handler wrote nothing');
    process.exit(1);
  }
  console.log(indent(afterClick));

  console.log('\n=== f. verdict ===');
  const loadEntries = (afterLoad.match(/entries\s+(\d+)/) || [])[1];
  const clickEntries = (afterClick.match(/entries\s+(\d+)/) || [])[1];
  console.log(`     entries before click: ${loadEntries}   after click: ${clickEntries}`);
  console.log(`     content changed on click: ${afterLoad !== afterClick}`);
  console.log(`     -> button ${afterLoad !== afterClick ? 'WORKS and is visibly different' : 'produced identical text'}`);
})();
