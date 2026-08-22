// Executes the REAL inline JS from frontend/index.html against a minimal
// DOM/Leaflet stub and the REAL running dev server, simulating two actual
// map clicks (via the captured map 'click' handler) and then clicking
// "Compute + show route" - to verify, empirically rather than by reading
// the code, that a same-node collision surfaces the server's clear error
// message instead of a fake "1 points, 0.00 km" route.
//
// Run (with dev_server.py running): node scripts/test_same_node_ui.js
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const HTML = fs.readFileSync(path.join(__dirname, '..', 'frontend', 'index.html'), 'utf8');
const script = HTML.match(/<script>([\s\S]*)<\/script>/)[1];

const elements = new Map();
function makeEl(id) {
  const classes = new Set();
  return {
    id, textContent: '', innerHTML: '',
    value: id === 'kValue' ? '1' : (id === 'conditionKind' ? 'closed' : '3'),
    checked: false, disabled: false, style: {},
    // Real DOM elements always have classList; this stub was missing it
    // until reportCacheResult() (which loadRoute() reaches via
    // computeForCurrentPoints()) became the first thing in this harness to
    // actually call it.
    classList: {
      add: (...c) => c.forEach((x) => classes.add(x)),
      remove: (...c) => c.forEach((x) => classes.delete(x)),
      contains: (c) => classes.has(c),
    },
    _handlers: {},
    addEventListener(evt, fn) { (this._handlers[evt] = this._handlers[evt] || []).push(fn); },
    async click() { for (const fn of (this._handlers.click || [])) await fn({}); },
  };
}
function getEl(id) {
  if (!elements.has(id)) elements.set(id, makeEl(id));
  return elements.get(id);
}

let mapClickHandler = null;
const layerStub = () => ({ addTo() { return this; }, clearLayers() {}, getBounds() { return {}; } });
const L = {
  map: () => ({
    setView() { return this; },
    on(evt, fn) { if (evt === 'click') mapClickHandler = fn; },
    fitBounds() {}, removeLayer() {},
  }),
  tileLayer: () => ({ addTo() { return this; } }),
  layerGroup: layerStub, marker: layerStub, polyline: layerStub, circleMarker: layerStub,
  latLngBounds: () => ({}),
};

// Real browsers resolve a relative fetch() URL (e.g. "route_astar.json",
// no leading slash) against the *page's* URL, not by string-concatenating
// it onto the origin - "http://host" + "route_astar.json" is missing the
// separating slash entirely. Using Node's URL class with an explicit base
// replicates that resolution correctly instead of reintroducing the same
// class of bug this test exists to catch, just in the test harness.
const PAGE_URL = 'http://localhost:8765/index.html';
const sandbox = {
  L,
  document: { getElementById: getEl },
  fetch: (url, opts) => fetch(new URL(url, PAGE_URL).toString(), opts),
  // setInterval/clearInterval added alongside the existing setTimeout pair
  // specifically because checkpoint 6's auto-reroute poll loop (the first
  // thing in this page to use setInterval) would otherwise fail to load in
  // this VM at all - the same "test-stub gap, not a frontend bug" pattern
  // as the classList/fetch-URL gaps documented in NOTES.md.
  setTimeout, clearTimeout, setInterval, clearInterval, console, URLSearchParams, JSON, Math, Date, Promise,
};
sandbox.window = sandbox;
vm.createContext(sandbox);
vm.runInContext(script, sandbox, { filename: 'index.html:<script>' });

const settle = (ms) => new Promise((r) => setTimeout(r, ms));

async function clickMap(lat, lng) {
  await mapClickHandler({ latlng: { lat, lng } });
}

async function runScenario(name, p1, p2, expectRoute) {
  console.log(`\n=== ${name} ===`);
  console.log(`  click 1: (${p1[0]}, ${p1[1]})`);
  console.log(`  click 2: (${p2[0]}, ${p2[1]})`);

  await clickMap(p1[0], p1[1]);
  await clickMap(p2[0], p2[1]);

  const reloadBtn = getEl('reload');
  console.log(`  #reload disabled after two clicks? ${reloadBtn.disabled} (must be false)`);

  getEl('routeInfo').textContent = '';
  await reloadBtn.click();
  await settle(2000); // real subprocess call, needs real time

  const result = getEl('routeInfo').textContent;
  console.log(`  routeInfo shows: "${result}"`);

  const looksLikeFakeRoute = /Route: 1 points, 0\.00 km/.test(result);
  const looksLikeRealRoute = /Route: \d+ points, \d+\.\d\d km/.test(result) && !looksLikeFakeRoute;
  const looksLikeClearError = /same road intersection/.test(result);

  if (expectRoute) {
    console.log(`  expected: a real multi-point route`);
    console.log(`  -> ${looksLikeRealRoute ? 'PASS' : 'FAIL'} (fake-route pattern present: ${looksLikeFakeRoute})`);
    return looksLikeRealRoute;
  } else {
    console.log(`  expected: the clear same-node error, NOT a fake route`);
    console.log(`  -> got fake "1 points, 0.00 km"?  ${looksLikeFakeRoute}`);
    console.log(`  -> got clear error message?       ${looksLikeClearError}`);
    console.log(`  -> ${(!looksLikeFakeRoute && looksLikeClearError) ? 'PASS' : 'FAIL'}`);
    return !looksLikeFakeRoute && looksLikeClearError;
  }
}

(async () => {
  const results = [];

  // Scenario 1: the real 212m-apart pair that collapses onto one node
  // (found by scanning the actual exported graph for a realistic sparse
  // gap - see the investigation notes).
  results.push(await runScenario(
    'Same-node collision (212m apart, real sparse-area pair)',
    [20.102739, 85.854541], [20.101467, 85.856055],
    /* expectRoute */ false,
  ));

  // Scenario 2 (regression): a normal, clearly-far-apart pair must still
  // produce a proper multi-point route, not get caught by the new guard.
  results.push(await runScenario(
    'Normal distant pair (airport area -> railway station area, ~3.5km)',
    [20.2444, 85.8178], [20.2679, 85.8425],
    /* expectRoute */ true,
  ));

  console.log(`\n=== SUMMARY: ${results.filter(Boolean).length}/${results.length} scenarios passed ===`);
  process.exit(results.every(Boolean) ? 0 : 1);
})();
