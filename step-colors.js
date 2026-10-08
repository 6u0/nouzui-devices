// 2/3: STEP -> step-colors.json. Parses STYLED_ITEM colours straight from the STEP file
// (occt-import-js misses colours set on assembly instances) and lists solids with face counts.
const fs = require('fs');
const text = fs.readFileSync(process.argv[2] || 'source/brain device2.step', 'latin1');
const data = text.slice(text.indexOf('DATA;') + 5, text.lastIndexOf('ENDSEC;'));
const E = new Map();
for (const raw of data.split(/;\s*\n/)) {
  const s = raw.replace(/\s*\n\s*/g, '');
  const m = /^#(\d+)=(.*)$/s.exec(s.trim());
  if (m) E.set(+m[1], m[2]);
}
const refs = (s) => [...s.matchAll(/#(\d+)/g)].map((x) => +x[1]);

function colourOf(id, seen = new Set()) {
  if (seen.has(id)) return null; seen.add(id);
  const s = E.get(id); if (!s) return null;
  const m = /^COLOUR_RGB\('(?:[^']|'')*',([^,]+),([^,]+),([^)]+)\)/.exec(s);
  if (m) return [+m[1], +m[2], +m[3]];
  for (const r of refs(s)) { const c = colourOf(r, seen); if (c) return c; }
  return null;
}

const itemColour = new Map();
for (const [id, s] of E) {
  const m = /^(?:OVER_RIDING_)?STYLED_ITEM\('[^']*',\((#[\d,#]+)\),#(\d+)/.exec(s);
  if (!m) continue;
  const c = colourOf(+m[1].slice(1).split(',')[0]);
  if (c) itemColour.set(+m[2], c);
}

const solids = [];
for (const [id, s] of E) {
  const m = /^(MANIFOLD_SOLID_BREP|BREP_WITH_VOIDS|SHELL_BASED_SURFACE_MODEL)\('((?:[^']|'')*)',(.*)\)$/.exec(s);
  if (!m) continue;
  const shells = refs(m[3]);
  const faces = shells.flatMap((sh) => refs(E.get(sh).replace(/^[A-Z_]+\('[^']*',/, '')));
  const faceCols = faces.map((f) => itemColour.get(f) || null);
  solids.push({ id, type: m[1], name: m[2], faces: faces.length, colour: itemColour.get(id) || null, faceCols, box: vertexBox(faces) });
}
// Bounding box of a solid's vertices, in the solid's own (part) coordinates. to-glb.js uses it to
// tell apart solids with the same face count (e.g. six identical servo mounts).
function vertexBox(faces) {
  const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity], seen = new Set();
  const stack = [...faces];
  while (stack.length) {
    const id = stack.pop(); if (seen.has(id)) continue; seen.add(id);
    const s = E.get(id); if (!s) continue;
    if (s.startsWith('VERTEX_POINT')) {
      const p = /\(([-\d.E+]+),([-\d.E+]+),([-\d.E+]+)\)/.exec(E.get(refs(s)[0]) || '');
      if (p) for (let a = 0; a < 3; a++) { mn[a] = Math.min(mn[a], +p[a + 1]); mx[a] = Math.max(mx[a], +p[a + 1]); }
      continue;
    }
    // walk topology only (faces -> loops -> edges -> vertices), not into surface / curve geometry
    if (/^(ADVANCED_FACE|FACE_SURFACE|FACE_OUTER_BOUND|FACE_BOUND|EDGE_LOOP|ORIENTED_EDGE|EDGE_CURVE)/.test(s)) stack.push(...refs(s));
  }
  return mn[0] === Infinity ? null : [mn, mx];
}
const hex = (c) => c ? '#' + c.map((v) => Math.round(v * 255).toString(16).padStart(2, '0')).join('') : '-';
for (const s of solids) {
  const fc = new Map(); s.faceCols.forEach((c) => { const k = hex(c); fc.set(k, (fc.get(k) || 0) + 1); });
  console.log(s.id, s.type, JSON.stringify(s.name), 'faces', s.faces, 'solid', hex(s.colour), 'faceCols', [...fc].map(([k, v]) => k + 'x' + v).join(' '));
}
fs.writeFileSync('step-colors.json', JSON.stringify(solids));
