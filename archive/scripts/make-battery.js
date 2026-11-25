// One-off: 603040 LiPo (source/603040.STEP, meshed by step-mesh.py into raw.json) -> source/battery-602535.json.
// The device uses a 602535 cell (6 × 25 × 35 mm), so the 603040 body (6 × 30 × 40) is scaled to that
// size in its plane, keeping the thickness. Its label (raised 0.01 mm, reading "603040 2.22Wh 600mAh")
// is flattened into the face and its side walls dropped, so no wrong text shows as CAD outlines.
// Placed in the device's CAD coordinates at the centre of the SIGGRAPH STEP's battery placeholder
// (7 × 38 × 28 mm block, x 3.2–10.2 / y -19.9–18.1 / z 0.2–28.2): thickness along x with the label face
// outward (+x), 35 mm along y, 25 mm along z.
// Usage (in a folder holding raw.json of 603040.STEP): node make-battery.js <out.json>
const fs = require('fs');
const src = JSON.parse(fs.readFileSync('raw.json', 'utf8')).meshes[0];
const SX = 25 / 30, SZ = 35 / 40, LABEL = 6.0;          // cell width / length scale; face the label sits on
const AT = [3.7, -0.9, 14.2];                            // centre of the placeholder (x = 3.7 + thickness 0..6)
const p = src.attributes.position.array, n = src.attributes.normal.array, ix = src.index.array;
const top = (k) => p[k * 3 + 1] > LABEL + 0.004;         // vertex on the raised label
const pos = [], nrm = [];
for (let k = 0; k < p.length / 3; k++) {
  const [bx, by0, bz] = [p[k * 3], p[k * 3 + 1], p[k * 3 + 2]];
  const by = top(k) ? LABEL : by0;
  pos.push(AT[0] + by, AT[1] + bz * SZ, AT[2] + bx * SX);  // (x, y, z) of the cell -> (z, x, y) of the device: a rotation
  let [nx, ny, nz] = [ny0(k), n[k * 3 + 2] / SZ, n[k * 3] / SX];
  const l = Math.hypot(nx, ny, nz) || 1;
  nrm.push(nx / l, ny / l, nz / l);
}
function ny0(k) { return n[k * 3 + 1]; }
// Colours from the SLDPRT (the STEP export carried only one): matte aluminium pouch, white label
// (text left out: it reads 603040), amber polyimide tape over the sealed end (z < -13 in the cell).
const COL = { pouch: [0.75, 0.75, 0.76], label: [0.93, 0.93, 0.92], tape: [0.78, 0.55, 0.12] };
const lab = [Infinity, -Infinity, Infinity, -Infinity];  // label rectangle (x, z) from its raised faces
for (let k = 0; k < p.length / 3; k++) if (top(k)) { lab[0] = Math.min(lab[0], p[k * 3]); lab[1] = Math.max(lab[1], p[k * 3]); lab[2] = Math.min(lab[2], p[k * 3 + 2]); lab[3] = Math.max(lab[3], p[k * 3 + 2]); }
const groups = { pouch: [], label: [], tape: [] };
for (let t = 0; t < ix.length; t += 3) {
  const v = [ix[t], ix[t + 1], ix[t + 2]], up = v.filter(top).length;
  if (up && up < 3 && v.every((k) => Math.abs(p[k * 3 + 1] - LABEL) < 0.02)) continue;  // label side wall
  const c = [0, 1, 2].map((a) => (p[v[0] * 3 + a] + p[v[1] * 3 + a] + p[v[2] * 3 + a]) / 3);
  const g = c[2] < -13 ? 'tape' : c[1] > LABEL - 0.01 && c[0] > lab[0] && c[0] < lab[1] && c[2] > lab[2] && c[2] < lab[3] ? 'label' : 'pouch';
  groups[g].push(...v);
}
const idx = [], faces = [];
for (const g in groups) { if (!groups[g].length) continue; faces.push({ first: idx.length / 3, last: (idx.length + groups[g].length) / 3 - 1, color: COL[g] }); idx.push(...groups[g]); }
const round = (a, d) => a.map((v) => Math.round(v * d) / d);
const out = {
  _note: 'LiPo 602535 for the brain device, made from source/603040.STEP by archive/scripts/make-battery.js. add-parts.js puts it in place of the STEP battery placeholder.',
  replaces: { min: [3.2, -19.9, 0.2], max: [10.2, 18.1, 28.2] },
  mesh: { name: 'lipo battery', color: COL.pouch,
    attributes: { position: { array: round(pos, 1e4) }, normal: { array: round(nrm, 1e5) } },
    index: { array: idx }, brep_faces: faces },
};
fs.writeFileSync(process.argv[2], JSON.stringify(out));
console.log('triangles', ix.length / 3, '->', idx.length / 3);
