// 3/3: raw.json (step-mesh.py output) + step-colors.json -> app/model.glb + app/model.json
// One node per mesh named "m<index>", with STEP colours baked into COLOR_0.
// Colours come from raw.json (read by OpenCascade's XCAF per solid and per face, instance-aware).
const fs = require('fs');
const res = JSON.parse(fs.readFileSync('raw.json', 'utf8'));
const solids = JSON.parse(fs.readFileSync('step-colors.json', 'utf8'));

// Solid names only (brain, shell…; XCAF does not expose them): step-mesh emits one mesh per solid
// *instance* while STEP lists each solid definition once, so walk both in order matching on face
// count. Not trusted for colours: equal face counts mis-pair small parts (resistors, film sensors).
// First try an exact fit: same face count and the same vertex bounding box (works for bodies placed
// without a transform, i.e. the printed parts at the top of the assembly).
const meshBox = (m) => {
  const p = m.attributes.position.array, mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let k = 0; k < p.length; k += 3) for (let a = 0; a < 3; a++) { mn[a] = Math.min(mn[a], p[k + a]); mx[a] = Math.max(mx[a], p[k + a]); }
  return [mn, mx];
};
const boxGap = (a, b) => Math.max(...[0, 1].flatMap((e) => [0, 1, 2].map((k) => Math.abs(a[e][k] - b[e][k]))));
let ptr = 0;
const exact = new Set();  // meshes whose solid was found by shape + position, not just face count
const match = res.meshes.map((m, mi) => {
  const n = m.brep_faces.length, mb = meshBox(m);
  let best = null, gap = 5;  // mm; vertex boxes miss bulging curved faces, so allow some slack (closest wins)
  for (const s of solids) if (s.faces === n && s.box && boxGap(s.box, mb) < gap) { gap = boxGap(s.box, mb); best = s; }
  if (best) { exact.add(mi); return best; }
  for (let k = ptr; k < solids.length; k++) if (solids[k].faces === n) { ptr = k + 1; return solids[k]; }
  for (let k = 0; k < solids.length; k++) if (solids[k].faces === n) return solids[k];
  return null;
});

// Colour overrides, keyed by STEP colour as 0-255 hex. M5StickC Plus orange -> slate.
const OVERRIDES = { db620b: [0x3f, 0x45, 0x51] };
const key = (c) => c.map((v) => Math.round(v * 255).toString(16).padStart(2, '0')).join('');
const toLinear = (c) => {
  const o = OVERRIDES[key(c)];
  return (o ? o.map((v) => v / 255) : c).map((v) => Math.pow(v, 2.2));
};
// Per-model position fixes, from to-glb.config.json in the working directory (optional):
//   { "shift": { "parts": ["brain", "brain_cover"], "by": [0, -9.5, -18.5] } }
// The brain device needs one (see that file); other models (headgear/) run without it.
// The whole model is then lifted so its lowest point is back at z = 0.
const CONFIG = fs.existsSync('to-glb.config.json') ? JSON.parse(fs.readFileSync('to-glb.config.json', 'utf8')) : {};
const SHIFT_PARTS = new Set((CONFIG.shift && CONFIG.shift.parts) || []);
const SHIFT_BY = (CONFIG.shift && CONFIG.shift.by) || [0, 0, 0];
const shiftOf = (i) => (SHIFT_PARTS.has(match[i] && match[i].name) ? SHIFT_BY : [0, 0, 0]);
let lift = Infinity;
res.meshes.forEach((m, i) => {
  const p = m.attributes.position.array, dz = shiftOf(i)[2];
  for (let k = 2; k < p.length; k += 3) lift = Math.min(lift, p[k] + dz);
});
lift = -lift;
// Each mesh's place in the CAD assembly, e.g. "M5StickC Plus/M5StickC v14/Grove socket v7".
// The viewer picks parts by these paths, so adding parts to the STEP does not break the catalogue.
// The top assembly name is dropped; a bare "COMPOUND" leaf takes the STEP solid name (brain, shell…)
// unless that is Fusion's generic "ボディN".
const paths = [];
// Repeated sub-assemblies (e.g. six identical servos) get "#1", "#2"… so each can be picked alone.
(function walk(n, at, tag = '') {
  const here = n.name ? [...at, n.name + tag] : at;
  for (const i of n.meshes) paths[i] = here.slice(1);  // drop the top assembly name
  const count = {}, seen = {};
  for (const c of n.children) if (c.children.length) count[c.name] = (count[c.name] || 0) + 1;
  n.children.forEach((c) => {
    const dup = c.children.length && count[c.name] > 1;
    seen[c.name] = (seen[c.name] || 0) + 1;
    walk(c, here, dup ? '#' + seen[c.name] : '');
  });
})(res.root, []);
const pathOf = (i) => {
  const p = [...(paths[i] || [])], solid = exact.has(i) && match[i].name;
  if (p[p.length - 1] === 'COMPOUND' && solid && !solid.startsWith('\\X2\\')) p[p.length - 1] = solid;
  return p.join('/');
};
const bufs = [];
let offset = 0;
const bufferViews = [], accessors = [], meshes = [], nodes = [];

function push(typed, target, byteStride) {
  const b = Buffer.from(typed.buffer, typed.byteOffset, typed.byteLength);
  const pad = (4 - (b.length % 4)) % 4;
  bufferViews.push({ buffer: 0, byteOffset: offset, byteLength: b.length, target, ...(byteStride ? { byteStride } : {}) });
  bufs.push(b, Buffer.alloc(pad));
  offset += b.length + pad;
  return bufferViews.length - 1;
}

// Bodies made of thousands of tiny planar faces (the STL-derived gel brain) come out faceted:
// weld shared corners and average the face normals so the surface shades smoothly.
const SMOOTH_MIN_FACES = 5000;
function weld(pos, idx) {
  const map = new Map(), remap = new Uint32Array(pos.length / 3), out = [];
  for (let v = 0; v < remap.length; v++) {
    const k = [0, 1, 2].map((a) => Math.round(pos[v * 3 + a] * 1e4)).join(',');
    let n = map.get(k);
    if (n === undefined) { n = out.length / 3; map.set(k, n); out.push(pos[v * 3], pos[v * 3 + 1], pos[v * 3 + 2]); }
    remap[v] = n;
  }
  const p = new Float32Array(out), ix = idx.map((v) => remap[v]), nr = new Float32Array(p.length);
  for (let t = 0; t < ix.length; t += 3) {
    const [a, b, c] = [ix[t] * 3, ix[t + 1] * 3, ix[t + 2] * 3];
    const e1 = [0, 1, 2].map((k) => p[b + k] - p[a + k]), e2 = [0, 1, 2].map((k) => p[c + k] - p[a + k]);
    const fn = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]]; // area-weighted
    for (const v of [a, b, c]) for (let k = 0; k < 3; k++) nr[v + k] += fn[k];
  }
  for (let v = 0; v < nr.length; v += 3) {
    const l = Math.hypot(nr[v], nr[v + 1], nr[v + 2]) || 1;
    nr[v] /= l; nr[v + 1] /= l; nr[v + 2] /= l;
  }
  return { pos: p, nrm: nr, idx: ix };
}

// "compact": true in to-glb.config.json stores normals and colours as 16-bit normalised integers
// (KHR_mesh_quantization; three.js reads them natively) and indices as 16-bit where they fit.
// About 40 % smaller with no visible difference; used by headgear/ to stay under the 16 MB host limit.
const COMPACT = !!CONFIG.compact;
function vec3x16(src, signed) {  // VEC3 -> 16-bit normalised, padded to a 4-byte-aligned stride of 8
  const out = signed ? new Int16Array(src.length / 3 * 4) : new Uint16Array(src.length / 3 * 4), s = signed ? 32767 : 65535;
  for (let v = 0, n = src.length / 3; v < n; v++) for (let a = 0; a < 3; a++) out[v * 4 + a] = Math.round(Math.max(signed ? -1 : 0, Math.min(1, src[v * 3 + a])) * s);
  return out;
}
function pushVec3(arr, signed) {  // returns accessor fields for a VEC3 attribute
  if (!COMPACT) return { bufferView: push(arr, 34962), componentType: 5126 };
  return { bufferView: push(vec3x16(arr, signed), 34962, 8), componentType: signed ? 5122 : 5123, normalized: true };
}
// "simplify": <mm> in to-glb.config.json drops triangles that change the surface by less than that
// (meshoptimizer). Every CAD face keeps its exact outline (borders are locked), so colours, edges
// and the fit between parts stay as they were: only the inside of finely tessellated faces thins out.
const SIMPLIFY = CONFIG.simplify || 0;
const { MeshoptSimplifier } = require('meshoptimizer');
function simplify(pos, nrm, col, idx) {
  const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let k = 0; k < pos.length; k += 3) for (let a = 0; a < 3; a++) { mn[a] = Math.min(mn[a], pos[k + a]); mx[a] = Math.max(mx[a], pos[k + a]); }
  const ext = Math.hypot(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]);  // the error is given relative to this
  if (!ext) return { pos, nrm, col, idx };
  const [out] = MeshoptSimplifier.simplify(idx, pos, 3, 0, SIMPLIFY / ext, ['LockBorder']);
  if (out.length === idx.length) return { pos, nrm, col, idx };
  // drop the vertices no triangle uses any more
  const remap = new Int32Array(pos.length / 3).fill(-1);
  let n = 0;
  for (const v of out) if (remap[v] < 0) remap[v] = n++;
  const pick = (src) => {
    if (!src) return src;
    const dst = new Float32Array(n * 3);
    remap.forEach((d, v) => { if (d >= 0) dst.set(src.subarray(v * 3, v * 3 + 3), d * 3); });
    return dst;
  };
  return { pos: pick(pos), nrm: pick(nrm), col: pick(col), idx: out.map((v) => remap[v]) };
}

function build() {
  const report = [];
  res.meshes.forEach((m, i) => {
    let pos = new Float32Array(m.attributes.position.array);
    const sh = shiftOf(i);
    for (let k = 0; k < pos.length; k += 3) { pos[k] += sh[0]; pos[k + 1] += sh[1]; pos[k + 2] += sh[2] + lift; }
    let nrm = m.attributes.normal ? new Float32Array(m.attributes.normal.array) : null;
    let idx = new Uint32Array(m.index.array);
    const smooth = m.brep_faces.length >= SMOOTH_MIN_FACES;
    const s = match[i];
    const base = m.color || [0.6, 0.6, 0.6];
    if (smooth) ({ pos, nrm, idx } = weld(pos, idx));  // one colour for the whole body
    let col = new Float32Array(pos.length);
    const lin = toLinear(base);
    for (let k = 0; k < col.length; k += 3) col.set(lin, k);
    if (!smooth) m.brep_faces.forEach((f) => {
      if (!f.color) return;
      const l = toLinear(f.color);
      for (let t = f.first; t <= f.last; t++) for (let v = 0; v < 3; v++) col.set(l, idx[t * 3 + v] * 3);
    });
    if (SIMPLIFY && !smooth) ({ pos, nrm, col, idx } = simplify(pos, nrm, col, idx));
    report.push(`${i} ${pathOf(i)} -> ${s ? s.id + ' ' + s.name : 'NONE'} ${base.map((v) => Math.round(v * 255)).join(',')}`);

    const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
    for (let k = 0; k < pos.length; k += 3) for (let a = 0; a < 3; a++) { mn[a] = Math.min(mn[a], pos[k + a]); mx[a] = Math.max(mx[a], pos[k + a]); }
    const attrs = {};
    accessors.push({ bufferView: push(pos, 34962), componentType: 5126, count: pos.length / 3, type: 'VEC3', min: mn, max: mx });
    attrs.POSITION = accessors.length - 1;
    if (nrm) { accessors.push({ ...pushVec3(nrm, true), count: nrm.length / 3, type: 'VEC3' }); attrs.NORMAL = accessors.length - 1; }
    accessors.push({ ...pushVec3(col, false), count: col.length / 3, type: 'VEC3' });
    attrs.COLOR_0 = accessors.length - 1;
    const small = COMPACT && pos.length / 3 <= 65536;
    accessors.push({ bufferView: push(small ? Uint16Array.from(idx) : idx, 34963), componentType: small ? 5123 : 5125, count: idx.length, type: 'SCALAR' });
    meshes.push({ name: 'm' + i, primitives: [{ attributes: attrs, indices: accessors.length - 1 }] });
    // extras.c = original STEP base colour (picks a surface finish), extras.path = assembly path (picks the part)
    nodes.push({ name: 'm' + i, mesh: i, extras: { c: key(base), path: pathOf(i) } });
  });
  fs.writeFileSync('match-report.txt', report.join('\n'));

  const bin = Buffer.concat(bufs);
  const gltf = {
    asset: { version: '2.0', generator: 'brain-device converter' },
    scene: 0, scenes: [{ nodes: nodes.map((_, i) => i) }],
    nodes, meshes, accessors, bufferViews, buffers: [{ byteLength: bin.length }],
    ...(COMPACT ? { extensionsUsed: ['KHR_mesh_quantization'], extensionsRequired: ['KHR_mesh_quantization'] } : {}),
  };
  let json = Buffer.from(JSON.stringify(gltf));
  json = Buffer.concat([json, Buffer.alloc((4 - (json.length % 4)) % 4, 0x20)]);
  const header = Buffer.alloc(12);
  header.writeUInt32LE(0x46546c67, 0); header.writeUInt32LE(2, 4);
  header.writeUInt32LE(12 + 8 + json.length + 8 + bin.length, 8);
  const jh = Buffer.alloc(8); jh.writeUInt32LE(json.length, 0); jh.writeUInt32LE(0x4e4f534a, 4);
  const bh = Buffer.alloc(8); bh.writeUInt32LE(bin.length, 0); bh.writeUInt32LE(0x004e4942, 4);
  fs.mkdirSync('app', { recursive: true });
  fs.writeFileSync('app/model.glb', Buffer.concat([header, jh, json, bh, bin]));
  // Same GLB, base64-wrapped in JSON: the artifact host serves .json but not .glb
  const glb = fs.readFileSync('app/model.glb');
  fs.writeFileSync('app/model.json', JSON.stringify({ glb: glb.toString('base64') }));
  console.log('wrote app/model.glb + app/model.json', ((12 + 16 + json.length + bin.length) / 1e6).toFixed(2), 'MB');
}
if (SIMPLIFY) MeshoptSimplifier.ready.then(build);
else build();
