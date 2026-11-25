// SIGGRAPH headgear = the new STEP (source/hedogggg.step) from head_move up + the older STEP
// (source/metaq brain.step) for everything below it. The new export changed the case, head_move and
// the electronics (XIAO nRF52840 instead of ESP32, TRRS jack for the ear-clip pulse sensor, reel, no
// ToF / MAX30100, no spring / coil cover), but it lost the servos and the stand, so those come from the
// old model. Both STEPs
// share the same CAD coordinates (the frame parts match to 0.1 mm), so nothing needs moving.
// Usage (in headgear/, see npm run headgear:convert): after meshing the old STEP, `node merge.js --keep-old`
// moves raw.json / step-colors.json to raw-old.json / step-colors-old.json; after meshing the new STEP,
// `node merge.js` writes the merged raw.json + step-colors.json for to-glb.js.
const fs = require('fs');
if (process.argv[2] === '--keep-old') {
  fs.renameSync('raw.json', 'raw-old.json'); fs.renameSync('step-colors.json', 'step-colors-old.json');
  process.exit(0);
}
const read = (f) => JSON.parse(fs.readFileSync(f, 'utf8'));
const OLD = read('raw-old.json'), OLDS = read('step-colors-old.json');
const NEW = read('raw.json'), NEWS = read('step-colors.json');

const box = (m) => {
  const p = m.attributes.position.array, mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let k = 0; k < p.length; k += 3) for (let a = 0; a < 3; a++) { mn[a] = Math.min(mn[a], p[k + a]); mx[a] = Math.max(mx[a], p[k + a]); }
  return [mn, mx];
};
const gap = (a, b) => Math.max(...[0, 1].flatMap((e) => [0, 1, 2].map((k) => Math.abs(a[e][k] - b[e][k]))));
// STEP solid name of a top-level body: same face count and vertex box (as to-glb.js), else the closest box
function nameOf(m, solids) {
  const b = box(m);
  let best = null, g = 5;
  for (const s of solids) if (s.faces === m.brep_faces.length && s.box && gap(s.box, b) < g) { g = gap(s.box, b); best = s; }
  return best && best.name;
}
const closest = (raw, ids, solid) => ids.reduce((a, i) => (gap(box(raw.meshes[i]), solid.box) < gap(box(raw.meshes[a]), solid.box) ? i : a));

const meshes = [], top = { name: 'metaq brain', meshes: [], children: [] };
function copy(raw, n) {  // a subtree with its meshes appended to the merged list
  return { name: n.name, meshes: n.meshes.map((i) => meshes.push(raw.meshes[i]) - 1), children: n.children.map((c) => copy(raw, c)) };
}
const leaf = (name, raw, i) => ({ name, meshes: [meshes.push(raw.meshes[i]) - 1], children: [] });

// ---- old model: everything below head_move
const UPPER_OLD = new Set(['components_case', 'components_case_cover', 'reelcover', 'head_move (1)', 'spring', 'coil_cover']);
const oldTop = OLD.root.children[0];
for (const c of oldTop.children) {
  if (c.name === 'COMPOUND') {
    const keep = { name: 'COMPOUND', meshes: [], children: [] };
    for (const i of c.meshes) {
      const n = nameOf(OLD.meshes[i], OLDS);
      if (UPPER_OLD.has(n)) continue;
      if (n && !n.startsWith('\\X2\\')) top.children.push(leaf(n, OLD, i));  // named printed part (top, servomount_1…)
      else keep.meshes.push(meshes.push(OLD.meshes[i]) - 1);
    }
    top.children.push(keep);
    continue;
  }
  if (/^(PCB_ESP32|PCA9685|VL53L0X|30100)/.test(c.name)) continue;  // replaced / gone in the new electronics
  // all servos stay (the new export lost every one of them; the sketch drives all 8): MG90S ×6,
  // the SG90 in the case (#6, head impact) and the SG90 on the stand (#7, mirror)
  top.children.push(copy(OLD, c));
}

// ---- new model: head_move and above
const newTop = NEW.root.children[0];
for (const c of newTop.children) {
  if (c.name === 'components') { for (const k of c.children) top.children.push(copy(NEW, k)); continue; }  // PCA9685, reel, XIAO
  if (c.name === 'COMPOUND') {
    for (const name of ['components_case', 'components_case_cover', 'reelcover', 'XIAO_cover']) {
      const i = c.meshes.find((j) => nameOf(NEW.meshes[j], NEWS) === name);
      if (i === undefined) throw new Error('missing ' + name);
      top.children.push(leaf(name, NEW, i));
    }
    // head_move is a solid with voids, so its face count differs from the mesh: take the closest box
    const hm = NEWS.find((s) => s.name === 'head_move (1)');
    top.children.push(leaf('head_move', NEW, closest(NEW, c.meshes, hm)));
    continue;
  }
  top.children.push(copy(NEW, c));  // TRRSBreakout
}

// Mirror linkage: the stand SG90's horn tip → a long rod → the tab on the right side of the floor plate
// "ボディ112" that carries the mirror (hinged at its back edge). The rod and the plate are unnamed stand
// bodies in the CAD; they go into the servo's assembly as "mirror_rod" / "mirror_plate" so the viewer
// can move them with the servo (MIRROR_LINK / buildMirrorLink() in app/index.html).
{
  const ROD = [[79, -110.2, 6.5], [84, -67.7, 92]], PLATE = [[-84.8, -150.7, 7.8], [86.8, -70.2, 12.7]];
  const keep = top.children.find((n) => n.name === 'COMPOUND');
  const all = (n) => [...n.meshes, ...n.children.flatMap(all)];
  const stand = top.children.find((n) => /Sg90/.test(n.name) && Math.min(...all(n).map((i) => box(meshes[i])[0][2])) < 140);
  const rod = keep.meshes.find((i) => gap(box(meshes[i]), ROD) < 1), plate = keep.meshes.find((i) => gap(box(meshes[i]), PLATE) < 1);
  if (rod === undefined || plate === undefined || !stand) throw new Error('mirror linkage not found');
  keep.meshes = keep.meshes.filter((i) => i !== rod && i !== plate);
  stand.children.push({ name: 'mirror_rod', meshes: [rod], children: [] }, { name: 'mirror_plate', meshes: [plate], children: [] });
}
fs.writeFileSync('raw.json', JSON.stringify({ success: true, root: { name: '', meshes: [], children: [top] }, meshes }));
fs.writeFileSync('step-colors.json', JSON.stringify(OLDS));  // names are baked into the tree above
console.log('merge: ' + meshes.length + ' meshes');
