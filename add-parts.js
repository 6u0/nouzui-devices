// Between step-mesh.py / step-colors.js and to-glb.js: brings in the parts the SIGGRAPH STEP lacks.
//  1) The gel brain + base cover from source/gel.json. They were meshed once from the older
//     "brain device2.step" (same CAD coordinates for the device), so to-glb.config.json's shift still
//     seats the device in the brain's cavity exactly as before.
//  2) The LiPo from source/battery-602535.json in place of the STEP's battery placeholder block (found by
//     its bounding box), as its own "lipo battery" node so the viewer can pick it by path.
// Usage: node add-parts.js
const fs = require('fs');
const raw = JSON.parse(fs.readFileSync('raw.json', 'utf8'));
const solids = JSON.parse(fs.readFileSync('step-colors.json', 'utf8'));
const top = raw.root.children[0];  // the top assembly; to-glb.js drops its name from the paths

if (solids.some((s) => s.name === 'brain')) console.log('add-parts: the STEP already has a brain, no gel added');
else {
  const gel = JSON.parse(fs.readFileSync('source/gel.json', 'utf8'));
  for (const m of gel.meshes) {
    raw.meshes.push(m);
    // a bare COMPOUND leaf takes the STEP solid name (brain / brain_cover) in to-glb.js
    top.children.push({ name: 'COMPOUND', meshes: [raw.meshes.length - 1], children: [] });
  }
  solids.push(...gel.solids);
  console.log('add-parts: added', gel.solids.map((s) => s.name).join(', '));
}

const bat = JSON.parse(fs.readFileSync('source/battery-602535.json', 'utf8'));
const box = (m) => {
  const p = m.attributes.position.array, mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let k = 0; k < p.length; k += 3) for (let a = 0; a < 3; a++) { mn[a] = Math.min(mn[a], p[k + a]); mx[a] = Math.max(mx[a], p[k + a]); }
  return [mn, mx];
};
const near = (m) => { const [mn, mx] = box(m); return [0, 1, 2].every((a) => Math.abs(mn[a] - bat.replaces.min[a]) < 0.5 && Math.abs(mx[a] - bat.replaces.max[a]) < 0.5); };
const hit = raw.meshes.findIndex(near);
if (hit < 0) console.log('add-parts: battery placeholder not found, LiPo not added');
else {
  raw.meshes[hit] = bat.mesh;
  (function unlink(n) { n.meshes = n.meshes.filter((i) => i !== hit); n.children.forEach(unlink); })(raw.root);
  top.children.push({ name: 'lipo battery', meshes: [hit], children: [] });
  console.log('add-parts: LiPo 602535 in place of mesh', hit);
}
fs.writeFileSync('raw.json', JSON.stringify(raw));
fs.writeFileSync('step-colors.json', JSON.stringify(solids));
