// 1/3: STEP -> raw.json (tessellated meshes + assembly tree) via occt-import-js
const fs = require('fs');
const STEP = process.argv[2] || 'source/brain device2.step';
const occt = require('occt-import-js')();

occt.then((o) => {
  const buf = new Uint8Array(fs.readFileSync(STEP));
  const res = o.ReadStepFile(buf, { linearUnit: 'millimeter', linearDeflectionType: 'bounding_box_ratio', linearDeflection: 0.001, angularDeflection: 0.5 });
  console.log('success', res.success, 'meshes', res.meshes.length);
  const walk = (n, d) => {
    console.log('  '.repeat(d) + (n.name || '(noname)') + ' meshes=' + JSON.stringify(n.meshes));
    n.children.forEach((c) => walk(c, d + 1));
  };
  walk(res.root, 0);
  res.meshes.forEach((m, i) => {
    const p = m.attributes.position.array;
    let mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9];
    for (let k = 0; k < p.length; k += 3) for (let a = 0; a < 3; a++) { mn[a] = Math.min(mn[a], p[k + a]); mx[a] = Math.max(mx[a], p[k + a]); }
    console.log(i, m.name, 'tris', m.index.array.length / 3, 'color', m.color, 'min', mn.map((v) => v.toFixed(1)).join(','), 'max', mx.map((v) => v.toFixed(1)).join(','));
  });
  fs.writeFileSync('raw.json', JSON.stringify(res));
});
