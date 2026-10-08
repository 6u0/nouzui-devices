const res = JSON.parse(require('fs').readFileSync('raw.json', 'utf8'));
res.meshes.forEach((m, i) => {
  const cols = new Set((m.brep_faces || []).map((f) => JSON.stringify(f.color && f.color.map((c) => +c.toFixed(2)))));
  console.log(i, m.name, 'faces', (m.brep_faces || []).length, 'faceColors', [...cols].join(' '));
});
