// One-off: rewrite explode layers to arms > shell > internals > M5 (top to bottom)
const fs = require('fs');
const f = 'app/index.html';
let s = fs.readFileSync(f, 'utf8');
const L = {
  shell: "stack: 'shell'", m5: 'layer: 0', screws: 'layer: 0.4', battery: 'layer: 1', hallcase: 'layer: 1.3',
  socket: 'layer: 1.6', vibrator: 'layer: 2', chargerbase: 'layer: 2', drvcase: 'layer: 2.7',
  drv: 'layer: 3.4', tp: 'layer: 3.4', hall: 'layer: 4', arm2: "stack: 'arms'", arm1: "stack: 'arms'",
};
for (const [id, v] of Object.entries(L)) {
  const re = new RegExp(`(\\{ id: '${id}',[^\\n]*?)layer: [\\d.]+`);
  if (!re.test(s)) throw new Error('no match ' + id);
  s = s.replace(re, `$1${v}`);
}
fs.writeFileSync(f, s);
console.log('ok');
