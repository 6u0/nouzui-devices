// One-off: replace everything before the import map in app/index.html with app/head.html
const fs = require('fs');
const f = 'app/index.html';
const s = fs.readFileSync(f, 'utf8');
const i = s.indexOf('<script type="importmap">');
if (i < 0) throw new Error('import map not found');
fs.writeFileSync(f, fs.readFileSync('app/head.html', 'utf8') + s.slice(i));
fs.unlinkSync('app/head.html');
console.log('ok');
