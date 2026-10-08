// app/index.html is written for the Claude Artifact host, which wraps the page in its own
// <!doctype html><head>… skeleton. This adds that skeleton so the page also runs on any static
// host (GitHub Pages, Netlify, a local server) and copies the model next to it.
//   node build-standalone.js [src=app] [out=dist]
// The headgear page goes to dist/headgear/, so the two pages' relative links work there too.
const fs = require('fs'), path = require('path');
const src = process.argv[2] || 'app', out = process.argv[3] || 'dist';
const page = fs.readFileSync(path.join(src, 'index.html'), 'utf8');
const head = `<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<style>body { margin: 0; }</style>
</head>
<body>
`;
fs.mkdirSync(out, { recursive: true });
fs.writeFileSync(path.join(out, 'index.html'), head + page + '\n</body>\n</html>\n');
fs.copyFileSync(path.join(src, 'model.json'), path.join(out, 'model.json'));
fs.copyFileSync(path.join(src, 'manifest.webmanifest'), path.join(out, 'manifest.webmanifest'));
console.log(`wrote ${out}/index.html + model.json + manifest.webmanifest`);
