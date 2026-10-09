// Tiny static server for local preview.  node serve.js [dir=app] [port=5173]
// Listens on all interfaces, so an iPad on the same Wi-Fi can open http://<this PC's IP>:5173
// Serving app/ also serves the headgear page at /headgear/ (from headgear/app), so the two pages
// link to each other locally the same way they do in dist/ (dist/ and dist/headgear/).
const http = require('http'), fs = require('fs'), path = require('path');
const dir = process.argv[2] || 'app';
const root = path.join(__dirname, dir);
const port = +(process.argv[3] || process.env.PORT || 5173);
const mounts = dir === 'app' ? { '/headgear/': path.join(__dirname, 'headgear', 'app') } : {};
const types = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript', '.json': 'application/json', '.glb': 'model/gltf-binary', '.css': 'text/css', '.webmanifest': 'application/manifest+json', '.png': 'image/png', '.svg': 'image/svg+xml' };
http.createServer((req, res) => {
  let url = decodeURIComponent(req.url.split('?')[0]);
  const prefix = Object.keys(mounts).find((m) => url === m.slice(0, -1) || url.startsWith(m));
  if (prefix && url === prefix.slice(0, -1)) { res.writeHead(301, { Location: prefix }); return res.end(); }  // /headgear -> /headgear/
  const base = prefix ? mounts[prefix] : root;
  if (prefix) url = '/' + url.slice(prefix.length);
  const p = path.join(base, url.replace(/\/$/, '/index.html'));
  if (!p.startsWith(base)) { res.writeHead(403); return res.end('forbidden'); }
  fs.readFile(p, (err, data) => {
    if (err) { res.writeHead(404); return res.end('not found'); }
    res.writeHead(200, { 'Content-Type': types[path.extname(p)] || 'application/octet-stream', 'Cache-Control': 'no-store' });
    res.end(data);
  });
}).listen(port, '0.0.0.0', () => {
  const os = require('os');
  const interfaces = os.networkInterfaces();
  let localIP = 'localhost';
  for (const name of Object.keys(interfaces)) {
    for (const iface of interfaces[name]) {
      if (iface.family === 'IPv4' && !iface.internal) {
        localIP = iface.address;
        break;
      }
    }
  }
  console.log(`serving ${root} on http://${localIP}:${port}`);
  for (const [m, d] of Object.entries(mounts)) console.log(`  and ${d} on http://${localIP}:${port}${m}`);
});
