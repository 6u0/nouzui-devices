// One-off: per-step callout lists for the しくみ tour + clearer callout subtitles
const fs = require('fs');
const f = 'app/index.html';
let s = fs.readFileSync(f, 'utf8');
const rep = (a, b) => { if (!s.includes(a)) throw new Error('missing: ' + a); s = s.replace(a, b); };

const labels = {
  "tab: '全体',": "['m5', 'hall', 'vibrator', 'battery']",
  "tab: '通信',": "['m5']",
  "tab: '磁石',": "['hall', 'socket', 'm5']",
  "tab: '圧力',": "['m5']",
  "tab: '衝撃',": "['m5']",
  "tab: '振動',": "['m5', 'drv', 'vibrator']",
  "tab: '電源',": "['tp', 'battery', 'm5']",
};
for (const [k, v] of Object.entries(labels)) rep(k, `${k} labels: ${v},`);

rep("en: 'lipo battery', meshes: [0], layer: 1, sub: '電源',", "en: 'lipo battery', meshes: [0], layer: 1, sub: '電源（GPIO5で電圧監視）',");
rep("en: 'vibrator', meshes: [1], layer: 2, sub: '出力',", "en: 'vibrator', meshes: [1], layer: 2, sub: '出力（心拍パターン）',");
rep("en: 'hall_sensor', meshes: [3], layer: 4, sub: '入力',", "en: 'hall_sensor', meshes: [3], layer: 4, sub: '入力（GPIO10）',");
rep("en: 'pinsocket_8', meshes: [15],", "en: 'pinsocket_8', sub: 'M5StickS3との接続', meshes: [15],");
fs.writeFileSync(f, s);
console.log('ok');
