// One-off restyle of app/index.html: teal accent, squarer controls, opaque panels, Noto Sans JP
const fs = require('fs');
const f = 'app/index.html';
let s = fs.readFileSync(f, 'utf8');
const rep = (a, b) => {
  if (!s.includes(a)) throw new Error('missing: ' + a);
  s = s.split(a).join(b);
};

// 4. typeface
rep('family=Zen+Kaku+Gothic+New:wght@400;500;700', 'family=Noto+Sans+JP:wght@400;500;700');
rep('--font-ui: "Zen Kaku Gothic New",', '--font-ui: "Noto Sans JP",');

// 1. accent colour
rep('--accent: #c9711c;          /* solder amber: callouts, active state, signal flow */', '--accent: #0e7c86;          /* teal: callouts, active state, signal flow */');
rep("|| '#c9711c'", "|| '#0e7c86'");

// 3. opaque panels with a light lift instead of frosted glass
rep('--panel: rgba(247, 249, 251, 0.86);', '--panel: #f8f9fa;');
s = s.replace(/\s*backdrop-filter: blur\(\d+px\); -webkit-backdrop-filter: blur\(\d+px\);/g, '');
rep('--r: 14px;', '--r: 10px;\n  --shadow: 0 1px 2px rgba(20, 26, 34, 0.08), 0 4px 14px rgba(20, 26, 34, 0.06);');
for (const sel of ['.titleblock {', '.modes {', '.rail {', '.chip {', '.sheet {', '.tour {', '.callout {'])
  rep(sel, `${sel}\n  box-shadow: var(--shadow);`);

// 2. squarer controls: no pills, no circles
rep('border-radius: 999px', 'border-radius: 9px');
rep('border-radius: 50%; background: var(--accent)', 'border-radius: 9px; background: var(--accent)');
rep('.chip .dot { width: 9px; height: 9px; border-radius: 50%;', '.chip .dot { width: 10px; height: 10px; border-radius: 3px;');
rep('cursor: pointer; border-radius: 50%; }', 'cursor: pointer; border-radius: 8px; }');
rep('.modes button {\n  border: 0; background: transparent; color: var(--ink); font: 500 14px/1 var(--font-ui);\n  padding: 11px 18px; border-radius: 9px;', '.modes button {\n  border: 0; background: transparent; color: var(--ink); font: 500 14px/1 var(--font-ui);\n  padding: 11px 18px; border-radius: 6px;');

fs.writeFileSync(f, s);
console.log('ok', (s.match(/50%|999px|backdrop/g) || []).join(' '));
