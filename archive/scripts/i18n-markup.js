// One-off: tag static UI text for translation, add the language switch, TC font
const fs = require('fs');
const f = 'app/index.html';
let s = fs.readFileSync(f, 'utf8');
const rep = (a, b) => { if (!s.includes(a)) throw new Error('missing: ' + a); s = s.replace(a, b); };

rep('family=Noto+Sans+JP:wght@400;500;700', 'family=Noto+Sans+JP:wght@400;500;700&family=Noto+Sans+TC:wght@400;500;700');
rep('[hidden] { display: none !important; }', `[hidden] { display: none !important; }
html[lang="zh-Hant"] { --font-ui: "Noto Sans TC", "PingFang TC", "Microsoft JhengHei", system-ui, sans-serif; }
.langs { top: calc(72px + env(safe-area-inset-top, 0px)); }
.langs button { padding: 8px 12px; min-height: 34px; font-size: 13px; }`);
rep('@media (max-width: 700px) {', '@media (max-width: 700px) {\n  .langs { top: calc(16px + env(safe-area-inset-top, 0px)); bottom: auto; }');

rep('<div id="stage" aria-label="脳デバイスの3Dモデル。ドラッグで回転、ピンチで拡大">', '<div id="stage" data-i18n-aria="stage" aria-label="脳デバイスの3Dモデル。ドラッグで回転、ピンチで拡大">');
rep('<div class="loading" id="loading">LOADING MODEL…</div>', '<div class="loading" id="loading" data-i18n="loading">LOADING MODEL…</div>');
rep('<h1>脳デバイスの中身</h1>', '<h1 data-i18n="title">脳デバイスの中身</h1>');
rep(`<nav class="modes" aria-label="表示モード">
  <button id="mode-explore" aria-pressed="true">観察</button>
  <button id="mode-tour" aria-pressed="false">しくみ</button>
</nav>`, `<nav class="modes" data-i18n-aria="modes" aria-label="表示モード">
  <button id="mode-explore" aria-pressed="true" data-i18n="explore">観察</button>
  <button id="mode-tour" aria-pressed="false" data-i18n="tour">しくみ</button>
</nav>

<nav class="modes langs" aria-label="言語 / 語言">
  <button id="lang-ja" aria-pressed="true" lang="ja">日本語</button>
  <button id="lang-zh" aria-pressed="false" lang="zh-Hant">繁中</button>
</nav>`);
rep('<div class="cap">分解<br>↑</div>', '<div class="cap"><span data-i18n="explode">分解</span><br>↑</div>');
rep('aria-label="高さ方向の分解量"', 'data-i18n-aria="track" aria-label="高さ方向の分解量"');
rep('<span class="dot"></span>外装を透かす</button>', '<span class="dot"></span><span data-i18n="ghost">外装を透かす</span></button>');
rep('<span class="dot"></span>自動回転</button>', '<span class="dot"></span><span data-i18n="spin">自動回転</span></button>');
rep('<button class="chip" id="t-reset">視点を戻す</button>', '<button class="chip" id="t-reset" data-i18n="reset">視点を戻す</button>');
rep('<button class="close" id="sheet-close" aria-label="閉じる">', '<button class="close" id="sheet-close" data-i18n-aria="close" aria-label="閉じる">');
rep('<button id="tour-prev">← 前へ</button><button id="tour-next">次へ →</button>', '<button id="tour-prev" data-i18n="prev">← 前へ</button><button id="tour-next" data-i18n="next">次へ →</button>');
fs.writeFileSync(f, s);
console.log('ok');
