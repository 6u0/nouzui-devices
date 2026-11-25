// ============================================================================
//  NozuiHeadgear_SIGGRAPH — XR作品「脳髄はものを考えるところに非ず」ヘッドギア側ファームウェア【SIGGRAPH 版】
//  ★ SIGGRAPH 展示版。脳デバイス NozuiBrain_SIGGRAPH.ino、椅子 NozuiChair_SIGGRAPH.ino と組で使う
//  台湾版 NozuiHeadgear_Taiwan.ino（180° 版）がベース。差分は nozui_siggraph_spec.md（§3・§5）
//  XIAO nRF52840 Sense（Seeed nRF52 Boards ← mbed ではない方）＋ PCA9685 ＋ MG90S ×6 ＋ サーボ 2 本
//  ＋ Grove 耳クリップ心拍センサ（D0）＋ 椅子（M5Atom）への UART（D6=TX / D7=RX）
//
//  ■ SIGGRAPH 版で足したこと
//    ・コマンド 0x03 [Phase] を受ける（脳が Phase の変化時・接続時・1 秒ごとに送る）。Phase が変わった瞬間に:
//        Phase6 に入った   → ミラー（#7, ch15）0°→170°（2 秒）、頭部衝撃（#6, ch14）30° → 0.5 秒後 0°、椅子を倒す
//        Phase6 から出た   → ミラー 170°→0°（2 秒）、椅子を起こす
//        Phase1 に入った   → 心拍の「最初の 10 拍を捨てる」を数え直す
//      Phase が不明（起動直後・通信が切れたあと）から 6 になったときは倒さない（揃えるだけ）
//    ・心拍（D0 の立ち上がり）を拾い、最初の 10 拍を捨てたあと拍ごとに HEART を Notify で脳へ送る
//      （鳴らすかどうかは脳が Phase3・4 で判断する）
//    ・脳との BLE が切れた／0x03 が 3 秒来ない → 6 本は台湾版どおりリセット。Phase は覚えたまま、ミラー・椅子は動かさない
//      （2026-11-23 に変更。以前はミラーを戻し椅子を起こしていた。倒れたままでも害はなく、通信が戻ったときに
//       「切れる前の Phase → 今の Phase」で判断できるので、切れている間に Phase6 に入っても倒れる）
//    ・椅子とは UART 9600 で 4 バイトの電文 [0xA5][種類][値][種類 XOR 値] をやりとりする（200ms ごとに合図）
//    ・椅子は起動から 10 秒以内にこちらの電文が届かないと、椅子を起こして停止する（椅子の再起動まで戻らない）。
//      なので起動時に USB シリアルを待たない。椅子が停止したら、ヘッドギアはシリアルに警告を出し、ほかはそのまま動く
//    ・シリアル: 6 / 7 … Phase を 6 / 7 にしたことにする   d / u … 椅子を倒す / 起こす   k … 心拍の表示 ON/OFF
//    ・記録（2026-11-11）：シリアルに出すものすべてと、BLE の接続・切断理由、1 秒ごとの受信数、心拍 1 拍ごと、30 秒ごとの状態、
//      loop の停止を、行ごとに [#起動番号 起動からの秒] を付けて内蔵 2MB フラッシュに残す（電源を切っても消えない。古い方から上書き）。
//      取り出しは同じフォルダの nozui_log_dump.py（PC の時刻も付けて logs/ に保存）。シリアル: L=出す / G=状態 / E を 2 回=消す
//    ・配線の切り分け（2026-11-11、D6 が壊れた XIAO の調査で追加）: w / D / S / T / R / H（? で一覧）
//      D・T・R はレジスタを直接読む。digitalRead は出力ピンだと「出そうとした値」を返すので、実際の電圧の確認には使えない
//
//  ■ 必要ライブラリ（記録）：Adafruit SPIFlash（ボード同梱。XIAO Sense の P25Q16H を QSPI で使う）
//
//  ===== ここから下は台湾版の説明（6 本のサーボについては変わっていない） =====
//
//  ■ 役割
//    BLE Peripheral（GATT Server）。脳デバイス（Central）から 7 バイトのコマンドを受け取り、
//    サーボを動かすだけ（送信なし・Meta Quest とは接続しない）。
//      0x00 = リセット＆停止（0° へゆっくり戻して PWM 完全停止）
//      0x01 = Phase2 トリガー（内蔵シーケンス 15 秒を 1 回再生 → 0° → 停止）
//      0x02 = Phase3/4 サーボ値 [0x02][s0..s5]（0〜100）
//
//  ■ 値の扱い（0〜100 ベース）
//    脳デバイスとの通信値・Phase2 の収録データ・内部の状態はすべて「0〜100（%）」で持ち、
//    サーボに書き込む直前の servoWrite() で 0〜SERVO_ANGLE_MAX° にマップする。
//    可動域を変えたいときは SERVO_ANGLE_MAX だけ書き換えればよい（データや脳デバイス側は触らなくてよい）。
//
//  ■ 「Core0 / Core1」について（仕様書との違い）
//    nRF52840 は Cortex-M4 のシングルコアなので setup1()/loop1() は存在しない（それは RP2040 の機能）。
//    代わりに FreeRTOS のタスク分離で仕様書の意図を実現している：
//      ・「Core0」相当 … Bluefruit の BLE コールバックタスク。onCmdWrite() は受信キューに積むだけ。
//      ・「Core1」相当 … loop() タスク。キューを取り出して状態遷移・I2C 書き込みを行う。
//    svState は loop() 側だけが書き換えるので、コールバックとの競合（仕様書 §8-5）は起きない。
//
//  ■ シリアルコマンド（115200, 脳デバイスなしの単体テスト用。BLE と同じ経路で処理される）
//    0 … コマンド 0x00（リセット＆停止）
//    1 … コマンド 0x01（Phase2 シーケンス）
//    5 … コマンド 0x02（全 ch 50%）   m … コマンド 0x02（全 ch 100%）   z … コマンド 0x02（全 ch 0%）
//    s … 状態表示   ? … ヘルプ
//
//  ■ LED：未接続＝赤点滅／接続中＝緑／コマンド受信＝青 0.05 秒
//
//  ■ 必要ライブラリ：Adafruit PWM Servo Driver Library（2.x）、Bluefruit（ボード同梱）
// ============================================================================
#include <bluefruit.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_SPIFlashBase.h>     // XIAO Sense 内蔵の 2MB フラッシュ（P25Q16H、QSPI）。ボード同梱

// ================================================================ 記録（ログ）：電源を切っても残る
// シリアルに出す文字はすべて、行ごとに「[#起動番号 起動からの秒]」を付けて内蔵フラッシュ（2MB）にも書く。
// 加えて、シリアルには出さず記録だけに残すもの：BLE の接続・切断理由、1 秒ごとの受信数、心拍 1 拍ごと、30 秒ごとの状態、loop の停止
// ・2MB を 4KB × 512 の輪にして古い方から上書き（数時間〜十数時間分）。起動のたびに新しい 4KB から書き始める
// ・書き込みは別タスク（loop と同じ優先度）。loop からは RAM に積むだけなので、サーボ・BLE の処理は待たない
// ・電源を急に切っても、失うのは最後の約 20ms 分だけ
// ・取り出し：nozui_log_dump.py（PC 時刻も合わせて保存する）、またはシリアルで L
static auto& USBSerial = Serial;       // 本物の USB シリアル（下で Serial を置き換える前に押さえておく）

#define LOG_SECTOR   4096u
#define LOG_HDR      16u               // セクタ先頭：[magic][通し番号][起動番号][予備]
#define LOG_MAGIC    0x474C5A4Eu       // "NZLG"
#define LOG_RAM      16384u            // フラッシュに書くまでためておく RAM

static Adafruit_FlashTransport_QSPI g_qspiTr;
static Adafruit_SPIFlashBase        g_qspi(&g_qspiTr);
// XIAO Sense のフラッシュ（Puya P25Q16H）。ボード同梱の Adafruit SPIFlash 4.2 の一覧には入っていないので、ここで渡す
static const SPIFlash_Device_t LOG_FLASH_DEV = {
  .total_size = (1UL << 21), .start_up_time_us = 10000, .manufacturer_id = 0x85, .memory_type = 0x60, .capacity = 0x15,
  .max_clock_speed_mhz = 55, .quad_enable_bit_mask = 0x02, .has_sector_protection = false, .supports_fast_read = true,
  .supports_qspi = true, .supports_qspi_writes = true, .write_status_register_split = false, .single_status_byte = false, .is_fram = false,
};
static uint8_t           g_logJedec[3] = {0};  // 見つからなかったときに表示する ID
static bool              g_logOK = false;
static uint32_t          g_logSectors = 0, g_logSec = 0, g_logPos = LOG_SECTOR, g_logSeq = 0, g_bootNo = 0;
static uint8_t           g_logRam[LOG_RAM] __attribute__((aligned(4)));
static volatile uint32_t g_logHead = 0, g_logTail = 0, g_logDropped = 0;
static volatile bool     g_logDumpReq = false, g_logEraseReq = false, g_logDumping = false;
static uint32_t          g_logEraseMaxMs = 0;  // 4KB の消去にかかった最長時間（確認用）
static bool              g_logLineStart = true;
static SemaphoreHandle_t g_logMtx = NULL;

// RAM に積む（g_logMtx を持った状態で呼ぶ）
static void logPushRaw(const uint8_t* b, size_t n){
  if(g_logHead - g_logTail + n > LOG_RAM){ g_logDropped += n; return; }
  for(size_t i = 0; i < n; i++) g_logRam[(g_logHead + i) % LOG_RAM] = b[i];
  g_logHead += n;
}
// 記録だけに書く（行頭に [#起動番号 秒] を付ける）
static void logWrite(const uint8_t* b, size_t n){
  if(!g_logOK || !n) return;
  if(!g_logMtx) g_logMtx = xSemaphoreCreateMutex();
  xSemaphoreTake(g_logMtx, portMAX_DELAY);
  for(size_t i = 0; i < n; i++){
    if(g_logLineStart){
      char p[64]; uint32_t ms = millis();
      int k;
      if(g_logDropped){
        k = snprintf(p, sizeof(p), "[#%lu %lu.%03lu] [LOG] ★ %lu バイト記録できなかった\n", (unsigned long)g_bootNo,
                     (unsigned long)(ms / 1000), (unsigned long)(ms % 1000), (unsigned long)g_logDropped);
        uint32_t before = g_logDropped; logPushRaw((const uint8_t*)p, k); if(g_logDropped == before) g_logDropped = 0;
      }
      k = snprintf(p, sizeof(p), "[#%lu %lu.%03lu] ", (unsigned long)g_bootNo, (unsigned long)(ms / 1000), (unsigned long)(ms % 1000));
      logPushRaw((const uint8_t*)p, k);
      g_logLineStart = false;
    }
    if(b[i] == '\r') continue;
    logPushRaw(&b[i], 1);
    if(b[i] == '\n') g_logLineStart = true;
  }
  xSemaphoreGive(g_logMtx);
}
// 記録だけに書く printf（シリアルには出さない）
static void logf(const char* fmt, ...){
  char buf[200];
  va_list ap; va_start(ap, fmt); int n = vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  if(n < 0) return;
  if(n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
  logWrite((const uint8_t*)buf, n);
}

// Serial の代わり：USB シリアルに出し、同じものを記録にも書く
class LogSerial : public Stream {
public:
  bool echo = true;                    // false のあいだは記録だけ（30 秒ごとの状態など）
  using Print::write;
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* b, size_t n) override {
    logWrite(b, n);
    if(echo && !g_logDumping) USBSerial.write(b, n);
    return n;
  }
  int  available() override { return USBSerial.available(); }
  int  read() override      { return USBSerial.read(); }
  int  peek() override      { return USBSerial.peek(); }
  void flush() override     { USBSerial.flush(); }
  void begin(uint32_t baud) { USBSerial.begin(baud); }
  operator bool()           { return (bool)USBSerial; }
};
static LogSerial LogOut;
#define Serial LogOut                  // ★ これより下の Serial.print などはすべて記録にも残る

static bool logFlashWait(uint32_t maxMs){   // フラッシュの消去・書き込みが終わるまで、loop に譲りながら待つ
  uint32_t t0 = millis();
  while(!g_qspi.isReady()){ if(millis() - t0 > maxMs) return false; vTaskDelay(1); }
  return true;
}
static void logOpenNextSector(){
  g_logSec = (g_logSec + 1) % g_logSectors;
  uint32_t t0 = millis();
  logFlashWait(1000);
  g_qspi.eraseSector(g_logSec);
  logFlashWait(1000);
  uint32_t dt = millis() - t0; if(dt > g_logEraseMaxMs) g_logEraseMaxMs = dt;
  uint32_t hdr[4] = { LOG_MAGIC, ++g_logSeq, g_bootNo, 0xFFFFFFFFu };
  g_qspi.writeBuffer(g_logSec * LOG_SECTOR, (const uint8_t*)hdr, sizeof(hdr));
  g_logPos = LOG_HDR;
}
// RAM にたまった分をフラッシュへ（1 回 256 バイトまで）。書いたら true
// フラッシュ（QSPI）は 4 バイト単位・4 バイト境界でしか書けないので、4 の倍数だけ書く。
// 端数が 0.3 秒残ったら NUL（0x00）で 4 バイトにそろえて書く（取り出すときに NUL は捨てる）
static bool logFlushChunk(){
  static uint32_t oddSince = 0;
  uint32_t avail = g_logHead - g_logTail;
  if(avail && avail < 4){
    if(!oddSince) oddSince = millis() | 1;
    if(millis() - oddSince >= 300){
      static const uint8_t z[3] = {0, 0, 0};
      xSemaphoreTake(g_logMtx, portMAX_DELAY);
      uint32_t a = g_logHead - g_logTail;
      if(a & 3) logPushRaw(z, 4 - (a & 3));
      xSemaphoreGive(g_logMtx);
      avail = g_logHead - g_logTail;
    }
  }
  avail &= ~3u;
  if(!avail) return false;
  oddSince = 0;
  if(g_logPos >= LOG_SECTOR) logOpenNextSector();
  uint32_t off = g_logTail % LOG_RAM;
  uint32_t n = avail;
  if(n > LOG_RAM - off) n = LOG_RAM - off;
  if(n > LOG_SECTOR - g_logPos) n = LOG_SECTOR - g_logPos;
  if(n > 256) n = 256;
  n &= ~3u;                            // ここまでの値はどれも 4 の倍数だが念のため
  if(!n) return false;
  logFlashWait(100);
  g_qspi.writeBuffer(g_logSec * LOG_SECTOR + g_logPos, &g_logRam[off], n);
  g_logPos += n;
  g_logTail += n;
  return true;
}
// 古い順にすべて USB へ出す（ヘッダ・未使用部分は出さない）
static void logDump(){
  while(logFlushChunk()) {}            // 端数（4 バイト未満）は次の書き込みで出る
  logFlashWait(1000);
  g_logDumping = true;                 // この間、ほかの表示は USB に出さない（記録には残る）
  USBSerial.print("\n=====NOZUI-LOG-BEGIN=====\n");
  static uint8_t buf[256];
  uint32_t total = 0;
  for(uint32_t i = 1; i <= g_logSectors; i++){
    uint32_t s = (g_logSec + i) % g_logSectors;    // 今のセクタの次（いちばん古い）から順に
    uint32_t hdr[4];
    g_qspi.readBuffer(s * LOG_SECTOR, (uint8_t*)hdr, sizeof(hdr));
    if(hdr[0] != LOG_MAGIC) continue;
    for(uint32_t a = LOG_HDR; a < LOG_SECTOR; a += sizeof(buf)){
      uint32_t len = min((uint32_t)sizeof(buf), LOG_SECTOR - a);
      g_qspi.readBuffer(s * LOG_SECTOR + a, buf, len);
      uint32_t used = 0; while(used < len && buf[used] != 0xFF) used++;
      uint32_t k = 0;
      for(uint32_t j = 0; j < used; j++) if(buf[j]) buf[k++] = buf[j];   // 4 バイトにそろえた NUL を捨てる
      if(k) { USBSerial.write(buf, k); total += k; }
      if(used < len) break;
      if((a & 1023) == 0) vTaskDelay(1);
    }
  }
  USBSerial.printf("\n=====NOZUI-LOG-END===== %lu bytes\n", (unsigned long)total);
  USBSerial.flush();
  g_logDumping = false;
}
// 記録をすべて消す（使っていたセクタだけ消す）
static void logEraseAll(){
  while(logFlushChunk()) {}
  g_logTail = g_logHead & ~3u;         // 端数は捨てる（消したあとの書き始めをそろえるため）
  uint32_t n = 0;
  for(uint32_t s = 0; s < g_logSectors; s++){
    uint32_t m; g_qspi.readBuffer(s * LOG_SECTOR, (uint8_t*)&m, 4);
    if(m == 0xFFFFFFFFu) continue;
    logFlashWait(1000); g_qspi.eraseSector(s); logFlashWait(1000); n++;
  }
  g_logSec = g_logSectors - 1; g_logPos = LOG_SECTOR; g_logSeq = 0;
  USBSerial.printf("[LOG] 記録を消した（%lu セクタ）\n", (unsigned long)n);
}
static void logTask(void*){
  for(;;){
    if(g_logDumpReq){ logDump(); g_logDumpReq = false; }
    if(g_logEraseReq){ logEraseAll(); g_logEraseReq = false; }
    if(logFlushChunk()) taskYIELD();   // loop と同じ優先度なので、書くたびに譲る
    else vTaskDelay(20);
  }
}
// setup の最初で呼ぶ：フラッシュを調べて、今回の起動番号と書き始める位置を決める
static void logBegin(){
  if(!g_qspi.begin(&LOG_FLASH_DEV, 1) || g_qspi.size() < 64 * LOG_SECTOR){
    uint8_t id[4] = {0};
    g_qspiTr.begin(); g_qspiTr.readCommand(0x9F, id, 4);   // JEDEC ID
    memcpy(g_logJedec, id, 3);
    return;
  }
  g_logSectors = g_qspi.size() / LOG_SECTOR;
  uint32_t maxSeq = 0, maxBoot = 0; int32_t head = -1;
  for(uint32_t s = 0; s < g_logSectors; s++){
    uint32_t hdr[3];
    g_qspi.readBuffer(s * LOG_SECTOR, (uint8_t*)hdr, sizeof(hdr));
    if(hdr[0] != LOG_MAGIC) continue;
    if(head < 0 || hdr[1] > maxSeq){ maxSeq = hdr[1]; head = s; }
    if(hdr[2] > maxBoot) maxBoot = hdr[2];
  }
  g_logSeq = maxSeq;
  g_bootNo = maxBoot + 1;
  g_logSec = (head < 0) ? g_logSectors - 1 : (uint32_t)head;   // 次に開くのはその次のセクタ
  g_logPos = LOG_SECTOR;
  g_logMtx = xSemaphoreCreateMutex();
  static const uint8_t nl = '\n';      // 前回の起動の最後の行が途中で切れていても、今回の 1 行目とつながらないように
  logPushRaw(&nl, 1);
  g_logOK = true;
  TaskHandle_t h;
  xTaskCreate(logTask, "log", 1024, NULL, TASK_PRIO_LOW, &h);
}
static const char* resetReasonText(uint32_t r){
  if(r & (1u << 0))  return "リセットボタン";
  if(r & (1u << 1))  return "ウォッチドッグ";
  if(r & (1u << 2))  return "ソフトウェア（書き込み・再起動）";
  if(r & (1u << 3))  return "CPU ロックアップ（異常停止）";
  if(r & (1u << 16)) return "System OFF からの復帰";
  if(r & (1u << 20)) return "USB 接続で復帰";
  return "電源投入（電源が入った・電圧が下がって落ちた、のどちらか）";
}

// ---------------------------------------------------------------- 設定
static const char* BLE_NAME = "NozuiHeadgear-X1";   // 脳デバイスはこの名前でスキャンする

#define SERVO_CH          6
#define VALUE_MAX         100      // 通信値・収録データ・内部状態の最大値（0〜100）
#define SERVO_ANGLE_MAX   60       // ★ 可動域：値 0〜100 → 0〜60° にマップ（サーボ書き込み直前で変換）
#define PCA9685_ADDR      0x40
#define I2C_CLOCK_HZ      400000   // 不安定なら 100000 に落とす（§8-7）

#define PULSE_MIN         102      // 500µs  (0°)   @50Hz / 4096
#define PULSE_MAX         491      // 2400µs (180°)

#define SLOT_MS           (50 / SERVO_CH)  // 1ch あたりの更新間隔 ≒ 8ms → 6ch で約 50ms
#define RESET_STEP        3        // SV_RESET で 1 スロット（8ms）あたり 0 へ近づける量 [値 0〜100]（3 ≒ 1.8°）
#define RESET_HOLD_MS     300      // 全 ch 0° を書き終えてから PWM を切るまでの待ち（サーボが物理的に戻る時間）
#define PHASE2_FRAME_MS   50       // Phase2 シーケンス 1 フレーム = 50ms（20Hz）
#define ACTIVE_TIMEOUT_MS 1000     // SV_ACTIVE 中に 0x02 がこの時間来なければ SV_RESET（無通信タイムアウト）

// 取り付け向きの都合で回転方向を逆にしたい ch は true にする（角度 a → 180-a で出力）
static const bool SERVO_REVERSE[SERVO_CH] = { false, false, true, true, true, false };

// サーボ番号 → PCA9685 の出力 ch（配線スペースの都合で ch0〜3, ch12〜13 を使用）
//                                         左前 左中 左後 右前 右中 右後
static const uint8_t SERVO_PIN[SERVO_CH] = {  0,   1,   2,   3,  12,  13 };

// ---------------------------------------------------------------- サーボ #6 頭部衝撃 / #7 ミラー（SIGGRAPH 版。旧 headset_esp32_v3 と同じ値）
// 6 本とは計算が違う：0〜180° = 500〜2500µs、逆回転は「可動域 − 角度」（旧 ESP32 版の effectiveAngle() と同じ）
#define EX_PULSE_MIN_US   500
#define EX_PULSE_MAX_US   2500
#define IMPACT_PIN        14       // PCA9685 の出力 ch（頭部衝撃）
#define MIRROR_PIN        15       // PCA9685 の出力 ch（ミラー）
#define IMPACT_MAX_DEG    35       // 可動域
#define MIRROR_MAX_DEG    170
static const bool IMPACT_REVERSE = true;
static const bool MIRROR_REVERSE = true;
#define IMPACT_HIT_DEG    30       // Phase6 に入った瞬間に動かす角度
#define IMPACT_HOLD_MS    500      // この時間たったら 0° に戻す
#define MIRROR_OPEN_DEG   170      // Phase6 に入った瞬間の目標
#define MIRROR_MOVE_MS    2000     // 0° ⇄ 170° にかける時間（直線）
#define EX_UPDATE_MS      20       // 2 本の更新間隔
#define EX_OFF_DELAY_MS   300      // 0° に戻してから PWM を切るまで

// ---------------------------------------------------------------- 心拍センサ（Grove 耳クリップ式、旧 headset_esp32_v3 と同じ判定）
#define PIN_HEART             D0
#define HEART_MIN_INTERVAL_MS 350  // 前の拍からこれ以内は必ず無視（171 BPM 相当）
#define HEART_REL_MIN         0.6f // さらに、最近の間隔の 60% より早い拍は二重検出として無視
#define HEART_REL_CAP_MS      450  // ただし上の「60%」はこれより長くしない（2026-11-11：平均が 1.2 秒にずれたまま戻らず、
                                   // 100 拍/分の心拍を 1 拍おきに捨てていた。上限があれば 133 拍/分までの拍は必ず通る）
// 2026-11-10 の実測：中央値 0.68 秒の心拍に、0.25〜0.45 秒の二重検出が 81 回中 25 回混じっていた
#define HEART_IGNORE_COUNT    10   // 最初のこの拍数は捨てる（Phase1 に入るたび・起動時に数え直す）

// ---------------------------------------------------------------- 椅子（M5Atom）との UART
#define CHAIR_BAUD          9600   // 椅子と合わせる（ノイズに強くするため 115200 から下げた。4 バイトで約 4ms）
#define CHAIR_PING_MS       200    // 生きている合図の間隔
#define CHAIR_ONLINE_MS     3000   // 椅子からこの時間届かなければ「つながっていない」（表示・同期用。椅子の見切りと合わせる）
#define BRAIN_TIMEOUT_MS    3000   // 脳から 0x03 がこの時間来なければ「切れた」（記録するだけ。何も動かさない）
enum : uint8_t { CH_HEAD = 0xA5, CH_DOWN = 0x01, CH_UP = 0x02, CH_STOP = 0x03, CH_PING = 0x0F, CH_STATUS = 0x81 };

// ---------------------------------------------------------------- GATT（カスタム 128-bit UUID、バイト列はリトルエンディアン）
// Service : AA000000-0000-0000-0000-000000000001
// CMD     : AA000000-0000-0000-0000-000000000002
// HEART   : AA000000-0000-0000-0000-000000000003（SIGGRAPH 版。Notify、[0x10][通し番号]）
static const uint8_t UUID_SVC[16] = { 0x01,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0xAA };
static const uint8_t UUID_CMD[16] = { 0x02,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0xAA };
static const uint8_t UUID_HEART[16] = { 0x03,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0xAA };

BLEService        hgSvc(UUID_SVC);
BLECharacteristic hgCmd(UUID_CMD);
BLECharacteristic hgHeart(UUID_HEART);

#define CMD_LEN 7
enum : uint8_t { CMD_RESET = 0x00, CMD_PHASE2 = 0x01, CMD_SERVO = 0x02, CMD_PHASE = 0x03 };
#define HEART_MSG 0x10

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

// ---------------------------------------------------------------- Phase2 シーケンスデータ（§4-5）
// 出典: trail7.clip (TouchDesigner Trail CHOP, 60fps, 900フレーム)
// 変換: 60fps→20Hz(1/3間引き)、値域は元データのまま 0〜100（角度への変換は servoWrite() で行う）
// 6ch × 300サンプル = 15秒分 @20Hz
// chの対応: [chan1, chan2, chan3, chan4, chan5, chan6] = [左前, 左中, 左後, 右前, 右中, 右後]
// nRF52 では const 配列は自動的に Flash に置かれるので PROGMEM / pgm_read_byte は使わず直接参照する。
#define PHASE2_DATA_CH      6
#define PHASE2_DATA_FRAMES  300

static const uint8_t PHASE2_DATA[PHASE2_DATA_FRAMES][PHASE2_DATA_CH] = {
  // --- frame   0  t=0.0s ---
  {  0,  0,  0,  0,  0,  0}, {  0,  0,  0,  0,  0,  0}, {  0,  0,  0,  0,  0,  0},
  {  0,  0,  0,  0,  0,  0}, {  0,  0,  0,  0,  0,  0}, {  0,  0,  0,  0,  0,  0},
  {  0,  0,  0,  0,  0,  0}, {  0,  0,  0,  0,  0,  0}, {  0,  0,  0,  0,  0,  0},
  {  0,  0,  0,  0,  0,  0}, {  0,  1,  0,  0,  0,  0}, {  0,  2,  0,  0,  0,  0},
  {  0,  2,  0,  0,  0,  0}, {  2,  7,  2,  0,  0,  0}, {  3,  8,  3,  2,  7,  2},
  {  6, 20,  9,  6, 19,  6}, {  8, 28, 20,  8, 24,  9}, {  8, 34, 33,  9, 30, 18},
  {  8, 35, 34, 10, 32, 20}, {  8, 36, 36, 10, 35, 24},
  // --- frame  20  t=1.0s ---
  {  7, 36, 42, 10, 35, 26}, {  6, 34, 43,  9, 32, 26}, {  6, 33, 43,  9, 32, 26},
  {  6, 34, 45, 10, 35, 30}, {  7, 38, 45, 11, 38, 31}, { 11, 45, 45, 12, 44, 35},
  { 11, 44, 41, 12, 46, 37}, { 11, 45, 40, 13, 48, 37}, { 11, 44, 38, 13, 47, 37},
  { 11, 44, 38, 13, 49, 39}, { 11, 44, 41, 13, 49, 40}, { 11, 45, 46, 12, 44, 38},
  {  9, 41, 45, 10, 37, 32}, {  4, 20, 27,  2,  8,  5}, {  2, 11, 16,  0,  0,  1},
  {  1,  4,  1,  0,  0,  0}, {  1,  4,  1,  0,  0,  0}, {  1,  4,  1,  0,  0,  0},
  {  1,  4,  1,  0,  0,  0}, {  1,  4,  1,  0,  0,  0},
  // --- frame  40  t=2.0s ---
  {  2,  6,  2,  0,  0,  0}, {  6, 17,  6,  5, 17,  5}, {  8, 24,  8,  8, 25,  8},
  {  9, 28,  9, 11, 33, 11}, { 10, 31, 11, 11, 37, 21}, { 10, 41, 36,  9, 35, 27},
  { 10, 41, 42,  8, 31, 28}, { 10, 41, 44,  5, 24, 25}, {  7, 34, 44,  0,  6, 14},
  {  4, 22, 33,  0,  1,  3},
  // --- frame  50  t=2.5s ---
  {  3, 17, 23,  0,  1,  2}, {  4, 14, 13,  0,  1,  1}, {  4, 12,  4,  0,  1,  0},
  {  6, 16,  6,  0,  2,  0}, {  6, 18,  6,  0,  2,  0}, {  9, 25,  9,  1,  7,  1},
  { 10, 29, 10,  5, 17,  5}, { 10, 31, 10,  7, 22,  7}, { 10, 31, 10,  9, 28, 10},
  { 10, 41, 37,  9, 30, 18}, {  8, 37, 41,  8, 27, 19}, {  4, 28, 43,  7, 23, 18},
  {  4, 28, 44,  7, 23, 18}, {  5, 32, 44,  7, 23, 18}, {  9, 41, 46,  9, 31, 20},
  { 10, 44, 46,  9, 32, 21}, { 10, 41, 40,  8, 29, 20}, {  8, 34, 33,  6, 22, 15},
  {  8, 27, 19,  3, 11,  5}, {  9, 27, 12,  0,  3,  0},
  // --- frame  70  t=3.5s ---
  { 10, 27, 10,  0,  3,  0}, { 10, 27, 10,  0,  3,  0}, { 10, 28, 10,  0,  3,  0},
  { 10, 33, 19,  4, 16,  5}, { 10, 33, 19,  4, 16,  5}, { 11, 35, 26, 18, 26,  9},
  {  9, 34, 30, 13, 25, 13}, {  6, 30, 33,  6, 23, 17}, {  9, 36, 35,  6, 24, 18},
  {  9, 38, 38,  7, 28, 26}, { 10, 38, 37, 11, 29, 23}, {  9, 39, 38,  7, 28, 23},
  {  9, 38, 38,  7, 26, 21}, {  9, 40, 42,  7, 25, 18}, { 10, 42, 43,  9, 29, 13},
  {  9, 38, 35,  9, 28, 12}, { 10, 39, 34, 10, 30, 13}, { 10, 40, 36, 10, 31, 13},
  { 10, 43, 43, 11, 37, 25}, {  9, 42, 44, 11, 40, 32},
  // --- frame  90  t=4.5s ---
  { 10, 43, 46, 15, 41, 34}, {  9, 41, 47,  7, 30, 33}, {  9, 41, 47,  6, 28, 30},
  {  9, 41, 45,  6, 23, 20}, { 10, 42, 45,  4, 17, 13}, {  9, 40, 47,  3, 11,  7},
  { 20, 42, 48, 83, 43,  8}, { 20, 44, 51, 83, 43,  8}, { 21, 46, 50, 87, 54, 12},
  { 22, 52, 55, 92, 67, 20},
  // --- frame 100  t=5.0s ---
  { 22, 53, 58, 93, 76, 33}, { 17, 52, 60, 60, 68, 45}, { 14, 50, 59, 39, 59, 45},
  { 11, 45, 53, 22, 45, 40}, { 10, 40, 44, 14, 33, 33}, {  8, 30, 27,  1, 13, 18},
  {  8, 29, 26,  1,  7,  7}, {  9, 26, 12,  0,  3,  0}, { 10, 27, 10,  0,  3,  0},
  {  9, 24,  9,  0,  3,  0}, {  8, 23,  8,  0,  2,  0}, {  7, 20,  7,  0,  2,  0},
  {  8, 22,  9,  0,  2,  0}, {  7, 23, 10,  7, 22,  7}, {  6, 23, 17,  9, 28, 11},
  {  1, 17, 29,  8, 24, 15}, {  1, 18, 33,  8, 24, 16}, {  2, 21, 34,  8, 25, 15},
  {  4, 26, 34,  8, 26, 15}, {  7, 35, 39,  8, 27, 16},
  // --- frame 120  t=6.0s ---
  {  8, 38, 41,  8, 26, 16}, {  8, 38, 42,  8, 27, 16}, {  7, 36, 45,  7, 23, 16},
  {  6, 34, 46,  7, 22, 15}, { 19, 44, 50, 90, 63, 19}, { 19, 46, 52, 92, 68, 21},
  { 20, 49, 55, 93, 76, 31}, { 21, 51, 57, 94, 80, 39}, { 21, 50, 56, 95, 81, 41},
  { 13, 49, 54, 29, 55, 41}, { 12, 50, 55, 18, 49, 42}, { 11, 49, 53, 12, 46, 42},
  { 11, 50, 56, 12, 46, 42}, { 11, 52, 58, 13, 48, 44}, { 22, 53, 60, 93, 80, 45},
  { 22, 53, 60, 94, 81, 44}, { 22, 54, 60, 94, 82, 44}, { 22, 52, 57, 92, 73, 35},
  { 21, 49, 51, 91, 66, 23}, { 17, 38, 36, 72, 47, 10},
  // --- frame 140  t=7.0s ---
  {  9, 21, 16, 32, 27,  7}, {  2, 12, 13,  6, 15,  6}, {  0, 13, 24,  6, 17,  9},
  {  2, 14, 19,  5, 15,  7}, {  1, 10, 19,  1,  5,  4}, {  1, 12, 26,  0,  0,  3},
  {  1,  5,  6,  0,  0,  0}, {  1,  9, 13,  0,  0,  1}, {  6, 16,  6,  0,  2,  0},
  {  6, 18,  6,  0,  2,  0},
  // --- frame 150  t=7.5s ---
  {  8, 23, 12,  0,  2,  0}, {  6, 29, 35,  6, 17,  9}, {  1, 17, 32,  6, 18, 10},
  {  1, 18, 34,  7, 21, 11}, {  1, 20, 37,  8, 23, 13}, {  1, 18, 35,  8, 22, 12},
  {  2, 18, 27,  6, 19, 10}, {  5, 25, 29,  7, 20, 10}, { 18, 31, 23, 86, 51,  9},
  { 20, 35, 24, 88, 55, 10}, { 20, 40, 30, 92, 68, 15}, { 21, 44, 37, 95, 75, 21},
  { 14, 45, 38, 41, 57, 27}, { 14, 45, 36, 33, 54, 27}, { 11, 48, 44, 14, 45, 27},
  { 11, 48, 45, 14, 44, 23}, { 10, 46, 45, 14, 43, 23}, { 10, 45, 45, 13, 39, 18},
  {  9, 38, 41,  3, 13,  7}, {  9, 40, 46,  3, 13,  8},
  // --- frame 170  t=8.5s ---
  { 20, 44, 48, 86, 50, 11}, { 20, 44, 48, 87, 52, 12}, { 17, 49, 52, 67, 61, 24},
  { 15, 49, 52, 49, 56, 28}, { 11, 47, 48, 19, 50, 36}, { 11, 46, 46, 13, 47, 35},
  { 10, 43, 41, 13, 46, 35}, { 10, 43, 41, 13, 45, 35}, { 10, 41, 41,  9, 34, 27},
  {  9, 39, 41,  8, 30, 25}, {  9, 36, 37,  0,  5,  4}, {  9, 33, 32,  0,  3,  2},
  {  8, 23, 12,  0,  2,  0}, {  7, 19,  7,  0,  2,  0}, {  8, 24,  8,  5, 16,  5},
  {  8, 28, 21,  6, 18,  7}, {  9, 38, 37,  9, 30, 20}, {  9, 38, 38,  8, 29, 23},
  {  6, 33, 44,  6, 24, 26}, {  6, 32, 45,  5, 22, 26},
  // --- frame 190  t=9.5s ---
  {  6, 30, 40,  2,  9, 10}, {  7, 31, 38,  2,  9,  6}, { 19, 35, 33, 77, 37,  5},
  { 20, 36, 33, 80, 38,  5}, { 19, 36, 31, 81, 42,  6}, { 18, 39, 35, 68, 45, 10},
  { 12, 36, 32, 26, 32, 17}, {  9, 28, 24, 15, 24, 14}, {  6, 19, 15,  0,  2,  2},
  {  6, 16,  8,  0,  2,  2},
  // --- frame 200  t=10.0s ---
  {  6, 20, 13,  0,  2,  1}, {  6, 23, 21,  0,  2,  1}, {  9, 34, 33,  0,  3,  3},
  {  9, 34, 33,  0,  3,  3}, {  9, 36, 33,  4, 15,  7}, { 10, 39, 36,  6, 19,  9},
  { 10, 44, 46,  8, 33, 31}, { 10, 44, 47,  8, 34, 32}, { 10, 44, 50,  7, 32, 33},
  { 10, 43, 50,  6, 26, 29}, { 10, 43, 48,  6, 25, 20}, { 10, 43, 47,  6, 21, 11},
  { 10, 42, 44,  6, 20, 10}, { 10, 42, 44,  6, 20, 10}, { 10, 47, 52, 10, 34, 20},
  { 11, 49, 53, 11, 36, 23}, { 11, 49, 55, 11, 42, 36}, { 10, 47, 53, 11, 40, 34},
  { 10, 41, 41,  6, 24, 21}, { 10, 38, 34,  5, 21, 15},
  // --- frame 220  t=11.0s ---
  {  9, 27, 13,  2,  9,  3}, {  9, 24,  9,  0,  3,  0}, { 10, 28, 11,  1,  8,  2},
  { 10, 30, 12,  3, 13,  3}, { 11, 46, 49,  6, 24, 20}, { 11, 48, 52,  9, 33, 25},
  { 12, 53, 58, 13, 48, 38}, { 12, 54, 59, 14, 48, 38}, { 12, 52, 57, 14, 49, 40},
  { 11, 49, 52, 13, 47, 40}, { 11, 46, 47, 11, 41, 37}, { 11, 44, 45,  8, 34, 33},
  { 11, 42, 37,  8, 30, 24}, { 11, 36, 22,  5, 19,  9}, { 11, 30, 11,  0,  3,  0},
  { 11, 30, 12,  0,  3,  0}, { 11, 31, 14,  0,  3,  0}, { 11, 32, 17,  0,  3,  0},
  {  6, 26, 29,  0,  2,  2}, {  3, 20, 31,  0,  1,  3},
  // --- frame 240  t=12.0s ---
  {  1, 14, 25,  2,  7,  5}, {  1, 14, 25,  3,  8,  6}, {  3, 19, 34, 18, 21, 10},
  {  3, 20, 33, 12, 18,  9}, {  8, 35, 35,  8, 25, 11}, {  9, 36, 31, 10, 30, 13},
  { 15, 43, 39, 54, 57, 20}, { 14, 44, 39, 44, 55, 21}, { 12, 44, 37, 23, 50, 24},
  { 11, 44, 37, 15, 47, 25},
  // --- frame 250  t=12.5s ---
  { 21, 45, 40, 95, 79, 31}, { 17, 47, 44, 66, 67, 32}, { 13, 49, 51, 28, 53, 34},
  { 11, 49, 53, 14, 48, 35}, {  9, 46, 52, 12, 40, 28}, {  9, 45, 52, 11, 36, 23},
  { 13, 43, 48, 42, 41, 14}, { 12, 42, 48, 31, 36, 14}, { 10, 44, 48, 11, 30, 14},
  { 10, 45, 48, 11, 34, 17}, { 20, 46, 47, 91, 67, 23}, { 16, 45, 47, 58, 56, 27},
  { 10, 39, 37, 14, 32, 20}, {  9, 33, 25,  8, 29, 18}, {  6, 19, 10,  3, 12,  8},
  {  5, 15,  5,  1,  7,  2}, {  5, 17, 11,  0,  3,  2}, {  6, 22, 20,  2,  7,  4},
  {  8, 30, 27,  5, 16,  9}, {  8, 33, 32,  6, 20, 12},
  // --- frame 270  t=13.5s ---
  {  9, 40, 38, 10, 34, 22}, { 10, 43, 42, 11, 38, 24}, { 10, 45, 46, 12, 42, 30},
  { 10, 46, 47, 12, 43, 31}, { 10, 47, 51, 12, 41, 33}, { 10, 46, 50, 11, 40, 32},
  {  8, 39, 48,  9, 32, 27}, {  5, 31, 46,  8, 27, 24}, {  4, 26, 39,  5, 15,  9},
  {  5, 26, 34,  2,  8,  6}, {  6, 23, 25,  1,  6,  4}, {  6, 24, 25,  2,  7,  4},
  {  6, 25, 25,  2,  9,  5}, {  6, 27, 26,  4, 13,  6}, {  9, 36, 34,  6, 19,  9},
  {  9, 38, 38,  6, 21, 11}, {  9, 42, 43, 10, 34, 21}, { 20, 46, 46, 92, 70, 24},
  { 21, 46, 45, 93, 72, 25}, { 15, 47, 48, 49, 59, 31},
  // --- frame 290  t=14.5s ---
  { 12, 48, 52, 23, 49, 34}, { 10, 48, 54, 13, 43, 35}, {  8, 40, 50,  9, 33, 30},
  {  7, 35, 46,  6, 23, 18}, {  7, 30, 34,  2,  9,  6}, {  8, 34, 34,  4, 14,  7},
  {  8, 29, 22,  4, 15,  6}, {  7, 26, 19,  4, 15,  6}, {  6, 24, 18,  5, 16,  7},
  {  6, 22, 17,  3, 10,  5}
};

// ---------------------------------------------------------------- 受信キュー（BLE コールバック → loop）
// 単一生産者（BLE タスク）／単一消費者（loop）のリングバッファ。インデックスは 1 バイトでアトミック。
struct CmdEvt { uint8_t d[CMD_LEN]; };
static const uint8_t QN = 16;
static CmdEvt           g_q[QN];
static volatile uint8_t g_qw = 0, g_qr = 0;
static volatile uint32_t g_rxCount = 0, g_rxDropped = 0, g_rxBadLen = 0;

static void pushCmd(const uint8_t* d){
  uint8_t n = (uint8_t)((g_qw + 1) % QN);
  if(n == g_qr){ g_rxDropped++; return; }
  memcpy(g_q[g_qw].d, d, CMD_LEN);
  g_qw = n;
  g_rxCount++;
}

// GATT Write コールバック（BLE タスク＝仕様書の「Core0」）。ここでは I2C に触らない。
static void onCmdWrite(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len){
  (void)conn_hdl; (void)chr;
  if(len < CMD_LEN){ g_rxBadLen++; return; }
  pushCmd(data);
}

// ---------------------------------------------------------------- 接続状態
static volatile bool g_conn = false;
static volatile bool g_disconnectEvt = false;
static volatile uint16_t g_connHdl = BLE_CONN_HANDLE_INVALID;

static void onConnect(uint16_t h){
  g_conn = true;
  g_connHdl = h;
  BLEConnection* c = Bluefruit.Connection(h);
  if(c) c->requestConnectionParameter(6);   // 7.5ms 希望（最終決定は Central 側）
  if(c){
    char peer[32] = "?"; c->getPeerName(peer, sizeof(peer));
    ble_gap_addr_t a = c->getPeerAddr();
    logf("[BLE] 接続：相手 %s（%02X:%02X:%02X:%02X:%02X:%02X）間隔 %.2f ms\n", peer,
         a.addr[5], a.addr[4], a.addr[3], a.addr[2], a.addr[1], a.addr[0], c->getConnectionInterval() * 1.25f);
  } else logf("[BLE] 接続\n");
}
// 切断理由（BLE の規格の番号）。0x08 = 電波が届かなくなった、0x13 = 相手が切った、0x16 = こちらが切った、0x3E = 接続できなかった
static const char* bleReasonText(uint8_t r){
  switch(r){
  case 0x08: return "電波が届かない（タイムアウト）";
  case 0x13: return "相手が切った";
  case 0x16: return "こちらが切った";
  case 0x22: case 0x28: return "応答なし（LMP/LL タイムアウト）";
  case 0x3B: return "接続パラメータを受け入れられない";
  case 0x3D: return "MIC 失敗";
  case 0x3E: return "接続を確立できなかった";
  default:   return "その他";
  }
}
static void onDisconnect(uint16_t h, uint8_t reason){
  (void)h;
  logf("[BLE] 切断：理由 0x%02X（%s）\n", reason, bleReasonText(reason));
  g_conn = false;
  g_connHdl = BLE_CONN_HANDLE_INVALID;
  g_disconnectEvt = true;                   // loop 側で SV_RESET にする
}

// ---------------------------------------------------------------- LED（LOW で点灯）  bit0=R bit1=G bit2=B
static uint8_t  g_flash = 0;
static uint32_t g_flashUntil = 0;
static void led(uint8_t rgb){
  digitalWrite(LED_RED,   (rgb & 1) ? LOW : HIGH);
  digitalWrite(LED_GREEN, (rgb & 2) ? LOW : HIGH);
  digitalWrite(LED_BLUE,  (rgb & 4) ? LOW : HIGH);
}
static void flash(uint8_t rgb, uint32_t ms){ g_flash = rgb; g_flashUntil = millis() + ms; }

// ---------------------------------------------------------------- サーボ低レベル
// 値 0〜100 → 角度 0〜SERVO_ANGLE_MAX° → PCA9685 のカウント（角度を整数に丸めず直接カウントへ）
static uint16_t valueToCount(uint8_t val){
  if(val > VALUE_MAX) val = VALUE_MAX;
  return (uint16_t)(PULSE_MIN + (uint32_t)val * SERVO_ANGLE_MAX * (PULSE_MAX - PULSE_MIN) / ((uint32_t)VALUE_MAX * 180));
}
static void servoWrite(uint8_t ch, uint8_t val){
  uint16_t cnt = valueToCount(val);
  if(SERVO_REVERSE[ch]) cnt = (uint16_t)(PULSE_MAX + PULSE_MIN - cnt);
  pwm.setPWM(SERVO_PIN[ch], 0, cnt);
}
static void servoOff(uint8_t ch){ pwm.setPWM(SERVO_PIN[ch], 0, 4096); }   // 常時 LOW → 脱力・バズ消滅
static void allServosOff(){ for(uint8_t i = 0; i < SERVO_CH; i++) servoOff(i); }

// ================================================================ SIGGRAPH 版：頭部衝撃・ミラー（loop だけが触る）
static uint16_t exCount(int maxDeg, bool reverse, float angle){
  float a = constrain(angle, 0.0f, (float)maxDeg);
  if(reverse) a = maxDeg - a;
  float us = EX_PULSE_MIN_US + (EX_PULSE_MAX_US - EX_PULSE_MIN_US) * (a / 180.0f);
  return (uint16_t)(us * 4096.0f / 20000.0f + 0.5f);
}
static void impactWrite(float deg){ pwm.setPWM(IMPACT_PIN, 0, exCount(IMPACT_MAX_DEG, IMPACT_REVERSE, deg)); }
static void mirrorWrite(float deg){ pwm.setPWM(MIRROR_PIN, 0, exCount(MIRROR_MAX_DEG, MIRROR_REVERSE, deg)); }
static void impactOff(){ pwm.setPWM(IMPACT_PIN, 0, 4096); }
static void mirrorOff(){ pwm.setPWM(MIRROR_PIN, 0, 4096); }

// 頭部衝撃：0=待機（PWM 停止）／1=打っている／2=0° に戻して PWM を切るのを待っている
static uint8_t  g_impStage = 0;
static uint32_t g_impT = 0;
static void impactFire(){
  impactWrite(IMPACT_HIT_DEG);
  g_impStage = 1;
  g_impT = millis();
  Serial.printf("[IMPACT] %d°\n", IMPACT_HIT_DEG);
}

// ミラー：今の角度から目標まで MIRROR_MOVE_MS かけて直線で動かす。170° では保持、0° に戻ったら PWM を切る
static float    g_mirCur = 0, g_mirFrom = 0, g_mirTo = 0;
static bool     g_mirMoving = false;
static uint32_t g_mirT0 = 0, g_mirOffAt = 0;
static void mirrorMoveTo(float deg){
  if(!g_mirMoving && g_mirCur == deg) return;   // すでにそこにいる
  g_mirFrom = g_mirCur;
  g_mirTo = deg;
  g_mirT0 = millis();
  g_mirMoving = true;
  g_mirOffAt = 0;
  Serial.printf("[MIRROR] %.0f° → %.0f°\n", g_mirFrom, g_mirTo);
}

static void updateExtras(uint32_t now){
  static uint32_t lastT = 0;
  if(now - lastT < EX_UPDATE_MS) return;
  lastT = now;

  // 経過時間は符号付きで見る：g_impT・g_mirT0 はコマンド処理で millis() から入るので、loop 先頭の now より新しいことがある
  int32_t impEl = (int32_t)(now - g_impT);
  if(g_impStage == 1 && impEl >= IMPACT_HOLD_MS){ impactWrite(0); g_impStage = 2; g_impT = now; }
  else if(g_impStage == 2 && impEl >= EX_OFF_DELAY_MS){ impactOff(); g_impStage = 0; }

  if(g_mirMoving){
    int32_t el = (int32_t)(now - g_mirT0);
    if(el < 0) el = 0;
    if(el >= MIRROR_MOVE_MS){
      g_mirCur = g_mirTo;
      g_mirMoving = false;
      if(g_mirTo == 0) g_mirOffAt = now + EX_OFF_DELAY_MS;
    } else {
      g_mirCur = g_mirFrom + (g_mirTo - g_mirFrom) * ((float)el / MIRROR_MOVE_MS);
    }
    mirrorWrite(g_mirCur);
  } else if(g_mirOffAt && (int32_t)(now - g_mirOffAt) >= 0){
    mirrorOff();
    g_mirOffAt = 0;
  }
}

// ================================================================ SIGGRAPH 版：心拍センサ
static volatile uint32_t g_hrLastT   = 0;
static volatile uint16_t g_hrCount   = 0;   // Phase1・起動から拾った拍数（捨てた分を含む）
static volatile uint8_t  g_hrPending = 0;   // まだ脳へ送っていない拍
static uint8_t  g_hrSeq  = 0;               // 送った拍の通し番号
static uint32_t g_hrSent = 0;
static bool     g_printHeart = false;

static volatile uint32_t g_hrAvgMs  = 0;   // 最近の拍の間隔（受け付けた拍だけで平均。0 = まだ分からない）
static volatile uint32_t g_hrRejected = 0;  // 二重検出として捨てた数（表示用）
static volatile uint32_t g_hrRejDt = 0, g_hrRejMin = 0;   // 最後に捨てた拍の、前の拍からの時間・そのときのしきい値（記録用）
static void onHeartBeat(){                  // 割り込み（D0 の立ち上がり）
  uint32_t now = millis();
  uint32_t dt = now - g_hrLastT;
  uint32_t minDt = HEART_MIN_INTERVAL_MS;
  if(g_hrAvgMs) minDt = max(minDt, min((uint32_t)HEART_REL_CAP_MS, (uint32_t)(g_hrAvgMs * HEART_REL_MIN)));
  if(dt < minDt){ g_hrRejDt = dt; g_hrRejMin = minDt; g_hrRejected++; return; }
  // 間隔の平均（2 秒を超える間は外れ値として平均に入れない。指が外れていた等）
  if(g_hrLastT && dt <= 2000) g_hrAvgMs = g_hrAvgMs ? (g_hrAvgMs * 3 + dt) / 4 : dt;
  g_hrLastT = now;
  if(g_hrCount < 0xFFFF) g_hrCount++;
  if(g_hrCount > HEART_IGNORE_COUNT && g_hrPending < 255) g_hrPending++;
}
static void heartReset(){
  noInterrupts();
  g_hrCount = 0;
  g_hrPending = 0;
  interrupts();
  Serial.printf("[HEART] 数え直し（最初の %d 拍は捨てる）\n", HEART_IGNORE_COUNT);
}
// 拾った拍を脳へ Notify（loop から毎回）
static void pumpHeart(){
  static uint16_t lastCount = 0;
  uint16_t cnt = g_hrCount;
  if(cnt != lastCount && cnt <= HEART_IGNORE_COUNT){
    if(g_printHeart) Serial.printf("[HEART] 拍 %u（捨てる）\n", cnt);
    else             logf("[HEART] 拍 %u（捨てる）\n", cnt);
  }
  lastCount = cnt;
  static uint32_t lastRej = 0;
  uint32_t rej = g_hrRejected;
  if(rej != lastRej){
    lastRej = rej;
    logf("[HEART] 二重検出として捨てた：前の拍から %lu ms（しきい値 %lu ms、累計 %lu）\n",
         (unsigned long)g_hrRejDt, (unsigned long)g_hrRejMin, (unsigned long)rej);
  }

  uint8_t n = g_hrPending;
  if(!n) return;
  noInterrupts(); g_hrPending = 0; interrupts();
  if(n > 2) n = 2;                          // 溜まっていても古い拍は鳴らさない
  for(uint8_t i = 0; i < n; i++){
    uint8_t b[2] = { HEART_MSG, ++g_hrSeq };
    uint16_t h = g_connHdl;
    bool ok = g_conn && h != BLE_CONN_HANDLE_INVALID && hgHeart.notifyEnabled(h) && hgHeart.notify(h, b, sizeof(b));
    if(ok) g_hrSent++;
    if(g_printHeart) Serial.printf("[HEART] 拍 #%u %s\n", g_hrSeq, ok ? "→ 脳" : "（脳が未接続・未購読）");
    else             logf("[HEART] 拍 #%u 間隔平均 %lu ms %s\n", g_hrSeq, (unsigned long)g_hrAvgMs, ok ? "→ 脳" : "（脳が未接続・未購読）");
  }
}

// ================================================================ SIGGRAPH 版：椅子との UART
static uint32_t g_chairRxT = 0;      // 最後に椅子から電文が届いた時刻（0 = まだ）
static uint8_t  g_chairStatus = 0;
static bool     g_chairOnline = false;
static uint32_t g_chairRxBad = 0;
static int8_t   g_phase = -1;        // 0x03 で知った Phase（-1 = 不明：起動直後だけ。脳との通信が切れても覚えたまま）

static void chairSend(uint8_t type, uint8_t val){
  uint8_t f[4] = { CH_HEAD, type, val, (uint8_t)(type ^ val) };
  Serial1.write(f, sizeof(f));
}
// 倒す・起こす・止めるは、命令のときだけ CHAIR_CMD_REPEAT 回送る（電文が 1 つ壊れても届くように）。
// 椅子はすでにその向きなら何もしないので、何回届いても 1 回分しか動かない（2026-11-23）
#define CHAIR_CMD_REPEAT     3
#define CHAIR_CMD_REPEAT_MS  40
static uint8_t  g_chRepType = 0, g_chRepLeft = 0;
static uint32_t g_chRepT = 0;
static void chairCmd(uint8_t type){
  chairSend(type, 0);
  g_chRepType = type; g_chRepLeft = CHAIR_CMD_REPEAT - 1; g_chRepT = millis();   // 新しい命令が来たら前の繰り返しは打ち切る
  Serial.printf("[CHAIR] %s を送信%s\n", type == CH_DOWN ? "倒す" : type == CH_UP ? "起こす" : "止める",
                g_chairOnline ? "" : "（★ 椅子から応答がない）");
}
static const char* chairRelayName(uint8_t st){ static const char* N[] = { "停止", "倒し中", "起こし中", "?" }; return N[st & 3]; }
static const char* chairPhysName(uint8_t st){ static const char* N[] = { "不明", "水平", "傾斜", "危険" }; return N[(st >> 2) & 3]; }

static void pollChair(uint32_t now){
  static uint8_t buf[4];
  static uint8_t n = 0;
  while(Serial1.available()){
    uint8_t c = (uint8_t)Serial1.read();
    if(n == 0 && c != CH_HEAD) continue;     // 先頭を探す
    buf[n++] = c;
    if(n < 4) continue;
    n = 0;
    if((uint8_t)(buf[1] ^ buf[2]) != buf[3]){ g_chairRxBad++; continue; }
    if(buf[1] == CH_STATUS){
      if(buf[2] != g_chairStatus)
        Serial.printf("[CHAIR] %s／%s%s%s\n", chairRelayName(buf[2]), chairPhysName(buf[2]), (buf[2] & 0x10) ? "／★ MPU6050 エラー" : "",
                      (buf[2] & 0x20) ? "／★ 椅子は停止中（通信が確かめられなかった。椅子の電源を入れ直すまで倒れない）" : "");
      g_chairStatus = buf[2];
      g_chairRxT = now ? now : 1;
    }
  }
  bool online = g_chairRxT && (now - g_chairRxT < CHAIR_ONLINE_MS);
  if(online != g_chairOnline){
    g_chairOnline = online;
    if(online){
      Serial.println("[CHAIR] 椅子とつながった");
      if(g_phase != 6) chairCmd(CH_UP);      // 傾いたまま起動していた場合に揃える（水平なら椅子は何もしない）
    } else {
      Serial.println("[CHAIR] ★ 椅子から応答がない（線・電源を確認）");
    }
  }
  if(g_chRepLeft && (int32_t)(now - g_chRepT) >= CHAIR_CMD_REPEAT_MS){   // 命令の 2 回目・3 回目
    g_chRepT = now; g_chRepLeft--;
    chairSend(g_chRepType, 0);
  }
  static uint32_t lastPing = 0;
  if(now - lastPing >= CHAIR_PING_MS){
    lastPing = now;
    chairSend(CH_PING, g_phase < 0 ? 0xFF : (uint8_t)g_phase);
  }
}

// ================================================================ SIGGRAPH 版：Phase（0x03）
static uint32_t g_lastPhaseRxT = 0;

// Phase が変わった瞬間だけ動かす（同じ Phase の 0x03 が届いても何もしない）
static bool g_brainLost = false;     // 脳との通信が切れている（表示用）
static void onPhase(int8_t p, const char* src){
  g_lastPhaseRxT = millis();
  if(g_brainLost){ g_brainLost = false; Serial.printf("[LINK] 脳から 0x03 が届くようになった（Phase %d → %d）\n", g_phase, p); }
  if(p < 0 || p > 8 || p == g_phase) return;
  int8_t from = g_phase;
  g_phase = p;
  Serial.printf("[PHASE] %d → %d（%s）\n", from, p, src);
  if(p == 1) heartReset();

  if(from < 0){
    // 不明から：揃えるだけ。Phase6 でも倒さない（通信が戻っただけで倒し直さないため）
    if(p != 6){ mirrorMoveTo(0); if(g_chairOnline) chairCmd(CH_UP); }
    return;
  }
  if(p == 6){
    mirrorMoveTo(MIRROR_OPEN_DEG);
    impactFire();
    chairCmd(CH_DOWN);
  } else if(from == 6){
    mirrorMoveTo(0);
    chairCmd(CH_UP);
  }
}

// 脳との通信が切れた：記録するだけ。Phase は覚えたまま、ミラー・椅子は動かさない（2026-11-23）
static void brainLinkLost(const char* why){
  if(g_brainLost) return;
  g_brainLost = true;
  Serial.printf("[LINK] %s（Phase %d のまま。ミラー・椅子は動かさない）\n", why, g_phase);
}

// ---------------------------------------------------------------- サーボ状態マシン（loop ＝仕様書の「Core1」）
enum : uint8_t { SV_OFF = 0, SV_RESET, SV_SEQ, SV_ACTIVE };
static const char* SV_NAME[] = { "SV_OFF", "SV_RESET", "SV_SEQ", "SV_ACTIVE" };

static uint8_t  svState = SV_OFF;
static uint8_t  servoLocal[SERVO_CH]   = {0};  // 今出したい値（0〜100）
static uint8_t  servoCurrent[SERVO_CH] = {0};  // 最後に出した値（0〜100、リセット補間の起点）
static uint8_t  updateCh = 0;                  // 次のスロットで書く ch
static uint32_t slotT = 0;

static int      phase2Frame = 0;
static uint32_t phase2T     = 0;
static uint32_t lastServoCmdT = 0;             // SV_ACTIVE のタイムアウト判定用
static uint32_t resetZeroT  = 0;               // SV_RESET で全 ch 0° になった時刻（0＝まだ）

static void setState(uint8_t s){
  if(s == svState) return;
  Serial.printf("[SV] %s → %s\n", SV_NAME[svState], SV_NAME[s]);
  svState = s;
  if(s == SV_RESET) resetZeroT = 0;
}

static void enterReset(){
  if(svState == SV_OFF) return;   // すでに停止中なら何もしない（PWM 停止済み）
  setState(SV_RESET);
}

static void startPhase2(){
  phase2Frame = 0;
  phase2T     = millis() - PHASE2_FRAME_MS;   // 次の updatePhase2Sequence() で即フレーム 0 を適用
  setState(SV_SEQ);
}

// 記録用：1 秒ごとに数えたコマンド（0x00・0x01・0x02・0x03・その他）と、0x02 の間隔の最大
static uint32_t g_secCmd[5] = {0};
static uint32_t g_sec02Gap = 0, g_last02T = 0;
static uint8_t  g_last02[SERVO_CH] = {0};

static void applyCmd(const uint8_t* d){
  g_secCmd[d[0] <= CMD_PHASE ? d[0] : 4]++;
  if(d[0] == CMD_SERVO){
    uint32_t t = millis();
    if(g_last02T && t - g_last02T > g_sec02Gap) g_sec02Gap = t - g_last02T;
    g_last02T = t;
    memcpy(g_last02, d + 1, SERVO_CH);
  } else if(d[0] != CMD_PHASE){
    logf("[RX] コマンド 0x%02X\n", d[0]);
  }
  if(d[0] != CMD_PHASE) flash(4, 50);   // 1 秒ごとの 0x03 では光らせない
  switch(d[0]){
  case CMD_RESET:
    enterReset();
    break;
  case CMD_PHASE2:
    startPhase2();
    break;
  case CMD_SERVO:
    for(uint8_t i = 0; i < SERVO_CH; i++){
      uint8_t v = d[1 + i];
      servoLocal[i] = (v > VALUE_MAX) ? VALUE_MAX : v;
    }
    lastServoCmdT = millis();
    setState(SV_ACTIVE);
    break;
  case CMD_PHASE:
    onPhase((int8_t)d[1], "脳");
    break;
  default:
    Serial.printf("[RX] 未知のコマンド 0x%02X（無視）\n", d[0]);
    break;
  }
}

static void drainCmds(){
  while(g_qr != g_qw){
    CmdEvt e = g_q[g_qr];
    g_qr = (uint8_t)((g_qr + 1) % QN);
    applyCmd(e.d);
  }
}

// Phase2：50ms ごとに 1 フレーム進める。終端で SV_RESET。
static void updatePhase2Sequence(uint32_t now){
  if(now - phase2T < PHASE2_FRAME_MS) return;
  phase2T += PHASE2_FRAME_MS;
  if(now - phase2T >= PHASE2_FRAME_MS) phase2T = now;   // 大きく遅れたら追いつこうとしない

  if(phase2Frame >= PHASE2_DATA_FRAMES){
    phase2Frame = 0;
    setState(SV_RESET);
    return;
  }
  for(uint8_t i = 0; i < PHASE2_DATA_CH; i++) servoLocal[i] = PHASE2_DATA[phase2Frame][i];
  phase2Frame++;
}

// 1 スロット（約 8ms）ごとに 1ch だけ I2C で書く（突入電流・I2C バーストの分散）
static void servoSlot(uint32_t now){
  if(now - slotT < SLOT_MS) return;
  slotT = now;

  switch(svState){

  case SV_OFF:
    break;   // PWM 停止済み。何もしない

  case SV_RESET: {
    bool allZero = true;
    for(uint8_t i = 0; i < SERVO_CH; i++){
      if(servoCurrent[i] > RESET_STEP){ servoCurrent[i] -= RESET_STEP; allZero = false; }
      else servoCurrent[i] = 0;
    }
    servoWrite(updateCh, servoCurrent[updateCh]);
    updateCh = (uint8_t)((updateCh + 1) % SERVO_CH);
    if(allZero){
      // 全 ch に 0° を書き終え（1 周分）、さらにサーボが戻りきる時間を待ってから PWM を切る
      if(resetZeroT == 0) resetZeroT = now;
      if(now - resetZeroT >= (uint32_t)SLOT_MS * SERVO_CH + RESET_HOLD_MS){
        allServosOff();
        memset(servoLocal, 0, sizeof(servoLocal));
        setState(SV_OFF);
      }
    }
  } break;

  case SV_SEQ:
    updatePhase2Sequence(now);
    if(svState != SV_SEQ) break;   // 終端に達して SV_RESET へ移った
    servoWrite(updateCh, servoLocal[updateCh]);
    servoCurrent[updateCh] = servoLocal[updateCh];
    updateCh = (uint8_t)((updateCh + 1) % SERVO_CH);
    break;

  case SV_ACTIVE:
    if((int32_t)(now - lastServoCmdT) >= (int32_t)ACTIVE_TIMEOUT_MS){   // 符号付き：lastServoCmdT が now より新しいことがある（2026-11-11 の誤リセット）
      Serial.println("[SV] 0x02 が途絶えた → リセット");
      setState(SV_RESET);
      break;
    }
    servoWrite(updateCh, servoLocal[updateCh]);
    servoCurrent[updateCh] = servoLocal[updateCh];
    updateCh = (uint8_t)((updateCh + 1) % SERVO_CH);
    break;
  }
}

// ---------------------------------------------------------------- シリアル（単体テスト）
static void printHelp(){
  Serial.printf("コマンド: 0=リセット / 1=Phase2 / 5=全ch50%% / m=全ch100%% / z=全ch0%% / s=状態 / ?=この表示（100%%=%d°）\n", SERVO_ANGLE_MAX);
  Serial.println("SIGGRAPH: 6 / 7=Phase を 6 / 7 にしたことにする / d=椅子を倒す / u=椅子を起こす / k=心拍の表示 ON/OFF");
  Serial.println("記録: L=記録をすべて出す / G=記録の状態 / E を 2 回=記録を消す / t<UNIX秒>=PC の時刻を記録（nozui_log_dump.py が使う）");
  Serial.println("配線の切り分け: w=椅子 UART のピンを読む / D=ピンを自分で動かして読む / S=ピン同士の短絡 / T=送信が D6 に出るか / R=ピンのレジスタ / H=D6 を 15 秒止める");
}
static void printLogInfo(){
  if(!g_logOK){ Serial.printf("[LOG] フラッシュが使えないので記録していない（ID=%02X %02X %02X）\n", g_logJedec[0], g_logJedec[1], g_logJedec[2]); return; }
  Serial.printf("[LOG] 起動 #%lu／フラッシュ %lu KB（4KB × %lu）／今のセクタ %lu の %lu バイト目／通し番号 %lu／RAM に未書き込み %lu バイト／記録できなかった %lu バイト／4KB の消去 最長 %lu ms\n",
                (unsigned long)g_bootNo, (unsigned long)(g_logSectors * 4), (unsigned long)g_logSectors, (unsigned long)g_logSec,
                (unsigned long)g_logPos, (unsigned long)g_logSeq, (unsigned long)(g_logHead - g_logTail), (unsigned long)g_logDropped,
                (unsigned long)g_logEraseMaxMs);
}
static void printStatus(){
  Serial.printf("[STAT] 接続=%d 状態=%s 受信=%lu 取りこぼし=%lu 長さ不正=%lu Phase2フレーム=%d 値(0〜100)=[%u %u %u %u %u %u]\n",
                g_conn, SV_NAME[svState], (unsigned long)g_rxCount, (unsigned long)g_rxDropped, (unsigned long)g_rxBadLen,
                phase2Frame, servoCurrent[0], servoCurrent[1], servoCurrent[2], servoCurrent[3], servoCurrent[4], servoCurrent[5]);
  Serial.printf("[STAT] Phase=%d ミラー=%.0f°%s 衝撃=%s 心拍=拾った %u・送った %lu・二重検出で捨てた %lu・平均間隔 %lu ms 椅子=%s（%s／%s%s%s）電文不正=%lu\n",
                g_phase, g_mirCur, g_mirMoving ? "（動作中）" : "", g_impStage ? "動作中" : "待機",
                g_hrCount, (unsigned long)g_hrSent, (unsigned long)g_hrRejected, (unsigned long)g_hrAvgMs,
                g_chairOnline ? "OK" : "応答なし", chairRelayName(g_chairStatus), chairPhysName(g_chairStatus),
                (g_chairStatus & 0x10) ? "／MPU6050 エラー" : "", (g_chairStatus & 0x20) ? "／★ 停止中" : "", (unsigned long)g_chairRxBad);
}
// 配線の確認（シリアル w）。椅子との UART を止め、D6・D7 を内部プルダウン／プルアップの入力にして読む（電圧は出さない）
static void checkChairWiring(){
  Serial1.end();
  // D8〜D10 は何もつながっていないピン。測り方が正しいかの見本（プルダウンで 0%、プルアップで 100% になるはず）
  const uint8_t pins[5] = { D6, D7, D8, D9, D10 };
  const char* names[5] = { "D6（ヘッドギアの TX）", "D7（ヘッドギアの RX）", "D8（見本・未接続）", "D9（見本・未接続）", "D10（見本・未接続）" };
  for(int i = 0; i < 5; i++){
    pinMode(pins[i], INPUT_PULLDOWN); delay(5);
    uint32_t n = 0, hiDn = 0, edges = 0; int prev = digitalRead(pins[i]);
    uint32_t t0 = millis();
    while(millis() - t0 < (i < 2 ? 600u : 100u)){ int v = digitalRead(pins[i]); n++; hiDn += v; if(v != prev){ edges++; prev = v; } }
    pinMode(pins[i], INPUT_PULLUP); delay(5);
    int hiUp = 0; for(int k = 0; k < 50; k++){ hiUp += digitalRead(pins[i]); delayMicroseconds(200); }
    Serial.printf("[WIRE] %s：プルダウンで HIGH %lu%%・変化 %lu 回／プルアップで HIGH %d%%\n", names[i],
                  (unsigned long)(hiDn * 100 / (n ? n : 1)), (unsigned long)edges, hiUp * 2);
    pinMode(pins[i], INPUT);
  }
  Serial.println("[WIRE] 読み方：椅子の TX（G26）は待機中 HIGH で 200ms ごとに変化する。D7 がそうなら正しい。D6 がそうなら TX と RX が逆");
  Serial1.begin(CHAIR_BAUD);
}

// 切り分け用（シリアル R）。D6=P1.11・D7=P1.12 を、ソフト（周辺機能）が握っていないかをチップのレジスタで直接見る
static void dumpUartPinRegs(){
  const uint32_t P6 = 0x20 | 11, P7 = 0x20 | 12;   // PSEL の書き方：bit5 = ポート1、bit0-4 = ピン番号、bit31 = 0 なら接続
  struct { const char* name; volatile uint32_t* reg; } ps[] = {
    {"UARTE0.TXD", &NRF_UARTE0->PSEL.TXD}, {"UARTE0.RXD", &NRF_UARTE0->PSEL.RXD},
    {"UARTE0.RTS", &NRF_UARTE0->PSEL.RTS}, {"UARTE0.CTS", &NRF_UARTE0->PSEL.CTS},
    {"UARTE1.TXD", &NRF_UARTE1->PSEL.TXD}, {"UARTE1.RXD", &NRF_UARTE1->PSEL.RXD},
    {"TWIM0.SCL", &NRF_TWIM0->PSEL.SCL}, {"TWIM0.SDA", &NRF_TWIM0->PSEL.SDA},
    {"TWIM1.SCL", &NRF_TWIM1->PSEL.SCL}, {"TWIM1.SDA", &NRF_TWIM1->PSEL.SDA},
    {"SPIM0.SCK", &NRF_SPIM0->PSEL.SCK}, {"SPIM0.MOSI", &NRF_SPIM0->PSEL.MOSI}, {"SPIM0.MISO", &NRF_SPIM0->PSEL.MISO},
    {"SPIM2.SCK", &NRF_SPIM2->PSEL.SCK}, {"SPIM2.MOSI", &NRF_SPIM2->PSEL.MOSI}, {"SPIM2.MISO", &NRF_SPIM2->PSEL.MISO},
    {"SPIM3.SCK", &NRF_SPIM3->PSEL.SCK}, {"SPIM3.MOSI", &NRF_SPIM3->PSEL.MOSI}, {"SPIM3.MISO", &NRF_SPIM3->PSEL.MISO},
    {"PDM.CLK", &NRF_PDM->PSEL.CLK}, {"PDM.DIN", &NRF_PDM->PSEL.DIN},
  };
  int hits = 0;
  for(auto& p : ps){
    uint32_t v = *p.reg;
    if(!(v & 0x80000000u) && ((v & 0x3F) == P6 || (v & 0x3F) == P7)){
      Serial.printf("[REG] ★ %s が %s を使っている（0x%08lX）\n", p.name, (v & 0x3F) == P6 ? "D6" : "D7", (unsigned long)v); hits++;
    }
  }
  NRF_PWM_Type* pwm[4] = { NRF_PWM0, NRF_PWM1, NRF_PWM2, NRF_PWM3 };
  for(int i = 0; i < 4; i++) for(int c = 0; c < 4; c++){
    uint32_t v = pwm[i]->PSEL.OUT[c];
    if(!(v & 0x80000000u) && ((v & 0x3F) == P6 || (v & 0x3F) == P7)){
      Serial.printf("[REG] ★ PWM%d.OUT%d が %s を使っている（ENABLE=%lu）\n", i, c, (v & 0x3F) == P6 ? "D6" : "D7", (unsigned long)pwm[i]->ENABLE); hits++;
    }
  }
  for(int c = 0; c < 8; c++){
    uint32_t v = NRF_GPIOTE->CONFIG[c];
    uint32_t pin = (v >> 8) & 0x3F, mode = v & 3;
    if(mode && (pin == P6 || pin == P7)){
      Serial.printf("[REG] ★ GPIOTE ch%d が %s を%s（0x%08lX）\n", c, pin == P6 ? "D6" : "D7", mode == 3 ? "出力として駆動している" : "割り込みに使っている", (unsigned long)v); hits++;
    }
  }
  Serial.printf("[REG] UARTE0.ENABLE=%lu（0 = 止まっている、8 = 動いている）\n", (unsigned long)NRF_UARTE0->ENABLE);
  for(int i = 0; i < 2; i++){
    uint32_t pin = 11 + i, cnf = NRF_P1->PIN_CNF[pin];
    Serial.printf("[REG] %s（P1.%lu）：向き=%s・プル=%s・出力値=%lu・入力値=%lu\n", i ? "D7" : "D6", (unsigned long)pin,
                  (cnf & 1) ? "出力" : "入力", ((cnf >> 2) & 3) == 3 ? "アップ" : ((cnf >> 2) & 3) == 1 ? "ダウン" : "なし",
                  (unsigned long)((NRF_P1->OUT >> pin) & 1), (unsigned long)((NRF_P1->IN >> pin) & 1));
  }
  Serial.println(hits ? "[REG] → 上の周辺機能がピンを握っている（ソフトが原因）" : "[REG] → D6・D7 を握っている周辺機能はない");
}
// R：UART を止めた状態とプルダウンにした状態で、上のレジスタを読む
static void checkUartPinRegs(){
  Serial.println("[REG] --- UART 動作中"); dumpUartPinRegs();
  Serial1.end();
  pinMode(D6, INPUT_PULLDOWN); pinMode(D7, INPUT_PULLDOWN); delay(5);
  Serial.println("[REG] --- UART を止め、D6・D7 をプルダウンにした状態"); dumpUartPinRegs();
  pinMode(D6, INPUT); pinMode(D7, INPUT);
  Serial1.begin(CHAIR_BAUD);
}

// D：D6・D7 を自分で LOW / HIGH に出し、ピンの実際の電圧（入力レジスタ）を読む。D8 は見本。出すのは 20ms だけ
// （digitalRead は出力ピンだと「出そうとした値」を返すので使わない）
static void checkPinDrive(){
  Serial1.end();
  const uint8_t pin[3] = { 11, 12, 13 };            // P1.11 = D6、P1.12 = D7、P1.13 = D8
  const char* name[3] = { "D6", "D7", "D8（見本）" };
  for(int i = 0; i < 3; i++){
    uint32_t bm = 1u << pin[i], keep = NRF_P1->PIN_CNF[pin[i]];
    for(int strong = 0; strong < 2; strong++){
      NRF_P1->PIN_CNF[pin[i]] = 0x1 | (strong ? (3u << 8) : 0);   // 出力・入力回路を接続（strong = 強い駆動）
      NRF_P1->OUTCLR = bm; delay(20); int lo = (NRF_P1->IN & bm) ? 1 : 0;
      NRF_P1->OUTSET = bm; delay(20); int hi = (NRF_P1->IN & bm) ? 1 : 0;
      Serial.printf("[DRIVE] %s・%s駆動：LOW を出すと %s、HIGH を出すと %s%s\n", name[i], strong ? "強い" : "普通の",
                    lo ? "HIGH のまま ★" : "LOW", hi ? "HIGH" : "LOW のまま ★",
                    (lo || !hi) ? " → ピンが外の電圧に負けている（ピンの故障か、電源・GND との短絡）" : " → 正常");
    }
    NRF_P1->PIN_CNF[pin[i]] = keep;
  }
  Serial1.begin(CHAIR_BAUD);
}
// S：ピン同士の短絡を探す。D6・D7 を 1 本ずつ LOW / HIGH に出し、ほかのピン（D0〜D10）がつられて変わるかを見る
// ほかのピンは入力のまま（内部プルアップ／プルダウンだけ）なので、短絡していなければ何も流れない。出すのは各 5ms
static void checkPinBridges(){
  Serial1.end();
  const uint8_t all[11] = { D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10 };
  const uint8_t drv[2] = { 11, 12 };                 // P1.11 = D6、P1.12 = D7
  const uint8_t drvD[2] = { 6, 7 };
  int found = 0;
  for(int k = 0; k < 2; k++){
    uint32_t bm = 1u << drv[k], keep = NRF_P1->PIN_CNF[drv[k]];
    for(int i = 0; i < 11; i++){
      if(i == drvD[k]) continue;
      if((k == 1) && i == 6) continue;                // D6–D7 は 1 回目で見ている
      uint8_t x = all[i];
      // 出すピンが LOW のとき、相手はプルアップ。HIGH のとき、相手はプルダウン。どちらでも相手がつられたら短絡
      NRF_P1->PIN_CNF[drv[k]] = 0x1;
      pinMode(x, INPUT_PULLUP);   NRF_P1->OUTCLR = bm; delay(5); int a = digitalRead(x);
      pinMode(x, INPUT_PULLDOWN); NRF_P1->OUTSET = bm; delay(5); int b = digitalRead(x);
      NRF_P1->PIN_CNF[drv[k]] = 0x2;                 // 出すのをやめる
      pinMode(x, INPUT_PULLUP);   delay(5); int a0 = digitalRead(x);   // 出していないときの相手の値（比べる用）
      pinMode(x, INPUT_PULLDOWN); delay(5); int b0 = digitalRead(x);
      bool follow = (a == 0 && a0 == 1) || (b == 1 && b0 == 0);
      if(follow){ Serial.printf("[SHORT] ★ D%d と D%d がつながっている（D%d が LOW→相手 %d、HIGH→相手 %d）\n", drvD[k], i, drvD[k], a, b); found++; }
      pinMode(x, INPUT);
    }
    NRF_P1->PIN_CNF[drv[k]] = keep;
  }
  if(!found) Serial.println("[SHORT] D6・D7 とほかのピン（D0〜D10）のあいだに短絡はない");
  Wire.begin(); Wire.setClock(I2C_CLOCK_HZ);         // D4・D5 の I2C を元に戻す
  pinMode(PIN_HEART, INPUT);
  Serial1.begin(CHAIR_BAUD);
}

// H：UART を止め、D6 を「入力・プルダウン」にしたまま 15 秒置く（その間はソフトは D6 に一切触れない）。テスターで D6–GND の電圧を測る用
static void holdPinForMeter(){
  Serial1.end();
  pinMode(D6, INPUT_PULLDOWN);
  Serial.println("[HOLD] D6 を入力・プルダウンにして 15 秒止めます。今のうちにテスターで D6–GND の電圧を測ってください");
  uint32_t t0 = millis(), hi = 0, n = 0;
  while(millis() - t0 < 15000){ hi += digitalRead(D6); n++; delay(1); }
  Serial.printf("[HOLD] 終了。この 15 秒、D6 が HIGH だった割合 %lu%%（0V 付近なら 0%% になるはず）\n", (unsigned long)(hi * 100 / n));
  pinMode(D6, INPUT);
  Serial1.begin(CHAIR_BAUD);
}

// T：UART で 0x00 を送りながら、D6 の実際の電圧（入力レジスタ）を読む。送信で LOW に落ちていれば TX は出ている
static void sampleTx(const char* label){
  const uint8_t z[8] = {0};
  uint32_t n = 0, lo = 0, edges = 0; int prev = 1;
  Serial.flush();
  Serial1.write(z, sizeof(z));
  uint32_t t0 = micros();
  while(micros() - t0 < 12000){                   // 9600bps × 8 バイト ≒ 8.3ms
    int v = (NRF_P1->IN >> 11) & 1; n++; lo += !v; if(v != prev){ edges++; prev = v; }
  }
  Serial.printf("[TX] %s：PIN_CNF=0x%08lX・送信中の D6 が LOW だった割合 %lu%%・変化 %lu 回 → %s\n", label,
                (unsigned long)NRF_P1->PIN_CNF[11], (unsigned long)(lo * 100 / n), (unsigned long)edges,
                edges ? "TX は D6 に出ている" : "★ 送っても D6 が動かない");
}
static void dumpUarte(const char* label){
  NRF_UARTE_Type* u = NRF_UARTE0;
  Serial.printf("[TX] %s：ENABLE=%lu PSEL.TXD=0x%lX BAUD=0x%08lX PTR=0x%08lX MAXCNT=%lu AMOUNT=%lu TXSTARTED=%lu ENDTX=%lu TXSTOPPED=%lu INTEN=0x%lX\n", label,
    (unsigned long)u->ENABLE, (unsigned long)u->PSEL.TXD, (unsigned long)u->BAUDRATE, (unsigned long)u->TXD.PTR,
    (unsigned long)u->TXD.MAXCNT, (unsigned long)u->TXD.AMOUNT, (unsigned long)u->EVENTS_TXSTARTED, (unsigned long)u->EVENTS_ENDTX,
    (unsigned long)u->EVENTS_TXSTOPPED, (unsigned long)u->INTENSET);
}
static void checkTxOutput(){
  dumpUarte("送る前");
  { const uint8_t z[4] = {0}; Serial1.write(z, 4); uint32_t t = micros(); while(micros() - t < 300){} dumpUarte("送った直後"); delay(10); dumpUarte("10ms 後"); }
  uint32_t keep = NRF_P1->PIN_CNF[11];
  NRF_P1->PIN_CNF[11] = 0x0;                     // 入力・入力回路を接続（UART が向きを自分で出力にするかを見る）
  delay(15); sampleTx("入力のまま");
  NRF_P1->OUTSET = 1u << 11;
  NRF_P1->PIN_CNF[11] = 0x1;                     // 出力・入力回路を接続（読めるようにしたうえで出力にする）
  delay(15); sampleTx("出力にした後");
  NRF_P1->PIN_CNF[11] = keep;
}

static void pollSerial(){
  while(Serial.available()){
    char ch = (char)Serial.read();
    uint8_t d[CMD_LEN] = {0};
    switch(ch){
    case '0': d[0] = CMD_RESET;  pushCmd(d); break;
    case '1': d[0] = CMD_PHASE2; pushCmd(d); break;
    case '5': d[0] = CMD_SERVO;  memset(d + 1, VALUE_MAX / 2, SERVO_CH); pushCmd(d); break;
    case 'm': d[0] = CMD_SERVO;  memset(d + 1, VALUE_MAX, SERVO_CH); pushCmd(d); break;
    case 'z': d[0] = CMD_SERVO;  pushCmd(d); break;
    case '6':
    case '7':
      if(g_phase < 0) g_phase = 0;   // 脳なしで試すとき、不明からだと倒さないので 0 から変わったことにする
      onPhase((int8_t)(ch - '0'), "シリアル");
      break;
    case 'd': chairCmd(CH_DOWN); break;
    case 'u': chairCmd(CH_UP); break;
    case 'w': checkChairWiring(); break;
    case 'R': checkUartPinRegs(); break;
    case 'D': checkPinDrive(); break;
    case 'T': checkTxOutput(); break;
    case 'S': checkPinBridges(); break;
    case 'H': holdPinForMeter(); break;
    case 'L': if(g_logOK) g_logDumpReq = true; else USBSerial.println("[LOG] フラッシュが使えないので記録していない"); break;
    case 'G': printLogInfo(); break;
    case 'E': {                          // 記録を消す：E を 3 秒以内に 2 回
      static uint32_t armT = 0;
      if(armT && millis() - armT < 3000){ armT = 0; if(g_logOK) g_logEraseReq = true; }
      else { armT = millis(); USBSerial.println("[LOG] 記録をすべて消すには、3 秒以内にもう一度 E"); }
      break;
    }
    case 't': {                          // PC の時刻（UNIX 秒）を受け取って記録に残す：t1731300000 + 改行
      uint32_t v = 0, t0 = millis(); int got = 0;
      while(millis() - t0 < 300 && got < 10){
        if(!Serial.available()){ delay(1); continue; }
        char c = (char)Serial.read();
        if(c >= '0' && c <= '9'){ v = v * 10 + (c - '0'); got++; } else break;
      }
      if(got == 10) Serial.printf("[TIME] PC 時刻 unix=%lu\n", (unsigned long)v);   // 10 桁そろったときだけ（1 桁落ちて日付がずれたことがある）
      else          USBSerial.printf("[TIME] 受け取れなかった（%d 桁）\n", got);
      break;
    }
    case 'k': g_printHeart = !g_printHeart; Serial.printf("[HEART] 表示 %s\n", g_printHeart ? "ON" : "OFF"); break;
    case 's': printStatus(); break;
    case '?': printHelp(); break;
    default: break;
    }
  }
}

// ---------------------------------------------------------------- setup / loop
void setup(){
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT); pinMode(LED_BLUE, OUTPUT);
  led(4);

  Serial.begin(115200);
  // SIGGRAPH 版は USB シリアルを待たない（椅子は起動から 10 秒以内に UART の電文が届かないと停止するため）。
  // 起動直後の表示は記録（フラッシュ）に残るので、あとから L か nozui_log_dump.py で見られる
  logBegin();
  Serial.println();
  Serial.println("=== NozuiHeadgear_SIGGRAPH（ヘッドギア・SIGGRAPH 版）===");
  if(g_logOK) Serial.printf("[LOG] 起動 #%lu。理由：%s（RESETREAS=0x%08lX）。記録はフラッシュ %lu KB に残す\n", (unsigned long)g_bootNo,
                            resetReasonText(readResetReason()), (unsigned long)readResetReason(), (unsigned long)(g_logSectors * 4));
  else        Serial.printf("[LOG] ★ 内蔵フラッシュが見つからないので、記録は残らない（動作はそのまま続ける）。ID=%02X %02X %02X\n",
                            g_logJedec[0], g_logJedec[1], g_logJedec[2]);

  // PCA9685（XIAO の Wire = D4:SDA / D5:SCL）
  if(!pwm.begin()) Serial.println("[PWM] PCA9685 が見つからない（配線・アドレス 0x40 を確認）");
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(I2C_CLOCK_HZ);
  allServosOff();                            // 起動時は脱力（SV_OFF）
  impactOff();                               // 頭部衝撃・ミラーも脱力
  mirrorOff();

  // 心拍センサ（D0 の立ち上がりで割り込み）
  pinMode(PIN_HEART, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_HEART), onHeartBeat, RISING);

  // 椅子（Serial1 = D6:TX / D7:RX）
  Serial1.begin(CHAIR_BAUD);

  // BLE（Peripheral 1 本、セキュリティなし）
  Bluefruit.autoConnLed(false);
  Bluefruit.begin(1, 0);
  Bluefruit.setTxPower(4);
  Bluefruit.setName(BLE_NAME);
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
  Bluefruit.Periph.setConnInterval(6, 12);   // 7.5〜15ms（台湾版は 8〜12）

  hgSvc.begin();
  hgCmd.setProperties(CHR_PROPS_WRITE_WO_RESP | CHR_PROPS_WRITE);  // 本番は WoR。WRITE はスマホアプリでのテスト用
  hgCmd.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  hgCmd.setFixedLen(CMD_LEN);
  hgCmd.setWriteCallback(onCmdWrite);
  hgCmd.begin();

  hgHeart.setProperties(CHR_PROPS_NOTIFY);   // 心拍 → 脳（SIGGRAPH 版）
  hgHeart.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  hgHeart.setFixedLen(2);
  hgHeart.begin();

  // 広告：名前は広告パケットに（脳デバイスが名前でフィルタ）、128-bit UUID はスキャンレスポンスに
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addName();
  Bluefruit.ScanResponse.addService(hgSvc);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  Serial.printf("広告中: %s\n", BLE_NAME);
  printHelp();
}

// 記録だけに残す定期の情報（1 秒ごとの受信数・30 秒ごとの状態・loop の停止）
static void logPeriodic(uint32_t now){
  static uint32_t lastLoop = 0, lastSec = 0, lastStat = 0;
  if(lastLoop && now - lastLoop > 50) logf("[LOOP] ★ loop が %lu ms 止まっていた\n", (unsigned long)(now - lastLoop));
  lastLoop = now;

  if(now - lastSec >= 1000){
    lastSec = now;
    uint32_t sum = g_secCmd[0] + g_secCmd[1] + g_secCmd[2] + g_secCmd[3] + g_secCmd[4];
    if(sum)
      logf("[RX/1s] 0x02×%lu（最大間隔 %lu ms、最後の値 %u %u %u %u %u %u）0x03×%lu 0x00×%lu 0x01×%lu 他×%lu 落とした累計 %lu\n",
           (unsigned long)g_secCmd[2], (unsigned long)g_sec02Gap, g_last02[0], g_last02[1], g_last02[2], g_last02[3], g_last02[4], g_last02[5],
           (unsigned long)g_secCmd[3], (unsigned long)g_secCmd[0], (unsigned long)g_secCmd[1], (unsigned long)g_secCmd[4], (unsigned long)g_rxDropped);
    memset(g_secCmd, 0, sizeof(g_secCmd));
    g_sec02Gap = 0;
  }
  if(now - lastStat >= 30000){
    lastStat = now;
    LogOut.echo = false;               // 記録だけ（シリアルには出さない）
    printStatus();
    LogOut.echo = true;
  }
}

void loop(){
  uint32_t now = millis();
  logPeriodic(now);

  // LED
  if(now < g_flashUntil) led(g_flash);
  else if(g_conn)        led(2);                           // 緑
  else                   led(((now / 500) % 2) ? 1 : 0);   // 赤点滅

  if(g_disconnectEvt){
    g_disconnectEvt = false;
    Serial.println("[BLE] 切断 → リセット");
    enterReset();
    brainLinkLost("脳との BLE が切断");
  }
  if(g_conn && g_phase >= 0 && now - g_lastPhaseRxT >= BRAIN_TIMEOUT_MS) brainLinkLost("脳から 0x03 が 3 秒来ない");

  pollSerial();
  drainCmds();
  servoSlot(now);
  updateExtras(now);
  pumpHeart();
  pollChair(now);

  // SIGGRAPH 版は心拍・椅子の遅れを小さくするため、停止中でも 1ms で回す（台湾版は停止中 5ms）
  delay(1);
}
