// ============================================================================
//  NozuiBrain_SIGGRAPH — XR作品「脳髄はものを考えるところに非ず」脳デバイス側ファームウェア【SIGGRAPH 版】
//  ★ SIGGRAPH 展示版。ヘッドギア NozuiHeadgear_SIGGRAPH.ino と組で使う
//  台湾版 NozuiBrain_Taiwan.ino がベース。差分は nozui_siggraph_spec.md（§3・§4）
//  XIAO nRF52840 Sense（Seeed nRF52 Boards ← mbed ではない方）＋ 圧力センサ 4ch（D0〜D3）
//  ＋ DRV8833（D4=AIN1 / D5=AIN2、VM=電池 BAT+、EEP は 3V3 へ）＋ 振動モータ（ERM、DC3V）
//
//  ■ SIGGRAPH 版で足したこと
//    ・ヘッドギアへコマンド 0x03 [Phase] を送る（Phase が変わった瞬間・接続した瞬間・1 秒ごと）。
//      ヘッドギアはこれでミラー・頭部衝撃・椅子を動かし、1 秒ごとの 0x03 を「生きている合図」にする
//    ・ヘッドギアの HEART キャラクタリスティックを Notify で購読し、心拍が届いたら
//      Phase3・4 のときだけ DRV8833 でモータを鳴らす（PWM 20kHz で 2ms ごとに強さを更新。
//      電池電圧に合わせて、モータにかかる電圧が MOTOR_RATED_V を超えないよう PWM の幅を抑える。
//      波形は HapticHeartTest で試したもの。HAPTIC_WAVE_NO で選ぶ）
//    ・PWM はハードウェアの PWM（HwPWM3）を直接使う（追加のライブラリ不要）
//    ・シリアル h … 心拍の振動を 1 回鳴らす（Phase に関係なく。波形の確認用）
//    ・起動したとき・台座に載せて給電が来たときに、心拍の振動を 1 回鳴らす（動いている・給電されたの合図）
//    ・圧力の向きは、新しいファームを書き込んだ直後の起動で押し込みにより決め、確定したら内蔵フラッシュに保存する。
//      以後の起動は保存した向きを使い、変えない（ゼロ点・最大値は毎回測る）。シリアル D で決め直し、c はゼロ点だけ（2026-11-11）
//    ・電池残量を Quest に 2 つの道で知らせる（どちらを Quest が使うか分からないため両方）
//        ① 標準の Battery Service（0x180F / 0x2A19、読み出し＋通知）… Android の Bluetooth 設定の残量はこの道
//        ② HID 入力レポート ID=4「Battery Strength」（本物の Xbox One S と同じ形）… Linux の HID 電池（入力機器の残量）の道
//      Quest が ① を読みに来た・通知を購読した・② を購読したことをシリアルに出す（どちらの道が効いているかの切り分け用）
//      ★ サービスと記述子が変わるので、書き込んだら Quest 側でペアリングを解除して組み直す（脳側はシリアル b）
//    ・シリアル v … 残量を 100% → 50% → 10% → 実測 の順に切り替える（Quest の表示確認用）
//    ・Quest との通信は台湾版のまま（心拍は Quest に送らない）
//
//  ===== ここから下は台湾版の説明（変わっていない） =====
//
//  ■ 役割
//    ① Peripheral（HID Gamepad）→ Meta Quest 3
//       ・Xbox One S（VID 0x045E / PID 0x02FD）として名乗る（XIAO_phase_test の disSvc/disPnp/REPORT_MAP を流用）
//       ・受信：出力レポート ID=3（Xbox の振動）→ strong=byte[3] → phase = (strong + 5) / 10 - 1
//         Quest は Android の VibratorManager で強さ (phase + 1) / 10 の振動を送る（Phase0=0.1 … Phase8=0.9）。
//         0.0 は「アプリのフォーカスが外れて振動が止まった」だけなので無視する（Phase0 と誤解しない）
//       ・送信：入力レポート ID=1（16 バイト）
//           ボタン1（A, bit0）= 投げ検出（Phase4 のみ）
//           ボタン2（B, bit1）= 台座外れ（Phase3 のみ）
//           ボタン4（X, bit3）= 給電ケーブルが外れている間ずっと 1（全 Phase。Quest は Phase0 で警告を出す。Issue #8）
//           ブレーキ（10bit）  = 圧力合計（Phase3/4 のみ）→ Unity leftTrigger 0.0〜1.0
//    ② Central（GATT Client）→ ヘッドギア（NozuiHeadgear-X1）へ 7 バイトのコマンドを Write Without Response
//       0x00 = リセット＆停止（Phase2・3・4 以外に変わった瞬間に 1 回）
//       0x01 = Phase2 トリガー（Phase2 に変わった瞬間に 1 回）
//       0x02 = サーボ値 [0x02][s0..s5]（0〜100、Phase3/4 中 20Hz）
//         値は 0〜100 ベースで送る。角度（可動域）への変換はヘッドギア側の SERVO_ANGLE_MAX で行うので、
//         可動域を変えるときにこちらを書き換える必要はない
//
//  ■ 「Core0 / Core1」について（仕様書との違い）
//    nRF52840 は Cortex-M4 のシングルコアなので setup1()/loop1() は存在しない（それは RP2040 の機能）。
//    代わりに FreeRTOS のタスク（Scheduler.startLoop）で仕様書の分担を再現している：
//      ・「Core0」相当 … loop()  ：BLE 送信 / Phase 管理 / LED / 電源監視
//      ・「Core1」相当 … loop1() ：圧力センサ読み取り・動的キャリブレーション（常時）/ IMU（Phase4 のみ）
//    ADC と I2C（IMU）に触るのは loop1() だけなので、タスク間で取り合いにならない。
//
//  ■ 仕様書からの変更点
//    ・トリガー（Rx）：Xbox のレポート形式では左トリガー＝「ブレーキ 10bit（0〜1023）」なので、
//      圧力合計 0〜400 → 0〜1023 にスケールして入れる（Unity では leftTrigger 0.0〜1.0 になる）
//    ・ボタンの ON 時間：仕様書は 5ms だが、押下と解放が Unity の同じフレームに届くと
//      wasPressedThisFrame が取りこぼすので、BUTTON_HOLD_MS（150ms）押してから離す（ノンブロッキング）
//    ・台座外れ（B）：外れた瞬間だけでなく、Phase3 に入った時点ですでに外れていればすぐ B を送る
//      （Phase2 のうちに持ち上げられても Phase3 で止まらないように）
//    ・投げ判定：台座から外した瞬間と Phase4 に入った瞬間から IMU_IGNORE_MS（500ms）は判定しない
//      （ポゴピンが外れた衝撃を投げと誤検出しないように）
//    ・ヘッドギアに（再）接続した瞬間、Phase が 0/1/5〜8 ならコマンド 0x00 を 1 回送って状態を合わせる
//    ・圧力：Phase1 に入るたびにゼロ点を測り直し、最大値を初期値に戻す（向きは残す）。
//      体験者ごとに基準を揃えるため（Issue #3）。向きまで戻すのは起動時とシリアル c だけ
//    ・給電ケーブルの状態（X）：外れている間は X を押したままにする（パルスの A・B とは別に、毎回のレポートに載せる）。
//      変わった瞬間と、Quest との暗号化が済んだ瞬間にもレポートを送る。Phase3・4 以外のハートビートでは、
//      外れている間は X を離したレポート → 押したレポートの順に送る（後から起動したアプリにも届くように。Issue #8）
//
//  ■ シリアルコマンド（115200, Quest なしの単体テスト用）
//    0〜8 … Phase を手動で設定（Quest から次の値が来たらそちらで上書き）
//    a … A ボタン（投げ）を送る    u … B ボタン（台座外れ）を送る
//    x … X（ケーブル外れ）を強制的に送る ON/OFF（USB でシリアルをつないでいると給電が「あり」になるので、その確認用）
//    p … 圧力値の表示 ON/OFF       s … 状態表示
//    b … ボンド消去して再起動      ? … ヘルプ
//
//  ■ LED：Quest も頭も未接続＝赤点滅／Quest のみ＝緑／頭のみ＝青点滅／両方＝水色
//        Phase 変化＝白 0.3 秒／投げ＝青 0.4 秒／台座外れ＝黄 0.4 秒
//
//  ■ 必要ライブラリ：Seeed Arduino LSM6DS3、Bluefruit（ボード同梱）
//  ■ Quest には新しい名前（NozuiBrain-X1）で見えるので、XIAO_phase_test のペアリングは解除してから組み直す
// ============================================================================
#include <bluefruit.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include "LSM6DS3.h"
#include <InternalFileSystem.h>        // 圧力の向きを保存する（ボード同梱。BLE のボンド情報と同じ場所）
using namespace Adafruit_LittleFS_Namespace;

#define DEBUG_IMU 0   // 1 で Phase4 の投げ判定中に |a|・ジャイロ・状態をシリアルに流す（閾値調整用）

// ---------------------------------------------------------------- BLE 設定
static const char* BLE_NAME      = "NozuiBrain-X1";
static const char* HEADGEAR_NAME = "NozuiHeadgear-X1";   // この名前のヘッドギアにだけ接続する

static const uint16_t PNP_VID = 0x045E;                   // Microsoft
static const uint16_t PNP_PID = 0x02FD, PNP_VER = 0x0408; // Xbox One S（いちばん安定）

static const uint32_t SEND_MS        = 50;    // Phase3/4 の HID / GATT 送信間隔（20Hz）
static const uint32_t HEARTBEAT_MS   = 1000;  // それ以外の Phase でのニュートラル入力レポート間隔
static const uint32_t BUTTON_HOLD_MS = 150;   // ボタンを押している時間（Unity の取りこぼし対策）
static const uint32_t STATUS_MS      = 5000;  // 状態行を出す間隔
static const uint32_t PHASE_SYNC_MS  = 1000;  // ヘッドギアへ 0x03（今の Phase）を送り直す間隔（生きている合図を兼ねる）

// ---------------------------------------------------------------- 心拍の振動（DRV8833 ＋ 振動モータ、SIGGRAPH 版）
// モータは ERM（一般的な DC3V の振動モータ）。DRV8833 の A 側（AIN1/AIN2 → AOUT1/AOUT2）で動かす。
// VM は電池（BAT+、3.0〜4.2V）から。モータの定格を超えないよう、PWM の幅の上限 = MOTOR_RATED_V ÷ 電池電圧。
// 波形は a/HapticHeartTest（DRV2605L 版）で確認したものと同じ。単体の確認は a/HapticHeartTest_DRV8833 で。
#define PIN_MOTOR_IN1     D4        // DRV8833 AIN1
#define PIN_MOTOR_IN2     D5        // DRV8833 AIN2
#define HAPTIC_STEP_MS    2         // 強さを更新する間隔
#define MOTOR_PWM_TOP     800       // 16MHz ÷ 800 = 20kHz（耳に聞こえない）
static const float   MOTOR_RATED_V   = 3.0f;   // ★ モータの定格電圧 [V]。強さ 255 のときモータにかかる電圧
static const float   HAPTIC_GAIN     = 1.0f;   // ★ 強さの倍率（0.1〜1.0）。強すぎたら下げる（ブレーキには掛けない）
static const int16_t HAPTIC_BRAKE    = -200;   // ブレーキの強さ（-255〜0）。拍の終わりに逆向きに短く駆動してキレを出す
static const uint16_t HAPTIC_BRAKE_MS = 15;    // ブレーキの長さ
#define HAPTIC_WAVE_NO    2         // ★ 波形の選択：1 = 旧 M5 版の「ドクン」（330ms）、2 = 「ドッ・クン」（約 370ms）

// 区間ごとに直線で強さを変える。強さは -255〜255（負はブレーキ）
struct HapticSeg { int16_t from, to; uint16_t ms; bool brake; };
#if HAPTIC_WAVE_NO == 1
static const HapticSeg HAPTIC_WAVE[] = {   // 旧 M5 版（brain_m5sticks3）と同じ ＋ ブレーキ
  {   0, 220, 50, false },
  { 220, 220, 80, false },
  { 220, 130, 60, false },
  { 130,  40, 25, false },
  {  40,  80, 35, false },
  {  80,   0, 80, false },
  { HAPTIC_BRAKE, HAPTIC_BRAKE, HAPTIC_BRAKE_MS, true },
};
#else
static const HapticSeg HAPTIC_WAVE[] = {   // 「ドッ・クン」：強い 1 打目、間を置いて少し弱い 2 打目
  {   0, 255,  20, false },   // ドッ
  { 255, 255,  50, false },
  { 255,   0,  40, false },
  { HAPTIC_BRAKE, HAPTIC_BRAKE, HAPTIC_BRAKE_MS, true },
  {   0,   0, 110, false },   // 間
  {   0, 180,  20, false },   // クン
  { 180, 180,  40, false },
  { 180,   0,  60, false },
  { HAPTIC_BRAKE, HAPTIC_BRAKE, HAPTIC_BRAKE_MS, true },
};
#endif
static const int HAPTIC_SEGS = sizeof(HAPTIC_WAVE) / sizeof(HAPTIC_WAVE[0]);

// ---------------------------------------------------------------- 圧力センサ（§3-5）
#define PRESSURE_CH       4
#define CALIB_SAMPLES     8        // 起動時の平均サンプル数
#define PRESSURE_MS       50       // 20Hz（50ms ごとに読み取り）
#define PRESSURE_OUT_MAX  100      // 出力の最大値（通信値 0〜100。角度への変換はヘッドギア側）
#define PEAK_MIN_THRESH   200      // 押し込みとみなす変化量・スケーリングの最低ピーク幅（raw ADC差）
#define PRESS_RELEASE_THRESH (PEAK_MIN_THRESH / 2)  // これを下回ったら押し込みが終わった（次の押し込みを数える）
#define DIR_CONFIRM_PRESSES 3      // 仮決めと同じ向きの押し込みがこの回数で向きを確定（仮決めの 1 回を含む）
#define DIR_FLIP_PRESSES    2      // 確定前に逆向きの押し込みがこの回数続いたら向きを反転
static const uint8_t PIN_PRESSURE[PRESSURE_CH] = { D0, D1, D2, D3 };

// ---------------------------------------------------------------- 投げ判定の閾値（XIAO_throw_uart と同じ。★ 実物で調整）
static const float    FREEFALL_G      = 0.3f;
static const uint32_t THROW_FF_MS     = 60;
static const uint32_t SLIP_FF_MS      = 150;
static const float    SWING_G         = 2.5f;
static const uint32_t SWING_WINDOW_MS = 500;
static const float    FLIGHT_END_G    = 0.6f;
static const float    R_FROM_CENTER_M = 0.01f;  // XIAO から脳の中心までの距離 [m]（遠心力の差し引き用）
static const float    SPIN_MAX_SUB_G  = 0.8f;
static const int      FLIGHT_END_N    = 3;
// ---- 床に落ちた / キャッチした の区別（★ 床の材質が決まったら実物で調整）
//   自由落下が終わってから PEAK_WINDOW_MS の間の |a| の最大値（衝撃のピーク）で分ける。
//   シリアルに毎回「[IMU] 床に着地 / キャッチ  衝撃ピーク X.XG」が出るので、床に落としたときと
//   手でキャッチしたときの値を数回ずつ見て、その間に LAND_SOFT_G を置く。
//   ・ピーク ≥ LAND_HARD_G                         → 床（すぐ確定）
//   ・ピーク ≥ LAND_SOFT_G で、静止が STILL_MS 続いた → 床（柔らかい床。転がって止まった）
//   ・それ以外（CATCH_MS たっても手の中で動いている／弱い衝撃のまま止まった）→ キャッチ（何も送らない）
//   柔らかい床で床に落ちてもキャッチ扱いになる → LAND_SOFT_G を下げる
//   キャッチでも床扱いになる                   → LAND_SOFT_G / LAND_HARD_G を上げる
static const uint32_t PEAK_WINDOW_MS  = 300;    // 衝撃のピークを探す時間
static const float    LAND_HARD_G     = 12.0f;   // これ以上なら即「床」（LSM6DS3 は ±16G まで測れる）
static const float    LAND_SOFT_G     = 7.0f;   // これ以上で、その後静止したら「床」
static const float    STILL_TOL_G     = 0.05f;  // 静止とみなす |a| の 1G からのずれ
static const float    STILL_GYRO_DPS  = 6.0f;   // 静止とみなす回転の速さ
static const uint32_t STILL_MS        = 400;    // 静止がこの時間続いたら「止まった」
static const uint32_t CATCH_MS        = 800;    // この時間たっても止まらなければ「キャッチ」
static const uint32_t COOLDOWN_MS     = 1000;
static const uint32_t SAMPLE_MS       = 5;      // 200Hz
static const uint32_t IMU_IGNORE_MS   = 500;    // 台座外れ・Phase4 開始からこの時間は投げ判定しない

// ---------------------------------------------------------------- 電源
static const bool     CHARGE_100MA  = true;     // P0.13=LOW で 100mA
static const uint32_t USB_STABLE_MS = 150;      // 接点のガタで一瞬切れても反応しない
static const uint32_t VBAT_MS       = 2000;     // 電池電圧を測る間隔
static const float    VBAT_DIV      = 1510.0f / 510.0f;
#define PIN_CHG 23                              // D23＝P0.17（LOW で充電中）

// ---------------------------------------------------------------- HID（XIAO_phase_test からそのまま流用）
// 出典：Mystfit/ESP32-BLE-CompositeHID（MIT License）XboxOneS_1914_HIDDescriptor
// 入力 ID=1（16 バイト）／出力 ID=3（8 バイト）／入力 ID=4（電池の強さ 1 バイト、SIGGRAPH 版で追加）
static const uint8_t REPORT_MAP[] = {
  0x05,0x01,0x09,0x05,0xA1,0x01,0x85,0x01,0x09,0x01,0xA1,0x00,
  0x09,0x30,0x09,0x31,0x15,0x00,0x27,0xFF,0xFF,0x00,0x00,0x95,
  0x02,0x75,0x10,0x81,0x02,0xC0,0x09,0x01,0xA1,0x00,0x09,0x32,
  0x09,0x35,0x15,0x00,0x27,0xFF,0xFF,0x00,0x00,0x95,0x02,0x75,
  0x10,0x81,0x02,0xC0,0x05,0x02,0x09,0xC5,0x15,0x00,0x26,0xFF,
  0x03,0x95,0x01,0x75,0x0A,0x81,0x02,0x15,0x00,0x25,0x00,0x75,
  0x06,0x95,0x01,0x81,0x03,0x05,0x02,0x09,0xC4,0x15,0x00,0x26,
  0xFF,0x03,0x95,0x01,0x75,0x0A,0x81,0x02,0x15,0x00,0x25,0x00,
  0x75,0x06,0x95,0x01,0x81,0x03,0x05,0x01,0x09,0x39,0x15,0x01,
  0x25,0x08,0x35,0x00,0x46,0x3B,0x01,0x66,0x14,0x00,0x75,0x04,
  0x95,0x01,0x81,0x42,0x75,0x04,0x95,0x01,0x15,0x00,0x25,0x00,
  0x35,0x00,0x45,0x00,0x65,0x00,0x81,0x03,0x05,0x09,0x19,0x01,
  0x29,0x0F,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x0F,0x81,0x02,
  0x15,0x00,0x25,0x00,0x75,0x01,0x95,0x01,0x81,0x03,0x05,0x0C,
  0x0A,0xB2,0x00,0x15,0x00,0x25,0x01,0x95,0x01,0x75,0x01,0x81,
  0x02,0x15,0x00,0x25,0x00,0x75,0x07,0x95,0x01,0x81,0x03,0x05,
  0x0F,0x09,0x21,0x85,0x03,0xA1,0x02,0x09,0x97,0x15,0x00,0x25,
  0x01,0x75,0x04,0x95,0x01,0x91,0x02,0x15,0x00,0x25,0x00,0x75,
  0x04,0x95,0x01,0x91,0x03,0x09,0x70,0x15,0x00,0x25,0x64,0x75,
  0x08,0x95,0x04,0x91,0x02,0x09,0x50,0x66,0x01,0x10,0x55,0x0E,
  0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x01,0x91,0x02,0x09,
  0xA7,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x01,0x91,0x02,
  0x65,0x00,0x55,0x00,0x09,0x7C,0x15,0x00,0x26,0xFF,0x00,0x75,
  0x08,0x95,0x01,0x91,0x02,0xC0,
  // 入力 ID=4：電池の強さ（Generic Device Controls / Battery Strength、0〜255）。本物の Xbox One S（1708）と同じ形。
  // Linux はこれを見つけると HID 機器の電池（power_supply）を作り、Android の入力機器の残量になる
  0x05,0x06,0x09,0x20,0x85,0x04,0x15,0x00,0x26,0xFF,0x00,0x75,
  0x08,0x95,0x01,0x81,0x02,
  0xC0
};
static uint16_t IN_LEN[4]  = { 16, 1, 1, 1 };   // 入力 ID=1 が本命、ID=4 が電池。ID=2,3 はダミー（送らない。Bluefruit は ID 順に並べるため）
static uint16_t OUT_LEN[3] = { 1, 1, 8 };   // 出力 ID=3 が本命。ID=1,2 はダミー

// 電池レポート（ID=4）を Quest が購読したかを見るために、入力レポートの中身に触れる子クラス
class NozuiHid : public BLEHidGeneric {
public:
  NozuiHid() : BLEHidGeneric(4, 3, 0) {}
  bool inputNotifyEnabled(uint16_t conn, uint8_t id){ return _chr_inputs[id - 1].notifyEnabled(conn); }
};
NozuiHid          blehid;
// 標準の Battery Service（BLEBas は中の特性を外から触れないので自前で作る。読まれた・購読されたを知るため）
BLEService        basSvc(UUID16_SVC_BATTERY);
BLECharacteristic basLvl(UUID16_CHR_BATTERY_LEVEL);
BLEService        disSvc(0x180A);
BLECharacteristic disMfr(0x2A29);
BLECharacteristic disMdl(0x2A24);
BLECharacteristic disPnp(0x2A50);

#define BTN_A  (1 << 0)   // 投げ
#define BTN_B  (1 << 1)   // 台座外れ
#define BTN_X  (1 << 3)   // 給電ケーブルが外れている（状態。押したまま）。Xbox の Button 4 → Android BUTTON_X → Unity buttonWest

// ---------------------------------------------------------------- ヘッドギア GATT（§2-2）
// Service : AA000000-0000-0000-0000-000000000001
// CMD     : AA000000-0000-0000-0000-000000000002
static const uint8_t UUID_SVC[16] = { 0x01,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0xAA };
static const uint8_t UUID_CMD[16] = { 0x02,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0xAA };
// HEART   : AA000000-0000-0000-0000-000000000003（SIGGRAPH 版。ヘッドギア → 脳の Notify、[0x10][通し番号]）
static const uint8_t UUID_HEART[16] = { 0x03,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0xAA };

BLEClientService        hgSvc(UUID_SVC);
BLEClientCharacteristic hgCmd(UUID_CMD);
BLEClientCharacteristic hgHeart(UUID_HEART);

#define CMD_LEN 7
enum : uint8_t { CMD_RESET = 0x00, CMD_PHASE2 = 0x01, CMD_SERVO = 0x02, CMD_PHASE = 0x03 };
#define HEART_MSG 0x10

LSM6DS3 imu(I2C_MODE, 0x6A);

// ---------------------------------------------------------------- タスク間共有（§3-1）
volatile uint8_t g_calibOut[PRESSURE_CH]   = {0};      // キャリブ済み圧力値 0〜100（loop1 書き / loop 読み）
volatile bool    g_calibReady[PRESSURE_CH] = {false};  // 向きが決まった（仮決めを含む。4ch 共通なので全部同じ値）
volatile bool    g_throwTrigger = false;               // 投げ検出（loop1 書き / loop 読み・クリア）
volatile char    g_imuEvt       = 0;                   // 表示用：'t' 投げ / 's' 取り落とし / 'l' 床に着地 / 'c' キャッチ
volatile float   g_imuPeak      = 0;                   // 表示用：着地・キャッチ時の衝撃のピーク [G]
volatile char    g_dirEvt       = 0;                   // 表示用：'p' 圧力の向きを仮決め / 'L' 確定 / 'F' 反転 / 'z' Phase1 のリセット
volatile bool    g_recalReq     = false;               // 圧力キャリブレーションのやり直し要求（loop 書き / loop1 読み）
volatile bool    g_rezeroReq    = false;               // Phase1 に入った：ゼロ点・最大値のリセット要求（向きは残す。loop 書き / loop1 読み）
volatile int8_t  g_phaseRx      = -1;                  // Quest から受信した Phase（BLE コールバック書き）
volatile int8_t  g_phaseNow     = 0;                   // loop が確定させた現在の Phase（loop1 が読む）
volatile bool    g_imuOk        = false;
volatile uint32_t g_imuIgnoreUntil = 0;              // この時刻までは投げ判定しない（loop / loop1 が書く）
volatile float   g_vbat         = 0;

// ---------------------------------------------------------------- 接続状態（BLE コールバック書き）
volatile uint16_t g_questConn = BLE_CONN_HANDLE_INVALID;
volatile uint16_t g_hgConn    = BLE_CONN_HANDLE_INVALID;
volatile bool     g_hgReady   = false;   // サービス／キャラクタリスティックの検出まで済んだ
volatile bool     g_hgJustConnected = false;
volatile bool     g_hgHeartOk = false;   // HEART の Notify を購読できた
volatile uint8_t  g_beatRx    = 0;       // 届いた心拍の数（BLE コールバックが増やす。loop は差分を見る）
volatile uint8_t  g_beatSeq   = 0;       // 最後に届いた通し番号（表示用）
volatile uint32_t g_beatMissed = 0;      // 通し番号が飛んだ数（＝BLE で届かなかった拍。切り分け用）
volatile uint8_t  g_beatGapEvt = 0;      // 直近に飛んだ数（loop の表示用）
volatile bool     g_beatHaveSeq = false;
static bool       g_printBeat = true;    // シリアル k で心拍 1 拍ずつの表示 ON/OFF

// ---------------------------------------------------------------- loop 側の状態
static int      g_phase     = 0;    // 現在の Phase（0〜8）
static int      g_prevPhase = -1;   // Phase 変化検出用（-1 = 起動直後）
static uint16_t g_buttons   = 0;
static uint16_t g_brake     = 0;    // 0〜1023
static uint32_t g_btnUpAt   = 0;
static bool     g_printPressure = false;
static bool     g_usbNow    = false;   // 台座（USB 給電）あり（updatePower がデバウンスして更新）
static bool     g_forceCableOff = false;   // シリアル x：給電があっても X（ケーブル外れ）を送る（テスト用）
static void     sendDetachB(const char* why);

// ================================================================ LED（LOW で点灯） bit0=R bit1=G bit2=B
static uint8_t  g_flash = 0;
static uint32_t g_flashUntil = 0;
static void led(uint8_t rgb){
  digitalWrite(LED_RED,   (rgb & 1) ? LOW : HIGH);
  digitalWrite(LED_GREEN, (rgb & 2) ? LOW : HIGH);
  digitalWrite(LED_BLUE,  (rgb & 4) ? LOW : HIGH);
}
static void flash(uint8_t rgb, uint32_t ms){ g_flash = rgb; g_flashUntil = millis() + ms; }

static bool questReady(){
  uint16_t h = g_questConn;
  if(h == BLE_CONN_HANDLE_INVALID) return false;
  BLEConnection* c = Bluefruit.Connection(h);
  return c && c->connected() && c->secured();   // HID は暗号化（ペアリング）済みの相手にだけ送る
}

static void updateLed(uint32_t now){
  if(now < g_flashUntil){ led(g_flash); return; }
  bool q = questReady(), h = g_hgReady;
  if(q && h)  led(6);                                    // 水色
  else if(q)  led(2);                                    // 緑
  else if(h)  led(((now / 500) % 2) ? 4 : 0);            // 青点滅
  else        led(((now / 500) % 2) ? 1 : 0);            // 赤点滅
}

// ================================================================ Quest：HID
// 入力レポート ID=1（16 バイト）
//   [0-7] X,Y,Z,Rz（16bit, 中央 0x8000）[8-9] ブレーキ 10bit [10-11] アクセル 10bit
//   [12] ハット（0=押されていない）[13-14] ボタン 15bit [15] Record
// releaseX = true: ケーブルが外れていても X を離したレポートにする（ハートビートで押し直す前に使う）
static void sendGamepad(bool releaseX = false){
  if(!questReady()) return;
  uint8_t r[16];
  memset(r, 0, sizeof(r));
  r[1] = 0x80; r[3] = 0x80; r[5] = 0x80; r[7] = 0x80;
  r[8]  = (uint8_t)(g_brake & 0xFF);
  r[9]  = (uint8_t)((g_brake >> 8) & 0x03);
  // A・B（パルス）に、ケーブルの状態 X を重ねる。X は g_buttons に入れない（updateButtons() が 0 に戻すため）
  uint16_t b = g_buttons | ((!releaseX && (!g_usbNow || g_forceCableOff)) ? BTN_X : 0);
  r[13] = (uint8_t)(b & 0xFF);
  r[14] = (uint8_t)(b >> 8);
  blehid.inputReport(g_questConn, 1, r, sizeof(r));
}

// ボタンを BUTTON_HOLD_MS だけ押す（離すのは updateButtons()）
static void pressButton(uint16_t mask){
  g_buttons |= mask;
  g_btnUpAt = millis() + BUTTON_HOLD_MS;
  sendGamepad();
}
static void updateButtons(uint32_t now){
  if(g_buttons && (int32_t)(now - g_btnUpAt) >= 0){
    g_buttons = 0;
    sendGamepad();
  }
}

// 出力レポート ID=3（Xbox 振動）→ Phase（§3-3）
// [enable][trigL][trigR][strong][weak][duration][delay][loop]
// Quest は (phase + 1) / 10 を送る → strong ≒ (phase + 1) × 10（Phase0=10 … Phase8=90）
// strong ≒ 0（0.0）はフォーカス喪失などで振動が止まっただけなので無視し、今の Phase を保つ
static void outputReportCb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len){
  const uint8_t* d = data;
  if(len == 9 && d[0] == 3){ d++; len = 8; }   // ID 込みで届いた場合に備える
  if(len < 5) return;
  uint8_t strong = d[3], weak = d[4];
  uint8_t src = (strong > 0 || weak == 0) ? strong : weak;   // 強が 0 で弱だけ来ていたら弱を採用
  int p = (src + 5) / 10 - 1;
  if(p < 0 || p > 8) return;   // 0.0（停止）と範囲外は無視
  g_phaseRx = (int8_t)p;
}
static void outDummyCb(uint16_t, BLECharacteristic*, uint8_t*, uint16_t){}

static void onPeriphConnect(uint16_t h){ g_questConn = h; }
static void onPeriphDisconnect(uint16_t h, uint8_t reason){
  if(h == g_questConn) g_questConn = BLE_CONN_HANDLE_INVALID;
}

// ================================================================ ヘッドギア：GATT Central
static void scanCb(ble_gap_evt_adv_report_t* report){
  uint8_t name[32] = {0};
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, name, sizeof(name) - 1)
     && strcmp((const char*)name, HEADGEAR_NAME) == 0){
    Bluefruit.Central.connect(report);
  } else {
    Bluefruit.Scanner.resume();
  }
}
// HEART の Notify（BLE タスク）。数を増やすだけで、モータ（DRV8833）には触らない
static void onHeartNotify(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len){
  (void)chr;
  if(len < 2 || data[0] != HEART_MSG) return;
  uint8_t seq = data[1];
  if(g_beatHaveSeq){
    uint8_t gap = (uint8_t)(seq - g_beatSeq - 1);
    if(gap > 0 && gap < 50){ g_beatMissed += gap; g_beatGapEvt += gap; }   // 50 以上はヘッドギアの再起動とみなす
  }
  g_beatHaveSeq = true;
  g_beatSeq = seq;
  g_beatRx++;
}

static void onCentralConnect(uint16_t h){
  if(!hgSvc.discover(h) || !hgCmd.discover()){
    Bluefruit.disconnect(h);   // 名前は合っているがサービスがない → 切ってスキャンし直す
    return;
  }
  // HEART がない（台湾版のヘッドギア）ときは、心拍なしでそのまま動かす
  g_hgHeartOk = hgHeart.discover() && hgHeart.enableNotify();
  g_hgConn = h;
  g_hgReady = true;
  g_hgJustConnected = true;
}
static void onCentralDisconnect(uint16_t h, uint8_t reason){
  if(h == g_hgConn){ g_hgConn = BLE_CONN_HANDLE_INVALID; g_hgReady = false; g_hgHeartOk = false; g_beatHaveSeq = false; }
}

static bool sendHeadgear(const uint8_t* buf){
  if(!g_hgReady) return false;
  return hgCmd.write(buf, CMD_LEN) == CMD_LEN;   // Write Without Response
}
static void sendCmd00_Reset(){
  uint8_t b[CMD_LEN] = { CMD_RESET, 0, 0, 0, 0, 0, 0 };
  bool ok = sendHeadgear(b);
  Serial.printf("[HG] 0x00 リセット＆停止 %s\n", ok ? "送信" : "（頭が未接続）");
}
static void sendCmd01_Phase2Trigger(){
  uint8_t b[CMD_LEN] = { CMD_PHASE2, 0, 0, 0, 0, 0, 0 };
  bool ok = sendHeadgear(b);
  Serial.printf("[HG] 0x01 Phase2 トリガー %s\n", ok ? "送信" : "（頭が未接続）");
}
// 0x03：今の Phase（SIGGRAPH 版）。変化時・接続時はログを出し、1 秒ごとの送り直しでは出さない
static uint32_t g_lastPhaseSync = 0;
static void sendCmd03_Phase(bool log){
  uint8_t b[CMD_LEN] = { CMD_PHASE, (uint8_t)g_phase, 0, 0, 0, 0, 0 };
  bool ok = sendHeadgear(b);
  g_lastPhaseSync = millis();
  if(log) Serial.printf("[HG] 0x03 Phase=%d %s\n", g_phase, ok ? "送信" : "（頭が未接続）");
}

// ================================================================ 圧力の送信（§3-5、loop 側）
static void interpolate4to6(const uint8_t calib[4], uint8_t servo[6]){
  servo[0] =  calib[0];
  servo[1] = (calib[0] + calib[1]) / 2;
  servo[2] =  calib[1];
  servo[3] =  calib[2];
  servo[4] = (calib[2] + calib[3]) / 2;
  servo[5] =  calib[3];
}
static void copyCalib(uint8_t calib[4]){
  for(int i = 0; i < PRESSURE_CH; i++) calib[i] = g_calibOut[i];
}
static void sendPressureToQuest(const uint8_t calib[4]){
  uint16_t sum = (uint16_t)calib[0] + calib[1] + calib[2] + calib[3];   // 0〜400
  uint32_t v = (uint32_t)sum * 1023 / (PRESSURE_OUT_MAX * PRESSURE_CH);  // → 0〜1023
  g_brake = (v > 1023) ? 1023 : (uint16_t)v;
  sendGamepad();
}
static void sendServoValues_Cmd02(const uint8_t calib[4]){
  uint8_t b[CMD_LEN];
  b[0] = CMD_SERVO;
  interpolate4to6(calib, &b[1]);
  sendHeadgear(b);
}

// ================================================================ Phase 管理（§3-3）
static void checkPhaseTransition(){
  int8_t rx = g_phaseRx;
  if(rx >= 0){ g_phaseRx = -1; g_phase = rx; }
  if(g_phase == g_prevPhase) return;
  int from = g_prevPhase, to = g_phase;
  g_prevPhase = g_phase;
  g_phaseNow = (int8_t)to;
  Serial.printf("[PHASE] %d → %d\n", from, to);
  flash(7, 300);

  // 先に今の Phase を知らせる（ミラー・頭部衝撃・椅子はこれで動く。遅れを小さくするため最初に送る）
  sendCmd03_Phase(true);

  // Phase2 へ → シーケンス開始／Phase3・4 へ → 0x02 の送信で追従開始／それ以外へ → リセット＆停止
  if(to == 2)                  sendCmd01_Phase2Trigger();
  else if(to != 3 && to != 4)  sendCmd00_Reset();
  if(to != 3 && to != 4 && g_brake != 0){ g_brake = 0; sendGamepad(); }   // 圧力表示を 0 に戻す

  // Phase1（体験の始まり）に入ったら、圧力のゼロ点・最大値をリセットする（向きは残す）
  if(to == 1) g_rezeroReq = true;

  // Phase3 に入った時点ですでに台座から外れていれば、すぐ B を送る（Phase2 中に持ち上げられた場合）
  if(to == 3 && !g_usbNow) sendDetachB("Phase3 開始時にすでに外れていた");
}

// ヘッドギアに（再）接続したとき、Phase に合わせて頭の状態を揃える
static void syncHeadgearOnConnect(){
  if(!g_hgJustConnected) return;
  g_hgJustConnected = false;
  Serial.printf("[HG] ヘッドギアに接続（心拍 %s）\n", g_hgHeartOk ? "購読 OK" : "なし ★ HEART が見つからない");
  sendCmd03_Phase(true);
  if(!(g_phase >= 2 && g_phase <= 4)) sendCmd00_Reset();
}

// ================================================================ 電源（台座外れ検出）
static bool usbPowered(){
  uint8_t sdOn = 0;
  sd_softdevice_is_enabled(&sdOn);
  if(!sdOn) return NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk;   // BLE 開始前（setup の最初）は直接読む
  uint32_t st = 0;
  sd_power_usbregstatus_get(&st);
  return st & POWER_USBREGSTATUS_VBUSDETECT_Msk;
}
static bool charging(){ return digitalRead(PIN_CHG) == LOW; }

// USB 給電の変化を見る。Phase3 で「切れた」→ BUTTON_B を 1 回。どの Phase でも X（ケーブルの状態）を送り直す
static void sendDetachB(const char* why){
  pressButton(BTN_B);
  Serial.printf("[HID] BUTTON_B（台座外れ：%s）\n", why);
  flash(3, 400);
}
static void updatePower(uint32_t now){
  static bool usbRaw = false, first = true;
  static uint32_t rawSince = 0;
  bool r = usbPowered();
  if(first){ g_usbNow = usbRaw = r; first = false; }
  if(r != usbRaw){ usbRaw = r; rawSince = now; }
  if(usbRaw != g_usbNow && now - rawSince >= USB_STABLE_MS){
    g_usbNow = usbRaw;
    Serial.printf("[PWR] USB 給電が%s\n", g_usbNow ? "来た（台座に付けた）" : "切れた（台座から外した）");
    if(g_usbNow) hapticStart(true);   // SIGGRAPH 版：給電が来た合図に 1 回鳴らす
    if(!g_usbNow){
      g_imuIgnoreUntil = now + IMU_IGNORE_MS;   // 外れた衝撃を投げと誤検出しない
      if(g_phase == 3){ sendDetachB("外れた瞬間"); return; }   // B のレポートに X も載る
    }
    sendGamepad();   // X（ケーブルの状態）が変わったことをすぐ知らせる
  }
}

// Quest との暗号化が済んだ瞬間に 1 回送る（ケーブルの状態をハートビートを待たずに知らせる）
static void syncQuestOnSecured(){
  static bool wasReady = false;
  bool r = questReady();
  if(r && !wasReady) sendGamepad();
  wasReady = r;
}

// ================================================================ Phase4：投げ（§3-4、loop 側）
static void checkThrow_Phase4(){
  if(!g_throwTrigger) return;
  g_throwTrigger = false;
  pressButton(BTN_A);
  Serial.println("[HID] BUTTON_A（投げ）");
  flash(4, 400);
}

// ================================================================ 心拍の振動：DRV8833 ＋ 振動モータ（SIGGRAPH 版、loop 側）
// DRV8833 の入力（xIN1 / xIN2）と動き:
//   1 / PWM の反転 … 正転・スロー減衰（オフの間はブレーキ。PWM の幅と回転がほぼ比例する）
//   PWM の反転 / 1 … 逆転・スロー減衰（波形のブレーキに使う）
//   0 / 0          … 惰性（止まっているとき）
// ここでは「IN2 を LOW にしている割合 = 強さ」になるよう、IN2 に (1 - 強さ) の PWM を出す。
static const uint32_t MOTOR_PWM_TOKEN = 0x4D4F5452;   // HwPWM3 の持ち主の印（analogWrite と取り合わないように）
static bool    g_motorOk   = false;   // PWM を確保できた
static int16_t g_motorLast = 0;       // 最後に出した強さ（-255〜255）
// モータが動いている間とその直後は電池の電圧が下がるので、残量の測定を待たせる
static const uint32_t MOTOR_SETTLE_MS = 200;
static volatile uint32_t g_motorQuietAt = 0;   // この時刻以降なら電池を測ってよい

// 今の電池電圧で、強さ 255 のときの PWM の幅（0〜1）。電池電圧が測れていなければ 4.2V とみなす（弱い側）
static float motorDutyMax(){
  float v = g_vbat;
  if(v < 3.0f || v > 4.5f) v = 4.2f;
  return constrain(MOTOR_RATED_V / v, 0.0f, 1.0f);
}

static void initHaptic(){
  if(!HwPWM3.takeOwnership(MOTOR_PWM_TOKEN)){ Serial.println("[MOTOR] HwPWM3 を確保できない ★ 心拍の振動は出ません"); return; }
  HwPWM3.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_1);   // 16MHz
  HwPWM3.setMaxValue(MOTOR_PWM_TOP);
  HwPWM3.addPin(PIN_MOTOR_IN1);
  HwPWM3.addPin(PIN_MOTOR_IN2);
  HwPWM3.writePin(PIN_MOTOR_IN1, 0);
  HwPWM3.writePin(PIN_MOTOR_IN2, 0);
  g_motorOk = true;
  g_motorLast = 0;
  Serial.printf("[MOTOR] DRV8833 D4/D5・PWM 20kHz・定格 %.1fV（電池 %.2fV → 幅の上限 %.0f%%）\n",
                MOTOR_RATED_V, g_vbat, motorDutyMax() * 100.0f);
}

// 強さ -255〜255（正 = 回す、負 = 逆向き＝ブレーキ、0 = 惰性）
static void drvLevel(int16_t level){
  if(!g_motorOk) return;
  level = constrain(level, -255, 255);
  if(level != 0) g_motorQuietAt = millis() + MOTOR_SETTLE_MS;   // 止めてから 200ms は測らない
  if(level == g_motorLast) return;
  g_motorLast = level;
  if(level == 0){
    HwPWM3.writePin(PIN_MOTOR_IN1, 0);
    HwPWM3.writePin(PIN_MOTOR_IN2, 0);
    return;
  }
  uint16_t on  = (uint16_t)(MOTOR_PWM_TOP * motorDutyMax() * (abs(level) / 255.0f) + 0.5f);
  uint16_t off = MOTOR_PWM_TOP - min(on, (uint16_t)MOTOR_PWM_TOP);   // 反対側の入力を HIGH にしておく割合
  if(level > 0){ HwPWM3.writePin(PIN_MOTOR_IN1, MOTOR_PWM_TOP); HwPWM3.writePin(PIN_MOTOR_IN2, off); }
  else         { HwPWM3.writePin(PIN_MOTOR_IN1, off); HwPWM3.writePin(PIN_MOTOR_IN2, MOTOR_PWM_TOP); }
}

// 波形の再生（旧 M5 版 processHaptic() と同じ考え方。区間ごとに直線で強さを変える）
static int      g_hapSeg    = -1;      // 再生中の区間（-1 = 止まっている）
static uint32_t g_hapSegT   = 0;       // 今の区間の開始時刻
static uint32_t g_hapLastT  = 0;
static bool     g_hapManual = false;   // シリアル h で鳴らした（Phase に関係なく最後まで鳴らす）
static uint32_t g_beatPlayed = 0;      // 鳴らした回数（表示用）

static void updateHaptic(uint32_t now){
  if(g_hapSeg < 0) return;
  // ★ 符号付きで見る（2026-11-23）。hapticStart() は自分で millis() を取るので、loop の先頭で取った now より新しいことがある。
  //   符号なしだと 3ms 以上ずれたときに経過時間が約 49 日になり、波形を「もう終わった」として 1 拍まるごと鳴らさなかった
  if((int32_t)(now - g_hapLastT) < (int32_t)HAPTIC_STEP_MS) return;
  g_hapLastT = now;
  int32_t elS = (int32_t)(now - g_hapSegT);
  uint32_t el = elS > 0 ? (uint32_t)elS : 0;
  while(g_hapSeg < HAPTIC_SEGS && el >= HAPTIC_WAVE[g_hapSeg].ms){
    el        -= HAPTIC_WAVE[g_hapSeg].ms;
    g_hapSegT += HAPTIC_WAVE[g_hapSeg].ms;
    g_hapSeg++;
  }
  if(g_hapSeg >= HAPTIC_SEGS){ g_hapSeg = -1; g_hapManual = false; drvLevel(0); return; }
  const HapticSeg& s = HAPTIC_WAVE[g_hapSeg];
  if(s.brake){ drvLevel(s.from); return; }   // ブレーキは倍率をかけない
  int32_t lv = s.from + ((int32_t)s.to - s.from) * (int32_t)el / (int32_t)s.ms;
  drvLevel((int16_t)(lv * HAPTIC_GAIN));
}
// 鳴っている途中で次の拍が来たら最初からやり直す（旧版と同じ）
static void hapticStart(bool manual){
  uint32_t now = millis();
  g_hapSeg    = 0;
  g_hapSegT   = now;
  g_hapLastT  = now - HAPTIC_STEP_MS;
  g_hapManual = manual;
  g_beatPlayed++;
  updateHaptic(now);
}
static void hapticStop(){
  g_hapSeg = -1;
  g_hapManual = false;
  drvLevel(0);
}
static bool hapticPhase(){ return g_phase == 3 || g_phase == 4; }

// 届いた心拍を見て、Phase3・4 なら鳴らす。Phase3・4 から出たら止める
static void checkHeartbeat(){
  static uint8_t seen = 0;
  uint8_t rx = g_beatRx;
  if(rx != seen){
    uint8_t n = (uint8_t)(rx - seen);
    seen = rx;
    uint8_t gap = g_beatGapEvt; g_beatGapEvt = 0;
    bool play = hapticPhase();
    if(play) hapticStart(false);
    if(g_printBeat){
      Serial.printf("[HEART] #%u を受信", g_beatSeq);
      if(n > 1) Serial.printf("（%u 拍まとめて）", n);
      if(gap)   Serial.printf("（★ その前の %u 拍が届いていない）", gap);
      if(play)  Serial.println(" → 鳴らした");
      else      Serial.printf(" → Phase=%d なので鳴らさない（鳴らすのは Phase3・4 だけ）\n", g_phase);
    }
  }
  if(g_hapSeg >= 0 && !g_hapManual && !hapticPhase()) hapticStop();
}

// ================================================================ loop1 側：圧力キャリブレーション（§3-5）
// 押したときの向き（増える / 減る）は 4ch で揃っている前提で、全 ch 共通の 1 つの向き p_sign を持つ。
// 向きは一発で決め切らない：
//   ① 最初の押し込み（どれかの ch が基準値から PEAK_MIN_THRESH 以上動いた）で、一番大きく動いた ch の向きに仮決め
//   ② その後の押し込みを再検証する
//      ・仮決めと同じ向き → 一致。DIR_CONFIRM_PRESSES 回で確定（以後は変えない）
//      ・逆向き（今の向きでは「基準値より下」で 0 に張り付くしかない）→ DIR_FLIP_PRESSES 回続いたら反転
//   「押し込み」は 1 回の握りを 1 回と数える（|delta| が PRESS_RELEASE_THRESH を下回るまでは同じ押し込み）。
static uint16_t p_base[PRESSURE_CH];   // 起動時のゼロ点（ADC raw）
static int16_t  p_peak[PRESSURE_CH];   // 正規化後の最大値
static volatile uint16_t p_raw[PRESSURE_CH];   // 表示用
static volatile int8_t  p_sign      = 0;       // 全 ch 共通の向き：+1 / -1、0=未決定
static volatile bool    p_dirLocked = false;   // 再検証が終わって確定した
static uint8_t  p_agree = 0, p_disagree = 0;
static bool     p_pressing = false;

// ---- 圧力の向きの保存（2026-11-11）
// 新しいファームを書き込んだ直後の起動では、これまでどおり押し込みで向きを決め、確定したら内蔵フラッシュに保存する。
// 次の起動からは保存した向きをそのまま使い、もう変えない（ゼロ点・最大値は起動のたび・Phase1 のたびに測り直す）。
// 「新しいファームかどうか」はプログラム自身の CRC で見分ける（中身が同じファームを書き直しても向きは残る。やり直すときはシリアル D）
#define PDIR_FILE  "/nozui_pdir.bin"
#define PDIR_MAGIC 0x52494450u          // "PDIR"
struct PDirRec { uint32_t magic; uint32_t fwCrc; int8_t sign; uint8_t pad[3]; };
extern uint32_t __etext, __data_start__, __data_end__;   // リンカが決める：プログラムの終わり・初期値つき変数の範囲
static uint32_t       g_fwCrc = 0;
static volatile int8_t g_savedSign = 0;  // 保存されていた向き（0 = ない／ファームが変わった）
static volatile bool   g_dirSaveReq = false;   // loop1 → loop：確定したので保存して
static volatile bool   g_dirForgetReq = false; // シリアル D：保存した向きを捨てて決め直す

static uint32_t firmwareCrc(){            // 0x27000（アプリの先頭）からプログラムの終わり＋初期値データまで
  const uint8_t* p = (const uint8_t*)0x27000;
  uint32_t len = (uint32_t)&__etext - 0x27000 + ((uint32_t)&__data_end__ - (uint32_t)&__data_start__);
  uint32_t crc = 0xFFFFFFFFu;
  for(uint32_t i = 0; i < len; i++){
    crc ^= p[i];
    for(int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
  }
  return ~crc;
}
static void loadPressureDir(){
  g_fwCrc = firmwareCrc();
  g_savedSign = 0;
  PDirRec r;
  File f(InternalFS);
  if(f.open(PDIR_FILE, FILE_O_READ)){
    bool ok = f.read(&r, sizeof(r)) == sizeof(r);
    f.close();
    if(ok && r.magic == PDIR_MAGIC && r.fwCrc == g_fwCrc && (r.sign == 1 || r.sign == -1)) g_savedSign = r.sign;
  }
}
static bool savePressureDir(int8_t sign){
  PDirRec r = { PDIR_MAGIC, g_fwCrc, sign, {0, 0, 0} };
  InternalFS.remove(PDIR_FILE);
  File f(InternalFS);
  if(!f.open(PDIR_FILE, FILE_O_WRITE)) return false;
  bool ok = f.write((const uint8_t*)&r, sizeof(r)) == sizeof(r);
  f.close();
  if(ok) g_savedSign = sign;
  return ok;
}
// 保存してある向きを使う（loop1 の起動時・D の後以外）
static void applySavedDir(){
  int8_t s = g_savedSign;
  if(!s) return;
  p_sign = s; p_dirLocked = true; p_agree = DIR_CONFIRM_PRESSES; p_disagree = 0;
  for(int ch = 0; ch < PRESSURE_CH; ch++) g_calibReady[ch] = true;
}

static void resetPeaks(){
  for(int ch = 0; ch < PRESSURE_CH; ch++) p_peak[ch] = PEAK_MIN_THRESH;
}

static void setCalibReady(bool r){
  for(int ch = 0; ch < PRESSURE_CH; ch++) g_calibReady[ch] = r;
}

// ゼロ点を測り直し、最大値を初期値に戻す（向き p_sign・p_dirLocked はそのまま）
static void rezeroPressure(){
  for(int ch = 0; ch < PRESSURE_CH; ch++){
    uint32_t sum = 0;
    for(int i = 0; i < CALIB_SAMPLES; i++){ sum += analogRead(PIN_PRESSURE[ch]); delay(5); }
    p_base[ch] = (uint16_t)(sum / CALIB_SAMPLES);
    g_calibOut[ch] = 0;
  }
  resetPeaks();
  p_pressing = false;
}

// 起動時・シリアル 'c'：ゼロ点と最大値に加えて、向きも未決定に戻す
static void calibrateBaseline(){
  rezeroPressure();
  p_sign = 0; p_dirLocked = false;
  p_agree = p_disagree = 0;
  setCalibReady(false);
}

// 押し込みを 1 回ずつ検出して、向きの仮決め・再検証・反転を行う
static void updateDirection(const int16_t delta[PRESSURE_CH]){
  int best = 0;
  for(int ch = 1; ch < PRESSURE_CH; ch++) if(abs(delta[ch]) > abs(delta[best])) best = ch;
  int16_t mag = (int16_t)abs(delta[best]);

  if(p_pressing){                                   // 押し込みの途中 → 離されるまで数えない
    if(mag < PRESS_RELEASE_THRESH) p_pressing = false;
    return;
  }
  if(mag <= PEAK_MIN_THRESH) return;
  p_pressing = true;                                // 新しい押し込み
  int8_t dir = (delta[best] > 0) ? +1 : -1;

  if(p_sign == 0){                                  // ① 仮決め
    p_sign = dir; p_agree = 1; p_disagree = 0;
    setCalibReady(true);
    g_dirEvt = 'p';
    return;
  }
  if(p_dirLocked) return;

  if(dir == p_sign){                                // ② 一致
    p_disagree = 0;
    if(++p_agree >= DIR_CONFIRM_PRESSES){ p_dirLocked = true; g_dirEvt = 'L'; g_dirSaveReq = true; }
  } else if(++p_disagree >= DIR_FLIP_PRESSES){      // ② 逆向きが続いた → 反転
    p_sign = dir; p_agree = 1; p_disagree = 0;
    resetPeaks();                                   // 逆向きで測った最大値は捨てる
    g_dirEvt = 'F';
  }
}

static void updateCalib(){
  int16_t delta[PRESSURE_CH];
  for(int ch = 0; ch < PRESSURE_CH; ch++){
    uint16_t raw = analogRead(PIN_PRESSURE[ch]);
    p_raw[ch] = raw;
    delta[ch] = (int16_t)raw - (int16_t)p_base[ch];
  }
  updateDirection(delta);

  int8_t sign = p_sign;
  for(int ch = 0; ch < PRESSURE_CH; ch++){
    if(sign == 0){ g_calibOut[ch] = 0; continue; }   // 向きが決まるまでは 0
    int16_t norm = delta[ch] * sign;                // 押すと正になる向きに統一
    if(norm < 0) norm = 0;
    if(norm > p_peak[ch]) p_peak[ch] = norm;        // 動的ピーク（下がらない）

    int32_t out = (int32_t)norm * PRESSURE_OUT_MAX / p_peak[ch];
    if(out > PRESSURE_OUT_MAX) out = PRESSURE_OUT_MAX;
    g_calibOut[ch] = (uint8_t)out;
  }
}

static float readVbat(){
  analogReference(AR_INTERNAL_2_4);
  analogSampleTime(40);   // 分圧の抵抗が大きいので取り込み時間を長く
  uint32_t sum = 0;
  for(int i = 0; i < 8; i++) sum += analogRead(PIN_VBAT);
  analogReference(AR_DEFAULT);   // 圧力センサ用の設定に戻す
  analogSampleTime(3);
  return sum / 8.0f / 4096.0f * 2.4f * VBAT_DIV;
}

// 電池電圧の切り分け（シリアル V）。loop1 で測って結果を残し、loop が表示する（ADC を 2 つのタスクで取り合わないため）
static volatile bool  g_vbatDiagReq = false, g_vbatDiagDone = false;
static volatile float g_vdA = 0, g_vdB = 0, g_vdC = 0;
static volatile uint16_t g_vdRawA = 0, g_vdRawB = 0;
static uint16_t readVbatRaw(int n){
  analogReference(AR_INTERNAL_2_4);
  analogSampleTime(40);
  uint32_t sum = 0;
  for(int i = 0; i < n; i++) sum += analogRead(PIN_VBAT);
  analogReference(AR_DEFAULT);
  analogSampleTime(3);
  return (uint16_t)(sum / n);
}
static float rawToVbat(uint16_t raw){ return raw / 4096.0f * 2.4f * VBAT_DIV; }
static void runVbatDiag(){
  // A：いつもの測り方（圧力を読んだ直後）
  updateCalib();
  g_vdRawA = readVbatRaw(8);
  g_vdA = rawToVbat(g_vdRawA);
  // B：ADC を 50ms 休ませてから 32 回平均
  delay(50);
  g_vdRawB = readVbatRaw(32);
  g_vdB = rawToVbat(g_vdRawB);
  // C：起動時と同じ関数
  g_vdC = readVbat();
  g_vbatDiagDone = true;
}

static void runCalibration(uint32_t now){
  static uint32_t lastT = 0, lastVbat = 0;
  if(g_vbatDiagReq){ g_vbatDiagReq = false; runVbatDiag(); }
  if(now - lastT >= PRESSURE_MS){
    lastT = now;
    updateCalib();
  }
  // 電池電圧も ADC を使うので同じタスクで測る（投げの判定中は止めない）
  // モータが動いている間・止めて 200ms 以内は測らない（次の回に回す）
  if(now - lastVbat >= VBAT_MS && g_phaseNow != 4 && g_motorLast == 0 && (int32_t)(now - g_motorQuietAt) >= 0){
    lastVbat = now;
    g_vbat = readVbat();
  }
}

// ================================================================ loop1 側：IMU ステートマシン（§3-4、Phase4 のみ）
enum { ST_IDLE, ST_FALLING, ST_FLYING, ST_CLASSIFY, ST_COOLDOWN };
static const char* ST_NAME[] = { "IDLE", "FALL", "FLY", "CLS", "COOL" };
static int      g_state = ST_IDLE;
static uint32_t g_t0 = 0, g_lastSwing = 0, g_lastSample = 0;
static int      g_endN = 0;
static bool     g_swung = false;
static float    g_peak = 0;            // 自由落下が終わってからの衝撃のピーク [G]
static uint32_t g_stillSince = 0;
static bool     g_throwInFlight = false, g_slipInFlight = false;

static void enter(int s){ g_state = s; g_t0 = millis(); }

static void onThrowEvent(){ g_throwInFlight = true; g_imuEvt = 't'; }
static void onSlipEvent() { g_slipInFlight  = true; g_imuEvt = 's'; }
static void onLandEvent(){                 // 床に落ちた
  g_imuPeak = g_peak;
  g_imuEvt = 'l';
  if(g_throwInFlight || g_slipInFlight){   // ルートA（t→l）/ ルートB（s→l）
    g_throwTrigger  = true;
  }
  g_throwInFlight = g_slipInFlight = false;
}
static void onCatchEvent(){                // 手でキャッチした → 何も送らない
  g_imuPeak = g_peak;
  g_imuEvt = 'c';
  g_throwInFlight = g_slipInFlight = false;
}

static void resetImu(){
  g_state = ST_IDLE; g_endN = 0; g_swung = false;
  g_throwInFlight = g_slipInFlight = false;
  g_lastSample = 0;
}

static void runIMU(uint32_t now){
  if(now - g_lastSample < SAMPLE_MS) return;
  g_lastSample = now;

  float ax = imu.readFloatAccelX(), ay = imu.readFloatAccelY(), az = imu.readFloatAccelZ();
  float gx = imu.readFloatGyroX(),  gy = imu.readFloatGyroY(),  gz = imu.readFloatGyroZ();
  float aRaw = sqrtf(ax*ax + ay*ay + az*az);   // [G]
  float g    = sqrtf(gx*gx + gy*gy + gz*gz);   // [deg/s]
  float w    = g * 0.0174533f;
  float spin = w * w * R_FROM_CENTER_M / 9.81f;   // 回転による遠心力を差し引く
  if(spin > SPIN_MAX_SUB_G) spin = SPIN_MAX_SUB_G;
  float a = aRaw - spin; if(a < 0) a = 0;

  // 台座から外した直後・Phase4 開始直後は判定しない（衝撃を振りかぶりや落下と見なさない）
  if((int32_t)(g_imuIgnoreUntil - now) > 0){ g_state = ST_IDLE; g_endN = 0; return; }

  if(aRaw > SWING_G) g_lastSwing = now;

#if DEBUG_IMU
  if(g_state != ST_IDLE && g_state != ST_COOLDOWN)
    Serial.printf("%lu,%s,%.2f,%.2f,%.1f\n", (unsigned long)(now - g_t0), ST_NAME[g_state], aRaw, a, g);
#endif

  switch(g_state){
  case ST_IDLE:
    if(a < FREEFALL_G){ g_swung = (now - g_lastSwing) < SWING_WINDOW_MS; enter(ST_FALLING); }
    break;
  case ST_FALLING:
    if(a >= FREEFALL_G){ enter(ST_IDLE); break; }
    if(g_swung && now - g_t0 >= THROW_FF_MS)       { onThrowEvent(); enter(ST_FLYING); g_endN = 0; }
    else if(!g_swung && now - g_t0 >= SLIP_FF_MS)  { onSlipEvent();  enter(ST_FLYING); g_endN = 0; }
    break;
  case ST_FLYING:
    if(a > FLIGHT_END_G){
      if(++g_endN >= FLIGHT_END_N){ g_peak = aRaw; g_stillSince = 0; enter(ST_CLASSIFY); }
    } else g_endN = 0;
    break;
  case ST_CLASSIFY: {                      // 床に落ちたか、キャッチしたか
    uint32_t dt = now - g_t0;
    if(dt <= PEAK_WINDOW_MS && aRaw > g_peak) g_peak = aRaw;
    if(g_peak >= LAND_HARD_G){ onLandEvent(); enter(ST_COOLDOWN); break; }
    bool still = fabsf(aRaw - 1.0f) < STILL_TOL_G && g < STILL_GYRO_DPS;
    if(still){
      if(!g_stillSince) g_stillSince = now;
      if(now - g_stillSince >= STILL_MS){
        if(g_peak >= LAND_SOFT_G) onLandEvent();   // 衝撃のあと止まった → 柔らかい床
        else                      onCatchEvent();  // 弱い衝撃のまま止まった → そっと受け止めた
        enter(ST_COOLDOWN); break;
      }
    } else g_stillSince = 0;
    if(dt >= CATCH_MS && !g_stillSince){ onCatchEvent(); enter(ST_COOLDOWN); }   // 手の中で動いている
    else if(dt >= CATCH_MS * 3)        { onCatchEvent(); enter(ST_COOLDOWN); }   // 念のための打ち切り
  } break;
  case ST_COOLDOWN:
    if(now - g_t0 >= COOLDOWN_MS) enter(ST_IDLE);
    break;
  }
}

// ================================================================ loop1（「Core1」相当のタスク）
static void setup1(){
  analogReadResolution(12);
  for(int ch = 0; ch < PRESSURE_CH; ch++) pinMode(PIN_PRESSURE[ch], INPUT_PULLUP);
  calibrateBaseline();
  applySavedDir();
}

static void loop1(){
  static bool inited = false;
  if(!inited){ setup1(); inited = true; }

  if(g_dirForgetReq){ g_dirForgetReq = false; g_recalReq = true; }
  if(g_recalReq){ g_recalReq = false; g_rezeroReq = false; calibrateBaseline(); applySavedDir(); }   // ゼロ点の測り直し。向きは保存済みならそれを使う
  if(g_rezeroReq){ g_rezeroReq = false; rezeroPressure(); g_dirEvt = 'z'; }      // Phase1：ゼロ点・最大値だけ（向きは残す）

  uint32_t now = millis();
  runCalibration(now);

  static bool wasP4 = false;
  bool p4 = (g_phaseNow == 4) && g_imuOk;
  if(p4 != wasP4){
    resetImu();
    if(p4){
      uint32_t until = now + IMU_IGNORE_MS;
      if((int32_t)(until - g_imuIgnoreUntil) > 0) g_imuIgnoreUntil = until;
    }
    wasP4 = p4;
  }
  if(p4) runIMU(now);

  delay(1);   // 他のタスク（BLE / loop）に譲る
}

// ================================================================ 電池残量（SIGGRAPH 版）
// LiPo の電圧 → 残量 % の目安（無負荷のおおよその値。充電中は電圧が高めに出るので % も多めになる）
static const float VBAT_TABLE[][2] = {
  {4.20f, 100}, {4.10f, 90}, {4.00f, 80}, {3.90f, 65}, {3.80f, 40},
  {3.75f, 25},  {3.70f, 12}, {3.60f, 5},  {3.30f, 0},
};
// 充電中は充電電流で端子電圧が持ち上がる。2026-11-10 に実機で測った値：
// 充電 100mA→50mA で 3.948V→3.922V（約 0.5Ω）→ 100mA 充電中は約 0.05V 高く出る
static const float VBAT_CHG_OFFSET = 0.05f;
static float batteryOcv(){
  float v = g_vbat;
  if(v > 0 && usbPowered() && charging()) v -= VBAT_CHG_OFFSET;
  return v;
}
static uint8_t vbatToPercent(float v){
  const int n = sizeof(VBAT_TABLE) / sizeof(VBAT_TABLE[0]);
  if(v >= VBAT_TABLE[0][0]) return 100;
  for(int i = 1; i < n; i++){
    if(v >= VBAT_TABLE[i][0]){
      float v0 = VBAT_TABLE[i][0], v1 = VBAT_TABLE[i - 1][0];
      float p0 = VBAT_TABLE[i][1], p1 = VBAT_TABLE[i - 1][1];
      return (uint8_t)(p0 + (v - v0) / (v1 - v0) * (p1 - p0) + 0.5f);
    }
  }
  return 0;
}
static const uint32_t BAT_UPDATE_MS  = 30000;   // 残量を変えるのは最短でもこの間隔（電圧の揺れで細かく変わらないように）
static const uint32_t HID_BAT_RESEND = 60000;   // HID の電池レポートは変わらなくてもこの間隔で送り直す（Phase3・4 以外）
static volatile uint8_t g_batPct = 0;           // 今知らせている残量 %（BAS の読み出しにもこの値を返す）
static int      g_batOverride = -1;              // シリアル v の確認用（-1 = 実測）
static volatile uint32_t g_basReads = 0;         // Quest が BAS の残量を読みに来た回数
static volatile bool g_basReadEvt = false;
static volatile int8_t g_basCccdEvt = -1;        // BAS の通知の購読が変わった（0 / 1）

// BAS の残量が読まれた（BLE のタスクで呼ばれる）。今の % をその場で返す
static void onBasRead(uint16_t conn, BLECharacteristic* chr, ble_gatts_evt_read_t* req){
  (void)chr; (void)req;
  uint8_t v = g_batPct;
  ble_gatts_rw_authorize_reply_params_t reply;
  memset(&reply, 0, sizeof(reply));
  reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
  reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
  reply.params.read.update = 1;
  reply.params.read.offset = 0;
  reply.params.read.len    = 1;
  reply.params.read.p_data = &v;
  sd_ble_gatts_rw_authorize_reply(conn, &reply);
  g_basReads++;
  g_basReadEvt = true;
}
static void onBasCccd(uint16_t conn, BLECharacteristic* chr, uint16_t value){
  (void)conn; (void)chr;
  g_basCccdEvt = (value & BLE_GATT_HVX_NOTIFICATION) ? 1 : 0;
}

static void initBatteryService(){
  basSvc.begin();
  basLvl.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  basLvl.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  basLvl.setFixedLen(1);
  basLvl.setReadAuthorizeCallback(onBasRead, false);   // その場で返す（遅らせると読み出しが待たされる）
  basLvl.setCccdWriteCallback(onBasCccd, false);
  basLvl.begin();
  basLvl.write8(g_batPct);
}

// HID 入力 ID=4（電池の強さ 0〜255）。Linux は 0 を無視するので最低 1 にする。
// 送れないとき（つながっていない・購読前）も値は特性に書かれるので、Quest が読みに来たらこの値が返る
static void sendBatteryHid(){
  uint8_t v = (uint8_t)max(1, (g_batPct * 255 + 50) / 100);
  blehid.inputReport(questReady() ? g_questConn : BLE_CONN_HANDLE_INVALID, 4, &v, 1);
}

// 残量を決めて、BAS と HID の両方に反映する
static void setBatteryPercent(uint8_t pct, bool notify){
  g_batPct = pct;
  basLvl.write8(pct);
  if(notify && questReady()) basLvl.notify8(g_questConn, pct);   // 購読されていなければ何も起きない
  sendBatteryHid();
}

static void updateBattery(uint32_t now){
  static uint32_t lastChange = 0, lastHid = 0;
  static bool wasReady = false;

  // 切り分け用の表示（BLE のタスクで立てたフラグを loop で出す）
  if(g_basReadEvt){ g_basReadEvt = false; Serial.printf("[BAT] Quest が Battery Service の残量を読んだ（%u%%、通算 %lu 回）← ① の道が生きている\n", g_batPct, (unsigned long)g_basReads); }
  int8_t ce = g_basCccdEvt;
  if(ce >= 0){ g_basCccdEvt = -1; Serial.printf("[BAT] Quest が Battery Service の通知を%s\n", ce ? "購読した ← ① の道が生きている" : "やめた"); }
  bool r = questReady();
  static bool hidSub = false;
  bool hs = r && blehid.inputNotifyEnabled(g_questConn, 4);
  if(hs != hidSub){ hidSub = hs; if(hs) Serial.println("[BAT] Quest が HID の電池レポート（ID=4）を購読した ← ② の道で送れる"); }

  // つながった（暗号化が済んだ）直後に今の値を両方の道で送る
  if(r && !wasReady){ setBatteryPercent(g_batPct, true); lastHid = now; }
  wasReady = r;

  // 残量の更新（シリアル v の確認中はその値のまま）
  if(g_batOverride < 0){
    float v = g_vbat;               // loop1 が 2 秒ごとに更新する（Phase4 中は止まる）
    if(v > 0){
      uint8_t pct = vbatToPercent(batteryOcv());
      if(pct != g_batPct && now - lastChange >= BAT_UPDATE_MS){
        lastChange = now;
        setBatteryPercent(pct, true);
        lastHid = now;
      }
    }
  }
  // Linux は届いた値を覚えるだけなので、Phase3・4 以外でときどき送り直す
  if(r && g_phase != 3 && g_phase != 4 && now - lastHid >= HID_BAT_RESEND){ sendBatteryHid(); lastHid = now; }
}

// ================================================================ シリアル
static void printHelp(){
  Serial.println("コマンド: 0〜8=Phase 設定 / a=Aボタン / u=Bボタン / x=X（ケーブル外れ）強制 ON/OFF / h=心拍の振動を 1 回 / k=心拍の表示 ON/OFF / v=残量の表示確認（100→50→10→実測）/ V=電池電圧の切り分け/ p=圧力表示 / c=圧力のゼロ点測り直し / D=圧力の向きを決め直す（保存し直す） / s=状態 / b=ボンド消去して再起動 / ?=ヘルプ");
}
static void printStatus(){
  Serial.printf("[STAT] Phase=%d Quest=%s 頭=%s USB=%s%s X=%s 電池=%.2fV IMU=%s\n",
                g_phase,
                questReady() ? "OK" : (g_questConn != BLE_CONN_HANDLE_INVALID ? "接続(未暗号化)" : "未接続"),
                g_hgReady ? "OK" : "未接続",
                g_usbNow ? "あり" : "なし", g_usbNow ? (charging() ? "(充電中)" : "(満充電)") : "",
                (!g_usbNow || g_forceCableOff) ? (g_forceCableOff ? "送信中(強制)" : "送信中") : "なし",
                g_vbat, g_imuOk ? "OK" : "NG");
  Serial.printf("[STAT] 電池残量=%u%%%s  BAS: 読まれた %lu 回・通知の購読 %s  HID 電池レポート: 購読 %s\n",
                g_batPct, g_batOverride >= 0 ? "（v で固定中）" : "", (unsigned long)g_basReads,
                questReady() && basLvl.notifyEnabled(g_questConn) ? "あり" : "なし",
                questReady() && blehid.inputNotifyEnabled(g_questConn, 4) ? "あり" : "なし");
  Serial.printf("[STAT] 心拍: 購読=%s 受信=%u（最後の番号 %u・届かなかった %lu）鳴らした=%lu  DRV8833=%s\n",
                g_hgHeartOk ? "OK" : "なし", g_beatRx, g_beatSeq, (unsigned long)g_beatMissed, (unsigned long)g_beatPlayed,
                g_motorOk ? "OK" : "なし");
  Serial.printf("[STAT] 圧力の向き=%s%s（一致 %u / 逆 %u）  raw/base/out:",
                p_sign == 0 ? "未決定" : (p_sign > 0 ? "+（押すと増える）" : "-（押すと減る）"),
                p_sign == 0 ? "" : (p_dirLocked ? " 確定" : " 仮"), p_agree, p_disagree);
  for(int ch = 0; ch < PRESSURE_CH; ch++)
    Serial.printf("  D%d=%u/%u/%u", ch, p_raw[ch], p_base[ch], g_calibOut[ch]);
  Serial.println();
}
static void pollSerial(){
  while(Serial.available()){
    char c = (char)Serial.read();
    if(c >= '0' && c <= '8'){ g_phase = c - '0'; Serial.printf("[CMD] Phase を %d に設定\n", g_phase); }
    else if(c == 'a'){ pressButton(BTN_A); Serial.println("[CMD] BUTTON_A"); }
    else if(c == 'u'){ pressButton(BTN_B); Serial.println("[CMD] BUTTON_B"); }
    else if(c == 'x'){
      g_forceCableOff = !g_forceCableOff;
      Serial.printf("[CMD] X（ケーブル外れ）の強制送信を %s\n", g_forceCableOff ? "ON" : "OFF");
      sendGamepad();
    }
    else if(c == 'k'){ g_printBeat = !g_printBeat; Serial.printf("[CMD] 心拍 1 拍ずつの表示を %s\n", g_printBeat ? "ON" : "OFF"); }
    else if(c == 'h'){ hapticStart(true); Serial.println("[CMD] 心拍の振動を 1 回"); }
    else if(c == 'V'){
      g_vbatDiagReq = true;
      uint32_t t0 = millis();
      while(!g_vbatDiagDone && millis() - t0 < 1000) delay(5);
      g_vbatDiagDone = false;
      Serial.printf("[VBAT] いつもの=%.3fV(raw %u)  50ms 休んでから=%.3fV(raw %u)  起動時と同じ=%.3fV  2 秒ごとの値=%.3fV\n",
                    g_vdA, g_vdRawA, g_vdB, g_vdRawB, g_vdC, g_vbat);
      Serial.printf("[VBAT] USB=%s 充電=%s DCDC=%lu VBAT_ENABLE=%d  起動から %lu 秒\n",
                    usbPowered() ? "あり" : "なし", charging() ? "中" : "していない",
                    (unsigned long)NRF_POWER->DCDCEN, digitalRead(VBAT_ENABLE), (unsigned long)(millis() / 1000));
    }
    else if(c == 'v'){
      static const int SEQ[] = { 100, 50, 10, -1 };
      static int i = -1;
      i = (i + 1) % 4;
      g_batOverride = SEQ[i];
      if(g_batOverride >= 0){ setBatteryPercent((uint8_t)g_batOverride, true); Serial.printf("[CMD] 残量を %d%% に固定して送った（Quest の表示を確認）\n", g_batOverride); }
      else { setBatteryPercent(g_vbat > 0 ? vbatToPercent(batteryOcv()) : g_batPct, true); Serial.printf("[CMD] 残量を実測に戻した（%u%%）\n", g_batPct); }
    }
    else if(c == 'p'){ g_printPressure = !g_printPressure; }
    else if(c == 'c'){ g_recalReq = true; Serial.println(g_savedSign ? "[CMD] 圧力のゼロ点を測り直します（センサに触らないでください）。向きは保存したまま"
                                                                      : "[CMD] 圧力キャリブレーションをやり直します（センサに触らないでください）"); }
    else if(c == 'D'){
      InternalFS.remove(PDIR_FILE); g_savedSign = 0; g_dirForgetReq = true;
      Serial.println("[CMD] 保存した圧力の向きを消しました。センサに触らずに待ち、そのあとどれかを 3 回押すと決め直して保存します");
    }
    else if(c == 's'){ printStatus(); }
    else if(c == 'b'){
      Serial.println("[CMD] ボンドを消去して再起動します。Quest 側でもペアリング解除してください");
      Serial.flush();
      Bluefruit.Periph.clearBonds();
      delay(200);
      NVIC_SystemReset();
    }
    else if(c == '?'){ printHelp(); }
  }
}

// ================================================================ setup / loop（「Core0」相当）
void setup(){
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT); pinMode(LED_BLUE, OUTPUT);
  led(4);

  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);                // ★ LOW のまま（充電中に HIGH は P0.31 を傷める）
  if(CHARGE_100MA){
    pinMode(PIN_CHARGING_CURRENT, OUTPUT);
    digitalWrite(PIN_CHARGING_CURRENT, LOW);     // P0.13＝LOW で 100mA
  }
  pinMode(PIN_CHG, INPUT_PULLUP);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while(!Serial && millis() - t0 < 2000) delay(10);
  Serial.println();
  Serial.println("=== NozuiBrain_SIGGRAPH（脳デバイス・SIGGRAPH 版）===");

#ifdef PIN_LSM6DS3TR_C_POWER
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(10);
#endif
  g_imuOk = (imu.begin() == 0);
  Serial.println(g_imuOk ? "[IMU] OK" : "[IMU] 見つからない ★ 投げ判定できません");

  // 電池電圧を先に 1 回測る（モータの電圧の上限と、Quest に最初に知らせる残量に使う）
  analogReadResolution(12);
  g_vbat = readVbat();
  g_batPct = vbatToPercent(batteryOcv());
  Serial.printf("[BAT] 電池 %.2fV%s → 残量 %u%%\n", g_vbat, (usbPowered() && charging()) ? "（充電中。0.05V 引いて計算）" : "", g_batPct);

  initHaptic();                                  // DRV8833（D4/D5 の PWM）

  Bluefruit.autoConnLed(false);
  Bluefruit.begin(1, 1);                         // Peripheral×1（Quest）＋ Central×1（ヘッドギア）
  Bluefruit.setTxPower(4);
  Bluefruit.setName(BLE_NAME);

  // ---- Peripheral：Quest 向け HID ----
  Bluefruit.Periph.setConnectCallback(onPeriphConnect);
  Bluefruit.Periph.setDisconnectCallback(onPeriphDisconnect);

  // Device Information：PnP ID（0x2A50）で Xbox One S を名乗る（これがないと振動＝Phase が届かない）
  static const char* MFR = "OUA";
  static const char* MDL = "Nozui Brain";
  uint8_t pnp[7] = { 0x02,
                     (uint8_t)(PNP_VID & 0xFF), (uint8_t)(PNP_VID >> 8),
                     (uint8_t)(PNP_PID & 0xFF), (uint8_t)(PNP_PID >> 8),
                     (uint8_t)(PNP_VER & 0xFF), (uint8_t)(PNP_VER >> 8) };
  disSvc.begin();
  disMfr.setProperties(CHR_PROPS_READ); disMfr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  disMfr.setFixedLen((uint16_t)strlen(MFR)); disMfr.begin(); disMfr.write((const void*)MFR, (uint16_t)strlen(MFR));
  disMdl.setProperties(CHR_PROPS_READ); disMdl.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  disMdl.setFixedLen((uint16_t)strlen(MDL)); disMdl.begin(); disMdl.write((const void*)MDL, (uint16_t)strlen(MDL));
  disPnp.setProperties(CHR_PROPS_READ); disPnp.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  disPnp.setFixedLen(7); disPnp.begin(); disPnp.write((const void*)pnp, 7);

  blehid.setHidInfo(0x0111, 0x00, 0x01);
  blehid.setReportLen(IN_LEN, OUT_LEN, NULL);
  blehid.setReportMap(REPORT_MAP, sizeof(REPORT_MAP));
  blehid.begin();
  blehid.setOutputReportCallback(1, outDummyCb);
  blehid.setOutputReportCallback(2, outDummyCb);
  blehid.setOutputReportCallback(3, outputReportCb);   // ★ Phase（Xbox 振動レポート ID=3）
  initBatteryService();                                // 標準の Battery Service（HID のあとに足す）
  sendBatteryHid();                                    // HID 電池レポートの初期値（読みに来たときに返る値）
  Bluefruit.Periph.setConnInterval(8, 12);             // 10〜15ms

  // ---- Central：ヘッドギア向け GATT Client ----
  hgSvc.begin();
  hgCmd.begin();
  hgHeart.setNotifyCallback(onHeartNotify);
  hgHeart.begin();
  Bluefruit.Central.setConnectCallback(onCentralConnect);
  Bluefruit.Central.setDisconnectCallback(onCentralDisconnect);
  Bluefruit.Central.setConnInterval(6, 12);      // 7.5〜15ms（心拍・Phase の遅れを小さく。台湾版は 8〜12）

  Bluefruit.Scanner.setRxCallback(scanCb);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);              // 100ms 周期で 50ms スキャン
  Bluefruit.Scanner.useActiveScan(false);              // ヘッドギアの名前は広告パケットに入っている
  Bluefruit.Scanner.start(0);

  // ---- 広告（Quest 向け）----
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_GAMEPAD);
  // 電池サービスも広告に載せる（本物の機器と同じ）。広告はこれで 31 バイトちょうど
  // （Flags 3＋TxPower 3＋Appearance 4＋UUID 2 つ 6＋名前「NozuiBrain-X1」15）
  Bluefruit.Advertising.addService(blehid, basSvc);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  InternalFS.begin();
  loadPressureDir();                       // loop1 を始める前に（setup1 が使う）
  Scheduler.startLoop(loop1, 2048, TASK_PRIO_LOW, "sense");   // 「Core1」相当

  Serial.printf("[BLE] %s で広告中／%s をスキャン中\n", BLE_NAME, HEADGEAR_NAME);
  if(g_savedSign) Serial.printf("[CAL] 圧力の向き：保存済み %s（ファーム CRC %08lX）。押して決め直す必要はありません\n",
                                g_savedSign > 0 ? "+（押すと増える）" : "-（押すと減る）", (unsigned long)g_fwCrc);
  else            Serial.printf("[CAL] ★ 新しいファーム（CRC %08lX）なので、圧力の向きを決めます。センサに触らずに数秒待ち、"
                                "そのあとどれかのセンサを 3 回押してください。確定すると保存して 1 回振動します\n", (unsigned long)g_fwCrc);
  printHelp();
  hapticStart(true);                             // 起動した合図に心拍の振動を 1 回（loop で鳴る）
}

void loop(){
  uint32_t now = millis();

  updateLed(now);
  updateBattery(now);
  pollSerial();
  syncHeadgearOnConnect();
  syncQuestOnSecured();
  updatePower(now);            // 先に台座の状態を確定させてから Phase 変化を処理する
  checkPhaseTransition();
  updateButtons(now);

  // 心拍の振動（SIGGRAPH 版）
  checkHeartbeat();
  updateHaptic(now);

  // 1 秒ごとにヘッドギアへ今の Phase を送り直す（ヘッドギアは 3 秒来ないと「切れた」とみなして椅子を起こす）
  if(g_hgReady && now - g_lastPhaseSync >= PHASE_SYNC_MS) sendCmd03_Phase(false);

  // IMU イベントの表示（loop1 から）
  char ev = g_imuEvt;
  if(ev){
    g_imuEvt = 0;
    if(ev == 't')      Serial.println("[IMU] THROW（振りかぶって投げた）");
    else if(ev == 's') Serial.println("[IMU] SLIP（取り落とした）");
    else if(ev == 'l') Serial.printf("[IMU] 床に着地  衝撃ピーク %.1fG（床の判定 ≥%.1fG / 静止なら ≥%.1fG）→ A を送る\n", g_imuPeak, LAND_HARD_G, LAND_SOFT_G);
    else if(ev == 'c') Serial.printf("[IMU] キャッチ  衝撃ピーク %.1fG → 何も送らない\n", g_imuPeak);
  }
  if(g_dirSaveReq){
    g_dirSaveReq = false;
    if(savePressureDir(p_sign)){
      Serial.printf("[CAL] ★ 圧力の向き %s を保存しました。次にファームを書き込むまで、この向きのまま変わりません\n", p_sign > 0 ? "+（押すと増える）" : "-（押すと減る）");
      hapticStart(true);                // 保存できた合図に 1 回鳴らす
    } else Serial.println("[CAL] ★ 圧力の向きを保存できなかった（今回の起動中はこの向きで動く）");
  }
  char de = g_dirEvt;
  if(de == 'z'){
    g_dirEvt = 0;
    Serial.println("[CAL] Phase1：圧力のゼロ点・最大値をリセット（向きはそのまま）");
  } else if(de){
    g_dirEvt = 0;
    Serial.printf("[CAL] 圧力の向きを%s → %s\n",
                  de == 'p' ? "仮決め" : de == 'L' ? "確定" : "反転",
                  p_sign > 0 ? "+（押すと増える）" : "-（押すと減る）");
  }

  static uint32_t lastSend = 0, lastHb = 0;
  switch(g_phase){
  case 3:
  case 4:
    if(g_phase == 4) checkThrow_Phase4();
    if(now - lastSend >= SEND_MS){
      lastSend = now; lastHb = now;
      uint8_t calib[PRESSURE_CH];
      copyCalib(calib);
      sendPressureToQuest(calib);
      sendServoValues_Cmd02(calib);
    }
    break;
  default:
    g_throwTrigger = false;   // Phase4 以外の投げは捨てる
    // ニュートラルの入力レポートを 1 秒ごとに（Android に生きているゲームパッドと認識させる）
    if(!g_buttons && now - lastHb >= HEARTBEAT_MS){
      lastHb = now;
      // Android はボタンを押した瞬間・離した瞬間しかアプリに伝えない。アプリを後から起動すると、
      // 押しっぱなしの X に気づけないので、外れている間は 1 秒ごとに X を離して押し直す（Issue #8）
      if(!g_usbNow || g_forceCableOff) sendGamepad(true);
      sendGamepad();
    }
    break;
  }

  if(g_printPressure){
    static uint32_t lastP = 0;
    if(now - lastP >= 200){
      lastP = now;
      Serial.printf("[P] raw=%4u %4u %4u %4u  out=%3u %3u %3u %3u  向き=%c%s  brake=%u\n",
                    p_raw[0], p_raw[1], p_raw[2], p_raw[3],
                    g_calibOut[0], g_calibOut[1], g_calibOut[2], g_calibOut[3],
                    p_sign == 0 ? '?' : (p_sign > 0 ? '+' : '-'), p_dirLocked ? "確定" : "", g_brake);
    }
  }

  static uint32_t lastStat = 0;
  if(now - lastStat >= STATUS_MS){ lastStat = now; printStatus(); }

  delay(1);
}
