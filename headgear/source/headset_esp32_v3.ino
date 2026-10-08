// ============================================================
//  headset_esp32.ino  v3.4
//  【v3.4修正内容】(フリーズ対策)
//    - WiFi: スリープ無効化 / 自動再接続 / persistent無効化
//    - WiFi再接続を非ブロッキング化（再接続中もサーボ・OSC受信が止まらない）
//    - WiFi切断中はOSC送信をスキップ
//    - VL53L0X: setTimeout を init() の前に移動 / 初期化失敗時は読み取りをスキップ
//    - タスクスタック拡大 (HeartTask 1024→2048 / CommTask 8192→12288)
//    - タスクウォッチドッグ追加（CommTaskが止まったら自動再起動 → 起動時に /ask 送信）
//    - 5秒ごとに空きヒープ/スタック残量をシリアル出力（原因調査用・不要なら削除可）
//  【v3.3修正内容】
//    - バッテリー廃止: 電圧読み取り(PIN_BATT)と /battery のOSC送信を削除
//    - VL53L0Xを1台に変更(handdistance側を廃止): XSHUTピンと /handdistance の送信を削除
//  【v3.2修正内容】
//    - unlockサーボ(idx:6)駆動後、0.5秒(500ms)経過時に自動で0°に戻す処理を追加
// ============================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>
#include <esp_task_wdt.h>   // 【v3.4追加】

// ============================================================
//  ネットワーク設定
// ============================================================
const char*    WIFI_SSID = "nouzui24";
const char*    WIFI_PASS = "nouzui1234";
const char*    PC_IP     = "192.168.30.12";
// const char* WIFI_SSID = "unhcr";
// const char* WIFI_PASS = "ppap2024";
// const char* PC_IP     = "172.20.10.13";
const uint16_t TX_PORT   = 9003;
const uint16_t RX_PORT   = 9000;

// ============================================================
//  OSCアドレス定数
// ============================================================
const char* OSC_PHASE     = "/phase";
const char* OSC_POWER     = "/power";
const char* OSC_UNLOCK    = "/unlock";
const char* OSC_MIRROR    = "/mirror";
const char* OSC_INIT      = "/init";
const char* OSC_REBOOT    = "/reboot";
const char* OSC_HEARTRATE = "/heartrate";
const char* OSC_BRAINDIST = "/braindistance";
const char* OSC_ASK       = "/ask";

// ============================================================
//  ピン定義
// ============================================================
#define PIN_LED     2
#define PIN_HEART   25

// ============================================================
//  PCA9685 / サーボ設定
// ============================================================
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ    50
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2500
#define NUM_SERVOS    8

#define SERVO_ONSET_STEP  512

//              idx: 0   1   2   3   4   5   6    7
int   servoMaxAngle[NUM_SERVOS] = { 40, 40, 40, 40, 40, 40, 35, 170 };
bool  servoReverse[NUM_SERVOS]  = { false, false, true, true, true, false, true, true };
float servoCurrentAngle[NUM_SERVOS];
float servoTargetAngle[NUM_SERVOS];
const uint32_t SERVO_UPDATE_INTERVAL = 20;

// ============================================================
//  ミラーサーボ(#7) 非同期制御
// ============================================================
bool     mirrorMoving      = false;
float    mirrorStartAngle  = 0.0f;
float    mirrorTargetAngle = 0.0f;
uint32_t mirrorStartTime   = 0;
const uint32_t MIRROR_DURATION = 2000;

// ============================================================
//  アンロックサーボ(#6) 自動復帰制御
// ============================================================
bool     unlockActive    = false;
uint32_t unlockStartTime = 0;
const uint32_t UNLOCK_HOLD_TIME = 500;

// ============================================================
//  心拍センサ (Grove / D25 / ISR)
// ============================================================
volatile bool     heartBeatDetected  = false;
volatile int      heartBeatCount     = 0;
const int         HEART_IGNORE_COUNT = 10;
volatile uint32_t lastHeartTime      = 0;
const uint32_t    HEART_MIN_INTERVAL = 200;

void IRAM_ATTR onHeartBeat() {
    uint32_t now = millis();
    if (now - lastHeartTime < HEART_MIN_INTERVAL) return;
    lastHeartTime = now;
    heartBeatCount++;
    if (heartBeatCount > HEART_IGNORE_COUNT) {
        heartBeatDetected = true;
    }
}

// ============================================================
//  距離センサ
// ============================================================
VL53L0X distSensor1;
bool    distSensorOK = false;   // 【v3.4追加】初期化成功フラグ
#define DIST_INTERVAL 100

// ============================================================
//  フェーズ / WiFi
// ============================================================
volatile int currentPhase = 0;
#define WIFI_CHECK_INTERVAL 5000
#define WIFI_RETRY_INTERVAL 10000   // 【v3.4追加】自動再接続で戻らない場合の手動再接続間隔

// ============================================================
//  ウォッチドッグ 【v3.4追加】
// ============================================================
#define WDT_TIMEOUT_S 5

// ============================================================
//  UDP
// ============================================================
WiFiUDP UdpRx;
WiFiUDP UdpTx;

// ============================================================
//  ユーティリティ: 角度 → PCA9685 パルス幅 tick
// ============================================================
uint16_t angleToPulseTick(float angle) {
    float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle / 180.0f);
    return (uint16_t)(us / (20000.0f / 4096.0f));
}

float effectiveAngle(int idx, float angle) {
    float a = constrain(angle, 0.0f, (float)servoMaxAngle[idx]);
    if (servoReverse[idx]) a = servoMaxAngle[idx] - a;
    return a;
}

// ─────────────────────────────────────────────────────────────
//  writeServo: PCA9685 への実際の物理書き込み
// ─────────────────────────────────────────────────────────────
void writeServo(int idx, float angle) {
    int channel    = (idx >= 4) ? (idx + 8) : idx;
    uint16_t onT   = (uint16_t)(idx * SERVO_ONSET_STEP);
    uint16_t pulse = angleToPulseTick(effectiveAngle(idx, angle));
    uint16_t offT  = onT + pulse;
    if (offT >= 4096) offT -= 4096;
    pca9685.setPWM(channel, onT, offT);
    servoCurrentAngle[idx] = angle;
    servoTargetAngle[idx]  = angle;
}

// ─────────────────────────────────────────────────────────────
//  updateServos: 定期フレーム（20ms周期）での書き込み処理
// ─────────────────────────────────────────────────────────────
void updateServos() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (abs(servoCurrentAngle[i] - servoTargetAngle[i]) > 0.01f) {
            writeServo(i, servoTargetAngle[i]);
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  writeServoStaggered: 複数サーボを stagger_ms ずつ時間差で駆動
// ─────────────────────────────────────────────────────────────
void writeServoStaggered(int startIdx, int count, float angle,
                         uint32_t stagger_ms = 15) {
    for (int i = startIdx; i < startIdx + count; i++) {
        writeServo(i, angle);
        if (i < startIdx + count - 1) {
            vTaskDelay(pdMS_TO_TICKS(stagger_ms));
        }
    }
}

// ============================================================
//  WiFi 接続
// ============================================================
void connectWiFi() {
    WiFi.persistent(false);       // 【v3.4追加】再接続のたびにフラッシュへ書き込まない
    WiFi.mode(WIFI_STA);          // 【v3.4追加】
    WiFi.setSleep(false);         // 【v3.4追加】モデムスリープ無効（UDP遅延・取りこぼし対策）
    WiFi.setAutoReconnect(true);  // 【v3.4追加】
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("[WiFi] 接続中");
    uint32_t t = millis();
    while (WiFi.status() != WL_CONNECTED) {
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        delay(300);
        Serial.print(".");
        if (millis() - t > 15000) {
            Serial.println("\n[WiFi] タイムアウト、再試行");
            WiFi.disconnect();
            delay(500);
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            t = millis();
        }
    }
    digitalWrite(PIN_LED, HIGH);
    Serial.printf("\n[WiFi] 接続完了 IP: %s\n",
                  WiFi.localIP().toString().c_str());
    UdpRx.begin(RX_PORT);
}

// ============================================================
//  OSC 送信ヘルパー
// ============================================================
void oscSendInt(const char* addr, int32_t val) {
    if (WiFi.status() != WL_CONNECTED) return;   // 【v3.4追加】切断中は送信しない
    OSCMessage msg(addr);
    msg.add((int32_t)val);
    UdpTx.beginPacket(PC_IP, TX_PORT);
    msg.send(UdpTx);
    UdpTx.endPacket();
    msg.empty();
}

void oscSendFloat(const char* addr, float val) {
    if (WiFi.status() != WL_CONNECTED) return;   // 【v3.4追加】切断中は送信しない
    OSCMessage msg(addr);
    msg.add((float)val);
    UdpTx.beginPacket(PC_IP, TX_PORT);
    msg.send(UdpTx);
    UdpTx.endPacket();
    msg.empty();
}

// ============================================================
//  OSC コールバック
// ============================================================
void handlePhase(OSCMessage& msg) {
    int ph = msg.getInt(0);
    if (ph < 1 || ph > 7) return;
    currentPhase = ph;
    Serial.printf("[OSC] /phase → %d\n", ph);
    if (ph == 2) {
        for (int i = 0; i < 6; i++) {
            servoTargetAngle[i] = 0.0f;
        }
    }
}

void handlePower(OSCMessage& msg) {
    if (currentPhase < 2 || currentPhase > 4) return;
    for (int i = 0; i < 6 && i < msg.size(); i++) {
        float raw   = msg.isFloat(i) ? msg.getFloat(i) : (float)msg.getInt(i);
        float angle = constrain(raw, 0.0f, 100.0f) / 100.0f * 90.0f;
        servoTargetAngle[i] = angle;
    }
}

void handleUnlock(OSCMessage& msg) {
    if (currentPhase != 5) return;
    int v = msg.isInt(0) ? msg.getInt(0) : (int)msg.getFloat(0);
    if (v == 1) {
        servoTargetAngle[6] = 30.0f;
        unlockStartTime     = millis();
        unlockActive        = true;
        Serial.println("[OSC] /unlock → servo6=30°");
    }
}

void handleMirror(OSCMessage& msg) {
    int v = msg.isInt(0) ? msg.getInt(0) : (int)msg.getFloat(0);
    if (v == 1 && currentPhase == 6) {
        mirrorStartAngle  = servoCurrentAngle[7];
        mirrorTargetAngle = 170.0f;
        mirrorStartTime   = millis();
        mirrorMoving      = true;
        Serial.println("[OSC] /mirror 1 → servo7: →170°");
    } else if (v == 0 && currentPhase == 7) {
        mirrorStartAngle  = servoCurrentAngle[7];
        mirrorTargetAngle = 0.0f;
        mirrorStartTime   = millis();
        mirrorMoving      = true;
        Serial.println("[OSC] /mirror 0 → servo7: →0°");
    }
}

void handleInit(OSCMessage& msg) {
    Serial.println("[OSC] /init → ソフトリセット");
    currentPhase   = 0;
    mirrorMoving   = false;
    unlockActive   = false;
    heartBeatCount = 0;
    for (int i = 0; i < NUM_SERVOS; i++) {
        servoTargetAngle[i] = 0.0f;
    }
    writeServoStaggered(0, NUM_SERVOS, 0.0f, 15);
    UdpRx.stop();
    if (!UdpRx.begin(RX_PORT)) {   // 【v3.4追加】受信ソケット再作成の失敗を検出 → 再起動で復帰
        Serial.println("[UDP] 受信ソケット再作成失敗 → 再起動");
        delay(100);
        ESP.restart();
    }
}

void handleReboot(OSCMessage& msg) {
    Serial.println("[OSC] /reboot → ESP32強制再起動");
    delay(100);
    ESP.restart();
}

// ============================================================
//  OSC 受信ディスパッチ
// ============================================================
void processOSC() {
    int size = UdpRx.parsePacket();
    if (size <= 0) return;
    uint8_t buf[512];
    int len = UdpRx.read(buf, sizeof(buf));
    OSCMessage msg;
    for (int i = 0; i < len; i++) msg.fill(buf[i]);
    if (!msg.hasError()) {
        msg.dispatch(OSC_PHASE,  handlePhase);
        msg.dispatch(OSC_POWER,  handlePower);
        msg.dispatch(OSC_UNLOCK, handleUnlock);
        msg.dispatch(OSC_MIRROR, handleMirror);
        msg.dispatch(OSC_INIT,   handleInit);
        msg.dispatch(OSC_REBOOT, handleReboot);
    }
}

// ============================================================
//  ミラーサーボ非同期計算（目標角度の更新のみ）
// ============================================================
void updateMirrorServo() {
    if (!mirrorMoving) return;
    uint32_t elapsed = millis() - mirrorStartTime;
    if (elapsed >= MIRROR_DURATION) {
        servoTargetAngle[7] = mirrorTargetAngle;
        mirrorMoving = false;
    } else {
        float t = (float)elapsed / MIRROR_DURATION;
        servoTargetAngle[7] = mirrorStartAngle
                              + (mirrorTargetAngle - mirrorStartAngle) * t;
    }
}

// ============================================================
//  アンロックサーボ自動復帰計算（目標角度の更新のみ）
// ============================================================
void updateUnlockServo() {
    if (!unlockActive) return;
    if (millis() - unlockStartTime >= UNLOCK_HOLD_TIME) {
        servoTargetAngle[6] = 0.0f;
        unlockActive = false;
        Serial.println("[UNLOCK] 自動復帰 → servo6=0°");
    }
}

// ============================================================
//  Core0 タスク: 心拍 ISR サポート（WDT 対策のみ）
// ============================================================
void heartTask(void* param) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================
//  Core1 メインタスク
// ============================================================
void commTask(void* param) {
    uint32_t lastDistTx      = 0;
    uint32_t lastDistRead    = 0;
    uint32_t lastWifiCheck   = 0;
    uint32_t lastServoUpdate = 0;

    bool     wifiLost        = false;   // 【v3.4追加】
    uint32_t wifiLostSince   = 0;       // 【v3.4追加】

    int   brainDistMm = 0;

    esp_task_wdt_add(NULL);             // 【v3.4追加】このタスクをウォッチドッグ監視に登録

    for (;;) {
        esp_task_wdt_reset();           // 【v3.4追加】
        uint32_t now = millis();

        // ── OSC 受信（目標角度の更新のみ）──
        processOSC();

        // ── ミラーサーボ計算（目標角度の更新のみ）──
        updateMirrorServo();

        // ── アンロックサーボ自動復帰計算 ──
        updateUnlockServo();

        // ── サーボ定期書き込み（20ms周期 / 50Hz）──
        if (now - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {
            updateServos();
            lastServoUpdate = now;
        }

        // ── 心拍送信（ISR フラグ確認）──
        if (currentPhase >= 3 && currentPhase <= 4 && heartBeatDetected) {
            heartBeatDetected = false;
            oscSendInt(OSC_HEARTRATE, (int32_t)1);
        }

        // ── 距離センサ読み取り（100ms間隔、フェーズ3のみ）──
        if (distSensorOK && currentPhase == 3 && now - lastDistRead >= DIST_INTERVAL) {
            uint16_t d1 = distSensor1.readRangeContinuousMillimeters();
            if (!distSensor1.timeoutOccurred()) brainDistMm = d1;
            lastDistRead = now;
        }

        // ── 距離センサ送信（50ms間隔、フェーズ3のみ）──
        if (currentPhase == 3 && now - lastDistTx >= 50) {
            oscSendInt(OSC_BRAINDIST, (int32_t)brainDistMm);
            lastDistTx = now;
        }

        // ── WiFi 状態確認（5秒ごと / ノンブロッキング）──
        if (now - lastWifiCheck >= WIFI_CHECK_INTERVAL) {
            lastWifiCheck = now;
            if (WiFi.status() != WL_CONNECTED) {
                if (!wifiLost) {
                    // 切断を初めて検出：自動再接続に任せる
                    wifiLost      = true;
                    wifiLostSince = now;
                    digitalWrite(PIN_LED, LOW);
                    Serial.println("[WiFi] 切断検出、自動再接続待ち...");
                } else if (now - wifiLostSince >= WIFI_RETRY_INTERVAL) {
                    // 自動再接続で戻らない場合のみ手動で再接続（待たずに次へ進む）
                    Serial.println("[WiFi] 手動で再接続");
                    WiFi.disconnect();
                    WiFi.begin(WIFI_SSID, WIFI_PASS);
                    wifiLostSince = now;
                }
            } else if (wifiLost) {
                // 復帰を検出
                wifiLost = false;
                digitalWrite(PIN_LED, HIGH);
                UdpRx.stop();
                UdpRx.begin(RX_PORT);
                Serial.printf("[WiFi] 再接続完了 IP: %s\n",
                              WiFi.localIP().toString().c_str());
            }

            // 【v3.4追加】原因調査用ログ（不要なら削除してOK）
            Serial.printf("[DIAG] heap=%u minHeap=%u stackFree=%u\n",
                          (unsigned)ESP.getFreeHeap(),
                          (unsigned)ESP.getMinFreeHeap(),
                          (unsigned)uxTaskGetStackHighWaterMark(NULL));
        }

        vTaskDelay(1);
    }
}

// ============================================================
//  setup
// ============================================================
void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED,   OUTPUT);
    pinMode(PIN_HEART, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_HEART), onHeartBeat, RISING);

    Wire.begin();

    pca9685.begin();
    pca9685.setOscillatorFrequency(27000000);
    pca9685.setPWMFreq(SERVO_FREQ);

    writeServoStaggered(0, NUM_SERVOS, 0.0f, 15);

    // 【v3.4修正】setTimeout を init() の前に設定（未設定だと init() が無限待ちになり得る）
    distSensor1.setTimeout(200);
    distSensorOK = distSensor1.init();
    if (distSensorOK) {
        distSensor1.startContinuous(50);
        Serial.println("[VL53L0X] 初期化完了");
    } else {
        Serial.println("[VL53L0X] 初期化失敗（距離センサなしで継続）");
    }

    connectWiFi();

    oscSendInt(OSC_ASK, 1);
    Serial.println("[BOOT] Sent /ask to PC");

    // 【v3.4追加】タスクウォッチドッグ設定（CommTask が WDT_TIMEOUT_S 秒止まったら再起動）
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t wdtCfg = {
        .timeout_ms     = WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    if (esp_task_wdt_reconfigure(&wdtCfg) != ESP_OK) {
        esp_task_wdt_init(&wdtCfg);
    }
#else
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif

    xTaskCreatePinnedToCore(
        heartTask, "HeartTask",
        2048, NULL, 5, NULL, 0      // 【v3.4修正】1024 → 2048
    );

    xTaskCreatePinnedToCore(
        commTask, "CommTask",
        12288, NULL, 4, NULL, 1     // 【v3.4修正】8192 → 12288
    );

    Serial.println("[BOOT] 起動完了");
}

// ============================================================
//  loop
// ============================================================
void loop() {
    vTaskDelay(portMAX_DELAY);
}