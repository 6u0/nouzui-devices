// ============================================================
//   brain_m5sticks3.ino  ―  脳デバイス ファームウェア v3.9
//   プラットフォーム: M5StickS3 (M5Unified)
//
//   変更点 (v3.8 -> v3.9):
//     ・給電判定を電圧トレンド方式から M5.Power.getVBUSVoltage() に変更
//       M5PM1の reg 0x24/0x25 を直接読むため安定
//       給電あり: VBUS >= 4000mV / 給電なし: VBUS < 4000mV
//     ・電圧トレンド用バッファ (m5VoltHistory等) を削除
//
//   追加:
//     ・本体ボタンのダブルクリックで「充電専用モード」へ移行
//     ・充電専用モードではWi-Fi/OSC/センサー/LED/振動等を停止
//     ・M5本体バッテリー残量とLiPo電圧のみ表示
//     ・充電専用モードからは通常動作へ戻らない
// ============================================================

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCData.h>

// ============================================================
//   ★ 設定値
// ============================================================

const char* WIFI_SSID = "nouzui24";
const char* WIFI_PASS = "nouzui1234";
const char* PC_IP     = "192.168.30.12";
const uint16_t TX_PORT = 9002;
const uint16_t RX_PORT = 9000;

const int PIN_PRESSURE[]   = {1, 2, 3, 4};
const int PIN_LIPO_V       = 5;
const int PIN_LIPO_CHRG    = 6;
const int PIN_HALL         = 10;
const int PIN_VIB_PWM      = 8;
const int PIN_VIB_IN2      = 7;
const int PIN_BTN          = 11;
const int PIN_LED_RED      = 0;
const int PIN_LED_GREEN    = 43;

const int ADC_MIN = 0;
const int ADC_MAX = 4095;
const float BAT_RESTORE = 2.0f;

const float IMPACT_THRESH_G    = 5.0f;
const uint32_t IMPACT_GUARD_MS = 500;

const uint32_t SEND_INTERVAL_MS   = 20;
const uint32_t BATTERY_SAMPLE_MS  = 200;
const uint32_t BATTERY_CHECK_MS   = 1000;

const uint32_t AUTO_SLEEP_MS   = 180000;
const uint32_t POWER_OFF_MS    = 10000;

// ============================================================
//   ダブルクリック設定
// ============================================================

// 1回目のクリックから、この時間以内にもう一度クリックされたら
// 充電専用モードへ移行
const uint32_t DOUBLE_CLICK_MS = 400;

// VBUS電圧の給電判定閾値 (mV)
// 給電あり: 通常4500〜5100mV / 給電なし: 0mV
// 4000mVをしきい値として余裕を持たせる
const int16_t VBUS_POWERED_THRESH_MV = 4000;

const char* OSC_PHASE     = "/phase";
const char* OSC_POWER     = "/power";
const char* OSC_HEARTRATE = "/heartrate";
const char* OSC_MAGNET    = "/magnet";
const char* OSC_GRAVITY   = "/gravity";
const char* OSC_INIT      = "/init";
const char* OSC_REBOOT    = "/reboot";
const char* OSC_ASK       = "/ask";
const char* OSC_BAT_M5    = "/battery/m5";
const char* OSC_BAT_LIPO  = "/battery/lipo";
const char* OSC_BAT_STATE = "/battery/state";
const char* OSC_BAT_REQ   = "/battery";

// ============================================================
//   Haptic設定
// ============================================================
static const int LEDC_CHANNEL  = 0;
static const int LEDC_FREQ_HZ  = 1000;
static const int LEDC_RES_BITS = 8;

static const uint8_t AMP_MAX  = 220;
static const uint8_t AMP_MID  = 130;
static const uint8_t AMP_MID2 = 80;
static const uint8_t AMP_LOW  = 40;
static const uint8_t AMP_ZERO = 0;

struct Phase {
    uint8_t ampStart;
    uint8_t ampEnd;
    uint16_t durationMs;
};

static const int NUM_FIXED_PHASES = 6;

static const Phase FIXED_PHASES[NUM_FIXED_PHASES] = {
    { AMP_ZERO, AMP_MAX,  50 },
    { AMP_MAX,  AMP_MAX,  80 },
    { AMP_MAX,  AMP_MID,  60 },
    { AMP_MID,  AMP_LOW,  25 },
    { AMP_LOW,  AMP_MID2, 35 },
    { AMP_MID2, AMP_ZERO, 80 }
};

static int s_currentPhase = NUM_FIXED_PHASES;
static uint32_t s_phaseStartMs = 0;
volatile bool osc_trigger_received = false;
volatile bool chargeOnlyMode = false;   // ★追加

// ============================================================
//   グローバル変数
// ============================================================
WiFiUDP UdpRx;
WiFiUDP UdpTx;

int currentPhase = 0;

float rawPressure[4]  = {};
float filtPressure[4] = {};
float outPressure[6]  = {};

bool prevHallState = LOW;
bool gravityFired  = false;
uint32_t phase4StartMs = 0;
float prevAccMag = 1.0f;

uint32_t lastSendMs = 0;
uint32_t lastBatterySampleMs = 0;
uint32_t lastBatteryMs = 0;
uint32_t lastInteractionMs = 0;

bool isBtnPressed = false;
uint32_t btnPressStartMs = 0;

// ダブルクリック判定用
uint32_t lastButtonReleaseMs = 0;
bool waitingForDoubleClick = false;

int dispMode = 0;

#define BAT_AVG_SIZE 10
float m5BatHistory[BAT_AVG_SIZE];
float lipoVHistory[BAT_AVG_SIZE];
int batAvgIndex = 0;

float m5BatLevelAvg = 0.0f;
float lipoVoltageAvg = 0.0f;
float m5BatLevel = 0.0f;
float lipoVoltage = 0.0f;
int32_t m5BatState = 0;
int32_t lipoBatState = 0;

// LiPo 充電ピン ノイズフィルタ
bool lipoChrgHistory[10] = {
    true, true, true, true, true,
    true, true, true, true, true
};

int lipoChrgIndex = 0;

bool prevM5Powered = false;
uint32_t powerUnpluggedMs = 0;
bool isUnpluggedEventActive = false;

String oscLogTx = "";
String oscLogRx = "";

// ============================================================
//   プロトタイプ宣言
// ============================================================
void taskOscRx(void* pvParam);

void updateBatteryState();
void sampleLipoChrgPin();
bool isLipoChargingFiltered();

void updateLEDs();
void processHaptic();
void handleButton();
void updateDisplay();

void readAndSendPower();
void checkImpact();

void addOscLogTx(String msg);
void addOscLogRx(String msg);

// 充電専用モード
void enterChargeOnlyMode();
void updateChargeOnlyDisplay();

// ============================================================
//   setup
// ============================================================
void setup() {
    auto cfg = M5.config();

    cfg.output_power = false;  // 外部5V入力時に必要

    M5.Power.setExtOutput(false); // 5VINをINPUTモードに明示設定
    M5.begin(cfg);

    Serial.begin(115200);

    // M5.begin()の後でEXT_5V出力をONにする
    // ホールセンサ等をEXT_5Vから給電している場合は true が必要
    M5.Power.setExtOutput(true);

    pinMode(PIN_LIPO_CHRG, INPUT_PULLUP);
    pinMode(PIN_HALL, INPUT_PULLUP);
    pinMode(PIN_BTN, INPUT_PULLUP);

    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);

    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);

    if (PIN_VIB_IN2 >= 0) {
        pinMode(PIN_VIB_IN2, OUTPUT);
        digitalWrite(PIN_VIB_IN2, LOW);
    }

    ledcAttach(PIN_VIB_PWM, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcWrite(PIN_VIB_PWM, 0);

    M5.Display.setRotation(0);
    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.fillScreen(TFT_BLACK);

    M5.Display.setCursor(5, 5);
    M5.Display.println("WiFi Connecting:");

    M5.Display.setCursor(5, 25);
    M5.Display.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print(".");
    }

    UdpRx.begin(RX_PORT);

    M5.Display.setCursor(5, 55);
    M5.Display.println("Connected.");

    M5.Display.setCursor(5, 75);
    M5.Display.println(WiFi.localIP().toString().c_str());

    delay(1000);

    M5.Display.setBrightness(0);

    lastInteractionMs = millis();

    // --------------------------------------------------------
    // バッファ初期埋め
    // --------------------------------------------------------
    bool initChrg = digitalRead(PIN_LIPO_CHRG);

    float initM5 =
        M5.Power.getBatteryLevel() / 100.0f;

    int initAdc =
        analogRead(PIN_LIPO_V);

    float initV =
        roundf(
            ((initAdc / 4095.0f) * 3.3f)
            * BAT_RESTORE
            * 100.0f
        ) / 100.0f;

    for (int i = 0; i < 10; i++) {
        lipoChrgHistory[i] = initChrg;
        m5BatHistory[i] = initM5;
        lipoVHistory[i] = initV;
    }

    m5BatLevelAvg  = initM5;
    lipoVoltageAvg = initV;

    // 初期給電状態をVBUSで判定
    prevM5Powered =
        (M5.Power.getVBUSVoltage() >= VBUS_POWERED_THRESH_MV);

    // --------------------------------------------------------
    // OSC受信タスク
    // --------------------------------------------------------
    xTaskCreatePinnedToCore(
        taskOscRx,
        "OscRx",
        4096,
        nullptr,
        1,
        nullptr,
        0
    );

    // --------------------------------------------------------
    // PCへASK送信
    // --------------------------------------------------------
    OSCMessage msg(OSC_ASK);
    msg.add((int32_t)1);

    UdpTx.beginPacket(PC_IP, TX_PORT);
    msg.send(UdpTx);
    UdpTx.endPacket();

    addOscLogTx(OSC_ASK);

    Serial.println("[BOOT] Ready.");
}

// ============================================================
//   loop
// ============================================================
void loop() {
    M5.update();

    uint32_t now = millis();

    sampleLipoChrgPin();

    handleButton();

    processHaptic();

    if (now - lastBatterySampleMs >= BATTERY_SAMPLE_MS) {
        lastBatterySampleMs = now;
        updateBatteryState();
    }

    if (now - lastBatteryMs >= BATTERY_CHECK_MS) {
        lastBatteryMs = now;

        OSCMessage msgM5(OSC_BAT_M5);
        msgM5.add(m5BatLevel);

        UdpTx.beginPacket(PC_IP, TX_PORT);
        msgM5.send(UdpTx);
        UdpTx.endPacket();

        OSCMessage msgLiPo(OSC_BAT_LIPO);
        msgLiPo.add(lipoVoltage);

        UdpTx.beginPacket(PC_IP, TX_PORT);
        msgLiPo.send(UdpTx);
        UdpTx.endPacket();

        updateDisplay();
    }

    updateLEDs();

    switch (currentPhase) {

        case 3:

            if (now - lastSendMs >= SEND_INTERVAL_MS) {
                lastSendMs = now;
                readAndSendPower();
            }

            {
                bool cur = digitalRead(PIN_HALL);

                if (prevHallState == LOW && cur == HIGH) {

                    OSCMessage msg(OSC_MAGNET);
                    msg.add((int32_t)1);

                    UdpTx.beginPacket(PC_IP, TX_PORT);
                    msg.send(UdpTx);
                    UdpTx.endPacket();

                    addOscLogTx(OSC_MAGNET);
                }

                prevHallState = cur;
            }

            break;

        case 4:

            if (now - lastSendMs >= SEND_INTERVAL_MS) {
                lastSendMs = now;
                readAndSendPower();
            }

            checkImpact();

            break;

        default:
            break;
    }

    if (
        dispMode != 0 &&
        (now - lastInteractionMs > AUTO_SLEEP_MS)
    ) {
        dispMode = 0;
        M5.Display.setBrightness(0);
    }
}

// ============================================================
//   Core 0: OSC 受信タスク
// ============================================================
void taskOscRx(void* pvParam) {

    while (true) {

        // ★追加: 充電専用モードなら安全な位置で自分自身を終了
        if (chargeOnlyMode) {
            vTaskDelete(NULL);
        }
        
        int pktSize = UdpRx.parsePacket();

        if (pktSize > 0) {

            OSCMessage msg;

            while (pktSize--)
                msg.fill(UdpRx.read());

            if (!msg.hasError()) {

                if (
                    msg.fullMatch(OSC_PHASE) &&
                    msg.isInt(0)
                ) {

                    int p = msg.getInt(0);

                    if (p != currentPhase) {

                        currentPhase = p;
                        gravityFired = false;

                        prevHallState =
                            digitalRead(PIN_HALL);

                        if (p == 4)
                            phase4StartMs = millis();

                        addOscLogRx(OSC_PHASE);
                    }
                }

                else if (
                    msg.fullMatch(OSC_HEARTRATE) &&
                    msg.isInt(0)
                ) {

                    if (msg.getInt(0) == 1) {

                        if (currentPhase >= 2) {
                            osc_trigger_received = true;
                        }

                        addOscLogRx(OSC_HEARTRATE);
                    }
                }

                else if (
                    msg.fullMatch(OSC_BAT_REQ) &&
                    msg.isInt(0)
                ) {

                    if (msg.getInt(0) == 1) {

                        OSCMessage rep(OSC_BAT_STATE);

                        rep.add(m5BatState);
                        rep.add(lipoBatState);

                        UdpTx.beginPacket(PC_IP, TX_PORT);
                        rep.send(UdpTx);
                        UdpTx.endPacket();

                        addOscLogRx(OSC_BAT_REQ);
                        addOscLogTx(OSC_BAT_STATE);
                    }
                }

                else if (msg.fullMatch(OSC_INIT)) {

                    currentPhase = 0;
                    gravityFired = false;

                    memset(
                        rawPressure,
                        0,
                        sizeof(rawPressure)
                    );

                    memset(
                        filtPressure,
                        0,
                        sizeof(filtPressure)
                    );

                    addOscLogRx(OSC_INIT);
                }

                else if (msg.fullMatch(OSC_REBOOT)) {
                    ESP.restart();
                }
            }
        }

        vTaskDelay(1);
    }
}

// ============================================================
//   Haptic 処理
// ============================================================
uint8_t lerp8(
    uint8_t a,
    uint8_t b,
    uint32_t elapsed,
    uint32_t duration
) {
    if (
        duration == 0 ||
        elapsed >= duration
    )
        return b;

    int32_t delta =
        (int32_t)b - (int32_t)a;

    return (uint8_t)(
        a +
        (
            delta *
            (int32_t)elapsed
        ) /
        (int32_t)duration
    );
}

void processHaptic() {

    uint32_t now = millis();

    if (osc_trigger_received) {

        osc_trigger_received = false;

        ledcWrite(PIN_VIB_PWM, 0);

        s_currentPhase = 0;
        s_phaseStartMs = now;
    }

    if (s_currentPhase < NUM_FIXED_PHASES) {

        const Phase& ph =
            FIXED_PHASES[s_currentPhase];

        uint32_t elapsed =
            now - s_phaseStartMs;

        if (elapsed >= ph.durationMs) {

            ledcWrite(
                PIN_VIB_PWM,
                ph.ampEnd
            );

            s_currentPhase++;
            s_phaseStartMs = now;

        } else {

            uint8_t duty =
                lerp8(
                    ph.ampStart,
                    ph.ampEnd,
                    elapsed,
                    ph.durationMs
                );

            ledcWrite(
                PIN_VIB_PWM,
                duty
            );
        }

    } else {

        ledcWrite(PIN_VIB_PWM, 0);
    }
}

// ============================================================
//   圧力取得 & 4ch→6ch拡張フィルタ処理
// ============================================================
void readAndSendPower() {

    for (int i = 0; i < 4; i++) {

        int adc =
            analogRead(PIN_PRESSURE[i]);

        rawPressure[i] =
            (float)map(
                adc,
                ADC_MIN,
                ADC_MAX,
                0,
                150
            );
    }

    for (int i = 0; i < 4; i++)
        filtPressure[i] = 0;

    for (int i = 0; i < 4; i++) {

        float v = rawPressure[i];

        filtPressure[i] += v * 0.8f;

        if (i == 0)
            filtPressure[1] += v * 0.3f;

        if (i == 1)
            filtPressure[0] += v * 0.3f;

        if (i == 2)
            filtPressure[3] += v * 0.3f;

        if (i == 3)
            filtPressure[2] += v * 0.3f;

        if (i < 2)
            filtPressure[i + 2] += v * 0.1f;
        else
            filtPressure[i - 2] += v * 0.1f;
    }

    outPressure[0] =
        filtPressure[0];

    outPressure[1] =
        (filtPressure[0] +
         filtPressure[1]) / 2.0f;

    outPressure[2] =
        filtPressure[1];

    outPressure[3] =
        filtPressure[2];

    outPressure[4] =
        (filtPressure[2] +
         filtPressure[3]) / 2.0f;

    outPressure[5] =
        filtPressure[3];

    OSCMessage msg(OSC_POWER);

    for (int i = 0; i < 6; i++) {

        if (outPressure[i] > 100.0f)
            outPressure[i] = 100.0f;

        msg.add(
            (int32_t)outPressure[i]
        );
    }

    UdpTx.beginPacket(PC_IP, TX_PORT);
    msg.send(UdpTx);
    UdpTx.endPacket();
}

// ============================================================
//   衝撃検知 (M5内蔵IMU)
// ============================================================
void checkImpact() {

    if (gravityFired)
        return;

    float ax, ay, az;

    if (
        M5.Imu.getAccel(
            &ax,
            &ay,
            &az
        )
    ) {

        float magG =
            sqrtf(
                ax * ax +
                ay * ay +
                az * az
            );

        if (
            millis() - phase4StartMs <
            IMPACT_GUARD_MS
        ) {

            prevAccMag = magG;
            return;
        }

        float delta =
            fabsf(
                magG -
                prevAccMag
            );

        prevAccMag = magG;

        if (delta > IMPACT_THRESH_G) {

            OSCMessage msg(OSC_GRAVITY);
            msg.add((int32_t)1);

            UdpTx.beginPacket(
                PC_IP,
                TX_PORT
            );

            msg.send(UdpTx);
            UdpTx.endPacket();

            gravityFired = true;

            addOscLogTx(OSC_GRAVITY);
        }
    }
}

// ============================================================
//   GPIO6 ノイズフィルタ処理
// ============================================================
void sampleLipoChrgPin() {

    lipoChrgHistory[lipoChrgIndex] =
        digitalRead(PIN_LIPO_CHRG);

    lipoChrgIndex =
        (lipoChrgIndex + 1) % 10;
}

bool isLipoChargingFiltered() {

    for (int i = 0; i < 10; i++) {

        if (lipoChrgHistory[i] == LOW)
            return true;
    }

    return false;
}

// ============================================================
//   バッテリー・電源ステータス更新 (200ms周期)
// ============================================================
void updateBatteryState() {

    int batLvl =
        M5.Power.getBatteryLevel();

    float rawM5Bat =
        batLvl / 100.0f;

    if (rawM5Bat > 1.0f)
        rawM5Bat = 1.0f;

    int adc =
        analogRead(PIN_LIPO_V);

    float vAdc =
        (adc / 4095.0f) * 3.3f;

    float rawLipoV =
        roundf(
            vAdc *
            BAT_RESTORE *
            100.0f
        ) / 100.0f;

    m5BatHistory[batAvgIndex] =
        rawM5Bat;

    lipoVHistory[batAvgIndex] =
        rawLipoV;

    batAvgIndex =
        (batAvgIndex + 1) %
        BAT_AVG_SIZE;

    float sumM5 = 0.0f;
    float sumLipo = 0.0f;

    for (int i = 0; i < BAT_AVG_SIZE; i++) {

        sumM5 +=
            m5BatHistory[i];

        sumLipo +=
            lipoVHistory[i];
    }

    m5BatLevelAvg =
        sumM5 / BAT_AVG_SIZE;

    lipoVoltageAvg =
        sumLipo / BAT_AVG_SIZE;

    // lipoVoltageAvg = 3.5f; //バッテリー偽装

    m5BatLevel =
        m5BatLevelAvg;

    lipoVoltage =
        lipoVoltageAvg;

    // ★ [新方式] VBUS電圧で給電判定
    // M5PM1の reg 0x24/0x25 を直接読む。
    // 給電あり=4000mV以上、なし=0mV

    int16_t vbusVolt =
        M5.Power.getVBUSVoltage();

    bool isM5Powered =
        (
            vbusVolt >=
            VBUS_POWERED_THRESH_MV
        );

    // m5BatState の決定
    if (isM5Powered) {

        if (batLvl >= 100) {
            m5BatState = 2;
        } else {
            m5BatState = 1;
        }

    } else {

        m5BatState = 0;
    }

    if (
        prevM5Powered &&
        !isM5Powered
    ) {

        if (currentPhase < 3) {

            powerUnpluggedMs =
                millis();

            isUnpluggedEventActive =
                true;
        }
    }

    prevM5Powered =
        isM5Powered;

    bool isChrgActive =
        isLipoChargingFiltered();

    if (!isM5Powered) {

        lipoBatState = 0;

    } else if (isChrgActive) {

        lipoBatState = 1;

    } else {

        lipoBatState = 2;
    }
}

// ============================================================
//   LED表示制御
// ============================================================
void updateLEDs() {

    uint32_t now = millis();

    bool redOn = false;
    bool greenOn = false;

    bool isChrgDone =
        !isLipoChargingFiltered();

    bool isFull =
        isChrgDone &&
        (m5BatLevelAvg >= 0.95f);

    bool isLow =
        (lipoVoltageAvg < 3.55f) ||
        (m5BatLevelAvg < 0.20f);

    bool isNormal =
        !isFull &&
        (
            (lipoVoltageAvg >= 3.55f) ||
            (m5BatLevelAvg >= 0.20f)
        );

    if (
        isUnpluggedEventActive &&
        (now - powerUnpluggedMs >= 3000)
    ) {
        isUnpluggedEventActive = false;
    }

    if (isUnpluggedEventActive) {

        if (isFull) {

            greenOn = true;

        } else if (isNormal) {

            greenOn =
                ((now / 250) % 2 == 0);

        } else if (isLow) {

            redOn = true;
        }

    } else if (isLow) {

        redOn =
            ((now / 250) % 2 == 0);

    } else {

        redOn = false;
        greenOn = false;
    }

    digitalWrite(
        PIN_LED_RED,
        redOn ? HIGH : LOW
    );

    digitalWrite(
        PIN_LED_GREEN,
        greenOn ? HIGH : LOW
    );
}

// ============================================================
//   UI / ボタン制御
// ============================================================
void handleButton() {

    bool rawBtn =
        (digitalRead(PIN_BTN) == LOW);

    if (rawBtn && !isBtnPressed) {

        isBtnPressed = true;

        btnPressStartMs =
            millis();

        lastInteractionMs =
            millis();

    }

    else if (!rawBtn && isBtnPressed) {

        isBtnPressed = false;

        uint32_t pressTime =
            millis() -
            btnPressStartMs;

        if (
            pressTime > 50 &&
            pressTime < POWER_OFF_MS
        ) {

            uint32_t now =
                millis();

            // ------------------------------------------------
            // ダブルクリック判定
            // ------------------------------------------------
            if (
                waitingForDoubleClick &&
                (now - lastButtonReleaseMs <= DOUBLE_CLICK_MS)
            ) {

                waitingForDoubleClick = false;

                // 充電専用モードへ
                enterChargeOnlyMode();

                // enterChargeOnlyMode() は戻らないので
                // ここには通常到達しない
                return;
            }

            // ------------------------------------------------
            // 通常のシングルクリック
            // ------------------------------------------------
            waitingForDoubleClick = true;
            lastButtonReleaseMs = now;

            dispMode =
                (dispMode + 1) % 3;

            if (dispMode == 0) {

                M5.Display.setBrightness(0);

            } else {

                M5.Display.setBrightness(128);
                updateDisplay();
            }
        }

    }

    else if (rawBtn && isBtnPressed) {

        if (
            millis() - btnPressStartMs >=
            POWER_OFF_MS
        ) {

            M5.Display.setBrightness(128);
            M5.Display.fillScreen(TFT_BLACK);

            M5.Display.setCursor(
                10,
                30
            );

            M5.Display.print(
                "Powering Off..."
            );

            delay(1000);

            M5.Power.powerOff();
        }
    }

    // ダブルクリック待機状態を一定時間で解除
    if (
        waitingForDoubleClick &&
        !rawBtn &&
        (millis() - lastButtonReleaseMs > DOUBLE_CLICK_MS)
    ) {

        waitingForDoubleClick = false;
    }
}

// ============================================================
//   ★ 充電専用モード
//
//   この関数に入った後は戻らない。
//   Wi-Fi / OSC / センサー / 振動 / LED等を停止し、
//   バッテリー残量表示だけを定期更新する。
// ============================================================
void enterChargeOnlyMode() {

    Serial.println(
        "[CHARGE MODE] Entering charge-only mode."
    );

    // --------------------------------------------------------
    // 通常出力を停止
    // --------------------------------------------------------

    // 振動モーター停止
    ledcWrite(
        PIN_VIB_PWM,
        0
    );

    if (PIN_VIB_IN2 >= 0) {
        digitalWrite(
            PIN_VIB_IN2,
            LOW
        );
    }

    // LED消灯
    digitalWrite(
        PIN_LED_RED,
        LOW
    );

    digitalWrite(
        PIN_LED_GREEN,
        LOW
    );

    // EXT_5V出力停止
    M5.Power.setExtOutput(false);

    // --------------------------------------------------------
    // OSC受信タスクを終了させてから Wi-Fi / UDP を停止
    // --------------------------------------------------------
    chargeOnlyMode = true;
    delay(50);   // 受信タスクがループ先頭に戻って終了するのを待つ

    UdpRx.stop();
    UdpTx.stop();

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    M5.Display.setBrightness(32);
    M5.Display.fillScreen(TFT_BLACK);
    updateChargeOnlyDisplay();

    uint32_t lastChargeSampleMs = millis();

    // ★追加: 長押し電源OFF用
    bool chargeBtnPressed = false;
    uint32_t chargeBtnStartMs = 0;

    while (true) {

        uint32_t now = millis();

        // ----------------------------------------------------
        // 充電残量だけ更新
        // ----------------------------------------------------
        if (
            now - lastChargeSampleMs >=
            BATTERY_SAMPLE_MS
        ) {

            lastChargeSampleMs = now;

            // M5本体バッテリー
            int batLvl =
                M5.Power.getBatteryLevel();

            float rawM5Bat =
                batLvl / 100.0f;

            if (rawM5Bat > 1.0f)
                rawM5Bat = 1.0f;

            // LiPo
            int adc =
                analogRead(PIN_LIPO_V);

            float vAdc =
                (adc / 4095.0f) * 3.3f;

            float rawLipoV =
                roundf(
                    vAdc *
                    BAT_RESTORE *
                    100.0f
                ) / 100.0f;

            // 通常モードと同じ10点平均
            m5BatHistory[batAvgIndex] =
                rawM5Bat;

            lipoVHistory[batAvgIndex] =
                rawLipoV;

            batAvgIndex =
                (batAvgIndex + 1) %
                BAT_AVG_SIZE;

            float sumM5 = 0.0f;
            float sumLipo = 0.0f;

            for (int i = 0; i < BAT_AVG_SIZE; i++) {

                sumM5 +=
                    m5BatHistory[i];

                sumLipo +=
                    lipoVHistory[i];
            }

            m5BatLevelAvg =
                sumM5 / BAT_AVG_SIZE;

            lipoVoltageAvg =
                sumLipo / BAT_AVG_SIZE;

            m5BatLevel =
                m5BatLevelAvg;

            lipoVoltage =
                lipoVoltageAvg;

            updateChargeOnlyDisplay();
        }

        // ----------------------------------------------------
        // ★追加: 10秒長押しで電源OFF
        // ----------------------------------------------------
        bool btnDown = (digitalRead(PIN_BTN) == LOW);

        if (btnDown && !chargeBtnPressed) {

            chargeBtnPressed = true;
            chargeBtnStartMs = now;

        } else if (!btnDown) {

            chargeBtnPressed = false;

        } else if (now - chargeBtnStartMs >= POWER_OFF_MS) {

            M5.Display.setBrightness(128);
            M5.Display.fillScreen(TFT_BLACK);
            M5.Display.setCursor(10, 30);
            M5.Display.print("Powering Off...");

            delay(1000);

            M5.Power.powerOff();
        }

        // ----------------------------------------------------
        // 通常処理は一切しない
        // ----------------------------------------------------
        delay(50);
    }
}

// ============================================================
//   充電専用モード表示
// ============================================================
void updateChargeOnlyDisplay() {

    M5.Display.fillScreen(TFT_BLACK);

    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextColor(
        TFT_WHITE,
        TFT_BLACK
    );

    M5.Display.setCursor(
        2,
        2
    );

    M5.Display.print(
        "CHARGE MODE"
    );

    // M5本体バッテリー
    M5.Display.setCursor(
        2,
        25
    );

    M5.Display.printf(
        "M5: %3.0f%%",
        m5BatLevel * 100.0f
    );

    // LiPo
    M5.Display.setCursor(
        2,
        45
    );

    M5.Display.printf(
        "LiPo: %.2fV",
        lipoVoltage
    );
}

// ============================================================
//   ディスプレイ描画処理
// ============================================================
void updateDisplay() {

    if (dispMode == 0)
        return;

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setFont(&fonts::Font2);

    int y = 2;
    int w = M5.Display.width();

    if (dispMode == 1) {

        uint32_t phaseColor;

        switch (currentPhase) {

            case 0:
                phaseColor = TFT_DARKGREY;
                break;

            case 1:
                phaseColor = TFT_CYAN;
                break;

            case 2:
                phaseColor = TFT_YELLOW;
                break;

            case 3:
                phaseColor = TFT_GREEN;
                break;

            case 4:
                phaseColor = TFT_ORANGE;
                break;

            default:
                phaseColor = TFT_WHITE;
                break;
        }

        M5.Display.fillRect(
            0,
            y,
            w,
            18,
            TFT_NAVY
        );

        M5.Display.setTextColor(
            phaseColor,
            TFT_NAVY
        );

        M5.Display.setCursor(
            5,
            y + 2
        );

        M5.Display.printf(
            "PHASE: %d",
            currentPhase
        );

        bool wifiOk =
            (WiFi.status() == WL_CONNECTED);

        M5.Display.setTextColor(
            wifiOk ?
                TFT_GREEN :
                TFT_RED,
            TFT_NAVY
        );

        M5.Display.setCursor(
            80,
            y + 2
        );

        M5.Display.printf(
            "WiFi:%s",
            wifiOk ? "OK" : "NG"
        );

        y += 22;

        M5.Display.setTextColor(
            TFT_WHITE,
            TFT_BLACK
        );

        M5.Display.setCursor(
            5,
            y
        );

        M5.Display.print(
            "[ M5 Battery ]"
        );

        y += 16;

        uint32_t batColor;

        if (m5BatLevel >= 0.5f)
            batColor = TFT_GREEN;

        else if (m5BatLevel >= 0.20f)
            batColor = TFT_YELLOW;

        else
            batColor = TFT_RED;

        int barW =
            (int)(
                m5BatLevel *
                100.0f
            );

        M5.Display.fillRect(
            5,
            y,
            100,
            10,
            TFT_DARKGREY
        );

        M5.Display.fillRect(
            5,
            y,
            barW,
            10,
            batColor
        );

        M5.Display.drawRect(
            5,
            y,
            100,
            10,
            TFT_WHITE
        );

        M5.Display.setTextColor(
            batColor,
            TFT_BLACK
        );

        M5.Display.setCursor(
            110,
            y
        );

        M5.Display.printf(
            "%3.0f%%",
            m5BatLevel * 100.0f
        );

        y += 14;

        const char* m5StateStr;
        uint32_t m5StateColor;

        switch (m5BatState) {

            case 0:
                m5StateStr = "Discharging";
                m5StateColor = TFT_WHITE;
                break;

            case 1:
                m5StateStr = "Charging...";
                m5StateColor = TFT_CYAN;
                break;

            case 2:
                m5StateStr = "Full";
                m5StateColor = TFT_GREEN;
                break;

            default:
                m5StateStr = "---";
                m5StateColor = TFT_WHITE;
                break;
        }

        M5.Display.setTextColor(
            m5StateColor,
            TFT_BLACK
        );

        M5.Display.setCursor(
            5,
            y
        );

        M5.Display.print(
            m5StateStr
        );

        y += 18;

        M5.Display.drawFastHLine(
            0,
            y,
            w,
            TFT_DARKGREY
        );

        y += 4;

        M5.Display.setTextColor(
            TFT_WHITE,
            TFT_BLACK
        );

        M5.Display.setCursor(
            5,
            y
        );

        M5.Display.print(
            "[ LiPo Battery ]"
        );

        y += 16;

        uint32_t lipoColor;

        if (lipoVoltage >= 3.70f)
            lipoColor = TFT_GREEN;

        else if (lipoVoltage >= 3.55f)
            lipoColor = TFT_YELLOW;

        else
            lipoColor = TFT_RED;

        M5.Display.setTextColor(
            lipoColor,
            TFT_BLACK
        );

        M5.Display.setCursor(
            5,
            y
        );

        M5.Display.printf(
            "%.2fV",
            lipoVoltage
        );

        y += 14;

        const char* lipoStateStr;
        uint32_t lipoStateColor;

        switch (lipoBatState) {

            case 0:
                lipoStateStr = "Not charging";
                lipoStateColor = TFT_WHITE;
                break;

            case 1:
                lipoStateStr = "Charging...";
                lipoStateColor = TFT_CYAN;
                break;

            case 2:
                lipoStateStr = "Full";
                lipoStateColor = TFT_GREEN;
                break;

            default:
                lipoStateStr = "---";
                lipoStateColor = TFT_WHITE;
                break;
        }

        M5.Display.setTextColor(
            lipoStateColor,
            TFT_BLACK
        );

        M5.Display.setCursor(
            5,
            y
        );

        M5.Display.print(
            lipoStateStr
        );

    } else if (dispMode == 2) {

        M5.Display.fillRect(
            0,
            y,
            w,
            18,
            TFT_NAVY
        );

        M5.Display.setTextColor(
            TFT_WHITE,
            TFT_NAVY
        );

        M5.Display.setCursor(
            5,
            y + 2
        );

        M5.Display.print(
            "OSC LOG Monitor"
        );

        y += 22;

        M5.Display.setTextColor(
            TFT_CYAN,
            TFT_BLACK
        );

        M5.Display.setCursor(
            5,
            y
        );

        M5.Display.print(
            "TX:"
        );

        y += 14;

        M5.Display.setTextColor(
            TFT_WHITE,
            TFT_BLACK
        );

        M5.Display.setCursor(
            10,
            y
        );

        M5.Display.print(
            oscLogTx != "" ?
                oscLogTx :
                "---"
        );

        y += 18;

        M5.Display.drawFastHLine(
            0,
            y,
            w,
            TFT_DARKGREY
        );

        y += 4;

        M5.Display.setTextColor(
            TFT_YELLOW,
            TFT_BLACK
        );

        M5.Display.setCursor(
            5,
            y
        );

        M5.Display.print(
            "RX:"
        );

        y += 14;

        M5.Display.setTextColor(
            TFT_WHITE,
            TFT_BLACK
        );

        M5.Display.setCursor(
            10,
            y
        );

        M5.Display.print(
            oscLogRx != "" ?
                oscLogRx :
                "---"
        );

        y += 18;

        M5.Display.drawFastHLine(
            0,
            y,
            w,
            TFT_DARKGREY
        );

        y += 4;

        uint32_t phaseColor;

        switch (currentPhase) {

            case 0:
                phaseColor = TFT_DARKGREY;
                break;

            case 1:
                phaseColor = TFT_CYAN;
                break;

            case 2:
                phaseColor = TFT_YELLOW;
                break;

            case 3:
                phaseColor = TFT_GREEN;
                break;

            case 4:
                phaseColor = TFT_ORANGE;
                break;

            default:
                phaseColor = TFT_WHITE;
                break;
        }

        M5.Display.setTextColor(
            phaseColor,
            TFT_BLACK
        );

        M5.Display.setCursor(
            5,
            y
        );

        M5.Display.printf(
            "PHASE: %d",
            currentPhase
        );
    }
}

// ============================================================
//   OSCログ
// ============================================================
void addOscLogTx(String msg) {

    oscLogTx = msg;

    if (dispMode == 2)
        updateDisplay();
}

void addOscLogRx(String msg) {

    oscLogRx = msg;

    if (dispMode == 2)
        updateDisplay();
}
