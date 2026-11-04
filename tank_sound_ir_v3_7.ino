/*
 * ESP32 坦克聲光 + 1/16 IR 對戰系統  v3.7 (IR 互通 + 燈光系統)
 * ------------------------------------------------------------
 * v3.6 -> v3.7：
 *  [IR  ] 取消「揀協議」：接收永遠同時比對所有已設定協議；開炮時依序發射所有「波形唔同」嘅協議
 *         (相同波形只發一次，例如 Tamiya 同 Heng Long)。每個協議各自有連續幀計數、鎖定時間，
 *         同一發射擊嘅後續協議唔會重複算命中。裝填時間按整個發射序列自動計算。
 *  [燈光] 6 路 PWM 燈光系統 (可調亮度、淡入淡出)：
 *         大燈 (近光/遠光，跟遙控頻道或網頁強制) / 尾燈 (跟大燈微亮，放開搖桿煞車全亮) /
 *         機槍閃光 / 炮口火光 (瞬亮再漸暗) / 受擊指示 (閃爍，並令大燈尾燈閃爍損壞效果) /
 *         輔助燈 (遙控頻道開關)；遙控斷線時大燈尾燈交替閃；開機 6 盞燈依序自檢。
 *  [網頁] 新增「燈光」分頁 (即時亮度、每路最大亮度、測試、大燈模式)；IR 分頁改為互通狀態。
 *  其餘 (遙控/音效/系統/記錄/診斷報告、RAM 快取、混音輸出) 與 v3.6 相同。
 * v3.7 -> v3.7.1 (腳位審查)：
 *  - IR 發射腳 GPIO14 -> GPIO33：GPIO14 係 JTAG 嘅 TMS 腳，開機時有內部上拉；GPIO33 冇開機/JTAG 功能
 *  - LEDC 頻道重新分配 (燈 0,1,2,4,5,8；IR 載波 7)：無論 channel->timer 對應方式係 (ch/2)%4 定 ch%4，
 *    38kHz 載波都唔會同 5kHz 燈光共用 timer
 *  - 新增編譯期腳位檢查：撞腳位 / 用咗 flash、USB 序列埠、開機腳、JTAG 腳 / 輸入專用腳做輸出 /
 *    電池腳撞其他功能 / LEDC 頻道共用 timer，全部會直接編譯失敗，並講明邊隻腳有問題
 *  - 診斷報告新增 [腳位] 一行
 * ------------------------------------------------------------
 * 接線摘要：
 *  I2S 功放 MAX98357A : BCLK=GPIO26  LRC(WS)=GPIO25  DIN=GPIO22  VIN=5V  GND=GND  (SD 懸空)
 *  iBUS 接收機        : iBUS 訊號 -> GPIO16 (RX2)   共地
 *  IR 接收頭 (38kHz)  : OUT=GPIO27  VCC=3.3V  GND
 *  IR 發射            : GPIO33 -> 1k -> NPN 基極 (基極再加 10k 落 GND)；集電極 -> IR LED 負極；LED 正極 -> 47~100R -> 5V
 *  燈光：紅/橙/黃 LED 可 GPIO -> 220~330R -> LED -> GND；白/藍 LED (順向電壓約 3V) 同大功率燈要用 NPN/MOSFET 由 5V 驅動
 *    大燈 GPIO4 / 機槍閃光 GPIO18 / 炮口火光 GPIO19 / 尾燈 GPIO21 / 受擊指示 GPIO23 / 輔助燈 GPIO32
 *    (GPIO4 唔好加外部上拉)
 *  (選用) 電池電壓    : 分壓後接 GPIO34 / 35 / 36 / 39 (ADC1 輸入專用腳；GPIO32、33 已用)
 * ------------------------------------------------------------
 * 編譯環境：Arduino-ESP32 2.0.17 (建議)；3.x 可編譯，但 legacy I2S 會有 deprecated 警告
 * 開發板：ESP32 Dev Module / Flash 4MB / Partition Scheme = No OTA (2MB APP/2MB SPIFFS)
 * 音效格式：WAV / PCM / Mono / 16-bit / 22050 Hz
 * 注意：資料夾名稱必須與 .ino 檔名一致 (tank_sound_ir_v3_7)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <stdarg.h>
#include "driver/i2s.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#if __has_include("esp_random.h")
#include "esp_random.h"
#endif
#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif

// ===================== 基本設定 =====================
#define FW_VERSION "v3.7.1"
const char* WIFI_SSID     = "Tank_Battle_Sound";
const char* WIFI_PASSWORD = "12345678";          // 建議自行更改 (>=8 字元)

constexpr i2s_port_t I2S_PORT     = I2S_NUM_0;
constexpr uint32_t   AUDIO_RATE   = 22050;
constexpr size_t     AUDIO_FRAMES = 128;         // 每次混音 frame 數 (約 5.8ms)
constexpr int        I2S_DMA_COUNT = 6;
constexpr int        I2S_DMA_LEN   = 128;        // 6x128 = 約 35ms 緩衝
constexpr uint32_t   BLOCK_PERIOD_US = (uint32_t)(1000000ull * AUDIO_FRAMES / AUDIO_RATE);

// --- 腳位 ---
constexpr int IBUS_RX_PIN = 16;                  // 若用 WROVER/PSRAM 模組，16/17 會衝突
constexpr int IBUS_TX_PIN = 17;
constexpr int I2S_BCLK_PIN = 26;
constexpr int I2S_WS_PIN   = 25;
constexpr int I2S_DATA_PIN = 22;
constexpr int LED_HEADLIGHT_PIN = 4;             // 大燈
constexpr int LED_TAIL_PIN      = 21;            // 尾燈 / 煞車燈
constexpr int LED_MG_PIN        = 18;            // 機槍閃光
constexpr int LED_FIRE_PIN      = 19;            // 炮口火光
constexpr int LED_HIT_PIN       = 23;            // 受擊指示
constexpr int LED_AUX_PIN       = 32;            // 輔助燈
constexpr int IR_RECEIVE_PIN    = 27;            // 38kHz 解調接收頭 (低電平=有訊號)；必須 < 32
constexpr int IR_SEND_PIN       = 33;            // 必須經電晶體驅動 IR LED，唔好直推 (舊版用 GPIO14：JTAG TMS 腳，開機有上拉)
static_assert(IR_RECEIVE_PIN < 32, "IR_RECEIVE_PIN must be GPIO0-31 (register read in ISR)");

// --- (選用) 電池電壓偵測：必須用 ADC1 腳 (GPIO32~39)，ADC2 喺 Wi-Fi 啟動後不能用 ---
#ifndef BATT_ADC_PIN_VALUE
#define BATT_ADC_PIN_VALUE -1
#endif
#ifndef BATT_LOW_MV_VALUE
#define BATT_LOW_MV_VALUE 0
#endif
constexpr int      BATT_ADC_PIN   = BATT_ADC_PIN_VALUE;   // -1 = 停用
constexpr float    BATT_DIV_RATIO = 3.0f;
constexpr uint32_t BATT_LOW_MV    = BATT_LOW_MV_VALUE;
static_assert(BATT_ADC_PIN < 0 || (BATT_ADC_PIN >= 32 && BATT_ADC_PIN <= 39), "BATT_ADC_PIN must be an ADC1 pin (GPIO32-39)");

// ---- 腳位編譯期檢查：任何改動令腳位撞車、或者用咗唔可用嘅腳位，會直接編譯失敗 (C++11 寫法，Arduino-ESP32 2.0.x 可用) ----
constexpr int PIN_LIST[] = { IBUS_RX_PIN, IBUS_TX_PIN, I2S_BCLK_PIN, I2S_WS_PIN, I2S_DATA_PIN, LED_HEADLIGHT_PIN, LED_TAIL_PIN,
                             LED_MG_PIN, LED_FIRE_PIN, LED_HIT_PIN, LED_AUX_PIN, IR_RECEIVE_PIN, IR_SEND_PIN };
constexpr int PIN_N = (int)(sizeof(PIN_LIST) / sizeof(PIN_LIST[0]));
constexpr bool pinsAllDifferent(int i, int j) {
  return i >= PIN_N ? true : (j >= PIN_N ? pinsAllDifferent(i + 1, i + 2) : (PIN_LIST[i] != PIN_LIST[j] && pinsAllDifferent(i, j + 1)));
}
constexpr bool pinInList(int p, int i) { return i < PIN_N && (PIN_LIST[i] == p || pinInList(p, i + 1)); }
constexpr bool pinReserved(int p) {   // 6-11 連 flash；1/3 USB 序列埠；0/2/5/12/15 開機腳；12-15 同時係 JTAG 腳 (13/14 開機有上拉)
  return (p >= 6 && p <= 11) || p == 1 || p == 3 || p == 0 || p == 2 || p == 5 || p == 12 || p == 13 || p == 14 || p == 15;
}
constexpr bool pinOutputOk(int p) { return p >= 0 && p <= 33 && !pinReserved(p); }   // 34~39 只可輸入
constexpr bool pinInputOk(int p)  { return p >= 0 && p <= 39 && !pinReserved(p); }
static_assert(pinsAllDifferent(0, 1), "兩個功能用咗同一隻 GPIO (見 PIN_LIST)");
#define PIN_OUT(p) static_assert(pinOutputOk(p), #p " 唔可以做輸出：GPIO6-11 連 flash、1/3 係 USB 序列埠、0/2/5/12/13/14/15 係開機或 JTAG 腳、34-39 只可輸入")
#define PIN_IN(p)  static_assert(pinInputOk(p),  #p " 唔可以用：GPIO6-11 連 flash、1/3 係 USB 序列埠、0/2/5/12/13/14/15 係開機或 JTAG 腳")
PIN_OUT(IBUS_TX_PIN); PIN_OUT(I2S_BCLK_PIN); PIN_OUT(I2S_WS_PIN); PIN_OUT(I2S_DATA_PIN);
PIN_OUT(LED_HEADLIGHT_PIN); PIN_OUT(LED_TAIL_PIN); PIN_OUT(LED_MG_PIN); PIN_OUT(LED_FIRE_PIN); PIN_OUT(LED_HIT_PIN); PIN_OUT(LED_AUX_PIN);
PIN_OUT(IR_SEND_PIN);
PIN_IN(IBUS_RX_PIN); PIN_IN(IR_RECEIVE_PIN);
static_assert(BATT_ADC_PIN < 0 || !pinInList(BATT_ADC_PIN, 0), "BATT_ADC_PIN 撞咗其他功能嘅腳位 (GPIO32、33 已用，請用 34/35/36/39)");

// --- iBUS 頻道預設對應 (0 起算)；網頁可修改並存入 NVS ---
constexpr int CH_LEFT   = 0;
constexpr int CH_RIGHT  = 1;
constexpr int CH_TURRET = 2;
constexpr int CH_ELEV   = 3;
constexpr int CH_FIRE   = 4;
constexpr int CH_MG     = 5;
constexpr int CH_LIGHT  = 6;     // 大燈：<1300 關、1300~1700 近光、>1700 遠光 (2 段掣 = 關/遠光；3 段掣 = 關/近光/遠光)
constexpr int CH_AUX    = 7;     // 輔助燈：>1500 開
constexpr uint32_t IBUS_TIMEOUT_MS = 500;

// --- 行為參數 ---
constexpr uint32_t AUTO_STOP_MS     = 10000;
#ifndef FIRE_RELOAD_MS_VALUE
#define FIRE_RELOAD_MS_VALUE 1300
#endif
#ifndef IR_RELOAD_DYNAMIC
#define IR_RELOAD_DYNAMIC 1
#endif
constexpr uint32_t FIRE_RELOAD_MS   = FIRE_RELOAD_MS_VALUE;
constexpr uint32_t HIT_COOLDOWN     = 900;
constexpr int      STICK_ACTIVE     = 80;
constexpr int      TURRET_ON        = 150;
constexpr int      TURRET_OFF       = 110;
constexpr uint32_t MAX_SOUND_BYTES  = 1500000;
constexpr int      IR_TX_CORE       = 1;
constexpr bool     IR_SERIAL_DUMP   = true;
constexpr uint32_t AUDIO_SLOW_US    = 20000;
constexpr uint32_t SETTINGS_DEBOUNCE_MS = 3000;
constexpr uint32_t SETTINGS_MAX_WAIT_MS = 30000;
constexpr int8_t   REQ_NONE         = -2;
constexpr uint32_t TEST_HOLD_MS     = 1500;
constexpr uint32_t LOW_HEAP_BYTES   = 40000;
constexpr uint32_t IR_NOISE_EPS     = 150;
constexpr int      LOG_N            = 48;

// --- 燈光參數 ---
constexpr uint32_t LIGHT_PWM_HZ     = 5000;
constexpr int      LIGHT_HEAD_LOW   = 90;      // 近光亮度 (0~255)
constexpr int      LIGHT_TAIL_RUN   = 50;      // 尾燈微亮
constexpr uint32_t LIGHT_BRAKE_MS   = 800;     // 放開搖桿後煞車燈全亮時間
constexpr uint32_t LIGHT_HIT_MS     = 1500;    // 受擊指示閃爍時間
constexpr uint32_t LIGHT_DAMAGE_MS  = 1200;    // 大燈/尾燈損壞閃爍時間
constexpr uint32_t LIGHT_TEST_MS    = 800;     // 網頁測試燈光時間
constexpr uint32_t LIGHT_CHASE_MS   = 150;     // 開機自檢每盞燈亮咗幾耐

// --- 混音音量 ---
constexpr float ENGINE_VOL = 0.55f;
const float VOICE_VOL[] = { 0.90f, 1.00f, 1.00f, 0.70f, 0.45f, 0.45f };

// --- IR 參數 ---
constexpr int      IR_FRAME_MAX       = 96;
constexpr uint32_t IR_FRAME_GAP_US    = 5000;
constexpr uint32_t IR_FRAME_TIMEOUT_US= 12000;
constexpr int      IR_TOL_ABS_US      = 120;
constexpr int      IR_TOL_PCT         = 30;
constexpr int      IR_LEN_TOL         = 2;
constexpr int      IR_LEN_TOL_MIN     = 15;
constexpr uint8_t  IR_FRAMES_FOR_HIT  = 3;
constexpr uint32_t IR_MATCH_WINDOW_MS = 120;
constexpr uint32_t IR_LATCH_RELEASE_MS= 150;
constexpr uint32_t IR_TX_MAX_US       = 1500000;  // 單個協議單次發射最長時間
constexpr uint32_t IR_SEQ_GAP_MS      = 50;       // 協議與協議之間嘅空隙
constexpr uint32_t IR_SHOT_MAX_MS     = 4500;     // 一發射擊 (全部協議) 最長時間，超出嘅協議唔發
constexpr uint32_t IR_CARRIER_HZ      = 38000;
constexpr int      IR_CARRIER_DUTY    = 128;
constexpr int      IR_LEDC_CH         = 7;        // 僅 2.x：IR 載波 38kHz 專用；燈光用 0,1,2,4,5,8 (見下面檢查)
constexpr size_t   IR_RING_SIZE       = 256;
constexpr int      LRN_MAX            = 6;

constexpr size_t RD_BUF = 1024;
constexpr size_t RD_BUF_ENGINE = 2048;
#ifndef RAM_CACHE_BUDGET_BYTES
#define RAM_CACHE_BUDGET_BYTES (96 * 1024)
#endif
#ifndef RAM_CACHE_MIN_FREE_BYTES
#define RAM_CACHE_MIN_FREE_BYTES (80 * 1024)
#endif
#ifndef PCM_ALLOC
#define PCM_ALLOC(n) malloc(n)
#define PCM_FREE(p) free(p)
#endif
constexpr uint32_t RAM_CACHE_BUDGET = RAM_CACHE_BUDGET_BYTES;
constexpr uint32_t RAM_CACHE_MIN_FREE = RAM_CACHE_MIN_FREE_BYTES;
constexpr float FADE_STEP = 1.0f / (0.012f * AUDIO_RATE);
constexpr float PITCH_IDLE = 0.65f, PITCH_MAX = 1.90f;

#define INC(x) ((x) = (x) + 1)

volatile uint32_t statFlashReads = 0, statFlashBytes = 0;

// ===================== 型別 (必須全部放喺第一個函式之前) =====================
enum EngineState : uint8_t { ENGINE_OFF = 0, ENGINE_STARTING, ENGINE_IDLE_RUN, ENGINE_STOPPING };
enum SoundId { S_ENGINE = 0, S_START, S_STOP, S_FIRE, S_MG, S_TURRET, S_ELEV, S_HIT, S_COUNT };
enum VoiceId { V_SEQ = 0, V_FIRE, V_HIT, V_MG, V_TURRET, V_ELEV, V_COUNT };
enum IRProtocol : uint8_t {
  IR_PROTO_TAMIYA = 0, IR_PROTO_TAMIYA_2SHOT, IR_PROTO_TAIGEN_V1, IR_PROTO_TAIGEN_V2V3,
  IR_PROTO_HENGLONG, IR_PROTO_COUNT
};
enum LearnState { LRN_IDLE = 0, LRN_WAIT, LRN_COLLECT, LRN_OK, LRN_FAIL };
enum RcFn { FN_LEFT = 0, FN_RIGHT, FN_TURRET, FN_ELEV, FN_FIRE, FN_MG, FN_LIGHT, FN_AUX, FN_COUNT };
enum LightId { LT_HEAD = 0, LT_TAIL, LT_MG, LT_FIRE, LT_HIT, LT_AUX, LT_COUNT };
enum WavReason : uint8_t {
  WR_OK = 0, WR_MISSING, WR_NOT_WAV, WR_NO_CHUNK, WR_NOT_PCM, WR_NOT_MONO, WR_BAD_RATE, WR_BAD_BITS,
  WR_OPEN_FAIL, WR_SIZE_CHANGED, WR_COUNT
};
enum AlertBit {
  AL_NO_LINK = 0, AL_WAV_MISSING, AL_WAV_BAD, AL_LOW_HEAP, AL_AUDIO_SLOW, AL_BROWNOUT, AL_IR_UNSET,
  AL_IR_NOISE, AL_LOW_BATT, AL_FS_FULL, AL_STACK_LOW, AL_CRASH_RESET, AL_IBUS_ERR
};

struct Inputs {
  int left = 1500, right = 1500, turret = 1500, elev = 1500;
  bool fire = false, mg = false, aux = false;
  uint8_t headLevel = 0;                        // 0 關 / 1 近光 / 2 遠光 (按遙控頻道)
};

struct IRProfile {
  uint8_t  valid, len, repeats, builtin;
  uint16_t periodUs;
  uint16_t d[IR_FRAME_MAX];                     // mark, space, mark, ... mark (接收頭量度值)
};

struct IRPlan {                                 // 一發射擊嘅發射序列
  uint8_t n;
  uint8_t idx[IR_PROTO_COUNT];
  uint32_t totalMs;
};

struct WavInfo {
  uint32_t dataOffset = 0, dataSize = 0, sampleRate = 0;
  uint16_t channels = 0, bitsPerSample = 0, audioFormat = 0;
};

struct SoundMeta {
  bool ok = false;
  uint8_t reason = WR_MISSING;
  uint32_t size = 0, dataOff = 0, dataLen = 0;
  uint32_t rate = 0;
  uint16_t fmt = 0, chans = 0, bits = 0;
  int16_t* ram = nullptr;
};

struct SoundItem { const char* name; const char* label; };
struct LogEntry { uint32_t seq; uint32_t ms; char msg[56]; };

struct FrameSnap {                              // 單個 IR frame 快照 (seqlock)
  volatile uint32_t seq = 0;
  uint16_t d[IR_FRAME_MAX];
  volatile uint8_t len = 0;
  volatile uint32_t ms = 0;
  volatile int8_t proto = -1;                   // >=0 命中嘅協議；-1 未識別；-2 學習中
  void put(const uint16_t* src, uint8_t n, int8_t pr) {
    if (n > IR_FRAME_MAX) n = IR_FRAME_MAX;
    seq = seq + 1;
    memcpy(d, src, n * sizeof(uint16_t));
    len = n; ms = millis(); proto = pr;
    seq = seq + 1;
  }
  bool get(uint16_t* out, uint8_t& n, uint32_t& t, int8_t& pr) const {
    for (int tries = 0; tries < 4; tries++) {
      uint32_t s1 = seq;
      if (s1 & 1u) continue;
      uint8_t l = len;
      if (l > IR_FRAME_MAX) l = IR_FRAME_MAX;
      memcpy(out, d, l * sizeof(uint16_t));
      uint32_t m = ms;
      int8_t p = proto;
      if (seq == s1) { n = l; t = m; pr = p; return true; }
    }
    return false;
  }
};

template <size_t BUFSZ>
struct WavReaderT {                             // 優先由 RAM 讀；否則以 BUFSZ 緩衝由 flash 串流
  File f;
  const int16_t* ram = nullptr;
  uint32_t ramSamples = 0;
  bool isOpen = false;
  bool ioFail = false;
  uint32_t dataOff = 0, dataLen = 0, bufStart = 0xFFFFFFFFu;
  uint16_t bufLen = 0;
  uint8_t buf[BUFSZ];

  bool open(const SoundMeta& m, const char* path) {
    close();
    if (!m.ok) return false;
    if (m.ram) { ram = m.ram; ramSamples = m.dataLen >> 1; dataLen = m.dataLen; isOpen = true; return true; }
    f = LittleFS.open(path, "r");
    if (!f) return false;
    if ((uint32_t)f.size() != m.size) { f.close(); return false; }
    dataOff = m.dataOff; dataLen = m.dataLen;
    bufStart = 0xFFFFFFFFu; bufLen = 0; isOpen = true;
    return true;
  }
  void close() {
    if (isOpen && !ram) f.close();
    ram = nullptr; ramSamples = 0; isOpen = false; ioFail = false;
    bufStart = 0xFFFFFFFFu; bufLen = 0;
  }
  uint32_t samples() const { return dataLen >> 1; }
  inline bool at(uint32_t idx, int16_t& s) {
    if (ram) {
      if (idx >= ramSamples) return false;
      s = ram[idx];
      return true;
    }
    uint32_t o = idx << 1;
    if (!isOpen || o + 2 > dataLen) return false;
    if (bufStart == 0xFFFFFFFFu || o < bufStart || o + 2 > bufStart + bufLen) {
      uint32_t n = dataLen - o;
      if (n > BUFSZ) n = BUFSZ;
      f.seek(dataOff + o);
      int got = (int)f.read(buf, n);
      INC(statFlashReads);
      if (got > 0) statFlashBytes = statFlashBytes + (uint32_t)got;
      if (got < 2) { bufStart = 0xFFFFFFFFu; bufLen = 0; ioFail = true; return false; }
      bufStart = o; bufLen = (uint16_t)(got & ~1);
    }
    uint32_t k = o - bufStart;
    s = (int16_t)(buf[k] | (buf[k + 1] << 8));
    return true;
  }
};
typedef WavReaderT<RD_BUF> WavReader;
typedef WavReaderT<RD_BUF_ENGINE> EngReader;

struct Voice {
  WavReader r;
  bool active = false, loop = false;
  int8_t snd = -1;
  uint32_t pos = 0;
  float gain = 1.f, target = 1.f;
};

struct EngineVoice {
  EngReader r;
  bool ready = false;
  float pos = 0, pitch = PITCH_IDLE, pitchPrev = PITCH_IDLE, fLoad = 0, jitter = 0, gain = 0;
  uint32_t retryAt = 0;
};

// ===================== 全域狀態 =====================
WebServer server(80);
Preferences prefs;

volatile EngineState engineState = ENGINE_OFF;
volatile bool ignitionCommand = false;
volatile bool webFireRequest  = false;
volatile bool hitCommand      = false;
volatile float globalVolume   = 0.8f;
volatile bool  ibusLinkOk     = false;
Inputs curIn;
uint8_t fnState = 0;                              // bit0 開炮掣 bit1 機槍 bit2 炮塔動 bit3 升降動 bit4 大燈亮 bit5 輔助燈

uint16_t debugChannels[14] = {1500,1500,1500,1500,1000,1000,1500,1500,1500,1500,1500,1500,1500,1500};
int8_t chMap[FN_COUNT] = { CH_LEFT, CH_RIGHT, CH_TURRET, CH_ELEV, CH_FIRE, CH_MG, CH_LIGHT, CH_AUX };
const int8_t CH_MAP_DEFAULT[FN_COUNT] = { CH_LEFT, CH_RIGHT, CH_TURRET, CH_ELEV, CH_FIRE, CH_MG, CH_LIGHT, CH_AUX };
const char* const FN_LABEL[FN_COUNT] = { "左履帶", "右履帶", "炮塔", "炮管升降", "主炮", "機槍", "大燈", "輔助燈" };
portMUX_TYPE channelMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool audioPauseRequested = false;
volatile bool audioPaused = false;

const SoundItem SOUND_LIST[S_COUNT] = {
  {"engine","引擎 Loop"}, {"start","點火"}, {"stop","熄匙"},
  {"fire","主炮"}, {"mg","機槍"}, {"turret","炮塔"}, {"elev","升降"}, {"hit","受擊"}
};
const char* const SOUND_PATH[S_COUNT] = {
  "/engine.wav", "/start.wav", "/stop.wav", "/fire.wav", "/mg.wav", "/turret.wav", "/elev.wav", "/hit.wav"
};
const char* const WAV_REASON_TXT[WR_COUNT] = {
  "OK", "缺少檔案", "不是 WAV 檔", "缺少 fmt/data 區塊", "不是 PCM 編碼", "不是單聲道", "取樣率不是 22050Hz",
  "不是 16-bit", "開檔失敗", "檔案已變更，請重新整理"
};
const char* const ENGINE_NAME[4] = { "熄火", "點火中", "運行中", "熄匙中" };
SoundMeta soundMeta[S_COUNT];
volatile uint32_t ramCacheBytes = 0;
volatile bool audioTaskStarted = false;
volatile uint32_t audioStackFree = 0, irStackFree = 0, loopStackFree = 0;
volatile uint32_t fsUsedCache = 0, fsTotalCache = 0;

Voice voices[V_COUNT];
EngineVoice eng;
uint32_t testUntil[V_COUNT];
volatile int8_t playReq = REQ_NONE;

volatile uint32_t wavPlays[S_COUNT], wavLastMs[S_COUNT], wavOpenFail[S_COUNT], wavIoErr[S_COUNT];
bool wavFailLogged[S_COUNT];
uint32_t wavIoLogMs[S_COUNT], wavFailCountMs[S_COUNT];

volatile uint32_t audioMaxUs = 0;
volatile uint32_t audioWinMaxUs = 0, audioMaxRecentUs = 0;
volatile uint32_t audioSlow = 0, audioBlocks = 0, audioSlowLastMs = 0;
volatile float audioAvgUs = 0;
uint32_t audioSlowLogMs = 0;

volatile uint32_t rateIbusHz = 0, rateIrEdges = 0, rateFlashReads = 0, rateFlashBytes = 0, rateIbusErr = 0;
volatile uint32_t battMv = 0;
volatile uint32_t webRequests = 0;
bool lowHeapLogged = false;
int bootResetReason = 0;

volatile uint32_t ibusFrames = 0, ibusBadSum = 0, ibusBadRange = 0, ibusDropBytes = 0, ibusFailsafe = 0, ibusMaxGapMs = 0;
uint16_t chMin[14], chMax[14];
bool prevLink = false;
EngineState prevEngine = ENGINE_OFF;

volatile bool settingsDirty = false;
volatile uint32_t settingsDirtyMs = 0;

LogEntry logBuf[LOG_N];
volatile uint32_t logSeq = 0;
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

// --- 燈光 ---
constexpr int LIGHT_PIN[LT_COUNT] = { LED_HEADLIGHT_PIN, LED_TAIL_PIN, LED_MG_PIN, LED_FIRE_PIN, LED_HIT_PIN, LED_AUX_PIN };
// LEDC 頻道 (僅 2.x)：同一個 timer 嘅頻道必須同頻率。唔肯定 channel->timer 對應係 (ch/2)%4 定 ch%4，所以兩種都唔可以撞：
// 燈 0,1,2,4,5 + 8 (第 2 組)，IR 載波 7；3 同 6 留空，令 7 喺兩種對應下都獨佔一個 timer。
constexpr int LIGHT_LEDC[LT_COUNT] = { 0, 1, 2, 4, 5, 8 };
constexpr int ledcTimerA(int ch) { return (ch / 2) % 4; }
constexpr int ledcTimerB(int ch) { return ch % 4; }
constexpr bool ledcShare(int a, int b) { return (a / 8) == (b / 8) && (ledcTimerA(a) == ledcTimerA(b) || ledcTimerB(a) == ledcTimerB(b)); }
constexpr bool lightChOk(int i, int j) {
  return i >= LT_COUNT ? true : (j >= LT_COUNT ? lightChOk(i + 1, i + 2) : (LIGHT_LEDC[i] != LIGHT_LEDC[j] && lightChOk(i, j + 1)));
}
constexpr bool irClearsLights(int i) { return i >= LT_COUNT ? true : (!ledcShare(IR_LEDC_CH, LIGHT_LEDC[i]) && irClearsLights(i + 1)); }
static_assert(IR_LEDC_CH >= 0 && IR_LEDC_CH <= 15, "IR_LEDC_CH 必須係 0~15");
static_assert(lightChOk(0, 1), "兩路燈用咗同一個 LEDC 頻道");
static_assert(irClearsLights(0), "IR 載波 (38kHz) 嘅 LEDC 頻道同燈光 (5kHz) 共用 timer：會令 IR 載波頻率錯誤");
const char* const LIGHT_NAME[LT_COUNT] = { "大燈", "尾燈 / 煞車燈", "機槍閃光", "炮口火光", "受擊指示", "輔助燈" };
const float LIGHT_RISE[LT_COUNT] = { 12.f, 255.f, 255.f, 255.f, 255.f, 12.f };   // 每個 block (5.8ms) 最多升幾多
const float LIGHT_FALL[LT_COUNT] = { 8.f, 6.f, 255.f, 5.f, 255.f, 8.f };
uint8_t lightPct[LT_COUNT] = { 100, 100, 100, 100, 100, 100 };               // 每路最大亮度 (%)
volatile int8_t headMode = 0;                       // 0 跟遙控 / 1 關 / 2 近光 / 3 遠光
float lightVal[LT_COUNT];                           // 目前亮度 0~255 (未計最大亮度)
uint8_t lightOut[LT_COUNT];                         // 實際輸出 0~255 (已計最大亮度，網頁顯示)
uint32_t lightDuty[LT_COUNT];
uint32_t lightTestUntil[LT_COUNT];
volatile int8_t lightTestReq = REQ_NONE;
uint32_t hitFlashUntil = 0, damageUntil = 0, brakeUntil = 0;

// --- IR ---
const char* const IR_PROTO_NAME[IR_PROTO_COUNT] = {"Tamiya", "Tamiya2Shot", "TaigenV1", "TaigenV2V3", "HengLong"};
IRProfile irProfiles[IR_PROTO_COUNT];
volatile int  irMarkExcessUs = 60;

volatile uint32_t irRing[IR_RING_SIZE];
volatile uint16_t irHead = 0;
volatile uint16_t irTail = 0;
volatile uint32_t irLastEdge = 0;
volatile uint32_t irEdges = 0, irRingDrops = 0;

volatile bool irTxBusy = false;
volatile uint32_t irRxBlockUntil = 0;
volatile uint32_t irRxHits = 0;
volatile uint32_t irTxCount = 0, irTxRequests = 0, irTxSkipped = 0, irLastTxMs = 0, irLastTxBurstMs = 0;
volatile uint32_t irTxByProto[IR_PROTO_COUNT];
volatile int8_t irSavePending = -1;
volatile int8_t irLearnReq = REQ_NONE;
volatile int8_t irResetReq = REQ_NONE;
TaskHandle_t irTxTaskHandle = NULL;

uint8_t  irMatchCnt[IR_PROTO_COUNT];                // 每個協議各自嘅連續命中幀、時間、鎖定
uint32_t irMatchMs[IR_PROTO_COUNT];
bool     irLatched[IR_PROTO_COUNT];
uint32_t irBlockUntil[IR_PROTO_COUNT];

volatile uint32_t irFramesSeen = 0, irFramesMatched = 0;
volatile uint32_t irLastHitAtMs = 0;
volatile int8_t   irLastHitProto = -1;
volatile uint32_t irFramesByProto[IR_PROTO_COUNT], irHitsByProto[IR_PROTO_COUNT];
FrameSnap irUnk, irLast;

volatile int8_t  irLearnSlot = -1;
volatile uint8_t irLearnState = LRN_IDLE;
uint16_t lrnFrames[LRN_MAX][IR_FRAME_MAX];
uint8_t  lrnLens[LRN_MAX];
uint8_t  lrnN = 0;
uint16_t lrnTotal = 0;
uint32_t lrnFirstStart = 0, lrnLastStart = 0, lrnLastMs = 0, lrnStartMs = 0;

#define IR_RX_LEVEL() ((REG_READ(GPIO_IN_REG) >> IR_RECEIVE_PIN) & 1u)

uint32_t lastIbusMs = 0, idleTime = 0, lastFireMs = 0;
bool lastFire = true, turretOn = false, elevOn = false, everFired = false;
volatile uint32_t statFireEdges = 0, statFireBlocked = 0;

File uploadFile;
const char* UPLOAD_TEMP_PATH = "/upload.tmp";
String uploadTargetPath, uploadResult = "尚未上載";
bool uploadWriteOK = false, uploadPausedByMe = false;
uint32_t uploadBytes = 0, uploadBudget = 0;
int uploadSoundIdx = -1;

// ===================== 小工具 =====================
static inline uint16_t chv(int idx) { return (idx >= 0 && idx < 14) ? debugChannels[idx] : (uint16_t)1500; }
static const char* reasonTxt(int r) { return WAV_REASON_TXT[(r >= 0 && r < WR_COUNT) ? r : WR_OPEN_FAIL]; }

int soundIndexByName(const String& n) {
  for (int i = 0; i < S_COUNT; i++) if (n == SOUND_LIST[i].name) return i;
  return -1;
}

// ===================== 事件記錄 =====================
void evLog(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void evLog(const char* fmt, ...) {
  char tmp[sizeof(LogEntry::msg)];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  size_t len = strlen(tmp);                       // 截斷時唔好留低半個 UTF-8 字
  if (len == sizeof(tmp) - 1) {
    int i = (int)len - 1;
    while (i >= 0 && ((uint8_t)tmp[i] & 0xC0) == 0x80) i--;
    if (i >= 0 && (uint8_t)tmp[i] >= 0xC0) {
      int need = ((uint8_t)tmp[i] >= 0xF0) ? 4 : ((uint8_t)tmp[i] >= 0xE0) ? 3 : 2;
      if ((int)len - i < need) tmp[i] = 0;
    }
  }
  uint32_t ms = millis();
  portENTER_CRITICAL(&logMux);
  uint32_t s = logSeq + 1;
  LogEntry& e = logBuf[s % LOG_N];
  e.seq = s; e.ms = ms;
  memcpy(e.msg, tmp, sizeof(tmp));
  logSeq = s;
  portEXIT_CRITICAL(&logMux);
}

bool logGet(uint32_t seq, LogEntry& out) {
  bool ok = false;
  portENTER_CRITICAL(&logMux);
  const LogEntry& e = logBuf[seq % LOG_N];
  if (e.seq == seq) { out = e; ok = true; }
  portEXIT_CRITICAL(&logMux);
  return ok;
}

// ===================== 統計 =====================
void noteBlockTime(uint32_t us) {
  INC(audioBlocks);
  audioAvgUs = audioAvgUs + ((float)us - audioAvgUs) * 0.01f;
  if (us > audioMaxUs) audioMaxUs = us;
  if (us > audioWinMaxUs) audioWinMaxUs = us;
  if (us > AUDIO_SLOW_US) {
    INC(audioSlow);
    uint32_t now = millis();
    audioSlowLastMs = now;
    if (now - audioSlowLogMs > 5000 || audioSlowLogMs == 0) { audioSlowLogMs = now; evLog("音訊處理偏慢：%u us", (unsigned)us); }
  }
}

void noteIoError(int snd) {
  if (snd < 0 || snd >= S_COUNT) return;
  INC(wavIoErr[snd]);
  uint32_t now = millis();
  if (now - wavIoLogMs[snd] > 5000 || wavIoLogMs[snd] == 0) { wavIoLogMs[snd] = now; evLog("音效讀取錯誤：%s", SOUND_LIST[snd].name); }
}

const char* resetReasonName(int r) {
  switch (r) {
    case ESP_RST_POWERON:   return "上電";
    case ESP_RST_EXT:       return "外部重設鍵";
    case ESP_RST_SW:        return "軟體重啟";
    case ESP_RST_PANIC:     return "程式崩潰 (panic)";
    case ESP_RST_INT_WDT:   return "中斷看門狗";
    case ESP_RST_TASK_WDT:  return "任務看門狗";
    case ESP_RST_WDT:       return "看門狗";
    case ESP_RST_DEEPSLEEP: return "深度睡眠喚醒";
    case ESP_RST_BROWNOUT:  return "欠壓 (Brownout)";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "未知";
  }
}

void resetStats() {
  audioMaxUs = 0; audioWinMaxUs = 0; audioMaxRecentUs = 0; audioSlow = 0; audioBlocks = 0; audioSlowLastMs = 0; audioAvgUs = 0;
  ibusFrames = 0; ibusBadSum = 0; ibusBadRange = 0; ibusDropBytes = 0; ibusFailsafe = 0; ibusMaxGapMs = 0;
  for (int i = 0; i < 14; i++) { chMin[i] = 0xFFFF; chMax[i] = 0; }
  for (int i = 0; i < S_COUNT; i++) { wavPlays[i] = 0; wavLastMs[i] = 0; wavOpenFail[i] = 0; wavIoErr[i] = 0; wavFailCountMs[i] = 0; }
  irFramesSeen = 0; irFramesMatched = 0; irRxHits = 0; irTxCount = 0; irTxRequests = 0; irTxSkipped = 0;
  irEdges = 0; irRingDrops = 0; irLastHitAtMs = 0; irLastHitProto = -1; irLastTxMs = 0; irLastTxBurstMs = 0;
  for (int i = 0; i < IR_PROTO_COUNT; i++) { irFramesByProto[i] = 0; irHitsByProto[i] = 0; irTxByProto[i] = 0; }
  statFlashReads = 0; statFlashBytes = 0; statFireEdges = 0; statFireBlocked = 0;
}

// ===================== 設定存取 (NVS) =====================
void markSettingsDirty() { settingsDirtyMs = millis(); settingsDirty = true; }

void loadSettings() {
  if (!prefs.begin("tank", false)) return;
  uint8_t v = prefs.getUChar("vol", 80);
  if (v > 100) v = 100;
  globalVolume = (float)v / 100.0f;
  uint8_t t = prefs.getUChar("tune", 60);
  if (t > 200) t = 200;
  irMarkExcessUs = (int)t;
  for (int i = 0; i < FN_COUNT; i++) {
    char key[4] = { 'm', (char)('0' + i), 0, 0 };
    int8_t m = prefs.getChar(key, CH_MAP_DEFAULT[i]);
    chMap[i] = (m >= -1 && m <= 13) ? m : CH_MAP_DEFAULT[i];
  }
  for (int i = 0; i < LT_COUNT; i++) {
    char key[4] = { 'l', (char)('0' + i), 0, 0 };
    uint8_t p = prefs.getUChar(key, 100);
    lightPct[i] = (p <= 100) ? p : 100;
  }
  uint8_t hm = prefs.getUChar("hm", 0);
  headMode = (hm <= 3) ? (int8_t)hm : 0;
  prefs.end();
}

void saveSettings() {
  if (!prefs.begin("tank", false)) return;
  prefs.putUChar("vol", (uint8_t)lroundf(globalVolume * 100.0f));
  prefs.putUChar("tune", (uint8_t)irMarkExcessUs);
  for (int i = 0; i < FN_COUNT; i++) {
    char key[4] = { 'm', (char)('0' + i), 0, 0 };
    prefs.putChar(key, chMap[i]);
  }
  for (int i = 0; i < LT_COUNT; i++) {
    char key[4] = { 'l', (char)('0' + i), 0, 0 };
    prefs.putUChar(key, lightPct[i]);
  }
  prefs.putUChar("hm", (uint8_t)headMode);
  prefs.end();
  evLog("設定已儲存");
}

// ===================== WAV =====================
static inline uint16_t le16(const uint8_t* b) { return (uint16_t)(b[0] | (b[1] << 8)); }
static inline uint32_t le32(const uint8_t* b) {
  return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

int inspectWavCode(File& f, WavInfo& info) {
  info = WavInfo();
  if (!f) return WR_OPEN_FAIL;
  uint32_t fsz = (uint32_t)f.size();
  if (fsz < 44) return WR_NOT_WAV;
  uint8_t hdr[12];
  f.seek(0);
  if (f.read(hdr, 12) != 12 || memcmp(hdr, "RIFF", 4) || memcmp(hdr + 8, "WAVE", 4)) return WR_NOT_WAV;
  bool gotFmt = false, gotData = false;
  uint32_t p = 12;
  while (p + 8 <= fsz && !(gotFmt && gotData)) {
    uint8_t ck[8];
    f.seek(p);
    if (f.read(ck, 8) != 8) break;
    uint32_t sz = le32(ck + 4), start = p + 8;
    if (!memcmp(ck, "fmt ", 4)) {
      uint8_t b[16];
      if (sz < 16 || start + 16 > fsz || f.read(b, 16) != 16) return WR_NO_CHUNK;
      info.audioFormat = le16(b);
      info.channels = le16(b + 2);
      info.sampleRate = le32(b + 4);
      info.bitsPerSample = le16(b + 14);
      gotFmt = true;
    } else if (!memcmp(ck, "data", 4)) {
      uint32_t avail = fsz - start;
      info.dataOffset = start;
      info.dataSize = (sz > avail) ? avail : sz;
      gotData = true;
    }
    uint64_t next = (uint64_t)start + sz + (sz & 1);
    if (next > fsz) break;
    p = (uint32_t)next;
  }
  info.dataSize &= ~1u;
  if (!gotFmt || !gotData) return WR_NO_CHUNK;
  if (info.audioFormat != 1) return WR_NOT_PCM;
  if (info.channels != 1) return WR_NOT_MONO;
  if (info.sampleRate != AUDIO_RATE) return WR_BAD_RATE;
  if (info.bitsPerSample != 16) return WR_BAD_BITS;
  if (info.dataSize < 4) return WR_NO_CHUNK;
  return WR_OK;
}

bool inspectWav(File& f, WavInfo& info) { return inspectWavCode(f, info) == WR_OK; }

void refreshFsUsage() {
  fsTotalCache = (uint32_t)LittleFS.totalBytes();
  fsUsedCache  = (uint32_t)LittleFS.usedBytes();
}

void freeSoundCache() {
  for (int i = 0; i < S_COUNT; i++) {
    if (soundMeta[i].ram) { PCM_FREE(soundMeta[i].ram); soundMeta[i].ram = nullptr; }
  }
  ramCacheBytes = 0;
}

const uint8_t CACHE_ORDER[] = { S_MG, S_TURRET, S_ELEV, S_HIT, S_FIRE, S_START, S_STOP };

void loadSoundCache() {
  uint32_t used = 0;
  if (RAM_CACHE_BUDGET > 0) {
    for (uint8_t k = 0; k < sizeof(CACHE_ORDER); k++) {
      int i = CACHE_ORDER[k];
      SoundMeta& m = soundMeta[i];
      if (!m.ok || m.dataLen == 0) continue;
      if (used + m.dataLen > RAM_CACHE_BUDGET) continue;
      if ((uint64_t)ESP.getFreeHeap() < (uint64_t)m.dataLen + RAM_CACHE_MIN_FREE) continue;
      int16_t* p = (int16_t*)PCM_ALLOC(m.dataLen);
      if (!p) continue;
      bool ok = false;
      File f = LittleFS.open(SOUND_PATH[i], "r");
      if (f) {
        f.seek(m.dataOff);
        ok = ((uint32_t)f.read((uint8_t*)p, m.dataLen) == m.dataLen);
        f.close();
      }
      if (!ok) { PCM_FREE(p); continue; }
      m.ram = p;
      used += m.dataLen;
    }
  }
  ramCacheBytes = used;
}

// 只可喺音訊暫停 (或音訊任務未啟動) 時呼叫，因為會釋放/重建 RAM 快取
void refreshSoundInfo() {
  if (audioTaskStarted && !audioPaused) { Serial.println("[WARN] refreshSoundInfo skipped: audio not paused"); return; }
  freeSoundCache();
  for (int i = 0; i < S_COUNT; i++) {
    soundMeta[i] = SoundMeta();
    if (!LittleFS.exists(SOUND_PATH[i])) { soundMeta[i].reason = WR_MISSING; continue; }
    File f = LittleFS.open(SOUND_PATH[i], "r");
    if (!f) { soundMeta[i].reason = WR_OPEN_FAIL; continue; }
    WavInfo wi;
    soundMeta[i].size = (uint32_t)f.size();
    int code = inspectWavCode(f, wi);
    soundMeta[i].reason = (uint8_t)code;
    soundMeta[i].fmt = wi.audioFormat; soundMeta[i].chans = wi.channels;
    soundMeta[i].rate = wi.sampleRate; soundMeta[i].bits = wi.bitsPerSample;
    if (code == WR_OK) {
      soundMeta[i].ok = true;
      soundMeta[i].dataOff = wi.dataOffset;
      soundMeta[i].dataLen = wi.dataSize;
    }
    f.close();
  }
  loadSoundCache();
  refreshFsUsage();
}

// ===================== 音效 Voice =====================
void voiceStop(int i) { voices[i].active = false; voices[i].r.close(); voices[i].snd = -1; }

bool voiceStart(int i, int snd, bool loop) {
  Voice& v = voices[i];
  voiceStop(i);
  if (!v.r.open(soundMeta[snd], SOUND_PATH[snd])) {
    uint32_t nf = millis();                          // 每個音效每秒最多計一次失敗
    if (wavFailCountMs[snd] == 0 || nf - wavFailCountMs[snd] > 1000) { INC(wavOpenFail[snd]); wavFailCountMs[snd] = nf; }
    if (!wavFailLogged[snd]) {
      wavFailLogged[snd] = true;
      const SoundMeta& m = soundMeta[snd];
      evLog("音效開啟失敗：%s (%s)", SOUND_LIST[snd].name, reasonTxt(m.ok ? (int)WR_SIZE_CHANGED : (int)m.reason));
    }
    return false;
  }
  wavFailLogged[snd] = false;
  INC(wavPlays[snd]);
  wavLastMs[snd] = millis();
  v.snd = (int8_t)snd;
  v.pos = 0; v.loop = loop; v.active = true; v.target = 1.f; v.gain = loop ? 0.f : 1.f;
  return true;
}

void voiceHold(int i, int snd, bool want) {
  Voice& v = voices[i];
  if (want) { if (!v.active) voiceStart(i, snd, true); else v.target = 1.f; }
  else if (v.active) v.target = 0.f;
}

static inline float voiceNext(int i) {
  Voice& v = voices[i];
  if (!v.active) return 0.f;
  if (v.gain < v.target) { v.gain += FADE_STEP; if (v.gain > v.target) v.gain = v.target; }
  else if (v.gain > v.target) {
    v.gain -= FADE_STEP;
    if (v.gain <= 0.f) { voiceStop(i); return 0.f; }
  }
  int16_t s;
  if (!v.r.at(v.pos, s)) {
    if (v.r.ioFail) noteIoError(v.snd);
    if (!v.loop) { voiceStop(i); return 0.f; }
    v.pos = 0;
    if (!v.r.at(0, s)) { if (v.r.ioFail) noteIoError(v.snd); voiceStop(i); return 0.f; }
  }
  v.pos++;
  return (float)s * v.gain;
}

// ===================== 引擎 =====================
void engineClose() { eng.r.close(); eng.ready = false; eng.gain = 0; }

void engineUpdatePitch(const Inputs& in) {
  constexpr float DEADZONE = 25.f, ACCEL = 0.0436f, DECEL = 0.0130f, LPF = 0.2256f, JIT = 0.0035f;
  float l = fabsf((float)in.left - 1500.f), r = fabsf((float)in.right - 1500.f);
  if (l < DEADZONE) l = 0;
  if (r < DEADZONE) r = 0;
  float load = (l + r) * 0.25f + fmaxf(l, r) * 0.5f;
  eng.fLoad += (load - eng.fLoad) * LPF;
  float target = PITCH_IDLE + (eng.fLoad / 500.f) * (PITCH_MAX - PITCH_IDLE);
  eng.pitchPrev = eng.pitch;
  float rate = (target > eng.pitch) ? ACCEL : DECEL;
  float p = eng.pitch + (target - eng.pitch) * rate;
  eng.jitter = eng.jitter * 0.9f + ((float)((int32_t)(esp_random() % 2001) - 1000) / 1000.f) * JIT;
  p += eng.jitter;
  if (p < PITCH_IDLE * 0.92f) p = PITCH_IDLE * 0.92f;
  if (p > PITCH_MAX * 1.08f) p = PITCH_MAX * 1.08f;
  eng.pitch = p;
}

static inline int32_t softLimit(int32_t x) {
  constexpr int32_t T = 24576;
  if (x > T) x = T + ((x - T) >> 2);
  else if (x < -T) x = -T + ((x + T) >> 2);
  if (x > 32767) x = 32767;
  if (x < -32768) x = -32768;
  return x;
}

void renderBlock(int16_t* out) {
  if (audioPaused) { memset(out, 0, AUDIO_FRAMES * 2 * sizeof(int16_t)); return; }
  engineUpdatePitch(curIn);
  bool anySound = eng.ready;
  for (int v = 0; v < V_COUNT; v++) anySound = anySound || voices[v].active;
  if (!anySound) { memset(out, 0, AUDIO_FRAMES * 2 * sizeof(int16_t)); return; }
  const bool engOn = (engineState == ENGINE_IDLE_RUN);
  const float gStep = 1.0f / (0.15f * AUDIO_RATE);
  float p = eng.pitchPrev;
  const float dp = (eng.pitch - eng.pitchPrev) / (float)AUDIO_FRAMES;
  const float vol = globalVolume;

  for (size_t i = 0; i < AUDIO_FRAMES; i++) {
    float x = 0.f;
    if (eng.ready) {
      if (engOn) { eng.gain += gStep; if (eng.gain > 1.f) eng.gain = 1.f; }
      else       { eng.gain -= gStep * 2.f; if (eng.gain < 0.f) eng.gain = 0.f; }
      uint32_t n = eng.r.samples();
      if (eng.pos >= (float)n) eng.pos -= (float)n;
      if (eng.pos < 0.f || eng.pos >= (float)n) eng.pos = 0.f;
      uint32_t pi = (uint32_t)eng.pos;
      float frac = eng.pos - (float)pi;
      int16_t s0 = 0, s1 = 0;
      if (eng.r.at(pi, s0)) {
        if (!eng.r.at((pi + 1 < n) ? pi + 1 : 0, s1)) s1 = s0;
        float amp = 0.82f + (p - PITCH_IDLE) * 0.28f;
        x += ((1.f - frac) * s0 + frac * s1) * amp * eng.gain * ENGINE_VOL;
      } else {
        if (eng.r.ioFail) noteIoError(S_ENGINE);
        engineClose();
      }
      eng.pos += p;
    }
    p += dp;
    for (int v = 0; v < V_COUNT; v++) x += voiceNext(v) * VOICE_VOL[v];
    int16_t m = (int16_t)softLimit((int32_t)(x * vol));
    out[i * 2] = out[i * 2 + 1] = m;
  }
}

// ===================== iBUS =====================
bool readIBUS(uint16_t* ch) {
  static uint8_t buf[32], idx = 0;
  bool ok = false;
  while (Serial2.available()) {
    uint8_t v = Serial2.read();
    if (idx == 0) { if (v == 0x20) buf[idx++] = v; else INC(ibusDropBytes); continue; }
    if (idx == 1) {
      if (v == 0x40) buf[idx++] = v;
      else { if (v != 0x20) idx = 0; INC(ibusDropBytes); }
      continue;
    }
    buf[idx++] = v;
    if (idx < 32) continue;
    idx = 0;
    uint16_t sum = 0xFFFF;
    for (int i = 0; i < 30; i++) sum -= buf[i];
    if (sum != (uint16_t)(buf[30] | (buf[31] << 8))) { INC(ibusBadSum); continue; }
    uint16_t tmp[14]; bool valid = true;
    for (int i = 0; i < 14; i++) {
      tmp[i] = (uint16_t)(buf[2 + i * 2] | (buf[3 + i * 2] << 8));
      if (tmp[i] < 800 || tmp[i] > 2200) { valid = false; break; }
    }
    if (!valid) { INC(ibusBadRange); continue; }
    memcpy(ch, tmp, sizeof(tmp)); ok = true;
    INC(ibusFrames);
  }
  return ok;
}

// ===================== 燈光系統 =====================
static void lightWrite(int i, uint32_t duty) {
  if (LIGHT_PIN[i] < 0) return;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(LIGHT_PIN[i], duty);
#else
  ledcWrite(LIGHT_LEDC[i], duty);
#endif
}

void lightsInit() {
  for (int i = 0; i < LT_COUNT; i++) {
    lightVal[i] = 0; lightOut[i] = 0; lightDuty[i] = 0;
    if (LIGHT_PIN[i] < 0) continue;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(LIGHT_PIN[i], LIGHT_PWM_HZ, 8);
#else
    ledcSetup(LIGHT_LEDC[i], LIGHT_PWM_HZ, 8);
    ledcAttachPin(LIGHT_PIN[i], LIGHT_LEDC[i]);
#endif
    lightWrite(i, 0);
  }
}

static uint8_t headLevelOf(uint16_t v) { return v < 1300 ? 0 : (v < 1700 ? 1 : 2); }

// 每個 block 執行一次 (音訊暫停時都會執行，所以上載期間燈光照常)
void lightsTick(uint32_t now) {
  static uint32_t t0 = 0;
  static bool prevActive = false;
  if (!t0) t0 = now | 1u;

  bool active = abs(curIn.left - 1500) > STICK_ACTIVE || abs(curIn.right - 1500) > STICK_ACTIVE;
  if (prevActive && !active && engineState != ENGINE_OFF) brakeUntil = now + LIGHT_BRAKE_MS;
  prevActive = active;

  int8_t tr = lightTestReq;                                  // 網頁測試：由音訊任務執行
  if (tr != REQ_NONE) { lightTestReq = REQ_NONE; if (tr >= 0 && tr < LT_COUNT) lightTestUntil[tr] = now + LIGHT_TEST_MS; }

  float tgt[LT_COUNT] = { 0, 0, 0, 0, 0, 0 };
  int lvl = (headMode == 0) ? (int)curIn.headLevel : (int)headMode - 1;
  tgt[LT_HEAD] = (lvl == 0) ? 0.f : (lvl == 1 ? (float)LIGHT_HEAD_LOW : 255.f);
  tgt[LT_TAIL] = (lvl > 0) ? (float)LIGHT_TAIL_RUN : 0.f;
  if ((int32_t)(brakeUntil - now) > 0) tgt[LT_TAIL] = 255.f;
  tgt[LT_MG]  = (curIn.mg && ((now / 50) & 1)) ? 255.f : 0.f;
  tgt[LT_AUX] = curIn.aux ? 255.f : 0.f;
  if ((int32_t)(hitFlashUntil - now) > 0) tgt[LT_HIT] = ((now / 60) & 1) ? 255.f : 0.f;
  if ((int32_t)(damageUntil - now) > 0 && !(((now / 40) * 2654435761u >> 13) & 1u)) { tgt[LT_HEAD] = 0; tgt[LT_TAIL] = 0; }
  if (lastIbusMs != 0 && !ibusLinkOk) {                      // 失控警示：大燈尾燈交替閃
    bool ph = (now / 250) & 1;
    tgt[LT_HEAD] = ph ? 255.f : 0.f; tgt[LT_TAIL] = ph ? 0.f : 255.f;
  }
  if (now - t0 < (uint32_t)LT_COUNT * LIGHT_CHASE_MS) {      // 開機自檢：6 盞燈依序亮
    int k = (int)((now - t0) / LIGHT_CHASE_MS);
    for (int i = 0; i < LT_COUNT; i++) tgt[i] = (i == k) ? 255.f : 0.f;
  }
  for (int i = 0; i < LT_COUNT; i++) if ((int32_t)(lightTestUntil[i] - now) > 0) tgt[i] = 255.f;

  for (int i = 0; i < LT_COUNT; i++) {
    float v = lightVal[i];
    if (tgt[i] > v) { v += LIGHT_RISE[i]; if (v > tgt[i]) v = tgt[i]; }
    else if (tgt[i] < v) { v -= LIGHT_FALL[i]; if (v < tgt[i]) v = tgt[i]; }
    lightVal[i] = v;
    uint32_t o = (uint32_t)(v + 0.5f);
    if (o > 255) o = 255;
    o = o * lightPct[i] / 100;
    lightOut[i] = (uint8_t)o;
    uint32_t duty = (o * o) / 255;                           // 近似 gamma 2.0
    if (duty != lightDuty[i]) { lightDuty[i] = duty; lightWrite(i, duty); }
  }
}

// ===================== IR 模組 (1/16) =====================
/*
 * 互通模式：
 *  - 接收：所有已設定協議同時比對，各自有連續幀計數同鎖定，唔需要揀。
 *  - 發射：依序發射所有「波形唔同」嘅已設定協議 (相同波形只發一次)。
 *  - Tamiya 內置波形：16 個 mark(~190us) / 15 個 space(~385us)，第 5 個 mark 後有 ~3.3ms 長間隔，
 *    frame 週期 ~20ms，開炮約 50 次(約 1 秒)，接收端需連續見到 3 次先算命中。
 *  - Heng Long 官方聲明可與 Tamiya 對戰，所以內置沿用 Tamiya 波形。
 *  - Tamiya 2-Shot / Taigen：未有可靠公開時序，請用「學習」功能錄真發射器。
 */
void IRAM_ATTR irISR() {
  uint32_t now = (uint32_t)esp_timer_get_time();
  uint32_t d = now - irLastEdge;
  irLastEdge = now;
  irEdges = irEdges + 1;
  uint32_t prevLevel = IR_RX_LEVEL() ? 0u : 1u;
  uint16_t h = irHead, nx = (uint16_t)((h + 1) & (IR_RING_SIZE - 1));
  if (nx == irTail) { irRingDrops = irRingDrops + 1; return; }
  irRing[h] = (d > 65535u ? 65535u : d) | (prevLevel << 16);
  irHead = nx;
}

void irMakeDefault(IRProfile& p) {
  memset(&p, 0, sizeof(p));
  p.len = 31;
  for (int i = 0; i < 31; i++) p.d[i] = (i & 1) ? 385 : 190;
  p.d[9] = 3300;
  p.repeats = 50; p.periodUs = 20000; p.valid = 1; p.builtin = 1;
}

static bool irProfileSane(const IRProfile& t) {
  if (!t.valid || t.len < 5 || t.len > IR_FRAME_MAX || !(t.len & 1)) return false;
  if (t.repeats < 1 || t.repeats > 200) return false;
  if (t.periodUs < 1000) return false;
  uint32_t sum = 0;
  for (int i = 0; i < t.len; i++) { if (t.d[i] < 20) return false; sum += t.d[i]; }
  return sum + 500 <= t.periodUs;
}

void irLoadProfiles() {
  for (int i = 0; i < IR_PROTO_COUNT; i++) {
    memset(&irProfiles[i], 0, sizeof(IRProfile));
    char path[24]; snprintf(path, sizeof(path), "/ir%d.bin", i);
    bool loaded = false;
    if (LittleFS.exists(path)) {
      File f = LittleFS.open(path, "r");
      if (f) {
        uint8_t magic[4];
        IRProfile tmp;
        if (f.read(magic, 4) == 4 && !memcmp(magic, "IRP1", 4) &&
            f.read((uint8_t*)&tmp, sizeof(tmp)) == sizeof(tmp) && irProfileSane(tmp)) {
          irProfiles[i] = tmp; irProfiles[i].builtin = 0; loaded = true;
        }
        f.close();
      }
    }
    if (!loaded && (i == IR_PROTO_TAMIYA || i == IR_PROTO_HENGLONG)) irMakeDefault(irProfiles[i]);
  }
}

bool irSaveProfile(int slot) {
  if (slot < 0 || slot >= IR_PROTO_COUNT) return false;
  char path[24]; snprintf(path, sizeof(path), "/ir%d.bin", slot);
  LittleFS.remove(path);
  if (irProfiles[slot].builtin || !irProfiles[slot].valid) return true;
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  f.write((const uint8_t*)"IRP1", 4);
  size_t w = f.write((const uint8_t*)&irProfiles[slot], sizeof(IRProfile));
  f.close();
  return w == sizeof(IRProfile);
}

void irDumpProfile(int slot) {
  const IRProfile& p = irProfiles[slot];
  Serial.printf("[IR] %s valid=%d len=%d repeats=%d period=%uus builtin=%d\n  ",
                IR_PROTO_NAME[slot], (int)p.valid, (int)p.len, (int)p.repeats, (unsigned)p.periodUs, (int)p.builtin);
  for (int i = 0; i < p.len; i++) Serial.printf("%u%c", (unsigned)p.d[i], (i & 1) ? ' ' : (i + 1 < p.len ? ',' : ' '));
  Serial.println();
}

void irCarrier(bool on) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(IR_SEND_PIN, on ? IR_CARRIER_DUTY : 0);
#else
  ledcWrite(IR_LEDC_CH, on ? IR_CARRIER_DUTY : 0);
#endif
}

void irInit() {
  pinMode(IR_RECEIVE_PIN, INPUT_PULLUP);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(IR_SEND_PIN, IR_CARRIER_HZ, 8);
#else
  ledcSetup(IR_LEDC_CH, IR_CARRIER_HZ, 8);
  ledcAttachPin(IR_SEND_PIN, IR_LEDC_CH);
#endif
  irCarrier(false);
  irHead = 0;
  irTail = 0;
  irLastEdge = (uint32_t)esp_timer_get_time();
  attachInterrupt(digitalPinToInterrupt(IR_RECEIVE_PIN), irISR, CHANGE);
  irLoadProfiles();
  Serial.println("[IR] ready (38kHz TX, all-protocol RX)");
}

bool irFrameMatches(const IRProfile& p, const uint16_t* d, uint8_t n) {
  if (!p.valid) return false;
  int tolLen = (p.len >= IR_LEN_TOL_MIN) ? IR_LEN_TOL : 0;
  int diff = (int)n - (int)p.len;
  if (diff < -tolLen || diff > tolLen) return false;
  uint8_t m = (n < p.len) ? n : p.len;
  for (uint8_t i = 0; i < m; i++) {
    int32_t a = d[i], b = p.d[i];
    int32_t tol = b * IR_TOL_PCT / 100;
    if (tol < IR_TOL_ABS_US) tol = IR_TOL_ABS_US;
    int32_t df = a - b; if (df < 0) df = -df;
    if (df > tol) return false;
  }
  return true;
}

void irLearnFinish(bool timeout) {
  int slot = irLearnSlot;
  if (slot < 0) return;
  if (timeout || lrnN < 3) {
    irLearnState = LRN_FAIL; irLearnSlot = -1;
    evLog("IR 學習失敗 (%s)：%s", IR_PROTO_NAME[slot], timeout ? "逾時，未收到訊號" : "訊號不足");
    return;
  }
  int bestLen = 0, bestCnt = 0;
  for (int i = 0; i < lrnN; i++) {
    int c = 0;
    for (int j = 0; j < lrnN; j++) if (lrnLens[j] == lrnLens[i]) c++;
    if (c > bestCnt) { bestCnt = c; bestLen = lrnLens[i]; }
  }
  if (bestCnt < 3 || bestCnt * 100 < lrnN * 60) {
    irLearnState = LRN_FAIL; irLearnSlot = -1;
    evLog("IR 學習失敗 (%s)：幀長度不一致", IR_PROTO_NAME[slot]);
    return;
  }
  IRProfile p; memset(&p, 0, sizeof(p));
  p.len = (uint8_t)bestLen;
  for (int e = 0; e < bestLen; e++) {
    uint16_t v[LRN_MAX]; int k = 0;
    for (int j = 0; j < lrnN; j++) if (lrnLens[j] == bestLen) v[k++] = lrnFrames[j][e];
    for (int a = 1; a < k; a++) { uint16_t t = v[a]; int b = a - 1; while (b >= 0 && v[b] > t) { v[b + 1] = v[b]; b--; } v[b + 1] = t; }
    p.d[e] = v[k / 2];
  }
  uint32_t dur = 0; for (int e = 0; e < bestLen; e++) dur += p.d[e];
  uint32_t period = dur + 8000;
  if (lrnTotal >= 2) period = (lrnLastStart - lrnFirstStart) / (lrnTotal - 1);
  if (period < dur + 1000) period = dur + 1000;
  if (period > 60000) period = 60000;
  p.periodUs = (uint16_t)period;
  p.repeats = (uint8_t)(lrnTotal < 3 ? 3 : (lrnTotal > 120 ? 120 : lrnTotal));
  p.valid = 1; p.builtin = 0;
  if (!irProfileSane(p)) {
    irLearnState = LRN_FAIL; irLearnSlot = -1;
    evLog("IR 學習失敗 (%s)：樣板不合理", IR_PROTO_NAME[slot]);
    return;
  }
  irProfiles[slot] = p;
  irSavePending = (int8_t)slot;
  irLearnState = LRN_OK;
  irLearnSlot = -1;
  evLog("IR 學習成功 (%s)：%d 元素、%d 次、週期 %u us", IR_PROTO_NAME[slot], (int)p.len, (int)p.repeats, (unsigned)p.periodUs);
}

void irLearnFeed(const uint16_t* d, uint8_t n, uint32_t startUs) {
  if (lrnTotal == 0) { lrnFirstStart = startUs; irLearnState = LRN_COLLECT; }
  lrnTotal++;
  lrnLastStart = startUs;
  lrnLastMs = millis();
  if (lrnN < LRN_MAX) { memcpy(lrnFrames[lrnN], d, n * sizeof(uint16_t)); lrnLens[lrnN] = n; lrnN++; }
}

static uint32_t irTxReps(const IRProfile& p) {
  uint32_t maxRep = IR_TX_MAX_US / (uint32_t)p.periodUs;
  if (maxRep < 1) maxRep = 1;
  return (p.repeats < maxRep) ? p.repeats : maxRep;
}
static uint32_t irBurstMs(const IRProfile& p) {
  if (!p.valid || p.periodUs == 0) return 0;
  return (uint32_t)(((uint64_t)irTxReps(p) * p.periodUs) / 1000u);
}

// 兩個協議嘅發射波形完全相同？ (Tamiya / Heng Long 內置就係)
static bool irSameWave(const IRProfile& a, const IRProfile& b) {
  return a.valid && b.valid && a.len == b.len && a.repeats == b.repeats && a.periodUs == b.periodUs &&
         memcmp(a.d, b.d, a.len * sizeof(uint16_t)) == 0;
}
// 同較前面某個協議波形相同嘅話，回傳嗰個協議；否則 -1
static int irDupOf(int i) {
  if (!irProfiles[i].valid) return -1;
  for (int j = 0; j < i; j++) if (irSameWave(irProfiles[j], irProfiles[i])) return j;
  return -1;
}
// 一發射擊嘅發射序列：已設定、波形唔同嘅協議，按編號順序；總時間唔可超過 IR_SHOT_MAX_MS
static void irBuildPlan(IRPlan& plan) {
  plan.n = 0; plan.totalMs = 0;
  for (int i = 0; i < IR_PROTO_COUNT; i++) {
    const IRProfile& p = irProfiles[i];
    if (!p.valid || p.periodUs == 0 || irDupOf(i) >= 0) continue;
    uint32_t add = irBurstMs(p) + (plan.n ? IR_SEQ_GAP_MS : 0);
    if (plan.n > 0 && plan.totalMs + add > IR_SHOT_MAX_MS) continue;
    plan.idx[plan.n++] = (uint8_t)i;
    plan.totalMs += add;
  }
}
static uint32_t fireReloadMs() {
  uint32_t r = FIRE_RELOAD_MS;
#if IR_RELOAD_DYNAMIC
  IRPlan plan; irBuildPlan(plan);
  uint32_t need = plan.totalMs + IR_LATCH_RELEASE_MS + 100;
  if (need > r) r = need;
#endif
  return r;
}
static uint32_t fireReloadLeftMs(uint32_t now) {
  if (!everFired) return 0;
  uint32_t el = now - lastFireMs, rl = fireReloadMs();
  return (el >= rl) ? 0 : rl - el;
}

void irOnFrame(const uint16_t* d, uint8_t n, uint32_t startUs) {
  if (irLearnSlot >= 0) { irLast.put(d, n, -2); irLearnFeed(d, n, startUs); return; }
  INC(irFramesSeen);
  int p = -1;
  for (int i = 0; i < IR_PROTO_COUNT && p < 0; i++) if (irFrameMatches(irProfiles[i], d, n)) p = i;   // 全部協議並行比對
  irLast.put(d, n, (int8_t)p);
  if (p < 0) { irUnk.put(d, n, -1); return; }
  INC(irFramesMatched);
  INC(irFramesByProto[p]);
  uint32_t now = millis();
  if (now - irMatchMs[p] > IR_MATCH_WINDOW_MS) irMatchCnt[p] = 0;
  irMatchMs[p] = now;
  if (irMatchCnt[p] < 255) irMatchCnt[p] = (uint8_t)(irMatchCnt[p] + 1);
  if (irMatchCnt[p] >= IR_FRAMES_FOR_HIT && !irLatched[p] && (int32_t)(now - irBlockUntil[p]) >= 0) {
    irLatched[p] = true; hitCommand = true;
    uint32_t blk = irBurstMs(irProfiles[p]) + 100;             // 同一協議嘅同一發射擊期間唔會再算命中
    irBlockUntil[p] = now + ((blk > HIT_COOLDOWN) ? blk : HIT_COOLDOWN);
    IRPlan plan; irBuildPlan(plan);                            // 序列中排後面嘅協議，係同一發射擊嘅後續部分，一併鎖定
    uint32_t rem = irBurstMs(irProfiles[p]);                   // 由而家計：自己呢個協議嘅整段 + 之後每個協議 (空隙 + 整段)
    bool found = false;
    for (int k = 0; k < plan.n; k++) {
      uint8_t q = plan.idx[k];
      if (found) {
        rem += IR_SEQ_GAP_MS + irBurstMs(irProfiles[q]);
        uint32_t until = now + rem + 100;
        if ((int32_t)(until - irBlockUntil[q]) > 0) irBlockUntil[q] = until;
      } else if (q == p) found = true;
    }
    INC(irRxHits);
    INC(irHitsByProto[p]);
    irLastHitAtMs = now; irLastHitProto = (int8_t)p;
    hitFlashUntil = now + LIGHT_HIT_MS; damageUntil = now + LIGHT_DAMAGE_MS;
    evLog("被擊中 [%s]", IR_PROTO_NAME[p]);
    Serial.println("[IR] HIT");
  }
}

void irProcessReceive() {
  static uint16_t cur[IR_FRAME_MAX];
  static uint8_t  curLen = 0;
  static bool     curActive = false, curOverflow = false;
  static uint32_t clockUs = 0, curStart = 0;

  uint32_t nowMs = millis();

  if (irTxBusy || (int32_t)(nowMs - irRxBlockUntil) < 0) {
    irTail = irHead; curActive = false; curLen = 0; return;
  }
  uint16_t tail = irTail;
  while (tail != irHead) {
    uint32_t e = irRing[tail];
    tail = (uint16_t)((tail + 1) & (IR_RING_SIZE - 1));
    uint32_t d = e & 0xFFFFu;
    bool isMark = ((e >> 16) & 1u) == 0;
    clockUs += d;
    if (!isMark && d >= IR_FRAME_GAP_US) {
      if (curActive && curLen >= 5 && (curLen & 1) && !curOverflow) irOnFrame(cur, curLen, curStart);
      curActive = true; curLen = 0; curOverflow = false; curStart = clockUs;
      continue;
    }
    if (!curActive) continue;
    if ((curLen & 1) == (isMark ? 1 : 0)) { curActive = false; continue; }
    if (curLen >= IR_FRAME_MAX) { curOverflow = true; continue; }
    cur[curLen++] = (uint16_t)d;
  }
  irTail = tail;

  if (curActive && curLen >= 5 && (curLen & 1) &&
      ((uint32_t)esp_timer_get_time() - irLastEdge) > IR_FRAME_TIMEOUT_US) {
    if (!curOverflow) irOnFrame(cur, curLen, curStart);
    curActive = false; curLen = 0;
  }

  for (int i = 0; i < IR_PROTO_COUNT; i++)
    if (irLatched[i] && (nowMs - irMatchMs[i]) > IR_LATCH_RELEASE_MS) { irLatched[i] = false; irMatchCnt[i] = 0; }
  if (irLearnState == LRN_COLLECT && (nowMs - lrnLastMs) > 400) irLearnFinish(false);
  else if (irLearnState == LRN_WAIT && (nowMs - lrnStartMs) > 20000) irLearnFinish(true);
}

void irLearnStart(int slot) {
  if (slot < 0 || slot >= IR_PROTO_COUNT) { irLearnSlot = -1; irLearnState = LRN_IDLE; return; }
  lrnN = 0; lrnTotal = 0; lrnStartMs = millis(); lrnLastMs = lrnStartMs;
  irLearnState = LRN_WAIT;
  irLearnSlot = (int8_t)slot;
  evLog("IR 學習開始 (%s)：請於 20 秒內對住接收頭開火", IR_PROTO_NAME[slot]);
}

void irResetProfile(int s) {
  if (s < 0 || s >= IR_PROTO_COUNT) return;
  memset(&irProfiles[s], 0, sizeof(IRProfile));
  if (s == IR_PROTO_TAMIYA || s == IR_PROTO_HENGLONG) irMakeDefault(irProfiles[s]);
  irSavePending = (int8_t)s;
  evLog("IR 協議已重設 (%s)", IR_PROTO_NAME[s]);
}

// ---- 發射 (獨立任務、優先級高於音訊) ----
static inline void spinUntil(int64_t t) { while (esp_timer_get_time() < t) { } }

static void irSendBurst(const IRProfile& p) {
  uint32_t reps = irTxReps(p);
  int ex = irMarkExcessUs;
  int64_t t0 = esp_timer_get_time() + 200;
  for (uint32_t r = 0; r < reps; r++) {
    int64_t fs = t0 + (int64_t)r * p.periodUs;
    int64_t rem = fs - esp_timer_get_time();
    if (rem > 3000) vTaskDelay(pdMS_TO_TICKS((rem - 2000) / 1000));
    spinUntil(fs);
    int64_t t = fs;
    for (uint8_t i = 0; i < p.len; i++) {
      bool mark = !(i & 1);
      int32_t dur = (int32_t)p.d[i] + (mark ? ex : -ex);
      if (dur < 30) dur = 30;
      irCarrier(mark);
      t += dur;
      spinUntil(t);
    }
    irCarrier(false);
  }
  irCarrier(false);
}

void irSendNow() {
  IRPlan plan; irBuildPlan(plan);
  if (plan.n == 0) { irTxBusy = false; return; }
  int64_t startUs = esp_timer_get_time();
  for (int k = 0; k < plan.n; k++) {
    if (k) vTaskDelay(pdMS_TO_TICKS(IR_SEQ_GAP_MS));
    IRProfile p = irProfiles[plan.idx[k]];                     // 複製一份，發射期間 learn/reset 都唔會影響
    irSendBurst(p);
    INC(irTxByProto[plan.idx[k]]);
  }
  irCarrier(false);
  INC(irTxCount);
  irLastTxBurstMs = (uint32_t)((esp_timer_get_time() - startUs) / 1000);
  irLastTxMs = millis();
  irRxBlockUntil = millis() + 150;
  irTxBusy = false;
}

void irTxTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    irSendNow();
    irStackFree = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
  }
}

void irRequestFire() {
  IRPlan plan; irBuildPlan(plan);
  if (irTxBusy || irLearnSlot >= 0 || !irTxTaskHandle || plan.n == 0) { INC(irTxSkipped); return; }
  irTxBusy = true;
  INC(irTxRequests);
  xTaskNotifyGive(irTxTaskHandle);
}

// ===================== 控制 / 狀態機 (非阻塞) =====================
static inline bool testHold(int voice, uint32_t now) { return (int32_t)(testUntil[voice] - now) > 0; }

void playTest(int snd, uint32_t now) {
  evLog("測試播放：%s", SOUND_LIST[snd].name);
  switch (snd) {
    case S_ENGINE: ignitionCommand = true; break;
    case S_START:  voiceStart(V_SEQ, S_START, false); break;
    case S_STOP:   voiceStart(V_SEQ, S_STOP, false); break;
    case S_FIRE:   voiceStart(V_FIRE, S_FIRE, false); break;
    case S_HIT:    voiceStart(V_HIT, S_HIT, false); break;
    case S_MG:     testUntil[V_MG] = now + TEST_HOLD_MS; break;
    case S_TURRET: testUntil[V_TURRET] = now + TEST_HOLD_MS; break;
    case S_ELEV:   testUntil[V_ELEV] = now + TEST_HOLD_MS; break;
    default: break;
  }
}

void controlTick() {
  uint32_t now = millis();

  int8_t lr = irLearnReq;
  if (lr != REQ_NONE) { irLearnReq = REQ_NONE; irLearnStart(lr); }
  int8_t rr = irResetReq;
  if (rr != REQ_NONE) { irResetReq = REQ_NONE; irResetProfile(rr); }

  if (audioPauseRequested) {
    if (!audioPaused) {
      for (int i = 0; i < V_COUNT; i++) voiceStop(i);
      engineClose();
      audioPaused = true;
    }
  } else audioPaused = false;

  irProcessReceive();

  uint16_t ch[14];
  if (readIBUS(ch)) {
    portENTER_CRITICAL(&channelMux);
    memcpy(debugChannels, ch, sizeof(ch));
    portEXIT_CRITICAL(&channelMux);
    if (lastIbusMs == 0) evLog("遙控已連線");
    else if (!prevLink) evLog("遙控訊號恢復 (中斷 %lu ms)", (unsigned long)(now - lastIbusMs));
    if (lastIbusMs != 0) { uint32_t gap = now - lastIbusMs; if (gap > ibusMaxGapMs) ibusMaxGapMs = gap; }
    lastIbusMs = now;
    for (int i = 0; i < 14; i++) { if (ch[i] < chMin[i]) chMin[i] = ch[i]; if (ch[i] > chMax[i]) chMax[i] = ch[i]; }
  }
  bool link = (lastIbusMs != 0) && (now - lastIbusMs < IBUS_TIMEOUT_MS);
  if (prevLink && !link) { INC(ibusFailsafe); evLog("遙控訊號中斷 (failsafe：全部回中)"); }
  prevLink = link;
  ibusLinkOk = link;

  Inputs in;
  if (link) {
    in.left = chv(chMap[FN_LEFT]); in.right = chv(chMap[FN_RIGHT]);
    in.turret = chv(chMap[FN_TURRET]); in.elev = chv(chMap[FN_ELEV]);
    in.fire = chv(chMap[FN_FIRE]) > 1500; in.mg = chv(chMap[FN_MG]) > 1500;
    in.headLevel = (chMap[FN_LIGHT] >= 0) ? headLevelOf(chv(chMap[FN_LIGHT])) : 0;
    in.aux = (chMap[FN_AUX] >= 0) && (chv(chMap[FN_AUX]) > 1500);
  }
  curIn = in;

  bool fireEdge = in.fire && !lastFire;
  lastFire = link ? in.fire : true;
  if (audioPaused) { lightsTick(now); return; }

  int8_t pr = playReq;
  if (pr != REQ_NONE) { playReq = REQ_NONE; if (pr >= 0 && pr < S_COUNT) playTest(pr, now); }

  if (ignitionCommand) {
    ignitionCommand = false;
    if (engineState == ENGINE_OFF) { engineState = ENGINE_STARTING; idleTime = now; voiceStart(V_SEQ, S_START, false); }
    else if (engineState == ENGINE_IDLE_RUN) { engineState = ENGINE_STOPPING; voiceStart(V_SEQ, S_STOP, false); }
  }
  bool active = abs(in.left - 1500) > STICK_ACTIVE || abs(in.right - 1500) > STICK_ACTIVE;
  switch (engineState) {
    case ENGINE_OFF:
      if (active) { engineState = ENGINE_STARTING; idleTime = now; voiceStart(V_SEQ, S_START, false); }
      break;
    case ENGINE_STARTING:
      if (!voices[V_SEQ].active) { engineState = ENGINE_IDLE_RUN; idleTime = now; }
      break;
    case ENGINE_IDLE_RUN:
      if (active) idleTime = now;
      else if (now - idleTime > AUTO_STOP_MS) { engineState = ENGINE_STOPPING; voiceStart(V_SEQ, S_STOP, false); }
      break;
    case ENGINE_STOPPING:
      if (!voices[V_SEQ].active) engineState = ENGINE_OFF;
      break;
  }
  if (engineState != prevEngine) { prevEngine = engineState; evLog("引擎：%s", ENGINE_NAME[(int)engineState & 3]); }
  if (engineState == ENGINE_IDLE_RUN) {
    if (!eng.ready && (int32_t)(now - eng.retryAt) >= 0) {
      if (eng.r.open(soundMeta[S_ENGINE], SOUND_PATH[S_ENGINE])) {
        eng.ready = true; eng.pos = 0; eng.gain = 0;
        INC(wavPlays[S_ENGINE]); wavLastMs[S_ENGINE] = now; wavFailLogged[S_ENGINE] = false;
      } else {
        eng.retryAt = now + 1000;
        INC(wavOpenFail[S_ENGINE]);
        if (!wavFailLogged[S_ENGINE]) {
          wavFailLogged[S_ENGINE] = true;
          const SoundMeta& m = soundMeta[S_ENGINE];
          evLog("引擎音效開啟失敗 (%s)", reasonTxt(m.ok ? (int)WR_SIZE_CHANGED : (int)m.reason));
        }
      }
    }
  } else if (eng.ready && eng.gain <= 0.001f) engineClose();

  bool reqFire = fireEdge || webFireRequest;
  if (fireEdge) INC(statFireEdges);
  webFireRequest = false;
  if (reqFire) {
    if (irLearnSlot >= 0) {
      INC(statFireBlocked);
      evLog("開炮被擋：IR 學習中");
    } else if (everFired && now - lastFireMs < fireReloadMs()) {
      INC(statFireBlocked);
      evLog("開炮被擋：裝填中 (尚餘 %lu ms)", (unsigned long)fireReloadLeftMs(now));
    } else {
      everFired = true; lastFireMs = now;
      voiceStart(V_FIRE, S_FIRE, false);
      lightVal[LT_FIRE] = 255.f;                               // 炮口火光：瞬亮，之後漸暗
      irRequestFire();
      IRPlan plan; irBuildPlan(plan);
      evLog("開炮｜IR %u 款｜%s", (unsigned)plan.n, irTxBusy ? "已發射" : "未發射");
    }
  }
  if (hitCommand) { hitCommand = false; voiceStart(V_HIT, S_HIT, false); }

  voiceHold(V_MG, S_MG, in.mg || testHold(V_MG, now));
  turretOn = (abs(in.turret - 1500) > (turretOn ? TURRET_OFF : TURRET_ON));
  elevOn   = (abs(in.elev - 1500)   > (elevOn   ? TURRET_OFF : TURRET_ON));
  voiceHold(V_TURRET, S_TURRET, turretOn || testHold(V_TURRET, now));
  voiceHold(V_ELEV, S_ELEV, elevOn || testHold(V_ELEV, now));
  fnState = (uint8_t)((in.fire ? 1 : 0) | (in.mg ? 2 : 0) | (turretOn ? 4 : 0) | (elevOn ? 8 : 0) | (lightOut[LT_HEAD] ? 16 : 0) | (in.aux ? 32 : 0));
  lightsTick(now);
}

void audioAndControlTask(void*) {
  static int16_t buf[AUDIO_FRAMES * 2];
  Serial2.setRxBufferSize(256);
  Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);
  Serial.println("[Task] Audio+Control start");
  uint32_t blockCount = 0;
  for (;;) {
    if ((++blockCount & 0xFF) == 0) audioStackFree = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    uint32_t t0 = (uint32_t)esp_timer_get_time();
    controlTick();
    renderBlock(buf);
    noteBlockTime((uint32_t)esp_timer_get_time() - t0);
    size_t w = 0;
    i2s_write(I2S_PORT, buf, sizeof(buf), &w, portMAX_DELAY);
  }
}

// ===================== 警示 (健康檢查) =====================
uint32_t computeAlerts() {
  uint32_t a = 0;
  uint32_t now = millis();
  if (!ibusLinkOk) a |= 1u << AL_NO_LINK;
  for (int i = 0; i < S_COUNT; i++) {
    if (soundMeta[i].ok) continue;
    if (soundMeta[i].reason == WR_MISSING) a |= 1u << AL_WAV_MISSING;
    else a |= 1u << AL_WAV_BAD;
  }
  if (ESP.getFreeHeap() < LOW_HEAP_BYTES) a |= 1u << AL_LOW_HEAP;
  if (audioSlowLastMs != 0 && now - audioSlowLastMs < 60000) a |= 1u << AL_AUDIO_SLOW;
  if (bootResetReason == ESP_RST_BROWNOUT) a |= 1u << AL_BROWNOUT;
  if (bootResetReason == ESP_RST_PANIC || bootResetReason == ESP_RST_INT_WDT ||
      bootResetReason == ESP_RST_TASK_WDT || bootResetReason == ESP_RST_WDT) a |= 1u << AL_CRASH_RESET;
  { IRPlan plan; irBuildPlan(plan); if (plan.n == 0) a |= 1u << AL_IR_UNSET; }
  if (rateIrEdges > IR_NOISE_EPS) a |= 1u << AL_IR_NOISE;
  if (BATT_ADC_PIN >= 0 && BATT_LOW_MV > 0 && battMv > 0 && battMv < BATT_LOW_MV) a |= 1u << AL_LOW_BATT;
  if (fsTotalCache > 0 && (uint64_t)fsUsedCache * 100 > (uint64_t)fsTotalCache * 90) a |= 1u << AL_FS_FULL;
  if ((audioStackFree != 0 && audioStackFree < 1024) || (irStackFree != 0 && irStackFree < 512) ||
      (loopStackFree != 0 && loopStackFree < 1024)) a |= 1u << AL_STACK_LOW;
  if (rateIbusErr >= 3) a |= 1u << AL_IBUS_ERR;
  return a;
}

// ===================== JSON 小工具 =====================
static void jkey(String& j, const char* k) { j += '"'; j += k; j += "\":"; }
static void jn(String& j, const char* k, long v) { jkey(j, k); j += String(v); j += ','; }
static void ju(String& j, const char* k, uint32_t v) { jkey(j, k); j += String((unsigned long)v); j += ','; }
static void jv(String& j, long v) { j += String(v); j += ','; }
static void jesc(String& j, const char* v) {
  j += '"';
  for (; *v; v++) {
    char c = *v;
    if (c == '"' || c == '\\') { j += '\\'; j += c; }
    else if ((uint8_t)c < 0x20) j += ' ';
    else j += c;
  }
  j += '"';
}
static void js(String& j, const char* k, const char* v) { jkey(j, k); jesc(j, v); j += ','; }
static void jopen(String& j, const char* k, char c) { if (k) jkey(j, k); j += c; }
static void jclose(String& j, char c) {
  if (j.length() && j[j.length() - 1] == ',') j.remove(j.length() - 1);
  j += c; j += ',';
}
static void jfin(String& j) { if (j.length() && j[j.length() - 1] == ',') j.remove(j.length() - 1); }
static void jarr16(String& j, const char* k, const uint16_t* a, int n) {
  jopen(j, k, '[');
  for (int i = 0; i < n; i++) jv(j, a[i]);
  jclose(j, ']');
}
static void sendJson(String& j) {
  jfin(j);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}
static long ageOf(uint32_t t, uint32_t now) { return t == 0 ? -1 : (long)(now - t); }

static int soundProgress(int snd) {
  if (snd == S_ENGINE) {
    uint32_t n = eng.r.samples();
    return (eng.ready && n) ? (int)((uint64_t)(uint32_t)eng.pos * 1000 / n) : -1;
  }
  for (int v = 0; v < V_COUNT; v++) {
    if (voices[v].active && voices[v].snd == snd) {
      uint32_t n = voices[v].r.samples();
      return n ? (int)((uint64_t)voices[v].pos * 1000 / n) : 0;
    }
  }
  return -1;
}


// ===================== 網頁 (單頁儀表板) =====================
const char PAGE_HTML[] = R"rawliteral(<!DOCTYPE html>
<html lang="zh-HK"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>坦克儀表板</title>
<style>
:root{--bg:#0f1114;--card:#1a1d22;--line:#2c3138;--fg:#e8eaed;--mut:#8b949e;--acc:#ff9800;--ok:#3fb950;--bad:#f85149;--warn:#d29922;--blue:#2f81f7}
*{box-sizing:border-box}
[hidden]{display:none!important}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,-apple-system,"Segoe UI","Microsoft JhengHei",sans-serif}
header{position:sticky;top:0;z-index:5;background:#0a0c0e;border-bottom:1px solid var(--line);padding:8px 10px 0}
.top{display:flex;align-items:center;gap:8px;flex-wrap:wrap}
.top b{font-size:16px;color:var(--acc)}
.pills{display:flex;gap:6px;flex-wrap:wrap;margin:6px 0}
.pill{display:inline-block;padding:2px 9px;border-radius:99px;background:#252a31;color:var(--fg);font-size:12px;white-space:nowrap}
.pill.ok{background:#12351d;color:#7ee787}.pill.bad{background:#4a1a1a;color:#ff9b94}.pill.warn{background:#3d2f0b;color:#f2cc60}.pill.info{background:#10294d;color:#79b8ff}
nav{display:flex;gap:2px;overflow-x:auto;scrollbar-width:none}
nav button{flex:0 0 auto;background:none;border:0;border-bottom:3px solid transparent;color:var(--mut);padding:8px 13px;font-size:14px;cursor:pointer;border-radius:0}
nav button.on{color:var(--fg);border-bottom-color:var(--acc)}
main{padding:10px;max-width:1100px;margin:0 auto}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:10px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:11px 12px;margin-bottom:10px;min-width:0}
.grid .card{margin-bottom:0}
h3{margin:0 0 8px;font-size:14px;color:var(--acc);font-weight:600}
.mut{color:var(--mut);font-size:12px}
.kv{display:flex;justify-content:space-between;gap:8px;padding:3px 0;border-bottom:1px dashed #262b31}
.kv:last-child{border-bottom:0}.kv span:first-child{color:var(--mut)}
button,select,input{font:inherit}
button{background:#30363d;color:#fff;border:0;border-radius:7px;padding:6px 11px;cursor:pointer}
.btns{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:8px}
.btns button{background:#1f6feb;color:#fff;padding:8px 12px;font-weight:600}
.btns button.gry,button.gry{background:#30363d}
button.red,.btns button.red{background:#da3633}button.grn{background:#238636}button.s{padding:4px 9px;font-size:12px;border-radius:6px}
select,input[type=number]{background:#0d1014;color:var(--fg);border:1px solid #3a4048;border-radius:6px;padding:5px 7px}
input[type=range]{width:100%}
input[type=file]{max-width:170px;font-size:12px;color:var(--mut)}
.bar{height:9px;background:#262b31;border-radius:5px;overflow:hidden;position:relative}
.bar i{display:block;height:100%;width:0;background:var(--acc);border-radius:5px;transition:width .25s}
.bar.g i{background:var(--ok)}.bar.b i{background:var(--blue)}
.alert{padding:6px 9px;border-radius:7px;margin:4px 0;font-size:13px;border-left:4px solid}
.alert.e{background:#3a1717;border-color:var(--bad)}.alert.w{background:#392d0e;border-color:var(--warn)}.alert.i{background:#102640;border-color:var(--blue)}.alert.k{background:#12301b;border-color:var(--ok)}
.vrow{display:grid;grid-template-columns:92px 1fr;gap:2px 8px;align-items:center;margin:5px 0}
.vrow .vn{color:var(--mut);font-size:12px}.vrow .vs{font-size:12px;min-height:16px}
.vrow.on .vn{color:var(--ok)}
.fnrow{display:grid;grid-template-columns:74px 1fr 62px;gap:8px;align-items:center;margin:5px 0}
.fnrow .vn{color:var(--mut);font-size:12px}.fnrow b{text-align:right;font-variant-numeric:tabular-nums;font-size:12px}
table{width:100%;border-collapse:collapse;font-size:13px}
th{color:var(--mut);font-weight:500;text-align:left;padding:4px 4px;border-bottom:1px solid var(--line);font-size:12px}
td{padding:4px 4px;border-bottom:1px solid #21262c;vertical-align:middle}
td.n{font-variant-numeric:tabular-nums;text-align:right}
.scroll{overflow-x:auto}
.rbar{position:relative;height:14px;background:#262b31;border-radius:4px;min-width:90px}
.rbar i{position:absolute;left:0;top:0;bottom:0;background:var(--acc);border-radius:4px;width:0}
.rbar u{position:absolute;top:-2px;bottom:-2px;width:2px;background:#fff8;text-decoration:none}
.rbar s{position:absolute;top:0;bottom:0;width:1px;background:#6e768166}
.srow{border:1px solid var(--line);border-radius:9px;padding:8px 10px;margin:8px 0;background:#15181c}
.srow.bad{border-color:#7d2b28}
.s-head{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
.s-info{margin:4px 0;font-size:12px;color:var(--mut)}
.s-act{display:flex;gap:6px;align-items:center;flex-wrap:wrap;margin-top:7px}
.bar.up{height:5px;margin-top:5px}
.lamp{width:22px;height:22px;border-radius:50%;background:#222;border:1px solid #444;flex:0 0 22px;transition:background .15s,box-shadow .15s}
.lrowl{display:grid;grid-template-columns:22px 1fr;gap:10px;align-items:center;padding:8px 0;border-bottom:1px dashed #262b31}
.lrowl:last-child{border-bottom:0}
.lctl{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-top:4px}
.lctl input[type=range]{flex:1;min-width:110px;width:auto}
canvas{width:100%;height:auto;background:#0b0d0f;border:1px solid var(--line);border-radius:8px;display:block}
pre,textarea{width:100%;background:#0b0d0f;color:#c9d1d9;border:1px solid var(--line);border-radius:8px;padding:8px;font:12px/1.4 ui-monospace,Consolas,monospace;white-space:pre-wrap;word-break:break-all}
textarea{height:260px}
.lrow{display:grid;grid-template-columns:78px 1fr;gap:8px;padding:3px 0;border-bottom:1px solid #20252b;font-size:13px}
.lrow time{color:var(--mut);font:12px ui-monospace,Consolas,monospace}
.mapgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:8px}
.mapgrid label{display:flex;flex-direction:column;gap:3px;font-size:12px;color:var(--mut)}
#toast{position:fixed;left:50%;bottom:18px;transform:translateX(-50%);background:#30363d;color:#fff;padding:8px 14px;border-radius:8px;opacity:0;pointer-events:none;transition:opacity .2s;z-index:20;max-width:90vw}
#toast.on{opacity:1}
.row2{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin:6px 0}
ul.how{margin:6px 0 0;padding-left:18px;font-size:13px;color:#c9d1d9}ul.how li{margin:3px 0}
</style></head><body>
<header>
 <div class="top"><b>坦克聲光 + IR</b><span id="fw" class="mut"></span><span id="off" class="pill bad" hidden>連線中斷，重試中…</span></div>
 <div class="pills"><span id="h-link" class="pill">遙控 —</span><span id="h-eng" class="pill">引擎 —</span><span id="h-ir" class="pill">IR —</span><span id="h-al" class="pill">警示 —</span></div>
 <nav id="tabs"></nav>
</header>
<main>

<section id="t-ov" class="tab">
 <div class="card"><h3>警示 / 健康檢查</h3><div id="al-list"></div></div>
 <div class="grid">
  <div class="card"><h3>快速操作</h3>
   <div class="btns"><button onclick="act('/ignition')">點火 / 熄匙</button><button class="red" onclick="act('/fire')">測試開炮 (聲+IR+燈)</button><button class="gry" onclick="act('/simhit')">模擬被擊中</button></div>
   <div class="kv"><span>音量</span><b id="vol-v">—</b></div><input type="range" id="vol" min="0" max="100">
  </div>
  <div class="card"><h3>引擎</h3>
   <div class="kv"><span>狀態</span><b id="o-eng">—</b></div>
   <div class="kv"><span>播放速度 (音調)</span><b id="o-pit">—</b></div><div class="bar"><i id="o-pitb"></i></div>
   <div class="kv"><span>油門負載</span><b id="o-ld">—</b></div><div class="bar b"><i id="o-ldb"></i></div>
  </div>
  <div class="card"><h3>聲道 (同時播放)</h3><div id="o-vo"></div></div>
  <div class="card"><h3>遙控 → 功能</h3><div id="o-fn"></div></div>
  <div class="card"><h3>燈光</h3><div id="o-lt" style="display:flex;gap:14px;flex-wrap:wrap"></div></div>
  <div class="card"><h3>IR 互通</h3>
   <div class="kv"><span>發射序列</span><b id="o-pr">—</b></div>
   <div class="kv"><span>發射次數 / 最後</span><b id="o-tx">—</b></div>
   <div class="kv"><span>被擊中次數 / 最後</span><b id="o-hit">—</b></div>
  </div>
  <div class="card"><h3>系統</h3>
   <div class="kv"><span>運行時間</span><b id="o-up">—</b></div>
   <div class="kv"><span>可用記憶體</span><b id="o-heap">—</b></div>
   <div class="kv"><span>音訊處理負載</span><b id="o-aload">—</b></div>
   <div class="kv"><span>Flash 讀取</span><b id="o-fr">—</b></div>
  </div>
 </div>
</section>

<section id="t-rc" class="tab" hidden>
 <div class="card"><h3>iBUS 連線</h3>
  <div class="grid" style="grid-template-columns:repeat(auto-fit,minmax(150px,1fr))">
   <div><div class="mut">狀態</div><b id="r-link">—</b></div><div><div class="mut">幀率</div><b id="r-hz">—</b></div>
   <div><div class="mut">最後一幀</div><b id="r-age">—</b></div><div><div class="mut">有效幀總數</div><b id="r-fr">—</b></div>
   <div><div class="mut">校驗錯誤</div><b id="r-bs">—</b></div><div><div class="mut">數值超出範圍</div><b id="r-br">—</b></div>
   <div><div class="mut">丟棄位元組</div><b id="r-dr">—</b></div><div><div class="mut">訊號中斷次數</div><b id="r-fo">—</b></div>
   <div><div class="mut">最大幀間隔</div><b id="r-gap">—</b></div>
  </div>
  <div class="row2"><button class="s gry" onclick="act('/rcreset',()=>poll())">重設最小/最大值</button><span class="mut">白線 = 本次開機以來嘅最小/最大值 (可用嚟檢查搖桿行程同中位)</span></div>
 </div>
 <div class="card"><h3>全部 14 個頻道</h3><div class="scroll"><table><thead><tr><th>CH</th><th>功能</th><th style="width:44%">數值</th><th>讀數</th><th>最小~最大</th></tr></thead><tbody id="rc-body"></tbody></table></div></div>
 <div class="card"><h3>頻道功能對應</h3><div class="mut" style="margin-bottom:6px">揀每個功能用邊個頻道 (儲存喺 ESP32，重啟後保留)。大燈：2 段掣 = 關 / 遠光，3 段掣 = 關 / 近光 / 遠光。</div>
  <div class="mapgrid" id="map-grid"></div>
  <div class="row2"><button class="s gry" onclick="act('/mapreset',()=>{loadCfg();poll()})">還原預設對應</button></div></div>
</section>

<section id="t-lt" class="tab" hidden>
 <div class="card"><h3>燈光系統</h3>
  <div class="row2"><label>大燈模式 <select id="l-hm"><option value="0">跟遙控頻道</option><option value="1">強制關</option><option value="2">強制近光</option><option value="3">強制遠光</option></select></label>
   <span class="mut">網頁強制模式優先於遙控 (測試接線時好用)</span></div>
  <div id="lt-list"></div>
 </div>
 <div class="card"><h3>燈光行為</h3>
  <ul class="how">
   <li><b>大燈</b>：遙控頻道 <span id="l-chh">—</span> 控制 (關 / 近光 / 遠光)，漸亮漸暗。</li>
   <li><b>尾燈</b>：大燈開啟時微亮；放開搖桿 (煞車) 時全亮 0.8 秒。</li>
   <li><b>機槍閃光</b>：機槍掣按住時高速閃爍。 <b>炮口火光</b>：開炮瞬間全亮再漸暗。</li>
   <li><b>受擊指示</b>：被 IR 擊中閃爍 1.5 秒，同時大燈尾燈閃爍 1.2 秒 (損壞效果)。</li>
   <li><b>輔助燈</b>：遙控頻道 <span id="l-cha">—</span> 開關。</li>
   <li><b>失控警示</b>：遙控斷線時大燈尾燈交替閃爍。 <b>開機</b>：6 盞燈依序自檢。</li>
  </ul>
  <div class="mut" style="margin-top:8px">紅 / 橙 / 黃 LED 可由 GPIO 串 220~330Ω 直接驅動 (約 5~6 mA)；白 / 藍 LED (順向電壓約 3V) 同大功率燈要用 NPN / MOSFET 由 5V 驅動，否則會好暗。亮度已用 PWM + 近似 gamma 校正。</div>
 </div>
</section>

<section id="t-wav" class="tab" hidden>
 <div class="card"><h3>儲存 / 讀取狀態</h3>
  <div class="grid" style="grid-template-columns:repeat(auto-fit,minmax(210px,1fr))">
   <div><div class="mut">音效 RAM 快取</div><b id="w-cache">—</b><div class="bar g"><i id="w-cacheb"></i></div></div>
   <div><div class="mut">檔案空間</div><b id="w-fs">—</b><div class="bar b"><i id="w-fsb"></i></div></div>
   <div><div class="mut">Flash 讀取 (串流播放)</div><b id="w-fr">—</b></div>
   <div><div class="mut">最後一次上載</div><b id="w-up">—</b></div>
  </div></div>
 <div id="wav-list"></div>
 <div class="mut">格式要求：WAV / PCM / 單聲道 / 16-bit / 22050 Hz。上載失敗會保留舊檔。</div>
</section>

<section id="t-ir" class="tab" hidden>
 <div class="grid">
  <div class="card"><h3>互通模式</h3>
   <div class="mut" style="margin-bottom:6px">唔使揀協議：接收會同時比對所有已設定協議；開炮會依序發射所有波形唔同嘅協議 (相同波形只發一次)。</div>
   <div class="kv"><span>發射序列</span><b id="i-seq">—</b></div>
   <div class="kv"><span>一發射擊總時間</span><b id="i-sm">—</b></div>
   <div class="row2"><label>脈衝補償 (µs) <input type="number" id="i-tune" min="0" max="200" step="5" style="width:80px"></label></div>
   <div class="kv"><span>學習狀態</span><b id="i-ls">—</b></div>
  </div>
  <div class="card"><h3>發射 (TX)</h3>
   <div class="kv"><span>已發射 / 請求 / 略過</span><b id="i-tx">—</b></div>
   <div class="kv"><span>最後發射</span><b id="i-txa">—</b></div>
   <div class="kv"><span>裝填倒數</span><b id="i-rl">—</b></div>
   <div class="kv"><span>開炮掣 / 被擋</span><b id="i-fe">—</b></div>
  </div>
  <div class="card"><h3>接收 (RX)</h3>
   <div class="kv"><span>見到嘅幀 / 命中樣板</span><b id="i-se">—</b></div>
   <div class="kv"><span>被擊中次數</span><b id="i-hi">—</b></div>
   <div class="kv"><span>最後被擊中</span><b id="i-ha">—</b></div>
   <div class="kv"><span>連續命中幀 / 鎖定</span><b id="i-mc">—</b></div>
   <div class="kv"><span>環境 IR 雜訊</span><b id="i-ep">—</b></div>
   <div class="kv"><span>ISR 邊緣總數 / 丟失</span><b id="i-ed">—</b></div>
   <div class="alert w" id="i-hint" hidden></div>
  </div>
 </div>
 <div class="card"><h3>協議樣板 (全部同時啟用)</h3><div class="scroll"><table><thead><tr><th>協議</th><th>狀態</th><th>元素</th><th>重複</th><th>週期</th><th>發射時間</th><th>已發射</th><th>收到幀</th><th>命中</th><th></th></tr></thead><tbody id="pf-body"></tbody></table></div></div>
 <div class="card"><h3>波形對比 (最後收到嘅一幀 vs 序列第一個協議樣板)</h3>
  <canvas id="scope" width="900" height="190"></canvas>
  <div class="mut" id="i-lf" style="margin-top:6px">—</div>
  <div class="mut" style="margin-top:4px">灰色虛線 = 樣板；彩色 = 實際收到 (綠 = 命中、紅 = 未識別、藍 = 學習中)。</div>
  <div class="row2"><b>最後一個未識別訊號</b> <span class="mut" id="i-ukm">—</span></div><pre id="i-uk">—</pre>
 </div>
</section>

<section id="t-sys" class="tab" hidden>
 <div class="grid">
  <div class="card"><h3>裝置</h3>
   <div class="kv"><span>韌體</span><b id="s-fw">—</b></div><div class="kv"><span>晶片</span><b id="s-chip">—</b></div>
   <div class="kv"><span>CPU / Flash</span><b id="s-cpu">—</b></div><div class="kv"><span>SDK</span><b id="s-sdk">—</b></div>
   <div class="kv"><span>運行時間</span><b id="s-up">—</b></div><div class="kv"><span>上次重啟原因</span><b id="s-rst">—</b></div>
  </div>
  <div class="card"><h3>記憶體</h3>
   <div class="kv"><span>可用</span><b id="s-hp">—</b></div><div class="kv"><span>歷來最低</span><b id="s-hm">—</b></div>
   <div class="kv"><span>最大連續區塊</span><b id="s-hx">—</b></div><div class="kv"><span>音效 RAM 快取</span><b id="s-cu">—</b></div>
   <div class="kv"><span>檔案空間</span><b id="s-fs">—</b></div>
  </div>
  <div class="card"><h3>音訊處理</h3>
   <div class="kv"><span>負載 (平均 / block 時長)</span><b id="s-ld">—</b></div><div class="bar"><i id="s-ldb"></i></div>
   <div class="kv"><span>每個 block 平均</span><b id="s-av">—</b></div><div class="kv"><span>最近 1 秒最大</span><b id="s-wn">—</b></div>
   <div class="kv"><span>開機以來最大</span><b id="s-mx">—</b></div><div class="kv"><span>偏慢次數 (&gt;<span id="s-th">20</span> ms)</span><b id="s-sl">—</b></div>
   <div class="kv"><span>處理過嘅 block</span><b id="s-bk">—</b></div>
  </div>
  <div class="card"><h3>任務堆疊餘量 (bytes)</h3>
   <div class="kv"><span>音訊/控制 (10240)</span><b id="s-ska">—</b></div><div class="kv"><span>IR 發射 (3072)</span><b id="s-ski">—</b></div>
   <div class="kv"><span>網頁 loop</span><b id="s-skl">—</b></div>
   <div class="mut">數值愈細愈危險；少於 ~1000 要留意。</div>
  </div>
  <div class="card"><h3>Wi-Fi</h3>
   <div class="kv"><span>SSID</span><b id="s-ssid">—</b></div><div class="kv"><span>位址 / 頻道</span><b id="s-ip">—</b></div>
   <div class="kv"><span>已連線裝置</span><b id="s-ws">—</b></div><div class="kv"><span>網頁請求總數</span><b id="s-rq">—</b></div>
  </div>
  <div class="card" id="s-battc"><h3>電池</h3><div class="kv"><span>電壓</span><b id="s-batt">—</b></div></div>
 </div>
 <div class="card"><h3>工具</h3>
  <div class="btns"><button onclick="report()">複製診斷報告</button><button class="gry" onclick="if(confirm('重設全部統計？'))act('/statsreset',()=>poll())">重設統計</button><button class="red" onclick="if(confirm('確定重啟 ESP32？'))reboot()">重啟 ESP32</button></div>
  <div class="mut">「診斷報告」包含遙控、燈光、音效、IR、記憶體同最近事件，貼俾我就可以直接分析。</div>
  <textarea id="rep" hidden readonly></textarea>
 </div>
</section>

<section id="t-log" class="tab" hidden>
 <div class="card"><h3>事件記錄 <span class="mut">(最新喺上面，最多 48 條)</span></h3>
  <div class="row2"><button class="s gry" onclick="clearLog()">清除畫面</button></div>
  <div id="log-body"></div></div>
</section>

</main>
<div id="toast"></div>
<script>
'use strict';
const $=id=>document.getElementById(id);
const E=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const ENG=['熄火','點火中','運行中','熄匙中'],VNAME=['點火/熄匙','主炮','受擊','機槍','炮塔','升降'];
const LS=['—','等待開火…','錄取中…','學習成功','學習失敗'];
const TABS=[['ov','總覽'],['rc','遙控'],['lt','燈光'],['wav','音效'],['ir','IR'],['sys','系統'],['log','記錄']];
const LCOL=['#fff3c4','#ff4a3d','#ffd166','#ff8a3d','#ff2e2e','#7fd1ff'];
const AL=[[0,'e','遙控無訊號：檢查 iBUS 接線、接收機同發射機'],[1,'w','有音效檔缺少 (見「音效」分頁)'],[2,'e','有音效檔格式錯誤 (見「音效」分頁)'],
 [3,'e','可用記憶體偏低'],[4,'w','最近 60 秒內音訊處理偏慢 (可能卡頓或爆音)'],[5,'e','上次重啟原因：欠壓 (Brownout)，請檢查電源 / UBEC / 電容'],
 [6,'w','冇任何可發射嘅 IR 協議 (請「學習」或重設 Tamiya)'],[7,'i','環境 IR 雜訊偏高 (強光或其他紅外線，可能影響命中判定)'],[8,'e','電池電壓偏低'],
 [9,'w','檔案空間接近滿 (>90%)'],[10,'w','任務堆疊餘量偏低'],[11,'e','上次重啟原因：程式崩潰 / 看門狗 (亦可能係欠壓)'],[12,'w','遙控資料錯誤偏多 (干擾或接線問題)']];
let cur='ov',cfg=null,live=null,logSeq=0,busy=false,fails=0,timer=0;
const q=async u=>{const r=await fetch(u,{cache:'no-store'});if(!r.ok)throw new Error(u+' '+r.status);return r.json()};
function toast(t){const e=$('toast');e.textContent=t;e.classList.add('on');clearTimeout(toast.t);toast.t=setTimeout(()=>e.classList.remove('on'),2400)}
function act(u,after){return fetch(u,{cache:'no-store'}).then(r=>r.text()).then(t=>{if(after)after(t);return t}).catch(()=>toast('請求失敗'))}
const age=ms=>ms<0?'—':ms<1000?ms+' ms 前':ms<60000?(ms/1000).toFixed(1)+' 秒前':ms<3600000?Math.floor(ms/60000)+' 分 '+Math.floor(ms%60000/1000)+' 秒前':Math.floor(ms/3600000)+' 小時前';
const dur=s=>{const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return (h?h+' 小時 ':'')+m+' 分 '+(s%60)+' 秒'};
const kb=b=>(b/1024).toFixed(b<10240?1:0)+' KB';
const pc=(v,a,b)=>Math.max(0,Math.min(100,(v-a)*100/(b-a)));
const w=(id,p)=>{$(id).style.width=p+'%'};
const clock=ms=>{const s=Math.floor(ms/1000);return String(Math.floor(s/3600)).padStart(2,'0')+':'+String(Math.floor(s/60)%60).padStart(2,'0')+':'+String(s%60).padStart(2,'0')};
function pill(id,cls,t){const e=$(id);e.className='pill '+cls;e.textContent=t}
function setIf(id,v){const e=$(id);if(document.activeElement!==e)e.value=v}
function sel(id,v){const e=$(id);if(document.activeElement!==e&&String(e.value)!==String(v))e.value=v}
function lampStyle(el,v,i){const a=v/255;el.style.background=a>0.02?LCOL[i]:'#222';el.style.opacity=a>0.02?(0.25+0.75*a):1;el.style.boxShadow=a>0.05?`0 0 ${4+14*a}px ${LCOL[i]}`:'none'}
function go(t){cur=t;document.querySelectorAll('.tab').forEach(s=>s.hidden=s.id!=='t-'+t);
 document.querySelectorAll('#tabs button').forEach(b=>b.classList.toggle('on',b.dataset.t===t));
 if(t==='log')clearLog();poll()}
function clearLog(){logSeq=0;$('log-body').innerHTML=''}

function build(){
 $('tabs').innerHTML=TABS.map(t=>`<button data-t="${t[0]}" onclick="go('${t[0]}')">${t[1]}</button>`).join('');
 $('o-vo').innerHTML=[...VNAME,'引擎 Loop'].map((n,i)=>`<div class="vrow" id="vr${i}"><span class="vn">${n}</span><span class="vs" id="vs${i}">—</span><span></span><div class="bar g"><i id="vb${i}"></i></div></div>`).join('');
 $('o-fn').innerHTML=cfg.fn.slice(0,6).map((n,i)=>`<div class="fnrow"><span class="vn">${E(n)}</span><div class="bar"><i id="fb${i}"></i></div><b id="fv${i}">—</b></div>`).join('');
 $('o-lt').innerHTML=cfg.lights.map((l,i)=>`<div style="text-align:center"><div class="lamp" id="ol${i}" style="margin:0 auto 4px"></div><div class="mut">${E(l.n.split(' ')[0])}</div></div>`).join('');
 let h='';for(let i=0;i<14;i++)h+=`<tr><td>CH${i+1}</td><td id="rf${i}" class="mut"></td><td><div class="rbar"><s style="left:50%"></s><i id="rb${i}"></i><u id="rn${i}" style="left:0"></u><u id="rx${i}" style="left:0"></u></div></td><td class="n" id="rv${i}">—</td><td class="n mut" id="rr${i}">—</td></tr>`;
 $('rc-body').innerHTML=h;
 const opt='<option value="-1">停用</option>'+Array.from({length:14},(_,i)=>`<option value="${i}">CH${i+1}</option>`).join('');
 $('map-grid').innerHTML=cfg.fn.map((n,i)=>`<label>${E(n)}<select id="mp${i}" onchange="act('/map?fn=${i}&ch='+this.value,()=>{loadCfg();poll()})">${opt}</select></label>`).join('');
 $('lt-list').innerHTML=cfg.lights.map((l,i)=>`<div class="lrowl"><div class="lamp" id="ll${i}"></div><div><div><b>${E(l.n)}</b> <span class="mut">GPIO ${l.pin>=0?l.pin:'停用'}</span> <span class="mut" id="lv${i}"></span></div>
  <div class="bar"><i id="lb${i}"></i></div>
  <div class="lctl"><span class="mut">最大亮度</span><input type="range" min="0" max="100" id="lm${i}"><b id="lp${i}" style="min-width:42px;text-align:right">100%</b><button class="s grn" onclick="act('/lighttest?i=${i}')">測試</button></div></div></div>`).join('');
 cfg.lights.forEach((l,i)=>{const r=$('lm'+i);r.oninput=()=>$('lp'+i).textContent=r.value+'%';r.onchange=()=>act('/lightmax?i='+i+'&v='+r.value)});
 $('l-hm').onchange=e=>act('/headmode?v='+e.target.value);
 $('wav-list').innerHTML=cfg.snd.map(s=>`<div class="srow" id="sr-${s.n}"><div class="s-head"><b>${E(s.l)}</b><span class="mut">${s.n}.wav</span><span class="pill" id="ss-${s.n}">—</span></div>
  <div class="s-info" id="si-${s.n}">—</div><div class="bar g"><i id="sb-${s.n}"></i></div>
  <div class="s-act"><button class="s grn" onclick="play('${s.n}')">▶ 測試</button><input type="file" id="uf-${s.n}" accept=".wav,audio/wav"><button class="s" onclick="upload('${s.n}')">上載</button><button class="s red" onclick="del('${s.n}','${E(s.l)}')">刪除</button></div>
  <div class="bar up b"><i id="up-${s.n}"></i></div></div>`).join('');
 $('i-tune').onchange=e=>act('/irtune?val='+e.target.value);
 $('vol').oninput=e=>$('vol-v').textContent=e.target.value+'%';
 $('vol').onchange=e=>act('/setVolume?val='+e.target.value);
 $('pf-body').innerHTML=cfg.pn.map((n,i)=>`<tr><td><b>${n}</b></td><td id="pa${i}"></td><td class="n" id="pb${i}"></td><td class="n" id="pc${i}"></td><td class="n" id="pd${i}"></td><td class="n" id="pe${i}"></td><td class="n" id="ph${i}"></td><td class="n" id="pf${i}"></td><td class="n" id="pg${i}"></td><td style="white-space:nowrap"><button class="s grn" onclick="act('/irlearn?slot=${i}',()=>poll())">學習</button> <button class="s gry" onclick="if(confirm('重設 ${n}？'))act('/irreset?slot=${i}',()=>poll())">重設</button></td></tr>`).join('');
 applyCfg();
}
async function loadCfg(){cfg=await q('/api/cfg');applyCfg()}
function applyCfg(){
 if(!cfg)return;$('fw').textContent=cfg.fw;
 if(document.activeElement!==$('vol')){$('vol').value=cfg.vol;$('vol-v').textContent=cfg.vol+'%'}
 setIf('i-tune',cfg.tune);sel('l-hm',cfg.hm);
 cfg.map.forEach((m,i)=>{const e=$('mp'+i);if(e)sel('mp'+i,m)});
 cfg.lights.forEach((l,i)=>{const r=$('lm'+i);if(r&&document.activeElement!==r){r.value=l.pct;$('lp'+i).textContent=l.pct+'%'}});
 const nm=c=>c<0?'(停用)':'CH'+(c+1);$('l-chh').textContent=nm(cfg.map[6]);$('l-cha').textContent=nm(cfg.map[7]);
}

async function tick(url,fn){const d=await q(url);try{fn(d)}catch(e){console.error('render '+url+': '+e.message);throw e}}
async function poll(){
 if(busy)return;busy=true;clearTimeout(timer);
 try{
  await tick('/api/live',renderLive);
  if(cur==='rc')await tick('/api/rc',renderRc);
  else if(cur==='wav')await tick('/api/wav',renderWav);
  else if(cur==='ir')await tick('/api/ir',renderIr);
  else if(cur==='sys')await tick('/api/sys',renderSys);
  else if(cur==='log')await tick('/api/log?since='+logSeq,renderLog);
  fails=0;$('off').hidden=true;
 }catch(e){if(++fails>=2)$('off').hidden=false}
 busy=false;timer=setTimeout(poll,(cur==='rc'||cur==='ov'||cur==='lt')?300:600);
}

function renderLive(L){
 live=L;
 pill('h-link',L.link?'ok':'bad','遙控 '+(L.link?'正常 '+L.hz+' Hz':'無訊號'));
 pill('h-eng','info','引擎 '+ENG[L.eng]);
 pill('h-ir','','IR 互通 '+L.ps+' 款｜擊中 '+L.hits+'｜發射 '+L.tx);
 const items=AL.filter(a=>L.al>>a[0]&1);
 const worst=items.some(a=>a[1]==='e')?'bad':items.some(a=>a[1]==='w')?'warn':items.length?'info':'ok';
 pill('h-al',worst,items.length?'警示 '+items.length:'一切正常');
 L.lt.forEach((v,i)=>{ if(cur==='ov')lampStyle($('ol'+i),v,i); if(cur==='lt'){lampStyle($('ll'+i),v,i);w('lb'+i,v*100/255);$('lv'+i).textContent=Math.round(v*100/255)+'%'} });
 if(cur!=='ov')return;
 $('al-list').innerHTML=items.length?items.map(a=>{let t=a[2];if(a[0]===1)t+=` (${L.se.wm} 個)`;if(a[0]===2)t+=` (${L.se.wb} 個)`;return `<div class="alert ${a[1]}">${E(t)}</div>`}).join(''):'<div class="alert k">✔ 一切正常</div>';
 $('o-eng').textContent=ENG[L.eng];
 $('o-pit').textContent=(L.pit/1000).toFixed(2)+'×';w('o-pitb',pc(L.pit/1000,0.55,2.1));
 $('o-ld').textContent=L.ld;w('o-ldb',pc(L.ld,0,500));
 for(let i=0;i<6;i++){const v=L.vo[i];vo(i,v[0],v[2])}
 vo(6,L.ev[0]?0:-1,L.ev[2]);
 for(let i=0;i<6;i++){const ch=L.map[i];const e=$('fv'+i);
  if(ch<0){e.textContent='停用';w('fb'+i,0);continue}
  const v=L.ch[ch];
  if(i>=4){const on=L.fs>>(i-4)&1;e.textContent=on?'ON':'OFF';w('fb'+i,on?100:0)}
  else{e.textContent='CH'+(ch+1)+' '+v;w('fb'+i,pc(v,1000,2000))}}
 $('o-pr').textContent=L.ps+' 款';
 $('o-tx').textContent=L.tx+' 次 / '+age(L.txa);
 $('o-hit').textContent=L.hits+' 次 / '+age(L.hta);
 $('o-up').textContent=dur(L.up);$('o-heap').textContent=kb(L.heap);
 $('o-aload').textContent=L.aload+'%';$('o-fr').textContent=L.fr+' 次/秒';
}
function vo(i,snd,prog){const on=snd>=0;$('vr'+i).classList.toggle('on',on);
 $('vs'+i).textContent=on?cfg.snd[snd].l:'—';w('vb'+i,on?prog/10:0)}

function renderRc(D){
 $('r-link').innerHTML=D.link?'<span style="color:var(--ok)">正常</span>':'<span style="color:var(--bad)">無訊號</span>';
 $('r-hz').textContent=D.hz+' Hz';$('r-age').textContent=age(D.age);$('r-fr').textContent=D.fr;
 $('r-bs').textContent=D.bs;$('r-br').textContent=D.br;$('r-dr').textContent=D.dr;$('r-fo').textContent=D.fo;$('r-gap').textContent=D.gap+' ms';
 for(let i=0;i<14;i++){
  const fns=D.map.map((c,f)=>c===i?cfg.fn[f]:null).filter(Boolean);
  $('rf'+i).textContent=fns.join(' / ');
  const v=D.ch[i];
  $('rv'+i).textContent=v;w('rb'+i,pc(v,900,2100));
  $('rn'+i).style.left=pc(D.mn[i]||v,900,2100)+'%';$('rx'+i).style.left=pc(D.mx[i]||v,900,2100)+'%';
  $('rr'+i).textContent=(D.mn[i]||'—')+'~'+(D.mx[i]||'—');
 }
 D.map.forEach((m,i)=>sel('mp'+i,m));
}

function renderWav(D){
 $('w-cache').textContent=kb(D.cu)+' / '+kb(D.cb);w('w-cacheb',D.cb?D.cu*100/D.cb:0);
 $('w-fs').textContent=kb(D.fu)+' / '+kb(D.ft);w('w-fsb',D.ft?D.fu*100/D.ft:0);
 $('w-fr').textContent=D.fr+' 次/秒，'+kb(D.fb)+'/秒';$('w-up').textContent=D.up;
 D.snd.forEach(s=>{
  const n=s.n,st=$('ss-'+n),row=$('sr-'+n);
  st.className='pill '+(s.ok?'ok':s.rs===1?'warn':'bad');st.textContent=s.ok?'OK':s.rt;row.classList.toggle('bad',!s.ok);
  let info;
  if(s.ok)info=`${kb(s.sz)}｜${(s.du/1000).toFixed(2)} 秒｜${s.ram?'已載入 RAM':'由 flash 串流'}｜播放 ${s.pl} 次｜最後 ${age(s.la)}`+(s.of?`｜<span style="color:var(--bad)">開檔失敗 ${s.of}</span>`:'')+(s.io?`｜<span style="color:var(--bad)">讀取錯誤 ${s.io}</span>`:'');
  else{info=s.rs===1?'檔案唔存在，請上載。':`實際格式：${s.hz?s.hz+' Hz':'—'} / ${s.ch||'—'} 聲道 / ${s.bt||'—'} bit (需要 22050 Hz / 1 聲道 / 16 bit PCM)`;if(s.of)info+=`｜開檔失敗 ${s.of}`}
  $('si-'+n).innerHTML=info;w('sb-'+n,s.pg>=0?s.pg/10:0);
 });
}
function play(n){act('/play?s='+n,()=>setTimeout(poll,120))}
function del(n,l){if(confirm('刪除「'+l+'」？'))act('/del?file='+n,()=>{toast('已刪除');poll()})}
function upload(n){
 const f=$('uf-'+n).files[0];if(!f){toast('請先揀一個 .wav 檔');return}
 const x=new XMLHttpRequest(),fd=new FormData();fd.append('wav',f,f.name);
 x.open('POST','/upload?file='+n);
 x.upload.onprogress=e=>{if(e.lengthComputable)w('up-'+n,e.loaded*100/e.total)};
 x.onload=()=>{w('up-'+n,0);q('/api/wav').then(d=>{toast(d.up);renderWav(d)}).catch(()=>toast(x.status===200?'上載成功':'上載失敗'))};
 x.onerror=()=>{w('up-'+n,0);toast('上載失敗：連線中斷')};
 x.send(fd);toast('上載中…');
}

function drawScope(cv,ref,fr,col){
 const c=cv.getContext('2d'),W=cv.width,H=cv.height;
 c.fillStyle='#0b0d0f';c.fillRect(0,0,W,H);
 const sum=a=>a.reduce((x,y)=>x+y,0),tot=Math.max(sum(ref),sum(fr),1000),sx=(W-24)/tot;
 c.strokeStyle='#222a31';c.lineWidth=1;c.font='11px sans-serif';c.fillStyle='#6e7681';
 const step=tot>20000?5000:tot>8000?2000:1000;
 for(let t=0;t<=tot;t+=step){const x=12+t*sx;c.beginPath();c.moveTo(x,0);c.lineTo(x,H-14);c.stroke();c.fillText((t/1000)+' ms',x+2,H-3)}
 const lane=(a,color,top,dash,label)=>{
  c.fillStyle='#6e7681';c.fillText(label,14,top-2);
  if(!a.length){c.fillText('(冇資料)',90,top+18);return}
  c.strokeStyle=color;c.lineWidth=2;c.setLineDash(dash?[5,3]:[]);c.beginPath();
  let x=12;const hi=top+4,lo=top+44;c.moveTo(x,lo);
  a.forEach((d,i)=>{const y=i%2===0?hi:lo;c.lineTo(x,y);x+=d*sx;c.lineTo(x,y)});
  c.lineTo(x,lo);c.stroke();c.setLineDash([]);
 };
 lane(ref,'#8b949e',14,true,'樣板');lane(fr,col,H/2+6,false,'實際收到');
}
function renderIr(D){
 setIf('i-tune',D.tn);
 $('i-seq').textContent=D.po.length?D.po.map(i=>cfg.pn[i]).join(' → '):'(冇)';
 $('i-sm').textContent=(D.tx.sm/1000).toFixed(2)+' 秒';
 let ls=LS[D.ls];if(D.ls===1)ls+=` (剩 ${Math.ceil(D.ll/1000)} 秒，${cfg.pn[D.lp]})`;if(D.ls===2)ls+=` (${D.lt} 幀，${cfg.pn[D.lp]})`;
 $('i-ls').textContent=ls;
 $('i-tx').textContent=D.tx.n+' / '+D.tx.rq+' / '+D.tx.sk;$('i-txa').textContent=age(D.tx.la)+(D.tx.lb?` (用咗 ${D.tx.lb} ms)`:'');
 $('i-rl').textContent=D.tx.rl?D.tx.rl+' ms / '+D.tx.rm+' ms':'已就緒';
 $('i-fe').textContent=D.tx.fe+' / '+D.tx.fb;
 $('i-se').textContent=D.rx.se+' / '+D.rx.ma;$('i-hi').textContent=D.rx.hi;
 $('i-ha').textContent=age(D.rx.ha)+(D.rx.hp>=0?' ['+cfg.pn[D.rx.hp]+']':'');
 $('i-mc').textContent=D.rx.mc+' / '+(D.rx.lt?'鎖定':'備戰')+(D.rx.bl?' (暫停接收 '+D.rx.bl+' ms)':'');
 $('i-ep').innerHTML=D.rx.ep+' 邊緣/秒'+(D.rx.ep>D.rx.nl?' <span style="color:var(--warn)">偏高</span>':' <span style="color:var(--ok)">正常</span>');
 $('i-ed').textContent=D.rx.ed+' / '+D.rx.dr;
 const h=$('i-hint');if(D.rx.ed>100&&D.rx.se===0&&D.ls===0){h.hidden=false;h.textContent='有收到紅外線訊號 (邊緣 '+D.rx.ed+' 個)，但一幀都分唔到：訊號之間嘅間隔可能少於 5 ms，或者雜訊太多。可以試「學習」，或者貼診斷報告俾我睇。'}else h.hidden=true;
 D.pf.forEach((p,i)=>{
  let st=p.v?(p.b?'<span class="pill ok">內置</span>':'<span class="pill info">已學習</span>'):'<span class="pill bad">未設定</span>';
  if(p.dp>=0)st+=` <span class="mut">= ${cfg.pn[p.dp]} (只發一次)</span>`;
  $('pa'+i).innerHTML=st;
  $('pb'+i).textContent=p.n||'—';$('pc'+i).textContent=p.rp||'—';$('pd'+i).textContent=p.pe?(p.pe/1000).toFixed(1)+' ms':'—';
  $('pe'+i).textContent=p.v?p.bm+' ms':'—';$('ph'+i).textContent=p.tc;$('pf'+i).textContent=p.fm;$('pg'+i).textContent=p.hm;
 });
 const lf=D.lf,col=lf.p>=0?'#3fb950':lf.p===-2?'#2f81f7':'#f85149';
 drawScope($('scope'),D.rf,lf.d,col);
 $('i-lf').textContent=lf.n?`最後一幀：${lf.n} 個元素，${age(lf.a)}，`+(lf.p>=0?'命中 '+cfg.pn[lf.p]:lf.p===-2?'學習中':'未識別'):'未收到任何 IR 訊號 (對住接收頭開火試下)';
 $('i-ukm').textContent=D.uk.n?`${D.uk.n} 個元素，${age(D.uk.a)}`:'';
 $('i-uk').textContent=D.uk.n?D.uk.d.join(','):'未有未識別訊號';
}

function renderSys(D){
 $('s-fw').textContent=D.fw;$('s-chip').textContent=D.cm+' rev'+D.cr;$('s-cpu').textContent=D.cf+' MHz / '+(D.fz/1048576)+' MB';
 $('s-sdk').textContent=D.sdk;$('s-up').textContent=dur(D.up);
 $('s-rst').innerHTML=E(D.rsn)+((D.rst===9||D.rst===4||D.rst===5||D.rst===6||D.rst===7)?' <span class="pill warn">留意</span>':'');
 $('s-hp').textContent=kb(D.hp);$('s-hm').textContent=kb(D.hm);$('s-hx').textContent=kb(D.hx);$('s-cu').textContent=kb(D.cu);$('s-fs').textContent=kb(D.fu)+' / '+kb(D.ft);
 const a=D.au;$('s-ld').textContent=a.ld+'% ('+a.av+' / '+a.bp+' µs)';w('s-ldb',Math.min(100,a.ld));
 $('s-av').textContent=a.av+' µs';$('s-wn').textContent=a.wn+' µs';$('s-mx').textContent=a.mx+' µs';$('s-th').textContent=a.th/1000;
 $('s-sl').textContent=a.sl+(a.sa>=0?' (最近 '+age(a.sa)+')':'');$('s-bk').textContent=a.bk;
 $('s-ska').textContent=D.sk.a||'—';$('s-ski').textContent=D.sk.i||'(未發射過)';$('s-skl').textContent=D.sk.l||'—';
 $('s-ssid').textContent=D.ssid;$('s-ip').textContent=D.ip+' / CH'+D.wc;$('s-ws').textContent=D.ws;$('s-rq').textContent=D.rq;
 $('s-battc').hidden=!D.bt.en;if(D.bt.en)$('s-batt').textContent=(D.bt.mv/1000).toFixed(2)+' V'+(D.bt.lo?' (警示 < '+(D.bt.lo/1000).toFixed(2)+' V)':'');
}
async function report(){
 let t;try{t=await (await fetch('/report',{cache:'no-store'})).text()}catch(e){toast('讀取失敗');return}
 const ta=$('rep');ta.hidden=false;ta.value=t;ta.focus();ta.select();ta.scrollTop=0;let ok=false;
 try{if(navigator.clipboard&&window.isSecureContext){await navigator.clipboard.writeText(t);ok=true}}catch(e){}
 if(!ok){try{ok=document.execCommand('copy')}catch(e){}}
 toast(ok?'已複製診斷報告，可直接貼上':'請手動全選複製下面文字');
}
function reboot(){act('/reboot',()=>toast('重啟中…等約 10 秒再重新整理'))}

function renderLog(D){
 if(D.sq<logSeq){clearLog()}
 const b=$('log-body');
 D.e.forEach(e=>{const d=document.createElement('div');d.className='lrow';d.innerHTML=`<time>${clock(e[1])}</time><span>${E(e[2])}</span>`;b.insertBefore(d,b.firstChild)});
 while(b.children.length>200)b.removeChild(b.lastChild);
 logSeq=D.sq;
}

(async()=>{try{cfg=await q('/api/cfg');build();go('ov')}catch(e){$('off').hidden=false;setTimeout(()=>location.reload(),2000)}})();
</script></body></html>
)rawliteral";

// ===================== 網頁：資料 API =====================
void handleApiLive() {
  uint32_t now = millis();
  uint16_t s[14];
  portENTER_CRITICAL(&channelMux); memcpy(s, debugChannels, sizeof(s)); portEXIT_CRITICAL(&channelMux);
  IRPlan plan; irBuildPlan(plan);
  String j; j.reserve(900);
  jopen(j, nullptr, '{');
  ju(j, "up", (uint32_t)(esp_timer_get_time() / 1000000));
  jn(j, "eng", (int)engineState);
  jn(j, "link", ibusLinkOk ? 1 : 0);
  ju(j, "hz", rateIbusHz);
  jn(j, "pit", (long)(eng.pitch * 1000.f));
  jn(j, "ld", (long)eng.fLoad);
  jn(j, "vol", (long)lroundf(globalVolume * 100));
  jarr16(j, "ch", s, 14);
  jopen(j, "map", '[');
  for (int i = 0; i < FN_COUNT; i++) jv(j, chMap[i]);
  jclose(j, ']');
  jn(j, "fs", fnState);
  jopen(j, "vo", '[');
  for (int v = 0; v < V_COUNT; v++) {
    jopen(j, nullptr, '[');
    jv(j, voices[v].active ? voices[v].snd : -1);
    jv(j, voices[v].active ? (long)(voices[v].gain * 1000.f) : 0);
    uint32_t n = voices[v].r.samples();
    jv(j, (voices[v].active && n) ? (long)((uint64_t)voices[v].pos * 1000 / n) : 0);
    jclose(j, ']');
  }
  jclose(j, ']');
  jopen(j, "ev", '[');
  jv(j, eng.ready ? 1 : 0); jv(j, (long)(eng.gain * 1000.f)); jv(j, soundProgress(S_ENGINE));
  jclose(j, ']');
  jopen(j, "lt", '[');
  for (int i = 0; i < LT_COUNT; i++) jv(j, lightOut[i]);
  jclose(j, ']');
  ju(j, "hits", irRxHits);
  ju(j, "tx", irTxCount);
  jn(j, "txa", ageOf(irLastTxMs, now));
  jn(j, "hta", ageOf(irLastHitAtMs, now));
  jn(j, "ps", plan.n);
  ju(j, "al", computeAlerts());
  ju(j, "heap", (uint32_t)ESP.getFreeHeap());
  jn(j, "aload", (long)(audioAvgUs * 100.f / (float)BLOCK_PERIOD_US));
  ju(j, "fr", rateFlashReads);
  jopen(j, "se", '{');
  jn(j, "wm", [] { int c = 0; for (int i = 0; i < S_COUNT; i++) if (!soundMeta[i].ok && soundMeta[i].reason == WR_MISSING) c++; return c; }());
  jn(j, "wb", [] { int c = 0; for (int i = 0; i < S_COUNT; i++) if (!soundMeta[i].ok && soundMeta[i].reason != WR_MISSING) c++; return c; }());
  jclose(j, '}');
  jclose(j, '}');
  sendJson(j);
}

void handleApiRc() {
  uint32_t now = millis();
  uint16_t s[14];
  portENTER_CRITICAL(&channelMux); memcpy(s, debugChannels, sizeof(s)); portEXIT_CRITICAL(&channelMux);
  uint16_t mn[14], mx[14];
  for (int i = 0; i < 14; i++) { mn[i] = (chMin[i] == 0xFFFF) ? 0 : chMin[i]; mx[i] = chMax[i]; }
  String j; j.reserve(700);
  jopen(j, nullptr, '{');
  jarr16(j, "ch", s, 14);
  jarr16(j, "mn", mn, 14);
  jarr16(j, "mx", mx, 14);
  jopen(j, "map", '[');
  for (int i = 0; i < FN_COUNT; i++) jv(j, chMap[i]);
  jclose(j, ']');
  jn(j, "link", ibusLinkOk ? 1 : 0);
  ju(j, "hz", rateIbusHz);
  ju(j, "fr", ibusFrames);
  ju(j, "bs", ibusBadSum);
  ju(j, "br", ibusBadRange);
  ju(j, "dr", ibusDropBytes);
  ju(j, "fo", ibusFailsafe);
  ju(j, "gap", ibusMaxGapMs);
  jn(j, "age", ageOf(lastIbusMs, now));
  ju(j, "to", IBUS_TIMEOUT_MS);
  jn(j, "fs", fnState);
  jclose(j, '}');
  sendJson(j);
}

void handleApiWav() {
  uint32_t now = millis();
  String j; j.reserve(2600);
  jopen(j, nullptr, '{');
  jopen(j, "snd", '[');
  for (int i = 0; i < S_COUNT; i++) {
    const SoundMeta& m = soundMeta[i];
    jopen(j, nullptr, '{');
    js(j, "n", SOUND_LIST[i].name);
    js(j, "l", SOUND_LIST[i].label);
    jn(j, "ok", m.ok ? 1 : 0);
    jn(j, "rs", m.reason);
    js(j, "rt", reasonTxt(m.reason));
    ju(j, "sz", m.size);
    ju(j, "dl", m.ok ? m.dataLen : 0);
    ju(j, "du", m.ok ? (uint32_t)((uint64_t)m.dataLen / 2 * 1000 / AUDIO_RATE) : 0);
    jn(j, "ram", m.ram ? 1 : 0);
    ju(j, "fm", m.fmt);
    ju(j, "ch", m.chans);
    ju(j, "hz", m.rate);
    ju(j, "bt", m.bits);
    ju(j, "pl", wavPlays[i]);
    jn(j, "la", ageOf(wavLastMs[i], now));
    ju(j, "of", wavOpenFail[i]);
    ju(j, "io", wavIoErr[i]);
    jn(j, "pg", soundProgress(i));
    jclose(j, '}');
  }
  jclose(j, ']');
  ju(j, "cu", ramCacheBytes);
  ju(j, "cb", RAM_CACHE_BUDGET);
  ju(j, "fu", fsUsedCache);
  ju(j, "ft", fsTotalCache);
  ju(j, "fr", rateFlashReads);
  ju(j, "fb", rateFlashBytes);
  ju(j, "mx", MAX_SOUND_BYTES);
  js(j, "up", uploadResult.c_str());
  ju(j, "hp", (uint32_t)ESP.getFreeHeap());
  jclose(j, '}');
  sendJson(j);
}

static void jframe(String& j, const char* k, const FrameSnap& f, uint32_t now) {
  uint16_t d[IR_FRAME_MAX]; uint8_t n = 0; uint32_t t = 0; int8_t p = -1;
  bool ok = f.get(d, n, t, p) && n > 0;
  jopen(j, k, '{');
  jn(j, "n", ok ? n : 0);
  jn(j, "a", ok ? (long)(now - t) : -1);
  jn(j, "p", ok ? p : -1);
  jopen(j, "d", '[');
  if (ok) for (int i = 0; i < n; i++) jv(j, d[i]);
  jclose(j, ']');
  jclose(j, '}');
}

void handleApiIr() {
  uint32_t now = millis();
  IRPlan plan; irBuildPlan(plan);
  String j; j.reserve(3200);
  jopen(j, nullptr, '{');
  jn(j, "tn", irMarkExcessUs);
  jn(j, "bz", irTxBusy ? 1 : 0);
  jn(j, "ls", irLearnState);
  jn(j, "lp", irLearnSlot);
  jn(j, "ln", lrnN);
  jn(j, "lt", lrnTotal);
  jn(j, "ll", irLearnState == LRN_WAIT ? (long)(20000 - (long)(now - lrnStartMs) > 0 ? 20000 - (long)(now - lrnStartMs) : 0) : 0);
  jopen(j, "po", '[');
  for (int k = 0; k < plan.n; k++) jv(j, plan.idx[k]);
  jclose(j, ']');
  jopen(j, "tx", '{');
  ju(j, "n", irTxCount);
  ju(j, "rq", irTxRequests);
  ju(j, "sk", irTxSkipped);
  jn(j, "la", ageOf(irLastTxMs, now));
  ju(j, "lb", irLastTxBurstMs);
  ju(j, "rl", fireReloadLeftMs(now));
  ju(j, "rm", fireReloadMs());
  ju(j, "sm", plan.totalMs);
  ju(j, "fe", statFireEdges);
  ju(j, "fb", statFireBlocked);
  jclose(j, '}');
  uint8_t mc = 0; bool lat = false;
  for (int i = 0; i < IR_PROTO_COUNT; i++) { if (irMatchCnt[i] > mc) mc = irMatchCnt[i]; if (irLatched[i]) lat = true; }
  jopen(j, "rx", '{');
  ju(j, "se", irFramesSeen);
  ju(j, "ma", irFramesMatched);
  ju(j, "hi", irRxHits);
  jn(j, "ha", ageOf(irLastHitAtMs, now));
  jn(j, "hp", irLastHitProto);
  ju(j, "mc", mc);
  jn(j, "lt", lat ? 1 : 0);
  jn(j, "bl", (int32_t)(irRxBlockUntil - now) > 0 ? (long)(irRxBlockUntil - now) : 0);
  ju(j, "ed", irEdges);
  ju(j, "dr", irRingDrops);
  ju(j, "ep", rateIrEdges);
  ju(j, "nl", IR_NOISE_EPS);
  jclose(j, '}');
  jopen(j, "pf", '[');
  for (int i = 0; i < IR_PROTO_COUNT; i++) {
    const IRProfile& p = irProfiles[i];
    jopen(j, nullptr, '{');
    jn(j, "v", p.valid); jn(j, "b", p.builtin); jn(j, "n", p.len); jn(j, "rp", p.repeats); jn(j, "pe", p.periodUs);
    jn(j, "dp", irDupOf(i));
    ju(j, "bm", irBurstMs(p)); ju(j, "fm", irFramesByProto[i]); ju(j, "hm", irHitsByProto[i]); ju(j, "tc", irTxByProto[i]);
    jclose(j, '}');
  }
  jclose(j, ']');
  jframe(j, "lf", irLast, now);
  jframe(j, "uk", irUnk, now);
  jopen(j, "rf", '[');
  if (plan.n > 0) { const IRProfile& cp = irProfiles[plan.idx[0]]; for (int i = 0; i < cp.len; i++) jv(j, cp.d[i]); }
  jclose(j, ']');
  jclose(j, '}');
  sendJson(j);
}

void handleApiSys() {
  uint32_t now = millis();
  String j; j.reserve(700);
  jopen(j, nullptr, '{');
  js(j, "fw", FW_VERSION);
  ju(j, "up", (uint32_t)(esp_timer_get_time() / 1000000));
  js(j, "cm", ESP.getChipModel());
  jn(j, "cr", ESP.getChipRevision());
  jn(j, "cf", ESP.getCpuFreqMHz());
  ju(j, "fz", (uint32_t)ESP.getFlashChipSize());
  js(j, "sdk", ESP.getSdkVersion());
  jn(j, "rst", bootResetReason);
  js(j, "rsn", resetReasonName(bootResetReason));
  ju(j, "hp", (uint32_t)ESP.getFreeHeap());
  ju(j, "hm", (uint32_t)ESP.getMinFreeHeap());
  ju(j, "hx", (uint32_t)ESP.getMaxAllocHeap());
  ju(j, "cu", ramCacheBytes);
  ju(j, "fu", fsUsedCache);
  ju(j, "ft", fsTotalCache);
  js(j, "ssid", WIFI_SSID);
  jn(j, "wc", WiFi.channel());
  jn(j, "ws", WiFi.softAPgetStationNum());
  js(j, "ip", WiFi.softAPIP().toString().c_str());
  jopen(j, "au", '{');
  jn(j, "av", (long)audioAvgUs);
  ju(j, "mx", audioMaxUs);
  ju(j, "wn", audioMaxRecentUs);
  ju(j, "sl", audioSlow);
  jn(j, "sa", ageOf(audioSlowLastMs, now));
  ju(j, "bk", audioBlocks);
  jn(j, "ld", (long)(audioAvgUs * 100.f / (float)BLOCK_PERIOD_US));
  ju(j, "bp", BLOCK_PERIOD_US);
  ju(j, "th", AUDIO_SLOW_US);
  jclose(j, '}');
  jopen(j, "sk", '{');
  ju(j, "a", audioStackFree); ju(j, "i", irStackFree); ju(j, "l", loopStackFree);
  jclose(j, '}');
  ju(j, "rq", webRequests);
  jopen(j, "bt", '{');
  jn(j, "en", BATT_ADC_PIN >= 0 ? 1 : 0);
  ju(j, "mv", battMv);
  ju(j, "lo", BATT_LOW_MV);
  jclose(j, '}');
  ju(j, "al", computeAlerts());
  jclose(j, '}');
  sendJson(j);
}

void handleApiLog() {
  uint32_t cur = logSeq;
  uint32_t since = (uint32_t)server.arg("since").toInt();
  if (since > cur) since = 0;
  uint32_t first = since + 1;
  if (cur >= LOG_N && first < cur - LOG_N + 1) first = cur - LOG_N + 1;
  String j; j.reserve(3600);
  jopen(j, nullptr, '{');
  ju(j, "sq", cur);
  ju(j, "now", millis());
  jopen(j, "e", '[');
  for (uint32_t s = first; s <= cur && s >= first; s++) {
    LogEntry e;
    if (!logGet(s, e)) continue;
    jopen(j, nullptr, '[');
    jv(j, (long)e.seq); jv(j, (long)e.ms);
    jesc(j, e.msg); j += ',';
    jclose(j, ']');
  }
  jclose(j, ']');
  jclose(j, '}');
  sendJson(j);
}

void handleApiCfg() {
  String j; j.reserve(1200);
  jopen(j, nullptr, '{');
  jn(j, "vol", (long)lroundf(globalVolume * 100));
  jn(j, "tune", irMarkExcessUs);
  jn(j, "hm", headMode);
  jopen(j, "map", '[');
  for (int i = 0; i < FN_COUNT; i++) jv(j, chMap[i]);
  jclose(j, ']');
  jopen(j, "fn", '[');
  for (int i = 0; i < FN_COUNT; i++) { jesc(j, FN_LABEL[i]); j += ','; }
  jclose(j, ']');
  jopen(j, "pn", '[');
  for (int i = 0; i < IR_PROTO_COUNT; i++) { jesc(j, IR_PROTO_NAME[i]); j += ','; }
  jclose(j, ']');
  jopen(j, "snd", '[');
  for (int i = 0; i < S_COUNT; i++) {
    jopen(j, nullptr, '{'); js(j, "n", SOUND_LIST[i].name); js(j, "l", SOUND_LIST[i].label); jclose(j, '}');
  }
  jclose(j, ']');
  jopen(j, "lights", '[');
  for (int i = 0; i < LT_COUNT; i++) {
    jopen(j, nullptr, '{'); js(j, "n", LIGHT_NAME[i]); jn(j, "pin", LIGHT_PIN[i]); jn(j, "pct", lightPct[i]); jclose(j, '}');
  }
  jclose(j, ']');
  js(j, "fw", FW_VERSION);
  jclose(j, '}');
  sendJson(j);
}

// ===================== 診斷報告 =====================
static void rp(String& r, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
static void rp(String& r, const char* fmt, ...) {
  char b[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  r += b;
}
static void fmtClock(char* out, size_t n, uint32_t ms) {
  uint32_t s = ms / 1000;
  snprintf(out, n, "%02lu:%02lu:%02lu.%03lu", (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60), (unsigned long)(ms % 1000));
}

void handleReport() {
  uint32_t now = millis();
  uint16_t s[14];
  portENTER_CRITICAL(&channelMux); memcpy(s, debugChannels, sizeof(s)); portEXIT_CRITICAL(&channelMux);
  IRPlan plan; irBuildPlan(plan);
  String r; r.reserve(5800);
  uint32_t up = (uint32_t)(esp_timer_get_time() / 1000000);
  rp(r, "=== 坦克聲光+IR 診斷報告 %s ===\n", FW_VERSION);
  rp(r, "運行 %lu:%02lu:%02lu｜上次重啟：%s｜%s rev%d @%dMHz｜Flash %uMB\n", (unsigned long)(up / 3600), (unsigned long)((up / 60) % 60), (unsigned long)(up % 60),
     resetReasonName(bootResetReason), ESP.getChipModel(), ESP.getChipRevision(), ESP.getCpuFreqMHz(), (unsigned)(ESP.getFlashChipSize() / 1048576));
  rp(r, "記憶體：可用 %u KB｜歷來最低 %u KB｜最大連續 %u KB｜音效 RAM 快取 %u KB\n", (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMinFreeHeap() / 1024),
     (unsigned)(ESP.getMaxAllocHeap() / 1024), (unsigned)(ramCacheBytes / 1024));
  rp(r, "音訊：block 平均 %d us / 最大 %u us / 最近1秒最大 %u us｜偏慢 %u 次｜負載 %d%%｜堆疊餘量 音訊 %u / IR %u / loop %u bytes\n",
     (int)audioAvgUs, (unsigned)audioMaxUs, (unsigned)audioMaxRecentUs, (unsigned)audioSlow, (int)(audioAvgUs * 100.f / (float)BLOCK_PERIOD_US),
     (unsigned)audioStackFree, (unsigned)irStackFree, (unsigned)loopStackFree);
  rp(r, "Flash 讀取：%u 次/秒 %u KB/秒｜檔案系統 %u / %u KB\n", (unsigned)rateFlashReads, (unsigned)(rateFlashBytes / 1024), (unsigned)(fsUsedCache / 1024), (unsigned)(fsTotalCache / 1024));
  rp(r, "引擎：%s｜音調 %d%%｜音量 %d%%\n", ENGINE_NAME[(int)engineState & 3], (int)(eng.pitch * 100.f), (int)lroundf(globalVolume * 100));
  rp(r, "\n[遙控] %s｜%u Hz｜有效幀 %u｜壞校驗 %u｜超出範圍 %u｜丟棄位元組 %u｜中斷 %u 次｜最大間隔 %u ms\n", ibusLinkOk ? "正常" : "無訊號", (unsigned)rateIbusHz,
     (unsigned)ibusFrames, (unsigned)ibusBadSum, (unsigned)ibusBadRange, (unsigned)ibusDropBytes, (unsigned)ibusFailsafe, (unsigned)ibusMaxGapMs);
  for (int i = 0; i < 14; i++) {
    const char* fn = "";
    for (int f = 0; f < FN_COUNT; f++) if (chMap[f] == i) { fn = FN_LABEL[f]; break; }
    rp(r, " CH%-2d %-8s %4u  (%u~%u)\n", i + 1, fn, (unsigned)s[i], (unsigned)(chMin[i] == 0xFFFF ? 0 : chMin[i]), (unsigned)chMax[i]);
  }
  rp(r, "\n[燈光] 大燈模式 %d｜亮度", (int)headMode);
  for (int i = 0; i < LT_COUNT; i++) rp(r, " %u", (unsigned)lightOut[i]);
  r += '\n';
  rp(r, "[腳位] iBUS RX%d TX%d｜I2S BCLK%d WS%d DIN%d｜IR 收%d 發%d｜燈 大%d 尾%d 槍%d 炮%d 擊%d 輔%d\n", IBUS_RX_PIN, IBUS_TX_PIN, I2S_BCLK_PIN, I2S_WS_PIN, I2S_DATA_PIN,
     IR_RECEIVE_PIN, IR_SEND_PIN, LED_HEADLIGHT_PIN, LED_TAIL_PIN, LED_MG_PIN, LED_FIRE_PIN, LED_HIT_PIN, LED_AUX_PIN);
  rp(r, "\n[音效] RAM 快取 %u / %u KB\n", (unsigned)(ramCacheBytes / 1024), (unsigned)(RAM_CACHE_BUDGET / 1024));
  for (int i = 0; i < S_COUNT; i++) {
    const SoundMeta& m = soundMeta[i];
    if (m.ok) rp(r, " %-7s OK   %5.1f KB %4.1fs %-5s 播放%u 開檔失敗%u 讀取錯誤%u\n", SOUND_LIST[i].name, m.size / 1024.0, (double)m.dataLen / 2 / AUDIO_RATE,
                 m.ram ? "RAM" : "flash", (unsigned)wavPlays[i], (unsigned)wavOpenFail[i], (unsigned)wavIoErr[i]);
    else rp(r, " %-7s 異常 %s (格式 fmt=%u %uch %uHz %ubit)\n", SOUND_LIST[i].name, reasonTxt(m.reason),
            (unsigned)m.fmt, (unsigned)m.chans, (unsigned)m.rate, (unsigned)m.bits);
  }
  rp(r, "\n[IR 互通] 發射序列 %u 款 共 %u ms｜脈衝補償 %d us｜發射 %u 次 (請求 %u、略過 %u)｜開炮掣 %u 次、被擋 %u 次\n", (unsigned)plan.n, (unsigned)plan.totalMs,
     (int)irMarkExcessUs, (unsigned)irTxCount, (unsigned)irTxRequests, (unsigned)irTxSkipped, (unsigned)statFireEdges, (unsigned)statFireBlocked);
  rp(r, " 接收：見 %u 幀 / 命中樣板 %u 幀 / 被擊中 %u 次｜環境邊緣 %u /秒 (共 %u)｜ISR 丟失 %u\n", (unsigned)irFramesSeen, (unsigned)irFramesMatched, (unsigned)irRxHits,
     (unsigned)rateIrEdges, (unsigned)irEdges, (unsigned)irRingDrops);
  for (int i = 0; i < IR_PROTO_COUNT; i++) {
    const IRProfile& p = irProfiles[i];
    int dp = irDupOf(i);
    rp(r, " %-12s %s len=%d 重複%d 週期%uus 幀%u 命中%u 發射%u%s\n", IR_PROTO_NAME[i], p.valid ? (p.builtin ? "內置" : "學習") : "未設定", (int)p.len, (int)p.repeats,
       (unsigned)p.periodUs, (unsigned)irFramesByProto[i], (unsigned)irHitsByProto[i], (unsigned)irTxByProto[i], dp >= 0 ? " (波形同前面相同，只發一次)" : "");
  }
  uint16_t d[IR_FRAME_MAX]; uint8_t n = 0; uint32_t t = 0; int8_t pr = -1;
  if (irUnk.get(d, n, t, pr) && n > 0) {
    rp(r, " 最後未識別訊號 (%lu ms 前, %d 元素)：", (unsigned long)(now - t), (int)n);
    for (int i = 0; i < n; i++) { rp(r, "%u%s", (unsigned)d[i], i + 1 < n ? "," : ""); }
    r += '\n';
  }
  rp(r, "\n[警示] 位元遮罩 0x%04lx\n", (unsigned long)computeAlerts());
  rp(r, "\n[最近記錄]\n");
  uint32_t cur = logSeq, first = (cur > 20) ? cur - 19 : 1;
  for (uint32_t q = first; q <= cur && q >= first; q++) {
    LogEntry e; char clk[16];
    if (!logGet(q, e)) continue;
    fmtClock(clk, sizeof(clk), e.ms);
    rp(r, " %s %s\n", clk, e.msg);
  }
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain; charset=utf-8", r);
}

// ===================== 網頁：動作 =====================
void handleRoot() { server.send_P(200, "text/html; charset=utf-8", PAGE_HTML); }

void handleSetVolume() {
  if (!server.hasArg("val")) { server.send(400, "text/plain", "Bad"); return; }
  long v = server.arg("val").toInt();
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  globalVolume = (float)v / 100.0f;
  markSettingsDirty();
  server.send(200, "text/plain", "OK");
}
void handleIgnition() { ignitionCommand = true; server.send(200, "text/plain", "OK"); }
void handleFireTest() { webFireRequest = true; server.send(200, "text/plain", "OK"); }
void handleFavicon() { server.send(204, "text/plain", ""); }
void handleSimHit() { hitCommand = true; hitFlashUntil = millis() + LIGHT_HIT_MS; damageUntil = millis() + LIGHT_DAMAGE_MS; evLog("模擬被擊中 (網頁)"); server.send(200, "text/plain", "OK"); }

void handlePlay() {
  int idx = soundIndexByName(server.arg("s"));
  if (idx < 0) { server.send(400, "text/plain", "Bad"); return; }
  playReq = (int8_t)idx;
  server.send(200, "text/plain", "OK");
}

void handleMap() {
  if (!server.hasArg("fn") || !server.hasArg("ch")) { server.send(400, "text/plain", "Bad"); return; }
  long f = server.arg("fn").toInt(), c = server.arg("ch").toInt();
  if (f < 0 || f >= FN_COUNT || c < -1 || c > 13) { server.send(400, "text/plain", "Bad"); return; }
  chMap[f] = (int8_t)c;
  markSettingsDirty();
  if (c < 0) evLog("頻道對應：%s → 停用", FN_LABEL[f]);
  else evLog("頻道對應：%s → CH%ld", FN_LABEL[f], c + 1);
  server.send(200, "text/plain", "OK");
}

void handleMapReset() {
  for (int i = 0; i < FN_COUNT; i++) chMap[i] = CH_MAP_DEFAULT[i];
  markSettingsDirty();
  evLog("頻道對應已還原預設");
  server.send(200, "text/plain", "OK");
}

void handleRcReset() {
  for (int i = 0; i < 14; i++) { chMin[i] = 0xFFFF; chMax[i] = 0; }
  server.send(200, "text/plain", "OK");
}

void handleStatsReset() { resetStats(); evLog("統計已重設"); server.send(200, "text/plain", "OK"); }

void handleReboot() {
  server.send(200, "text/plain", "OK");
  evLog("遠端重啟");
  delay(300);
  ESP.restart();
}

void handleIrTune() {
  long v = server.arg("val").toInt();
  if (v < 0) v = 0;
  if (v > 200) v = 200;
  irMarkExcessUs = (int)v;
  markSettingsDirty();
  server.send(200, "text/plain", "OK");
}
void handleIrLearn() {
  int v = server.hasArg("slot") ? (int)server.arg("slot").toInt() : -1;
  if (v < 0 || v >= IR_PROTO_COUNT) { server.send(400, "text/plain", "Bad"); return; }
  irLearnReq = (int8_t)v;
  server.send(200, "text/plain", "OK");
}
void handleIrReset() {
  int s = server.hasArg("slot") ? (int)server.arg("slot").toInt() : -1;
  if (s < 0 || s >= IR_PROTO_COUNT) { server.send(400, "text/plain", "Bad"); return; }
  irResetReq = (int8_t)s;
  server.send(200, "text/plain", "OK");
}

void handleLightMax() {                               // /lightmax?i=0..5&v=0..100
  if (!server.hasArg("i") || !server.hasArg("v")) { server.send(400, "text/plain", "Bad"); return; }
  long i = server.arg("i").toInt(), v = server.arg("v").toInt();
  if (i < 0 || i >= LT_COUNT) { server.send(400, "text/plain", "Bad"); return; }
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  lightPct[i] = (uint8_t)v;
  markSettingsDirty();
  server.send(200, "text/plain", "OK");
}
void handleLightTest() {
  long i = server.hasArg("i") ? server.arg("i").toInt() : -1;
  if (i < 0 || i >= LT_COUNT) { server.send(400, "text/plain", "Bad"); return; }
  lightTestReq = (int8_t)i;                           // 由音訊任務執行
  server.send(200, "text/plain", "OK");
}
void handleHeadMode() {
  long v = server.hasArg("v") ? server.arg("v").toInt() : -1;
  if (v < 0 || v > 3) { server.send(400, "text/plain", "Bad"); return; }
  headMode = (int8_t)v;
  markSettingsDirty();
  server.send(200, "text/plain", "OK");
}

// ---- 上載 / 刪除 ----
bool pauseAudio(uint32_t ms) {
  audioPauseRequested = true;
  uint32_t t = millis();
  while (!audioPaused && millis() - t < ms) delay(1);
  return audioPaused;
}
void resumeAudio() {
  audioPauseRequested = false;
  uint32_t t = millis();
  while (audioPaused && millis() - t < 400) delay(1);
}

void handleFileUpload() {
  HTTPUpload& u = server.upload();
  if (u.status == UPLOAD_FILE_START) {
    uploadResult = "失敗"; uploadWriteOK = false; uploadTargetPath = ""; uploadPausedByMe = false; uploadSoundIdx = -1;
    String n = server.hasArg("file") ? server.arg("file") : "";
    uploadSoundIdx = soundIndexByName(n);
    if (uploadSoundIdx < 0) { uploadResult = "名稱無效"; return; }
    if (!pauseAudio(1500)) { uploadResult = "音訊無法暫停"; audioPauseRequested = false; return; }
    uploadPausedByMe = true;
    uploadTargetPath = SOUND_PATH[uploadSoundIdx];
    LittleFS.remove(UPLOAD_TEMP_PATH);
    size_t total = LittleFS.totalBytes(), used = LittleFS.usedBytes();
    size_t freeB = (total > used) ? total - used : 0;
    uploadBudget = (freeB > 16384) ? (uint32_t)(freeB - 16384) : 0;
    if (uploadBudget > MAX_SOUND_BYTES) uploadBudget = MAX_SOUND_BYTES;
    uploadBytes = 0;
    uploadFile = LittleFS.open(UPLOAD_TEMP_PATH, "w");
    if (!uploadFile) { uploadResult = "無法建檔"; return; }
    uploadWriteOK = true;
  } else if (u.status == UPLOAD_FILE_WRITE) {
    if (uploadWriteOK && uploadFile) {
      uploadBytes += u.currentSize;
      if (uploadBytes > uploadBudget) { uploadWriteOK = false; uploadResult = "檔案太大/空間不足"; }
      else if (uploadFile.write(u.buf, u.currentSize) != u.currentSize) { uploadWriteOK = false; uploadResult = "寫入失敗"; }
    }
  } else if (u.status == UPLOAD_FILE_END) {
    if (uploadFile) uploadFile.close();
    bool ok = false;
    int code = WR_OPEN_FAIL;
    WavInfo wi;
    if (uploadWriteOK) {
      File tf = LittleFS.open(UPLOAD_TEMP_PATH, "r");
      if (tf) { code = inspectWavCode(tf, wi); ok = (code == WR_OK); tf.close(); }
    }
    if (!uploadWriteOK || !ok) {
      LittleFS.remove(UPLOAD_TEMP_PATH);
      if (uploadWriteOK) {
        char b[96];
        if (code == WR_BAD_RATE || code == WR_NOT_MONO || code == WR_BAD_BITS || code == WR_NOT_PCM)
          snprintf(b, sizeof(b), "格式不符：%s (實際 %uHz / %uch / %ubit)", reasonTxt(code), (unsigned)wi.sampleRate, (unsigned)wi.channels, (unsigned)wi.bitsPerSample);
        else snprintf(b, sizeof(b), "格式不符：%s", reasonTxt(code));
        uploadResult = b;
      }
    } else {
      LittleFS.remove(uploadTargetPath.c_str());
      if (LittleFS.rename(UPLOAD_TEMP_PATH, uploadTargetPath.c_str())) uploadResult = "上載成功";
      else { uploadResult = "替換失敗"; LittleFS.remove(UPLOAD_TEMP_PATH); }
    }
    uploadWriteOK = false;
    if (uploadPausedByMe) { refreshSoundInfo(); resumeAudio(); }
    uploadPausedByMe = false;
    if (uploadSoundIdx >= 0) evLog("上載 %s：%s", SOUND_LIST[uploadSoundIdx].name, uploadResult.c_str());
  } else if (u.status == UPLOAD_FILE_ABORTED) {
    if (uploadFile) uploadFile.close();
    LittleFS.remove(UPLOAD_TEMP_PATH);
    uploadWriteOK = false; uploadResult = "已中止";
    if (uploadPausedByMe) { refreshSoundInfo(); resumeAudio(); }
    uploadPausedByMe = false;
    evLog("上載已中止");
  }
}

void handleUploadFinished() {
  bool ok = uploadResult == "上載成功";
  String h = "<html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'></head>"
             "<body style='background:#111;color:#fff;text-align:center;padding:40px;font-family:sans-serif'><h2>";
  h += ok ? "✅ " : "⚠️ ";
  h += uploadResult + "</h2><a href=/ style=color:#4fc3f7>返回</a></body></html>";
  server.send(ok ? 200 : 400, "text/html; charset=utf-8", h);
}

void handleDelete() {
  int idx = soundIndexByName(server.arg("file"));
  if (idx < 0) { server.send(400, "text/plain", "Bad"); return; }
  if (!pauseAudio(1500)) { audioPauseRequested = false; server.send(503, "text/plain", "Busy"); return; }
  LittleFS.remove(SOUND_PATH[idx]);
  refreshSoundInfo();
  resumeAudio();
  evLog("刪除音效 %s", SOUND_LIST[idx].name);
  server.send(200, "text/plain", "OK");
}

// ===================== Setup / Loop =====================
void setupI2S() {
  i2s_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = AUDIO_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = I2S_DMA_COUNT;
  cfg.dma_buf_len = I2S_DMA_LEN;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;
  i2s_pin_config_t pins;
  memset(&pins, 0, sizeof(pins));
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = I2S_BCLK_PIN;
  pins.ws_io_num = I2S_WS_PIN;
  pins.data_out_num = I2S_DATA_PIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;
  if (i2s_driver_install(I2S_PORT, &cfg, 0, NULL) != ESP_OK) Serial.println("[ERR] i2s install");
  if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK) Serial.println("[ERR] i2s pins");
  i2s_zero_dma_buffer(I2S_PORT);
}

static void regGet(const char* path, void (*fn)()) {
  server.on(path, HTTP_GET, [fn]() { INC(webRequests); fn(); });
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Tank Sound + IR 1/16 " FW_VERSION " ===");

  bootResetReason = (int)esp_reset_reason();
  for (int i = 0; i < 14; i++) { chMin[i] = 0xFFFF; chMax[i] = 0; }
  evLog("開機 %s｜重啟原因：%s", FW_VERSION, resetReasonName(bootResetReason));

  loadSettings();
  lightsInit();
  if (!LittleFS.begin(true, "/littlefs", 12)) Serial.println("[ERR] LittleFS"); else Serial.println("[OK] LittleFS");
  LittleFS.remove(UPLOAD_TEMP_PATH);
  irInit();

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 1, 0, 4)) Serial.println("[ERR] softAP");
  WiFi.setSleep(false);
  Serial.print("[AP] "); Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  regGet("/favicon.ico", handleFavicon);
  regGet("/api/live", handleApiLive);
  regGet("/api/rc", handleApiRc);
  regGet("/api/wav", handleApiWav);
  regGet("/api/ir", handleApiIr);
  regGet("/api/sys", handleApiSys);
  regGet("/api/log", handleApiLog);
  regGet("/api/cfg", handleApiCfg);
  regGet("/report", handleReport);
  regGet("/setVolume", handleSetVolume);
  regGet("/ignition", handleIgnition);
  regGet("/fire", handleFireTest);
  regGet("/simhit", handleSimHit);
  regGet("/play", handlePlay);
  regGet("/map", handleMap);
  regGet("/mapreset", handleMapReset);
  regGet("/rcreset", handleRcReset);
  regGet("/statsreset", handleStatsReset);
  regGet("/reboot", handleReboot);
  regGet("/irtune", handleIrTune);
  regGet("/irlearn", handleIrLearn);
  regGet("/irreset", handleIrReset);
  regGet("/lightmax", handleLightMax);
  regGet("/lighttest", handleLightTest);
  regGet("/headmode", handleHeadMode);
  regGet("/del", handleDelete);
  server.on("/upload", HTTP_POST, handleUploadFinished, handleFileUpload);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  refreshSoundInfo();
  Serial.printf("[OK] sounds cached in RAM: %u bytes, free heap: %u\n", (unsigned)ramCacheBytes, (unsigned)ESP.getFreeHeap());
  evLog("音效 RAM 快取 %u KB｜可用記憶體 %u KB", (unsigned)(ramCacheBytes / 1024), (unsigned)(ESP.getFreeHeap() / 1024));
  for (int i = 0; i < S_COUNT; i++) if (!soundMeta[i].ok) evLog("音效 %s：%s", SOUND_LIST[i].name, reasonTxt(soundMeta[i].reason));
  setupI2S();
  xTaskCreatePinnedToCore(irTxTask, "IRTx", 3072, NULL, 4, &irTxTaskHandle, IR_TX_CORE);
  xTaskCreatePinnedToCore(audioAndControlTask, "AudioCtl", 10240, NULL, 3, NULL, 1);
  audioTaskStarted = true;
  Serial.println("[OK] Setup done");
}

void irPrintUnknownIfNew() {
  static uint32_t lastPrintedMs = 0, lastPrintAt = 0;
  if (!IR_SERIAL_DUMP) return;
  uint16_t d[IR_FRAME_MAX];
  uint8_t len = 0;
  uint32_t ms = 0;
  int8_t pr = -1;
  if (!irUnk.get(d, len, ms, pr) || len == 0 || ms == lastPrintedMs) return;
  if (millis() - lastPrintAt < 1500) return;
  lastPrintedMs = ms; lastPrintAt = millis();
  Serial.printf("[IR] unknown frame len=%u: ", (unsigned)len);
  for (int i = 0; i < len; i++) Serial.printf("%u%s", (unsigned)d[i], (i + 1 < len) ? "," : "\n");
}

void sampleRates() {
  static uint32_t last = 0, pFrames = 0, pEdges = 0, pReads = 0, pBytes = 0, pErr = 0;
  uint32_t now = millis();
  if (now - last < 1000) return;
  last = now;
  uint32_t f = ibusFrames, e = irEdges, r = statFlashReads, b = statFlashBytes, er = ibusBadSum + ibusBadRange;
  rateIbusHz = f - pFrames; pFrames = f;
  rateIrEdges = e - pEdges; pEdges = e;
  rateFlashReads = r - pReads; pReads = r;
  rateFlashBytes = b - pBytes; pBytes = b;
  rateIbusErr = er - pErr; pErr = er;
  audioMaxRecentUs = audioWinMaxUs; audioWinMaxUs = 0;
  loopStackFree = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
  uint32_t heap = (uint32_t)ESP.getFreeHeap();
  if (heap < LOW_HEAP_BYTES && !lowHeapLogged) { lowHeapLogged = true; evLog("可用記憶體偏低：%u KB", (unsigned)(heap / 1024)); }
  else if (heap > LOW_HEAP_BYTES + 8000) lowHeapLogged = false;
  if (BATT_ADC_PIN >= 0) {
    uint32_t acc = 0;
    for (int i = 0; i < 8; i++) acc += analogReadMilliVolts(BATT_ADC_PIN);
    battMv = (uint32_t)((acc / 8.0f) * BATT_DIV_RATIO);
  }
}

void serviceHousekeeping() {
  if (irSavePending >= 0) {
    int s = irSavePending; irSavePending = -1;
    bool paused = pauseAudio(1000);
    irSaveProfile(s);
    if (paused) resumeAudio(); else audioPauseRequested = false;
    irDumpProfile(s);
  }
  if (settingsDirty) {
    uint32_t age = millis() - settingsDirtyMs;
    if (age > SETTINGS_DEBOUNCE_MS && (engineState == ENGINE_OFF || age > SETTINGS_MAX_WAIT_MS)) {
      settingsDirty = false;
      bool paused = pauseAudio(1000);
      saveSettings();
      if (paused) resumeAudio(); else audioPauseRequested = false;
    }
  }
  irPrintUnknownIfNew();
  sampleRates();
}

void loop() {
  server.handleClient();
  serviceHousekeeping();
  delay(2);
}
