// ESP32 Karaoke + Autocorrelation pitch detection (improved)
// + DFPlayer CLONE (flat root files, uses play(trackNum))


#include <IRremote.hpp>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "driver/i2s.h"
#include <math.h>
#include "DFRobotDFPlayerMini.h"


// ===== RGB LED PINS =====
#define PIN_R 14
#define PIN_G 19
#define PIN_B 16


// ===== DFPLAYER (clone) =====
HardwareSerial mp3(2); // UART2
DFRobotDFPlayerMini dfplayer;

//void setColor(int r, int g, int b) {
//  analogWrite(PIN_R, r);
//  analogWrite(PIN_G, g);
//  analogWrite(PIN_B, b);
//}


// =================== USER CONFIG ===================
const char* ssid     = "pinakacute sa komsai";
const char* password = "ninaninanina";
const String baseURL = "https://delrosario-nina.github.io/jocSING/";


LiquidCrystal_I2C lcd(0x27, 16, 2);


// IR pin
#define IR_RECEIVE_PIN 32


// I2S (INMP441)
#define I2S_WS  25
#define I2S_SCK 26
#define I2S_SD  27


#define SAMPLE_RATE       16000
#define I2S_BLOCK_SAMPLES 2048
#define MAX_LRC_LINES     300
#define MAX_PITCH_POINTS  800


const float FREQ_TOLERANCE_RATIO = 0.03f;
const unsigned long DETECT_INTERVAL_MS = 120;
const float RMS_THRESHOLD = 0.004f;
const int SMOOTH_N = 3;


// =================== STRUCTS ===================
struct LyricLine { float time; String text; };
struct PitchPoint { float time; float freq; };


static LyricLine  lyrics[MAX_LRC_LINES];
static PitchPoint pitchPoints[MAX_PITCH_POINTS];
static int  lyricCount  = 0;
static int  pitchCount  = 0;


static int32_t i2s_read_buffer[I2S_BLOCK_SAMPLES];
static float   floatBuf[I2S_BLOCK_SAMPLES];
static float   smoothBuf[SMOOTH_N];
static int     smoothIdx = 0;


String inputCode = "";
bool   playing   = false;
unsigned long songStartMs = 0;


// =========================================================
// HTTP
// =========================================================
String httpGetString(const String &url) {
  HTTPClient http;
  http.begin(url);
  int code = http.GET();
  String payload = "";
  if (code == 200) payload = http.getString();
  Serial.printf("[HTTP] GET %s → %d\n", url.c_str(), code);
  http.end();
  return payload;
}


// =========================================================
// PARSERS
// =========================================================
void parseLRC(const String &data) {
  lyricCount = 0;
  int idx = 0;


  while (idx < data.length() && lyricCount < MAX_LRC_LINES) {
    int nl = data.indexOf('\n', idx);
    if (nl == -1) nl = data.length();


    String line = data.substring(idx, nl);
    line.trim();


    if (line.startsWith("[")) {
      int br = line.indexOf(']');
      int colon = line.indexOf(':');


      if (br > 0 && colon > 0) {
        int   mm = line.substring(1, colon).toInt();
        float ss = line.substring(colon + 1, br).toFloat();


        lyrics[lyricCount].time = mm * 60 + ss;
        lyrics[lyricCount].text = line.substring(br + 1);
        lyricCount++;
      }
    }
    idx = nl + 1;
  }
}


void parsePitchFile(const String &data) {
  pitchCount = 0;
  int idx = 0;


  while (idx < data.length() && pitchCount < MAX_PITCH_POINTS) {
    int nl = data.indexOf('\n', idx);
    if (nl == -1) nl = data.length();


    String line = data.substring(idx, nl);
    line.trim();


    if (line.startsWith("[")) {
      int br = line.indexOf(']');
      int colon = line.indexOf(':');


      if (br > 0 && colon > 0) {
        int   mm = line.substring(1, colon).toInt();
        float ss = line.substring(colon + 1, br).toFloat();
        float f  = line.substring(br + 1).toFloat();


        pitchPoints[pitchCount].time = mm * 60 + ss;
        pitchPoints[pitchCount].freq = f;
        pitchCount++;
      }
    }
    idx = nl + 1;
  }
}


float expectedPitchAt(float t) {
  if (pitchCount == 0) return 0;
  if (t <= pitchPoints[0].time) return pitchPoints[0].freq;
  if (t >= pitchPoints[pitchCount - 1].time) return pitchPoints[pitchCount - 1].freq;


  for (int i = 0; i < pitchCount - 1; i++) {
    if (t >= pitchPoints[i].time && t < pitchPoints[i + 1].time) {
      float dt   = pitchPoints[i + 1].time - pitchPoints[i].time;
      float frac = (t - pitchPoints[i].time) / dt;
      return pitchPoints[i].freq +
             frac * (pitchPoints[i + 1].freq - pitchPoints[i].freq);
    }
  }
  return 0;
}


// =========================================================
// I2S MIC SETUP
// =========================================================
void setupI2S() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };


  i2s_pin_config_t pins = {
    .bck_io_num   = I2S_SCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = -1,
    .data_in_num  = I2S_SD
  };


  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
}


// =========================================================
// DSP HELPERS
// =========================================================
float computeRMS(float *x, int n) {
  double s = 0;
  for (int i = 0; i < n; i++) s += x[i] * x[i];
  return sqrt(s / n);
}


void applyHann(float *x, int n) {
  for (int i = 0; i < n; i++) {
    float w = 0.5f * (1 - cosf(2 * M_PI * i / (n - 1)));
    x[i] *= w;
  }
}


float detectPitchAutocorr(float *x, int n, float sampleRate) {
  if (n < 64) return -1;


  double mean = 0;
  for (int i = 0; i < n; i++) mean += x[i];
  mean /= n;
  for (int i = 0; i < n; i++) x[i] -= mean;


  float rms = computeRMS(x, n);
  if (rms < RMS_THRESHOLD) return -1;


  applyHann(x, n);


  int maxLag = min(n / 2, (int)(sampleRate / 50.0f));
  int minLag = max(2, (int)(sampleRate / 2000.0f));


  double E0 = 0;
  for (int i = 0; i < n; i++) E0 += x[i] * x[i];


  float best = 0;
  int bestLag = -1;


  for (int lag = minLag; lag <= maxLag; lag++) {
    double num = 0, Ek = 0;


    for (int i = 0; i < n - lag; i++) {
      num += x[i] * x[i + lag];
      Ek  += x[i + lag] * x[i + lag];
    }


    if (Ek <= 1e-12 || E0 <= 1e-12) continue;


    double val = num / sqrt(E0 * Ek);
    if (val > best) {
      best = val;
      bestLag = lag;
    }
  }


  if (bestLag < 1) return -1;
  return sampleRate / bestLag;
}


float smoothPitchValue(float v) {
  smoothBuf[smoothIdx] = v;
  smoothIdx = (smoothIdx + 1) % SMOOTH_N;


  float t[SMOOTH_N];
  int c = 0;


  for (int i = 0; i < SMOOTH_N; i++)
    if (smoothBuf[i] > 0) t[c++] = smoothBuf[i];


  if (c == 0) return -1;


  for (int i = 1; i < c; i++) {
    float key = t[i];
    int j = i - 1;
    while (j >= 0 && t[j] > key) {
      t[j + 1] = t[j];
      j--;
    }
    t[j + 1] = key;
  }


  return (c % 2 ? t[c / 2] : (t[c / 2] + t[c / 2 - 1]) * 0.5);
}


// =========================================================
// LYRIC DISPLAY
// =========================================================
void showLyric(const String &text) {
  String top = text;
  String bot = "";


  if (top.length() > 16) {
    bot = top.substring(16, min((size_t)32, top.length()));
    top = top.substring(0, 16);
  }


  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(top);
  lcd.setCursor(0, 1);
  lcd.print(bot);
}


// =========================================================
// FILE FETCHING
// =========================================================
bool fetchFilesForCode(const String &code) {
  String lrcUrl   = baseURL + code + ".lrc";
  String pitchUrl = baseURL + code + "_pitch.txt";


  String lrc = httpGetString(lrcUrl);
  if (lrc.length() == 0) return false;
  parseLRC(lrc);


  String p = httpGetString(pitchUrl);
  if (p.length() == 0) pitchCount = 0;
  else parsePitchFile(p);


  return true;
}


// =========================================================
// PLAYBACK LOOP (FIXED VERSION)
// =========================================================
void playbackLoop(int trackNum) {
  if (lyricCount == 0) {
    lcd.clear();
    lcd.print("No song found!");
    delay(900);
    return;
  }


  // Start DFPlayer *here*
  dfplayer.stop();
  dfplayer.volume(20);   // safe audible volume
  dfplayer.play(trackNum);


  Serial.println("DFPlayer playing track inside loop...");


  delay(50);  // let DFPlayer begin output so timing matches 0-sec lyrics


  setupI2S();


  playing = true;
  songStartMs = millis();


  int currentIdx = 0;
  unsigned long lastDetect = 0;


  for (int i = 0; i < SMOOTH_N; i++)
    smoothBuf[i] = -1;


  lcd.clear();


  Serial.println("Playback loop started.");


  while (playing) {
    float t = (millis() - songStartMs) / 1000.0f;


    // LYRIC UPDATE
    if (currentIdx + 1 < lyricCount &&
        t >= lyrics[currentIdx + 1].time) {
      currentIdx++;
    }


    showLyric(lyrics[currentIdx].text);


    // PITCH DETECTION
    if (millis() - lastDetect >= DETECT_INTERVAL_MS) {
      lastDetect = millis();


      size_t bytesRead = 0;
      esp_err_t res = i2s_read(
        I2S_NUM_0,
        (void*)i2s_read_buffer,
        I2S_BLOCK_SAMPLES * sizeof(int32_t),
        &bytesRead,
        50 / portTICK_PERIOD_MS
      );


      int got = bytesRead / sizeof(int32_t);


      if (res == ESP_OK && got > 0) {
        int n = got;


        for (int i = 0; i < n; i++)
          floatBuf[i] = float(i2s_read_buffer[i]) / 2147483648.0f;


        float detected = detectPitchAutocorr(floatBuf, n, SAMPLE_RATE);
        float smoothed = detected > 0 ? smoothPitchValue(detected) : -1;
        float target   = expectedPitchAt(t);


        if (target > 0 && smoothed > 0) {
          float ratio = smoothed / target;


        //  if (fabs(ratio - 1.0f) <= FREQ_TOLERANCE_RATIO)       setColor(0,255,0);  // good
        //  else if (ratio < 1.0f)                                setColor(0,0,255);  // low
        //  else                                                  setColor(255,0,0);  // high
        }
      }
    }


    // END CONDITION (FIXED)
    if (currentIdx >= lyricCount - 1 &&
        t > lyrics[lyricCount - 1].time + 5.0f) {   // more generous end timing
      Serial.println("END of lyrics reached.");
      break;
    }


    // STOP FROM REMOTE
    if (IrReceiver.decode()) {
      String k = decodeKey(IrReceiver.decodedIRData.decodedRawData);
      IrReceiver.resume();


      if (k == "-") {
        Serial.println("STOP from IR.");
        playing = false;
        dfplayer.stop();
      }
    }
  }


  i2s_driver_uninstall(I2S_NUM_0);


  dfplayer.stop();
  playing = false;


  lcd.clear();
  lcd.print("Song finished!");
  delay(1200);
  lcd.clear();
  lcd.print("Enter song code:");
}


// =========================================================
// SETUP
// =========================================================
void setup() {
  Serial.begin(115200);
  delay(300);


  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Connecting WiFi");


  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);


  WiFi.begin(ssid, password);
  int tries = 0;


  while (WiFi.status() != WL_CONNECTED && tries < 25) {
    delay(300);
    Serial.print(".");
    tries++;
  }


  Serial.println();
  Serial.println(WiFi.status() == WL_CONNECTED ? "WiFi OK" : "WiFi FAIL");


  // ===== DFPLAYER =====
  Serial.println("== DFPLAYER INIT ==");
  mp3.begin(9600, SERIAL_8N1, 17, 5);
  delay(500);


  if (!dfplayer.begin(mp3)) {
    Serial.println("DFPlayer FAIL!");
  } else {
    Serial.println("DFPlayer OK.");
    dfplayer.volume(18);
    dfplayer.EQ(DFPLAYER_EQ_NORMAL);
  }

  // Disable RGB pins to avoid UART / noise interference
pinMode(PIN_R, OUTPUT);
pinMode(PIN_G, OUTPUT);
pinMode(PIN_B, OUTPUT);

digitalWrite(PIN_R, LOW);
digitalWrite(PIN_G, LOW);
digitalWrite(PIN_B, LOW);



  lcd.clear();
  lcd.print("Enter song code:");
}


// =========================================================
// LOOP
// =========================================================
void loop() {
  if (IrReceiver.decode()) {
    String key = decodeKey(IrReceiver.decodedIRData.decodedRawData);
    IrReceiver.resume();


    Serial.print("Key: ");
    Serial.println(key);


    // NUMBER INPUT
    if (key >= "0" && key <= "9") {
      inputCode += key;


      // last 5 bits only
      if (inputCode.length() > 5)
        inputCode = inputCode.substring(inputCode.length() - 5);


      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Code (bin):");
      lcd.setCursor(0,1);
      lcd.print(inputCode);
    }


    // CLEAR
    else if (key == "-") {
      inputCode = "";
      lcd.clear();
      lcd.print("Cleared!");
      delay(500);
      lcd.clear();
      lcd.print("Enter song code:");
    }


    else if (key == "EQ") {
      if (inputCode.length() == 0) {
        lcd.clear();
        lcd.print("No song code");
        delay(500);
        lcd.clear();
        lcd.print("Enter song code:");
      } else {
        lcd.clear();
        lcd.print("Loading...");
        Serial.println("Fetching for code: " + inputCode);


        if (fetchFilesForCode(inputCode)) {
          int trackNum = strtol(inputCode.c_str(), NULL, 2);
          if (trackNum <= 0) trackNum = 1;


          Serial.print("Playing track: ");
          Serial.println(trackNum);


          playbackLoop(trackNum);
        } else {
          lcd.clear(); lcd.print("No song found!");
          delay(900);
        }


        inputCode = "";
      }
    }
  }
}


// =========================================================
// IR MAP
// =========================================================
String decodeKey(unsigned long value) {
  switch (value) {
    case 0xF807FF00: return "-";
    case 0xF609FF00: return "EQ";
    case 0xE916FF00: return "0";
    case 0xF30CFF00: return "1";
    case 0xE718FF00: return "2";
    case 0xA15EFF00: return "3";
    case 0xF708FF00: return "4";
    case 0xE31CFF00: return "5";
    case 0xA55AFF00: return "6";
    case 0xBD42FF00: return "7";
    case 0xAD52FF00: return "8";
    case 0xB54AFF00: return "9";
    default:         return "UNKNOWN";
  }
}
