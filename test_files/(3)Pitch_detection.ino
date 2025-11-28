// ESP32 Karaoke + Autocorrelation pitch detection (improved)
// - IR remote for song code input
// - Fetch .lrc and <code>_pitch.txt from GitHub Pages (baseURL)
// - INMP441 (I2S) input
// - Hann-windowed normalized autocorrelation with RMS gating
// - Median smoothing (3 samples) for stability
// - Compare live pitch to expected pitch, print verdict to Serial
//
// Make sure your pitch file naming is EXACT: <code>_pitch.txt
// e.g. https://delrosario-nina.github.io/jocSING/11111_pitch.txt
//
// Uses LiquidCrystal_I2C library and IRremote library


#include <IRremote.hpp>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "driver/i2s.h"
#include <math.h>


// ===== RGB LED PINS (using analogWrite, NOT ledcSetup) =====
#define PIN_R 14
#define PIN_G 19
#define PIN_B 16


void setColor(int r, int g, int b) {
  analogWrite(PIN_R, r);
  analogWrite(PIN_G, g);
  analogWrite(PIN_B, b);
}




// =================== USER CONFIG ===================
const char* ssid = "pinakacute sa komsai";  
const char* password = "ninaninanina";      


const String baseURL = "https://delrosario-nina.github.io/jocSING/"; // base for LRC and pitch


// I2C LCD address (0x27 common). Change if your module uses 0x3F
LiquidCrystal_I2C lcd(0x27, 16, 2);


// IR pin
#define IR_RECEIVE_PIN 32


// I2S (INMP441) pins (as you've used)
#define I2S_WS  25   // LRCL/WS
#define I2S_SCK 26   // BCLK
#define I2S_SD  27   // DOUT


// I2S & pitch detection parameters
#define SAMPLE_RATE       16000      // 16 kHz is adequate and lighter on CPU than 44.1k
#define I2S_BLOCK_SAMPLES 2048       // samples per reading (128 ms @ 16 kHz)
#define MAX_LRC_LINES     300
#define MAX_PITCH_POINTS  800


// pitch correctness tolerance (relative)
const float FREQ_TOLERANCE_RATIO = 0.03f; // ±3%


// detection interval (ms) — how often we sample & check pitch
const unsigned long DETECT_INTERVAL_MS = 120; // ~8Hz checks


// RMS threshold: below this we treat as silence
const float RMS_THRESHOLD = 0.004f; // tuneable


// smoothing window size for detected pitch
const int SMOOTH_N = 3;


// =================== DATA STRUCTS ===================
struct LyricLine {
  float time; // seconds
  String text;
};
static LyricLine lyrics[MAX_LRC_LINES];
static int lyricCount = 0;


struct PitchPoint {
  float time; // seconds
  float freq; // Hz (0 => rest)
};
static PitchPoint pitchPoints[MAX_PITCH_POINTS];
static int pitchCount = 0;


// I2S read buffer (32-bit read)
static int32_t i2s_read_buffer[I2S_BLOCK_SAMPLES];
// float buffer for autocorr
static float floatBuf[I2S_BLOCK_SAMPLES];
// circular smoothing buffer
static float smoothBuf[SMOOTH_N];
static int smoothIdx = 0;


// playback state
String inputCode = "";
bool playing = false;
unsigned long songStartMs = 0;


// =================== UTILITY: HTTP GET ===================
String httpGetString(const String &url) {
  HTTPClient http;
  http.begin(url);
  int code = http.GET();
  String payload = "";
  if (code == 200) {
    payload = http.getString();
    Serial.printf("[HTTP] GET %s -> 200 OK\n", url.c_str());
  } else {
    Serial.printf("[HTTP] GET %s -> %d\n", url.c_str(), code);
  }
  http.end();
  return payload;
}


// =================== PARSERS ===================
void parseLRC(const String &data) {
  lyricCount = 0;
  int idx = 0;
  while (idx < (int)data.length() && lyricCount < MAX_LRC_LINES) {
    int nl = data.indexOf('\n', idx);
    if (nl == -1) nl = data.length();
    String line = data.substring(idx, nl);
    line.trim();
    if (line.length() > 0 && line.startsWith("[")) {
      int br = line.indexOf(']');
      int colon = line.indexOf(':');
      if (br > 0 && colon > 0) {
        int mm = line.substring(1, colon).toInt();
        float ss = line.substring(colon + 1, br).toFloat();
        String txt = line.substring(br + 1);
        lyrics[lyricCount].time = mm * 60 + ss;
        lyrics[lyricCount].text = txt;
        lyricCount++;
      }
    }
    idx = nl + 1;
  }
  Serial.printf("Parsed LRC: %d lines\n", lyricCount);
}


void parsePitchFile(const String &data) {
  pitchCount = 0;
  int idx = 0;
  while (idx < (int)data.length() && pitchCount < MAX_PITCH_POINTS) {
    int nl = data.indexOf('\n', idx);
    if (nl == -1) nl = data.length();
    String line = data.substring(idx, nl);
    line.trim();
    if (line.length() > 0 && line.startsWith("[")) {
      int br = line.indexOf(']');
      int colon = line.indexOf(':');
      if (br > 0 && colon > 0) {
        int mm = line.substring(1, colon).toInt();
        float ss = line.substring(colon + 1, br).toFloat();
        String fld = line.substring(br + 1);
        fld.trim();
        float f = fld.toFloat();
        pitchPoints[pitchCount].time = mm * 60 + ss;
        pitchPoints[pitchCount].freq = f;
        pitchCount++;
      }
    }
    idx = nl + 1;
  }
  Serial.printf("Parsed pitch file: %d points\n", pitchCount);
}


// linear interpolate expected pitch at time t
float expectedPitchAt(float t) {
  if (pitchCount == 0) return 0.0f;
  if (t <= pitchPoints[0].time) return pitchPoints[0].freq;
  if (t >= pitchPoints[pitchCount-1].time) return pitchPoints[pitchCount-1].freq;
  for (int i = 0; i < pitchCount - 1; ++i) {
    if (t >= pitchPoints[i].time && t < pitchPoints[i+1].time) {
      float dt = pitchPoints[i+1].time - pitchPoints[i].time;
      if (dt <= 1e-6f) return pitchPoints[i].freq;
      float frac = (t - pitchPoints[i].time) / dt;
      return pitchPoints[i].freq + frac * (pitchPoints[i+1].freq - pitchPoints[i].freq);
    }
  }
  return pitchPoints[pitchCount-1].freq;
}


// =================== I2S init/read ===================
void setupI2S() {
  i2s_config_t i2s_config = {
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


  setColor(0,0,0);


  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };


  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
  Serial.println("I2S driver installed");
}


// read specified samples into i2s_read_buffer, return samples read
int readI2SSamples(int32_t *buf, int samples) {
  size_t bytesRead = 0;
  esp_err_t res = i2s_read(I2S_NUM_0, (void*)buf, samples * sizeof(int32_t), &bytesRead, 500);
  if (res != ESP_OK) return 0;
  return (int)(bytesRead / sizeof(int32_t));
}


// =================== DSP helpers ===================
// Apply Hann window in-place (n samples)
void applyHann(float *x, int n) {
  for (int i = 0; i < n; ++i) {
    float w = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (n - 1)));
    x[i] *= w;
  }
}


// compute RMS of buffer
float computeRMS(float *x, int n) {
  double s = 0.0;
  for (int i = 0; i < n; ++i) s += (double)x[i] * (double)x[i];
  return sqrt((float)(s / n));
}


// normalized autocorrelation and best lag search
// returns estimated frequency (Hz) or -1 on failure
float detectPitchAutocorr(float *x, int n, float sampleRate) {
  if (n < 64) return -1.0f;


  // remove mean
  double mean = 0.0;
  for (int i = 0; i < n; ++i) mean += x[i];
  mean /= n;
  for (int i = 0; i < n; ++i) x[i] -= (float)mean;


  // RMS gate
  float rms = computeRMS(x, n);
  if (rms < RMS_THRESHOLD) return -1.0f;


  // window to reduce spectral leakage
  applyHann(x, n);


  // normalized autocorrelation: r[k] = sum(x[i]*x[i+k]) / (sqrt(E0*Ek))
  // search lags for pitch: limit to realistic musical range (50-2000 Hz)
  int maxLag = min(n / 2, (int)(sampleRate / 50.0f)); // low freq limit
  int minLag = max(2, (int)(sampleRate / 2000.0f));   // high freq limit
  float bestVal = 0.0f;
  int bestLag = -1;


  // Precompute energy for denominator optimization (energy of each lag window)
  // compute E0 = sum(x[i]^2)
  double E0 = 0.0;
  for (int i = 0; i < n; ++i) E0 += (double)x[i] * (double)x[i];


  for (int lag = minLag; lag <= maxLag; ++lag) {
    double num = 0.0;
    double Ek = 0.0;
    // compute numerator and Ek
    for (int i = 0; i < n - lag; ++i) {
      num += (double)x[i] * (double)x[i + lag];
      Ek += (double)x[i + lag] * (double)x[i + lag];
    }
    if (Ek <= 1e-12 || E0 <= 1e-12) continue;
    double denom = sqrt(E0 * Ek);
    double val = num / denom;
    if (val > bestVal) {
      bestVal = (float)val;
      bestLag = lag;
    }
  }


  if (bestLag <= 0) return -1.0f;


  // Parabolic interpolation around bestLag for sub-sample accuracy
  // compute correlation at bestLag-1, bestLag, bestLag+1
  auto corrAt = [&](int lag) -> double {
    double num = 0.0;
    double Ek = 0.0;
    for (int i = 0; i < n - lag; ++i) {
      num += (double)x[i] * (double)x[i + lag];
      Ek += (double)x[i + lag] * (double)x[i + lag];
    }
    if (Ek <= 1e-12 || E0 <= 1e-12) return 0.0;
    return num / sqrt(E0 * Ek);
  };


  double y1 = (bestLag - 1 >= minLag) ? corrAt(bestLag - 1) : 0.0;
  double y2 = corrAt(bestLag);
  double y3 = (bestLag + 1 <= maxLag) ? corrAt(bestLag + 1) : 0.0;


  double denom = (y1 - 2.0 * y2 + y3);
  double delta = 0.0;
  if (fabs(denom) > 1e-12) delta = 0.5 * (y1 - y3) / denom;
  double lagRefined = (double)bestLag + delta;


  double freq = sampleRate / lagRefined;
  if (freq < 50.0 || freq > 2000.0) return -1.0f;
  return (float)freq;
}


// median smoothing over small window (SMOOTH_N)
float smoothPitch(float v) {
  // put new value
  smoothBuf[smoothIdx] = v;
  smoothIdx = (smoothIdx + 1) % SMOOTH_N;
  // copy to temp and compute median ignoring negatives (invalid)
  float temp[SMOOTH_N];
  int c = 0;
  for (int i = 0; i < SMOOTH_N; ++i) {
    if (smoothBuf[i] > 0) {
      temp[c++] = smoothBuf[i];
    }
  }
  if (c == 0) return -1.0f;
  // simple insertion sort for small c
  for (int i = 1; i < c; ++i) {
    float key = temp[i];
    int j = i - 1;
    while (j >= 0 && temp[j] > key) { temp[j + 1] = temp[j]; j--; }
    temp[j + 1] = key;
  }
  // median
  if (c % 2 == 1) return temp[c / 2];
  else return (temp[c / 2 - 1] + temp[c / 2]) * 0.5f;
}


// =================== Play loop / fetch workflow ===================
void showLyric(const String &text) {
  String top = text;
  String bot = "";
  if (top.length() > 16) {
    bot = top.substring(16, min((size_t)32, top.length()));
    top = top.substring(0,16);
  } else {
    bot = "";
  }
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(top);
  lcd.setCursor(0,1);
  if (bot.length()) lcd.print(bot);
  else lcd.print("                ");
}


bool fetchFilesForCode(const String &code) {
  String lrcUrl = baseURL + code + ".lrc";
  String lrcData = httpGetString(lrcUrl);
  if (lrcData.length() == 0) {
    Serial.println("LRC not found on GitHub.");
    return false;
  }
  parseLRC(lrcData);


  String pitchUrl = baseURL + code + "_pitch.txt";
  String pitchData = httpGetString(pitchUrl);
  if (pitchData.length() == 0) {
    Serial.println("Pitch not found on GitHub; continuing with no pitch comparisons.");
    pitchCount = 0;
  } else {
    parsePitchFile(pitchData);
  }
  return true;
}


// =================== Playback + pitch detection ===================
void playbackLoop() {
  if (lyricCount == 0) {
    lcd.clear(); lcd.print("No song found!");
    delay(800);
    return;
  }


  setupI2S(); // initialize mic
  playing = true;
  songStartMs = millis();


  int currentIdx = 0;
  unsigned long lastDetect = 0;


  // initialize smoothing buffer to invalid
  for (int i = 0; i < SMOOTH_N; ++i) smoothBuf[i] = -1.0f;
  smoothIdx = 0;


  Serial.println("Playback started. Lyrics on LCD; pitch results to Serial.");


  while (playing) {
    float t = (millis() - songStartMs) / 1000.0f;


    // advance lyric index
    if (currentIdx + 1 < lyricCount && t >= lyrics[currentIdx + 1].time) currentIdx++;


    // show lyric
    String curLyric = (currentIdx < lyricCount) ? lyrics[currentIdx].text : "";
    showLyric(curLyric);


    // sample and detect pitch periodically
    if ((millis() - lastDetect) >= DETECT_INTERVAL_MS) {
      lastDetect = millis();


      // read I2S block
      int got = readI2SSamples(i2s_read_buffer, I2S_BLOCK_SAMPLES);
      if (got <= 0) {
        Serial.println("I2S read fail");
        // continue, maybe transient
      } else {
        // convert to float [-1..1]
        int convertCount = got;
        if (convertCount > I2S_BLOCK_SAMPLES) convertCount = I2S_BLOCK_SAMPLES;
        for (int i = 0; i < convertCount; ++i) {
          // INMP441 24-bit left-justified inside 32-bit signed; normalize by 2^31
          floatBuf[i] = (float)i2s_read_buffer[i] / 2147483648.0f;
        }


        // detect pitch
        float detected = detectPitchAutocorr(floatBuf, convertCount, (float)SAMPLE_RATE);
        float smoothed = -1.0f;
        if (detected > 0) smoothed = smoothPitch(detected);


        float expected = expectedPitchAt(t);


        // Print / judge
        if (expected <= 0.001f) {
          if (smoothed > 0)
            Serial.printf("[%.2fs] LIVE: %.1f Hz (no target)\n", t, smoothed);
          else
            Serial.printf("[%.2fs] LIVE: -- Hz (no target)\n", t);
        } else {
          if (smoothed <= 0) {
            Serial.printf("[%.2fs] LIVE: -- Hz  TARGET: %.1f Hz\n", t, expected);
          } else {
            float ratio = smoothed / expected;
            if (fabs(ratio - 1.0f) <= FREQ_TOLERANCE_RATIO) {
                Serial.printf("[%.2fs] GOOD     you: %.1f Hz  target: %.1f Hz\n", t, smoothed, expected);
                setColor(0, 255, 0);    // GREEN
            } else if (ratio < 1.0f) {
                Serial.printf("[%.2fs] TOO LOW  you: %.1f Hz  target: %.1f Hz\n", t, smoothed, expected);
                setColor(0, 0, 255);    // BLUE
            } else {
                Serial.printf("[%.2fs] TOO HIGH you: %.1f Hz  target: %.1f Hz\n", t, smoothed, expected);
                setColor(255, 0, 0);    // RED
            }
          }
        }
      } // got samples
    } // detect interval


    // stop when at end of song + buffer
    if (currentIdx >= lyricCount - 1 && t > (lyrics[lyricCount-1].time + 4.0f)) {
      Serial.println("End of song reached.");
      break;
    }


    // allow '-' to stop playback
    if (IrReceiver.decode()) {
      String k = decodeKey(IrReceiver.decodedIRData.decodedRawData);
      if (k == "-") {
        Serial.println("Playback interrupted by '-'.");
        playing = false;
        IrReceiver.resume();
        break;
      }
      IrReceiver.resume();
    }


    delay(10);
  } // while playing


  i2s_driver_uninstall(I2S_NUM_0);
  playing = false;
  lcd.clear();
  lcd.print("Song finished!");
  delay(1200);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Enter song code:");
}


// =================== IR + main ===================
void setup() {
  Serial.begin(115200);
  delay(200);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connecting WiFi");


  // IR
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);


  // connect WiFi
  WiFi.begin(ssid, password);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(300);
    Serial.print(".");
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    lcd.setCursor(0,1);
    lcd.print("WiFi connected");
    Serial.println("WiFi connected: " + WiFi.localIP().toString());
  } else {
    lcd.setCursor(0,1);
    lcd.print("WiFi failed");
    Serial.println("WiFi connect failed");
  }
  delay(700);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Enter song code:");
}


void loop() {
  if (IrReceiver.decode()) {
    unsigned long raw = IrReceiver.decodedIRData.decodedRawData;
    String key = decodeKey(raw);
    Serial.print("Key: "); Serial.println(key);


    if (key >= "0" && key <= "9") {
      inputCode += key;
      if (inputCode.length() > 6) inputCode = inputCode.substring(inputCode.length() - 6);
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("Code:");
      lcd.setCursor(0,1); lcd.print(inputCode);
    } else if (key == "-") {
      if (playing) playing = false;
      inputCode = "";
      lcd.clear(); lcd.setCursor(0,0); lcd.print("Cleared!");
      delay(600);
      lcd.clear(); lcd.setCursor(0,0); lcd.print("Enter song code:");
    } else if (key == "EQ") {
      if (inputCode.length() == 0) {
        lcd.clear(); lcd.setCursor(0,0); lcd.print("No song code");
        delay(700);
        lcd.clear(); lcd.setCursor(0,0); lcd.print("Enter song code:");
      } else {
        lcd.clear(); lcd.setCursor(0,0); lcd.print("Loading...");
        Serial.println("Fetching files for code: " + inputCode);
        bool ok = fetchFilesForCode(inputCode);
        if (ok && lyricCount > 0) {
          playbackLoop();
        } else {
          lcd.clear(); lcd.setCursor(0,0); lcd.print("No song found!");
          Serial.println("No song data for code: " + inputCode);
          delay(1000);
          lcd.clear();
          lcd.setCursor(0,0); lcd.print("Enter song code:");
        }
        inputCode = "";
      }
    }
    IrReceiver.resume();
  }


  delay(20);
}


// =================== IR decode map ===================
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
    default: return "UNKNOWN";
  }
}
