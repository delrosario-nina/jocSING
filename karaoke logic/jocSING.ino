/****************************************************
   ESP32 Karaoke – FINAL FULL VERSION
   MM + ChatGPT
   - Binary song codes
   - Correct lyric mapping for high/low pitch versions
   - Pitch smoothing + fallback
   - Scoring works
   - "-" button stops song
****************************************************/


#include <IRremote.hpp>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "driver/i2s.h"
#include "DFRobotDFPlayerMini.h"
#include <math.h>


/****************************************************
   WIFI + SERVER
****************************************************/
const char* ssid     = "Montagot_2.4";
const char* password = "noel1024";
const String baseURL = "https://delrosario-nina.github.io/jocSING/";


/****************************************************
   DFPLAYER
****************************************************/
HardwareSerial mp3(2);
DFRobotDFPlayerMini dfplayer;


/****************************************************
   LCD
****************************************************/
LiquidCrystal_I2C lcd(0x27, 16, 2);


/****************************************************
   IR
****************************************************/
#define IR_RECEIVE_PIN 32


/****************************************************
   RGB LED
****************************************************/
#define PIN_R 14
#define PIN_G 19
#define PIN_B 16


void setupPWM() {
  ledcAttach(PIN_R, 5000, 8);
  ledcAttach(PIN_G, 5000, 8);
  ledcAttach(PIN_B, 5000, 8);
}


void setColor(int r, int g, int b) {
  ledcWrite(PIN_R, r);
  ledcWrite(PIN_G, g);
  ledcWrite(PIN_B, b);
}


/****************************************************
   I2S MIC CONFIG
****************************************************/
#define SAMPLE_RATE 44100
#define SAMPLES     1024


#define I2S_WS   25
#define I2S_SD   27
#define I2S_SCK  26


int32_t i2sBuffer[SAMPLES];
float   floatBuffer[SAMPLES];


i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 8,
  .dma_buf_len = 256,
  .use_apll = false
};


i2s_pin_config_t pin_config = {
  .bck_io_num   = I2S_SCK,
  .ws_io_num    = I2S_WS,
  .data_out_num = -1,
  .data_in_num  = I2S_SD
};


/****************************************************
   RMS
****************************************************/
float computeRMS(float* buf, int count) {
  double sum = 0;
  for (int i=0;i<count;i++) sum += buf[i]*buf[i];
  return sqrt(sum/count);
}


/****************************************************
   AUTOCORR PITCH DETECTION
****************************************************/
float estimatePitch(float* samples, int n, int sample_rate) {
  int minLag = sample_rate / 1000;
  int maxLag = sample_rate / 80;


  float best=0;
  int bestLag=-1;


  for(int lag=minLag; lag<=maxLag; lag++){
    float corr=0;
    for(int i=0;i<n-lag;i++)
      corr += samples[i]*samples[i+lag];
    if (corr > best) {
      best=corr;
      bestLag=lag;
    }
  }
  if(bestLag<=0) return -1;
  return (float)sample_rate / bestLag;
}


/****************************************************
   SMOOTHING (Median)
****************************************************/
#define SMOOTH_N 5
float smoothBuf[SMOOTH_N];
int smoothIdx=0;


float smoothPitch(float p) {
  if (p <= 0) return p;


  smoothBuf[smoothIdx] = p;
  smoothIdx = (smoothIdx+1) % SMOOTH_N;


  float t[SMOOTH_N];
  int c=0;


  for(int i=0;i<SMOOTH_N;i++)
    if(smoothBuf[i]>0)
      t[c++]=smoothBuf[i];


  if (c==0) return -1;


  for(int i=1;i<c;i++){
    float key=t[i];
    int j=i-1;
    while(j>=0 && t[j]>key){
      t[j+1]=t[j];
      j--;
    }
    t[j+1]=key;
  }


  return (c%2 ? t[c/2] : 0.5*(t[c/2]+t[c/2-1]));
}


/****************************************************
   LRC + PITCH FILE ARRAYS
****************************************************/
#define MAX_LRC_LINES 300
struct LyricLine { float time; String text; };
LyricLine lyrics[MAX_LRC_LINES];
int lyricCount=0;


#define MAX_PITCH_POINTS 800
struct PitchPoint { float time; float freq; };
PitchPoint pitchPoints[MAX_PITCH_POINTS];
int pitchCount=0;


bool songPlaying=false;
unsigned long songStartMs=0;
int currentLyric=0;


int totalSamples=0;
int accurateSamples=0;


String inputCode="";
float lastValidPitch=0;


/****************************************************
   HTTP
****************************************************/
String httpGetString(String url){
  HTTPClient http;
  http.begin(url);
  int code=http.GET();
  String data = (code==200)? http.getString() : "";
  http.end();
  return data;
}


/****************************************************
   PARSE LRC
****************************************************/
void parseLRC(String d){
  lyricCount=0;
  int idx=0;


  while(idx<d.length() && lyricCount<MAX_LRC_LINES){
    int nl=d.indexOf('\n',idx);
    if(nl==-1) nl=d.length();
    String line=d.substring(idx,nl); line.trim();


    if(line.startsWith("[")){
      int br=line.indexOf("]");
      int col=line.indexOf(":");
      if(br>0 && col>0){
        int mm=line.substring(1,col).toInt();
        float ss=line.substring(col+1,br).toFloat();
        lyrics[lyricCount].time = mm*60 + ss;
        lyrics[lyricCount].text = line.substring(br+1);
        lyricCount++;
      }
    }
    idx=nl+1;
  }
}


/****************************************************
   PARSE PITCH
****************************************************/
void parsePitchFile(String d){
  pitchCount=0;
  int idx=0;


  while(idx<d.length() && pitchCount<MAX_PITCH_POINTS){
    int nl=d.indexOf('\n',idx);
    if(nl==-1) nl=d.length();


    String line=d.substring(idx,nl); line.trim();


    if(line.startsWith("[")){
      int br=line.indexOf("]");
      int col=line.indexOf(":");
      if(br>0 && col>0){
        int mm=line.substring(1,col).toInt();
        float ss=line.substring(col+1,br).toFloat();
        float f=line.substring(br+1).toFloat();


        pitchPoints[pitchCount].time = mm*60+ss;
        pitchPoints[pitchCount].freq = f;
        pitchCount++;
      }
    }
    idx=nl+1;
  }
}


/****************************************************
   LYRIC MAPPING FIX
****************************************************/
String resolveLyricCode(String code) {
  // Buko
  if(code=="0101" || code=="0110") return "0100";


  // Magkabilang Mundo
  if(code=="1000" || code=="1001") return "0111";


  // No Erase
  if(code=="1011" || code=="1100") return "1010";


  // otherwise use own code
  return code;
}


/****************************************************
   EXPECTED PITCH + FALLBACK
****************************************************/
float expectedPitchAt(float t){
  if(pitchCount==0) return lastValidPitch;


  if(t <= pitchPoints[0].time){
    if(pitchPoints[0].freq>0) lastValidPitch=pitchPoints[0].freq;
    return lastValidPitch;
  }


  if(t >= pitchPoints[pitchCount-1].time){
    if(pitchPoints[pitchCount-1].freq>0)
      lastValidPitch=pitchPoints[pitchCount-1].freq;
    return lastValidPitch;
  }


  for(int i=0;i<pitchCount-1;i++){
    if(t>=pitchPoints[i].time && t<pitchPoints[i+1].time){
      float dt=pitchPoints[i+1].time - pitchPoints[i].time;
      float frac=(t-pitchPoints[i].time)/dt;
      float f = pitchPoints[i].freq +
                frac*(pitchPoints[i+1].freq - pitchPoints[i].freq);
      if(f>0) lastValidPitch=f;
      return lastValidPitch;
    }
  }


  return lastValidPitch;
}


/****************************************************
   LYRIC DISPLAY + SCROLL
****************************************************/
int currentLyricIndex=-1;
String currentLyricText="";
int scrollPos=0;
unsigned long lyricScrollTimer=0;
bool lyricScrolling=false;


const unsigned long LYRIC_HOLD_MS=500;
const unsigned long LYRIC_SCROLL_MS=210;


void startNewLyric(int idx){
  if(idx<0 || idx>=lyricCount) return;


  currentLyricIndex=idx;
  currentLyricText=lyrics[idx].text;
  scrollPos=0;
  lyricScrollTimer=millis();
  lyricScrolling=false;


  lcd.clear();
  lcd.setCursor(0,0);


  if(currentLyricText.length()<=16)
    lcd.print(currentLyricText);
  else
    lcd.print(currentLyricText.substring(0,16));
}


void updateLyricScrolling(){
  if(!songPlaying) return;
  if(currentLyricIndex<0) return;
  if(currentLyricText.length()<=16) return;


  unsigned long now=millis();


  if(!lyricScrolling){
    if(now - lyricScrollTimer < LYRIC_HOLD_MS) return;
    lyricScrolling=true; lyricScrollTimer=now;
  }


  if(now - lyricScrollTimer >= LYRIC_SCROLL_MS){
    lyricScrollTimer=now;


    if(scrollPos+16 < currentLyricText.length()){
      scrollPos++;
      String win=currentLyricText.substring(scrollPos, scrollPos+16);
      while(win.length()<16) win+=" ";
      lcd.setCursor(0,0);
      lcd.print(win);
    }
  }
}


/****************************************************
   STOP SONG
****************************************************/
void stopSong(){
  dfplayer.stop();
  songPlaying=false;


  float score = (totalSamples>0) ?
                 (100.0f * accurateSamples/totalSamples) : 0;


  lcd.clear();
  lcd.print("Final Score:");
  lcd.setCursor(0,1);
  lcd.print((int)score);
  lcd.print("/100");


  delay(4000);


  lcd.clear();
  lcd.print("Enter code:");


  totalSamples=0;
  accurateSamples=0;


  currentLyricIndex=-1;
  currentLyricText="";
  scrollPos=0;
  lyricScrolling=false;
}


/****************************************************
   SETUP
****************************************************/
void setup(){
  Serial.begin(115200);
  delay(300);


  setupPWM();
  lcd.init();
  lcd.backlight();


  lcd.clear();
  lcd.print("Connecting WiFi");


  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);


  WiFi.begin(ssid,password);
  while(WiFi.status()!=WL_CONNECTED) delay(200);


  lcd.clear();
  lcd.print("WiFi OK");


  mp3.begin(9600, SERIAL_8N1, 17, 5);
  delay(400);
  dfplayer.begin(mp3);
  dfplayer.volume(12);


  i2s_driver_install(I2S_NUM_0,&i2s_config,0,NULL);
  i2s_set_pin(I2S_NUM_0,&pin_config);


  lcd.clear();
  lcd.print("Enter code:");
}


/****************************************************
   LOOP
****************************************************/
void loop(){


  /************** IR INPUT **************/
  if(IrReceiver.decode()){
    String key="";
    unsigned long raw=IrReceiver.decodedIRData.decodedRawData;
    IrReceiver.resume();


    // MAP IR RAW
    switch(raw){
      case 0xF807FF00: key="-"; break;
      case 0xF609FF00: key="EQ"; break;
      case 0xE916FF00: key="0"; break;
      case 0xF30CFF00: key="1"; break;
      case 0xE718FF00: key="2"; break;
      case 0xA15EFF00: key="3"; break;
      case 0xF708FF00: key="4"; break;
      case 0xE31CFF00: key="5"; break;
      case 0xA55AFF00: key="6"; break;
      case 0xBD42FF00: key="7"; break;
      case 0xAD52FF00: key="8"; break;
      case 0xB54AFF00: key="9"; break;
      default: key="UNKNOWN"; break;
    }


    /**************** "-" BUTTON FIX ****************/
    if(key=="-"){


      if(songPlaying){
        stopSong();
        return;
      }


      inputCode="";
      lcd.clear();
      lcd.print("Cleared!");
      delay(400);
      lcd.clear();
      lcd.print("Enter code:");
      return;
    }


    /**************** ADD DIGIT ****************/
    if(key>="0" && key<="9"){
      inputCode+=key;


      if(inputCode.length()>4)
        inputCode=inputCode.substring(inputCode.length()-4);


      lcd.clear();
      lcd.print("Code (bin):");
      lcd.setCursor(0,1);
      lcd.print(inputCode);
    }


    /**************** PLAY SONG ****************/
    if(key=="EQ"){
      if(inputCode.length()==0){
        lcd.clear();
        lcd.print("No code!");
        delay(500);
        lcd.clear();
        lcd.print("Enter code:");
        return;
      }


      lcd.clear();
      lcd.print("Loading...");


      String lyricCode = resolveLyricCode(inputCode);


      String lrc      = httpGetString(baseURL+lyricCode+".lrc");
      String pitchTxt = httpGetString(baseURL+inputCode+"_pitch.txt");


      parseLRC(lrc);
      parsePitchFile(pitchTxt);


      int trackNum = strtol(inputCode.c_str(), NULL, 2);
      if(trackNum<=0) trackNum=1;


      lastValidPitch=0;


      dfplayer.stop();
      delay(120);
      dfplayer.play(trackNum);


      songStartMs=millis();
      songPlaying=true;
      currentLyric=0;
      totalSamples=0;
      accurateSamples=0;


      currentLyricIndex=-1;
      scrollPos=0;
      lyricScrolling=false;


      if(lyricCount>0) startNewLyric(0);


      inputCode="";
    }
  }


  /************** SONG ACTIVE **************/
  if(songPlaying){


    float t = (millis()-songStartMs)/1000.0f;


    while(currentLyric+1<lyricCount &&
          t>=lyrics[currentLyric+1].time){
      currentLyric++;
      startNewLyric(currentLyric);
    }


    updateLyricScrolling();


    size_t bytesRead=0;
    i2s_read(I2S_NUM_0, i2sBuffer, sizeof(i2sBuffer), &bytesRead, portMAX_DELAY);


    int count=bytesRead/4;
    if(count>0){


      double mean=0;
      for(int i=0;i<count;i++){
        float sample=(float)(i2sBuffer[i]>>8)/8388608.0f;
        floatBuffer[i]=sample;
        mean+=sample;
      }
      mean/=count;


      for(int i=0;i<count;i++)
        floatBuffer[i]-=mean;


      float rms=computeRMS(floatBuffer,count);


      if(rms < 0.010f){
        setColor(0,0,255);
      } else {


        float detected = smoothPitch( estimatePitch(floatBuffer,count,SAMPLE_RATE) );
        float target   = expectedPitchAt(t);


        if(detected<=0 || target<=0){
          setColor(0,0,255);
        }
        else{
          float ratio=detected/target;


          if(fabs(ratio-1.0f) <= 0.15f){
            setColor(0,255,0);
            accurateSamples++;
          }
          else if(ratio < 1.0f){
            setColor(0,0,255);
          }
          else{
            setColor(255,0,0);
          }


          totalSamples++;
        }
      }
    }


    if(currentLyric>=lyricCount-1 &&
       t>lyrics[lyricCount-1].time + 5){
      stopSong();
    }
  }


}



