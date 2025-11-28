#include <IRremote.hpp>    
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define IR_RECEIVE_PIN 32

LiquidCrystal_I2C lcd(0x27, 16, 2);


String inputCode = "";


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== IR Remote Debug Start ===");
  Serial.println("Initializing...");


  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter song code:");


  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.print("IR Receiver started on pin ");
  Serial.println(IR_RECEIVE_PIN);
  Serial.println("Waiting for remote input...");
}


void loop() {
  if (IrReceiver.decode()) {
    Serial.println("\n--- SIGNAL DETECTED ---");
    Serial.print("Protocol: ");
    Serial.println(IrReceiver.decodedIRData.protocol);
    Serial.print("Address: ");
    Serial.println(IrReceiver.decodedIRData.address, HEX);
    Serial.print("Command: ");
    Serial.println(IrReceiver.decodedIRData.command, HEX);
    Serial.print("Raw Data: 0x");
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);


    unsigned long keyValue = IrReceiver.decodedIRData.decodedRawData;
    String keyName = decodeKey(keyValue);
    Serial.print("Key interpreted as: ");
    Serial.println(keyName);


    if (keyName >= "0" && keyName <= "9") {
      inputCode += keyName;
      if (inputCode.length() > 5) {
        inputCode = inputCode.substring(inputCode.length() - 5);
      }


      updateLCD();
    }


    //to change later
    else if (keyName == "-"){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Enter song code: ")
    }


    //entering the input
    else if (keyName == "EQ") {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Start!");
      lcd.setCursor(0, 1);


      if (inputCode.length() == 0) {
        lcd.print("No song code");
        Serial.println("EQ pressed — no song code");
      } else {
        lcd.print("Playing #" + inputCode);
        Serial.print("EQ pressed — playing song #");
        Serial.println(inputCode);
        //add case logic here for each of the songs
      }


      delay(2000);
      inputCode = "";
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Enter song code:");
    }


    IrReceiver.resume();
  }


  // keep looping even if no signal
  delay(50);
}


// --- LCD update function ---
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Code:");
  lcd.setCursor(0, 1);
  lcd.print(inputCode);
  Serial.print("Current input: ");
  Serial.println(inputCode);
}


// --- designate values to remote keys ---
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
