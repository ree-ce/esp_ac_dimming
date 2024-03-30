#include <Ticker.h>  
Ticker timer1;       

#define PIN_TRIAC_PULSE 4 
#define PIN_PIR_IN 12
#define PIN_ZERO_CROSS 5
#define MAX_BRIGHTNESS_LEVEL 300
#define LOOP_DELAY_NON_DIMMING 500
#define LOOP_DELAY_DIMMING 10

// 1000000us / 60hz / 2 = 8333us
bool interruptEnabled = true;
int last_chop_time = 0;
const long delay_to_dark = 0.2 * 60 * 1000;  // 5 minutes in milliseconds
const int dimming_time = 1000;
const int minimum_brightness_level = 50;
int max_brightness_level = MAX_BRIGHTNESS_LEVEL;
const long changeInterval =
    dimming_time / (max_brightness_level - minimum_brightness_level);
unsigned long lastTriggerTime = 0;
unsigned long triggerStartTime = 0;
int brightness = 500;
int brightness_level = 300;

unsigned long lastChangeTime = 0;
bool triggered = false;
bool auto_mode = true;
bool brighter_in_progress = false;
bool darker_in_progress = false;
int triggerCount = 0;

void setup() {
  brightness = dimming_map(brightness_level);
  Serial.begin(115200);
  Serial.setTimeout(500);
  pinMode(PIN_TRIAC_PULSE, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ZERO_CROSS), handleInturrupt, FALLING);
  pinMode(PIN_PIR_IN, INPUT);  // PIR input
}

IRAM_ATTR void handleInturrupt() {
  if (interruptEnabled) {
    int chop_time = (8.33 * brightness);
    if (last_chop_time != chop_time) {
      last_chop_time = chop_time;
      // Serial.print("chop_time:");
      // Serial.println(chop_time);
    }

    digitalWrite(PIN_TRIAC_PULSE, HIGH);
    delayMicroseconds(chop_time);
    digitalWrite(PIN_TRIAC_PULSE, LOW);
  }
}

int dimming_map(int val) {
  int data1 = map(val, 0, 1000, 900, 90);
  return data1;
}
bool trigger1 = false;
void loop() {
  if(brighter_in_progress || darker_in_progress){
    delay(LOOP_DELAY_DIMMING);
  }
  else{
    delay(LOOP_DELAY_NON_DIMMING);
    }

  if (auto_mode) {
    if (digitalRead(PIN_PIR_IN) == HIGH) {
      lastTriggerTime = millis();
      if (triggerStartTime == 0) {
        triggerStartTime = millis();
        trigger1 = true;
      }
      if (trigger1) {
        if (millis() - triggerStartTime >= 4000) {
          digitalWrite(PIN_TRIAC_PULSE, LOW);
          interruptEnabled = true;
          brighter_in_progress = true;
          darker_in_progress = false;
          trigger1 = false;
          triggerStartTime = 0;
        }
      }
    } else {
      if (trigger1 && millis() - triggerStartTime >= 7000) {
        trigger1 = false;
        triggerStartTime = 0;
      }
    }

    if (!brighter_in_progress) {
      int remainingTime = millis() - lastTriggerTime;
      if (remainingTime >= delay_to_dark) {
        if (millis() - lastChangeTime >= changeInterval) {
          if (brightness_level > minimum_brightness_level) {
            darker_in_progress = true;
            brightness_level--;                                                                                                                 
            brightness = dimming_map(brightness_level);
          }
          lastChangeTime = millis();
        }
      }
      else if(!darker_in_progress){
        Serial.println("Remaining time to decrease brightness: " + String(delay_to_dark - remainingTime) + " ms");
      }
    }

    if (brighter_in_progress && millis() - lastChangeTime >= changeInterval) {
      if (brightness_level < max_brightness_level) {
        brightness_level++;
        brightness = dimming_map(brightness_level);
      } else {
        brighter_in_progress = false;
      }
      lastChangeTime = millis();
    }
  }

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');

    if (command == "on") {
      digitalWrite(PIN_TRIAC_PULSE, LOW);
      interruptEnabled = false;
      auto_mode = false;
      Serial.println("ON");
    } else if (command == "off") {
      digitalWrite(PIN_TRIAC_PULSE, HIGH);
      interruptEnabled = false;
      auto_mode = false;
      Serial.println("OFF");
    } else if (command == "auto") {
      auto_mode = true;
      Serial.println("AUTO MODE ON");
    } else {
      Serial.println(command);
      int reading = command.toInt();
      auto_mode = false;
      if (reading >= 0) {
        brightness = dimming_map(reading);
        interruptEnabled = true;
        Serial.print("brightness: ");
        Serial.print(brightness);
      }
    }
    Serial.println("");
  }
}