#include <Ticker.h>

#include <cmath>  // for std::ceil
Ticker timer1;

#define PIN_TRIAC_PULSE 4
#define PIN_PIR_IN 12
#define PIN_ZERO_CROSS 5
#define LOOP_DELAY_NON_DIMMING 500
#define LOOP_DELAY_DIMMING 20
#define DEBOUNCE_TIME 3000

// 1000000us / 60hz / 2 = 8333us
bool interruptEnabled = true;
const int dimming_time = 3000;
int int_current_brightness = 0;
int current_level = 2;
int current_brightness = 300;
int target_brightness = 300;
unsigned long lastChangeTime = 0;
bool triggered = false;
bool auto_mode = true;
float dimming_step = 0;
bool dimming_in_progress = false;
int triggerCount = 0;
int start_time_in_current_level = 0;
static unsigned long lastTriggerTime = 0;

struct Level {
  int brightness;
  unsigned long timeToPreviousLevel;
};

Level levels[] = {
    {0, -1}, {50, 10000}, {70, 10000}, {100, 10000}, {200, 20000}, {300, 20000},
};

void set_level(int level) {
  struct Level level_value = levels[current_level];
  target_brightness = level_value.brightness;
  dimming_step = static_cast<float>(target_brightness - current_brightness) /
                 (dimming_time / LOOP_DELAY_DIMMING);
  if (dimming_step > 0 && dimming_step < 1) {
    dimming_step = 1;
  } else if (dimming_step < 0 && dimming_step > -1) {
    dimming_step = -1;
  }
  dimming_in_progress = true;

  Serial.print("current_brightness: ");
  Serial.print(current_brightness);
  Serial.print(" target_brightness: ");
  Serial.print(target_brightness);
  Serial.print(" dimming_step: ");
  Serial.println(dimming_step);
}

void setup() {
  current_level = sizeof(levels) / sizeof(levels[0]) - 1;
  current_brightness = levels[current_level].brightness;
  int_current_brightness = dimming_map(current_brightness);
  Serial.begin(115200);
  Serial.setTimeout(500);
  pinMode(PIN_TRIAC_PULSE, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ZERO_CROSS), handleInturrupt, FALLING);
  pinMode(PIN_PIR_IN, INPUT);  // PIR input
}

IRAM_ATTR void handleInturrupt() {
  if (interruptEnabled) {
    int chop_time = (8.33 * int_current_brightness);

    digitalWrite(PIN_TRIAC_PULSE, HIGH);
    delayMicroseconds(chop_time);
    digitalWrite(PIN_TRIAC_PULSE, LOW);
  } else {
    digitalWrite(PIN_TRIAC_PULSE, HIGH);
  }
}

int dimming_map(int val) {
  interruptEnabled = val != 0;
  return map(val, 0, 1000, 900, 90);
}

void loop() {
  if (dimming_in_progress) {
    delay(LOOP_DELAY_DIMMING);
  } else {
    delay(LOOP_DELAY_NON_DIMMING);
  }

  if (auto_mode) {
    struct Level level = levels[current_level];
    int current_time = millis();
    int last_level = current_level;

    if ((current_time - start_time_in_current_level) > level.timeToPreviousLevel &&
        level.timeToPreviousLevel > 0) {
      current_level--;
      if (current_level <= 0) {
        current_level = 0;
      }
      start_time_in_current_level = millis();
    } else {
      // level up with any trigger
      if (digitalRead(PIN_PIR_IN) == HIGH) {
        unsigned long currentTime = start_time_in_current_level = millis();
        if (currentTime - lastTriggerTime >= DEBOUNCE_TIME) {
          current_level++;
          int level_count = sizeof(levels) / sizeof(levels[0]);
          if (current_level >= level_count) {
            current_level = level_count - 1;
          }
          lastTriggerTime = currentTime;
        }
      }
    }
    if (last_level != current_level) {
      Serial.print("level change from last_level:");
      Serial.print(last_level);
      Serial.print(" to current_level:");
      Serial.println(current_level);

      set_level(current_level);
    }

    if (current_brightness != target_brightness) {
      current_brightness += dimming_step;
      if (current_brightness < 0) {
        current_brightness = 0;
      } else if (current_brightness > 1000) {
        current_brightness = 1000;
      }

      if (abs(target_brightness - current_brightness) < abs(dimming_step)) {
        current_brightness = target_brightness;
        dimming_in_progress = false;
      }

      int_current_brightness = dimming_map(current_brightness);
      // Serial.print("current_brightness: ");
      // Serial.println(current_brightness);
    } else {
      dimming_in_progress = false;
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
        current_brightness = dimming_map(reading);
        interruptEnabled = true;
        // Serial.print("current_brightness: ");
        // Serial.print(current_brightness);
      }
    }
    Serial.println("");
  }
}