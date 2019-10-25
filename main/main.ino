/*
  Main class for the Vitameter Basel
*/

#include <Wire.h>
#include <BLE_wcs.h>
#include <stdio.h>
#include <string>
#include <Adafruit_CCS811.h>
#include <Adafruit_VEML6075.h>
#include <Adafruit_ADXL335.h>
#include <InterfaceOut.h>
#include <Values.h>
#include "C:\Users\Yumi\Desktop\Vitameter\config.h"

// For debugging reasons. Keep defines to see currently measured values shown over according channel.
#define SHOW_SERIAL
#define SHOW_BLE

uint8_t state = LIGHT_SLEEP;
/*
void setup() {}
void loop() {}
*/

// Bluetooth related
BLE_wcs ble;
std::string sent;
std::string oldSent;
std::string processed = "prodef";
std::string msg = "";
uint32_t bleTimer = 0;
uint32_t bleShow = 0;
uint32_t bleShowFreq = 3000;
uint32_t bleMsgFreq = 3000;


Values values;
Adafruit_CCS811 ccs;
Adafruit_VEML6075 uv = Adafruit_VEML6075();
Adafruit_ADXL335 pedo;

InterfaceOut vib(VIBRATION_PIN);
InterfaceOut ledRed(LEDRED_PIN);
InterfaceOut ledGreen(LEDGREEN_PIN);
InterfaceOut ledBlue(LEDBLUE_PIN);
InterfaceOut sensors(SENSORS_EN_PIN);

bool firstBoot = 1;
bool reactivateWarning = 0;

// Button related
volatile bool checkBT = 0;
volatile bool checkPW = 0;
volatile bool checkWA = 0;
volatile uint32_t powerDebounceTimer = 0;
volatile uint32_t btDebounceTimer = 0;
volatile uint32_t waDebounceTimer = 0;
volatile uint32_t btButtonPressed = 0;
volatile uint32_t pwButtonPressed = 0;
volatile uint32_t waButtonPressed = 0;

// other timers
uint32_t ms = 0;
uint32_t warningTimeout = 0;
uint8_t warningCounter = 0;
uint32_t warningVibTimeout = 0;
bool goalVib = 0;
uint8_t vibCounter = 0;
uint32_t vibTimeout = 0;
uint32_t showFreq = UV_FREQ;
uint32_t sleepTime = 0;
uint32_t lastEmptied = 0;
uint32_t pedoTimeout = 0;
uint32_t uvTimeout = 0;
uint32_t airTimeout = 0;
uint32_t showTimeout = 0;


//////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Serial communication init
  Serial.begin(BAUDRATE);
  while (!Serial) {
    delay(10);
  }

  // I2C communication to sensors init
  Wire.begin(SDA_PIN, SCL_PIN);

  // Buttons init
  pinMode(POWER_PIN, INPUT);
  pinMode(BLUETOOTH_PIN, INPUT);
  // pinMode(WARNING_PIN, INPUT); // TODO add later

  // ADC init
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(Z_PIN, INPUT);
  
  // Thresholds for sensor values init
  values.init();
  values.warnCO2 = 1;
  values.warnVOC = 1;
  values.warnUVI = 1;
  sensors.on();
  delay(500);
  pedo.calibrate();
  values.pedoEnable = 1;
  ble.init("Vitameter low energy");
}


//////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  ms = millis();

  if (checkBT || checkPW || checkWA) {
    checkButtonState();
  }

  if (reactivateWarning && ms > warningTimeout) {
    Serial.println("Warnings reactivated");
    Serial.println();
    values.warnCO2 = 1;
    values.warnVOC = 1;
    reactivateWarning = 0;
  }

  if (values.warning) {
    ledRed.on();
  } else {
    ledRed.off();
  }
  
  if (ms > bleTimer) {
    bleTimer = ms + bleMsgFreq;
  
    if (values.dataWanted_all) {
      // Send over UART
      Serial.println("all data wanted");
      Serial.println(values.prepareAllData().c_str());
      // TODO something like "while not done" needed?
      values.dataWanted_all = 0;  // Why TODO here? Hört es auf zu senden wenn 0 gesetzt wird?
    }
    if (values.dataWanted_CO2) {
      Serial.println("CO2 Data wanted");
      Serial.println(values.prepareCO2Data().c_str());
      values.dataWanted_CO2 = 0;
    } else if (values.dataWanted_UVI) {
      Serial.println("UVI Data wanted");
      Serial.println(values.prepareUVIData().c_str());
      values.dataWanted_UVI = 0;
    } else if (values.dataWanted_steps) {
      Serial.println("Steps Data wanted");
      Serial.println(values.prepareStepData().c_str());
      values.dataWanted_steps = 0;
    } else {
      sent = ble.getMessage();
      processed = values.processMessage(sent);
      ble.write(processed);
  
      if (sent != oldSent) {
        Serial.print("sent");
        Serial.println(sent.c_str());
        Serial.print("message processed:   ");
        Serial.println(processed.c_str());
        Serial.print("parameter:   ");
        Serial.println(values.parameter.c_str());
        Serial.print("value is:   ");
        Serial.println(values._value);
        Serial.println("");
        Serial.println("");
        oldSent = sent;
      }
    }
  }  
 
  //__________________________________________________________________
  if (state == LIGHT_SLEEP) {
    
    if (values.uvi_idx >= 1) {
      values.storeRAMToFlash();
    }
    goLightSleep();
  }
  //__________________________________________________________________
  else if (state == SENSORS_ACTIVE) {
    if (values.warning) {
      handleWarning();
    }
    else {
      warningCounter = 0;
    }
    if (values.pedoEnable && ms > pedoTimeout) {
      uint16_t x = pedo.getPedo();
      
      // Step registered
      if (pedo.flag) {
        Serial.println(x);
        bool goalAchieved = values.storeSteps(x);
        if (goalAchieved) {
          // vibrate in short intervals
          goalVib = 1;
          vibTimeout = ms;
        }
        pedoTimeout = ms + WAIT_AFTER_STEP;
        pedo.flag = 0;
      }
      else {
        pedoTimeout += PEDO_FREQ;
      }
      if (goalVib && ms > vibTimeout) {
        vib.toggle();
        vibTimeout = ms + 200;
        vibCounter++;
      }
      if (vibCounter > 4) {
        goalVib = 0;
        vibCounter= 0;
        vib.off();
      }
    }
    if (ms > uvTimeout) {
      uvTimeout += values.uvFreq;
      uint8_t u = uv.readUVI();
      values.storeUVI(u);
    }
    if (ms > airTimeout) {
      ccs.readData();
      airTimeout += values.aqFreq;
      uint16_t c = ccs.geteCO2();
      uint16_t v = ccs.getTVOC();
      values.storeCO2(c);
      values.storeVOC(v);
      values.storeTemp(ccs.calculateTemperature());
    }

#ifdef SHOW_SERIAL
    if (ms > showTimeout) {
      String msg;
      msg = "Measurement number: ";
      Serial.print("Measurement number: ");
      Serial.println(values.co2_idx);
  
      Serial.print("CO2: ");
      Serial.println(values.getLastCO2());
      Serial.print("TVOC: ");
      Serial.println(values.getLastVOC());
      Serial.print("UV: ");
      Serial.println(values.getLastUVI());
      Serial.println();
      Serial.println();
      showTimeout += showFreq;
    }
#endif

    // Store RAM into flash if array is full.
    if (values.uvi_idx == UVI_STORAGE_SIZE) {
      values.storeRAMToFlash();
    }
    
    //_____ Go sleep until next timeout ________________________________

    if (!(checkBT || checkPW) && !values.pedoEnable && !values.warning) {
      ms = millis();
      if (uvTimeout > ms) {
        uint32_t timeLeft = uvTimeout - ms;
        if (timeLeft > 2000) {
          delay(500);
          gpio_wakeup_enable(GPIO_NUM_36, GPIO_INTR_LOW_LEVEL);
          gpio_wakeup_enable(GPIO_NUM_34, GPIO_INTR_LOW_LEVEL);
          // gpio_wakeup_enable(GPIO_NUM_23, GPIO_INTR_LOW_LEVEL); // TODO add back in on new one
          esp_sleep_enable_gpio_wakeup();
          goLightSleepTimeout(timeLeft - 500);
          if (esp_sleep_get_wakeup_cause() == 7) {
            if (digitalRead(POWER_PIN) == PRESSED_BUTTON_LEVEL) {
              checkPW = 1; 
              pwButtonPressed = ms;
            } else if (digitalRead(BLUETOOTH_PIN) == PRESSED_BUTTON_LEVEL) {
              checkBT = 1; 
              btButtonPressed = ms;
            } else if (digitalRead(WARNING_PIN) == PRESSED_BUTTON_LEVEL) {
              checkWA = 1; 
              btButtonPressed = ms;
            }
          }
        }
      }
    }
  }
 //___Bluetooth low energy  communication on (is always on) ____________________________________
  if (bluetoothOn) {
 
   if (ms > bleShow) {
      bleShow = ms + 3000;
      
#ifdef SHOW_BLE
      msg = "";
      msg = "Measurement number: ";
      msg += values.getUint8AsString(values.uvi_idx);
      msg += "\n";
      ble.write(msg);
      msg = "CO2: ";
      msg += values.getUint16AsString(values.getLastCO2());
      msg += "\n";
      ble.write(msg);
      msg = "TVOC: ";
      msg += values.getUint8AsString(values.getLastVOC());
      msg += "\n";
      ble.write(msg);
      msg = "UVI: ";
      msg += values.getUint8AsString(values.getLastUVI());
      msg += "\n";
      ble.write(msg);
      msg = "Steps: ";
      msg += values.getUint16AsString(values.getLastStep());
      msg += "\n";
      msg += "\n";
      ble.write(msg);
#endif
    }
  }  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

void goLightSleepTimeout(uint64_t sleepMillis) {
  esp_sleep_enable_timer_wakeup(sleepMillis * 1000);
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
}

void goLightSleep() {
  Serial.println("Enter sleep");
  detachInterrupt(digitalPinToInterrupt(POWER_PIN));
  detachInterrupt(digitalPinToInterrupt(BLUETOOTH_PIN));
  delay(1000);
  bluetoothOn = 0;
  values.clearAllWarnings();
  vib.off();
  ledGreen.off();
  ledRed.off();
  ledBlue.off();
  gpio_wakeup_enable(GPIO_NUM_36, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_34, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  wakeUp();
}

void setTimeouts() {
  ms = millis();
  pedoTimeout = PEDO_FREQ + ms;
  uvTimeout = UV_FREQ + ms;
  airTimeout = AQ_FREQ + ms;
  
  showTimeout = ms + showFreq;
  bleShow = ms + bleShowFreq;
}

void wakeUp() {
  // If button still pressed after 1 second
  delay(1000);
  
  if (digitalRead(POWER_PIN) == PRESSED_BUTTON_LEVEL) {
    Serial.println("WAKE UP, POWER UP!");
    state = SENSORS_ACTIVE;
    Serial.print("UVI thresh: ");
    Serial.println(EEPROM.read(UVI_THRESH_ADDR));
    Serial.print("CO2 thresh: ");
    Serial.println(values.getCO2Thresh());
    Serial.print("VOC thresh: ");
    Serial.println(EEPROM.read(VOC_THRESH_ADDR));
    Serial.print("Temp thresh: ");
    Serial.println(EEPROM.read(TEMP_THRESH_ADDR));
  
    Serial.print("Step goal: ");
    Serial.println(values.getStepGoal());
    
    ledGreen.on();
    sensorsInit();
    setTimeouts();
  }
  else if (digitalRead(BLUETOOTH_PIN) == PRESSED_BUTTON_LEVEL) {
    Serial.println("ONLY BLUETOOTH");
    ledBlue.on();
    bluetoothOn = 1;
    state = ONLY_BT;
    bleTimer = millis();
  }
  else {
    // Ignore if button is pressed less than a second
    state = LIGHT_SLEEP;
    return;
  }
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
  attachInterrupt(digitalPinToInterrupt(POWER_PIN), pwButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BLUETOOTH_PIN), btButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(WARNING_PIN), waButtonISR, FALLING);
  Serial.println("Good morning!");
}

void checkButtonState() {
  ms = millis();
  
  // Check Bluetooth button
  if (checkBT) {
    if (ms > btButtonPressed + 1200) {
      checkBT = 0;
      if (digitalRead(BLUETOOTH_PIN) == PRESSED_BUTTON_LEVEL) {
        if (bluetoothOn) {
            bluetoothOn = 0;
            // values.clearAllMemory();  // TODO change later. Now here to test.
            values.resetSteps();
            ledBlue.off();
            Serial.println("BT off");
            if (state == ONLY_BT) {
              Serial.println("Good Night");
              state = LIGHT_SLEEP;
            }
          }
        else {
          bluetoothOn = 1;
          ledBlue.on();
          Serial.println("dataWantel_all");
          values.dataWanted_all = 1;
        }
      }
    } else if (ms > btButtonPressed + 300) {
      if (digitalRead(BLUETOOTH_PIN) == !PRESSED_BUTTON_LEVEL) {
        checkBT = 0;
        if (values.warning) {
          vib.off();
          dismissWarning();
        }
      }
    }
  }
  // Check Power Button
  if (checkPW) {
    if (ms > pwButtonPressed + 1200) {
      checkPW = 0;
      if (digitalRead(POWER_PIN) == PRESSED_BUTTON_LEVEL) {
        if (state == SENSORS_ACTIVE) {
          state = LIGHT_SLEEP;  
        }
        else {
          // Wake up sensors.
          state = SENSORS_ACTIVE;
          ledGreen.on();
          sensorsInit();
          setTimeouts();
        }
      }     
    } else if (ms > pwButtonPressed + 300) {
      if (digitalRead(POWER_PIN) == !PRESSED_BUTTON_LEVEL) {
        checkPW = 0;
      }
    }
  }
  
  // Check Warning Button
  if (checkWA) {
    if (ms > waButtonPressed + 1200) {
      checkWA = 0;
      if (digitalRead(WARNING_PIN) == PRESSED_BUTTON_LEVEL) {
        if (state == SENSORS_ACTIVE) {
          state = LIGHT_SLEEP;  
        }
        else {
          // Wake up sensors.
          state = SENSORS_ACTIVE;
          ledGreen.on();
          sensorsInit();
          setTimeouts();
        }
      }     
    } else if (ms > waButtonPressed + 300) {
      if (digitalRead(WARNING_PIN) == !PRESSED_BUTTON_LEVEL) {
        checkWA = 0;
      }
    }
  }  
}

void pwButtonISR() {
  ms = millis();
  if (powerDebounceTimer < ms) {
    powerDebounceTimer = ms + 100;
    pwButtonPressed = ms;
    checkPW = 1;
  }
}

void btButtonISR() {
  ms = millis();
  // If bluetooth on
  if (btDebounceTimer < ms) {
    btDebounceTimer = ms + 100;
    btButtonPressed = ms;
    checkBT = 1;
  }
}

void waButtonISR() {
  ms = millis();
  if (waDebounceTimer < ms) {
    waDebounceTimer = ms + 100;
    waButtonPressed = ms;
    checkWA = 1;
  }
}

void sensorsInit() {
  bool error = 0;
  sensors.on();
  delay(500); 
  pedo.calibrate();
  // UV
  if (!uv.begin()) {
    Serial.println("Failed to communicate with VEML6075 UV sensor! Please check your wiring.");
    error = 1;
  }
  else {
    Serial.println("Found VEML6075 (UV) sensor");
  }
  // Air Quality init
  if (!ccs.begin()) {
    Serial.println("Failed to start Air Quality sensor! Please check your wiring.");
    error = 1;
  }
  else {
    Serial.println("Found CCS811 (Air Quality) sensor");
  }
  while (error) {
    ledRed.on();
    delay(1000);
    ledRed.off();
    delay(1000);
    if (digitalRead(POWER_PIN) == PRESSED_BUTTON_LEVEL) {
      state = LIGHT_SLEEP;
      return;
    }
  }
  
  if (firstBoot) {
    while (!ccs.available());
    float t = ccs.calculateTemperature();
    ccs.setTempOffset(t - 23.0);
    firstBoot = 0;
  }
  
}

void handleWarning() {
  ledRed.on();
  ms = millis();
  if (warningVibTimeout < ms) {
    warningCounter++;
    warningVibTimeout = ms + 500;
    vib.toggle();
  }
  if (warningCounter >= 10) {
    dismissWarning();
  }
}

void dismissWarning() {
  String s;
  if (values.getUVIFlag()) {
    values.warnUVI = 0;
    values.clearUVIFlag();
    s = "UVI";
  }
  else if (values.getTempFlag()) {
    values.warnTemp = 0;
    values.clearTempFlag();
    s = "Temp";
  }
  else if (values.getVOCFlag()) {
    values.warnVOC = 0;
    values.clearVOCFlag();
    s = "VOC";
  }
  else if (values.getCO2Flag()) {
    values.warnCO2 = 0;
    values.clearCO2Flag();
    s = "CO2";
  }
  Serial.print("Warning dismissed: ");
  Serial.println(s);
  warningCounter = 0;
  warningTimeout = millis() + 120000; // Reactivate after 30 s
  reactivateWarning = 1;
  ledRed.off();
  vib.off();
}
