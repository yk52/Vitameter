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

// Bluetooth related
BLE_wcs ble;
std::string sent;
std::string oldSent;
std::string processed = "prodef";
std::string msg = "";


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
bool ignoreWarning = 0;


// Button related
volatile bool checkBT = 0;
volatile bool checkPW = 0;
volatile bool checkWA = 0;
volatile uint32_t pwDebounceTimer = 0;
volatile uint32_t btDebounceTimer = 0;
volatile uint32_t waDebounceTimer = 0;
volatile uint32_t btButtonPressed = 0;
volatile uint32_t pwButtonPressed = 0;
volatile uint32_t waButtonPressed = 0;

// other timers
uint32_t wakeUpTime = 0;
uint32_t warningMs = 900000;  // 15 min
uint32_t ms = 0;
uint32_t warningTimeout = 0;
uint8_t warningVibCounter = 0;
uint32_t warningVibTimeout = 0;
bool goalVib = 0;
uint8_t vibCounter = 0;
uint32_t vibTimeout = 0;
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
  pinMode(WARNING_PIN, INPUT);

  // ADC init
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(Z_PIN, INPUT);
  
  // Thresholds for sensor values init
  values.init();
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
  if (values.dataWanted_all) {
    sendDataOverUart();
  }
  checkBLE();

  // Light sleep mode if Vitameter is turned off_____________________
  if (state == LIGHT_SLEEP) {
    if (values.uvi_idx >= 1) {
      bool memFull = values.storeRAMToFlash();
      if (memFull) {
        ble.write("Memory capacity exceeded. Data overflow.\n");
        Serial.print("Memory capacity exceeded. Data overflow.\n");
      }
    }
    goLightSleep();
  }
  //__________________________________________________________________
  else if (state == SENSORS_ACTIVE) {
    handleWarning();
    takeMeasurements();
    if (ms > showTimeout) {
      showMeasurements();
      showTimeout += values.showFreq;
    }
  }
}  


//////////////////////////////////////////////////////////////////////////////////////////////////////

void sendDataOverUart(void) {
  ledBlue.on();
  values.dataWanted_all = 0;

  ble.write("Print data over Serial Port...\n");
  Serial.println(values.prepareAllData().c_str());

  /*// TODO for testing reasons
  Serial.print("co2 flash ");
  Serial.println(values.getCurrentCO2FlashIdx());
  Serial.print("voc flash ");
  Serial.println(values.getCurrentVOCFlashIdx());
  Serial.print("uv flash ");
  Serial.println(values.getCurrentUVIFlashIdx());
  */
  
  delay(5000); 
  ledBlue.off();  
}

void takeMeasurements(void) {
  bool arrayFull = 0;
  if (values.pedoEnable && ms > pedoTimeout) {
    uint16_t x = pedo.getPedo();
    // Step registered
    if (pedo.flag) {
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
    if (vibCounter > 2) {
      ble.write("Step goal achieved\n");
      goalVib = 0;
      vibCounter= 0;
      vib.off();
    }
  }

  if (ms > uvTimeout) {
    uvTimeout += values.uvFreq;
    uint8_t u = uv.readUVI();
    arrayFull = values.storeUVI(u);
  }
  
  if (ms > airTimeout) {
    ccs.readData();
    airTimeout += values.aqFreq;
    uint16_t c = ccs.geteCO2();
    uint16_t v = ccs.getTVOC();
    values.storeVOC(v);
    arrayFull = values.storeCO2(c);
  }
  if (arrayFull) {
    bool memFull = values.storeRAMToFlash();
    if (memFull) {
      ble.write("Memory capacity exceeded. Data overflow.\n");
      Serial.print("Memory capacity exceeded. Data overflow.\n");
    }
  }
}

void showMeasurements(void) {
#ifdef SHOW_BLE
    uint8_t measNr = values.uvi_idx;
    if (measNr == 0) {
      return;
    }
    msg = "";
    msg = "Measurement Nr: ";
    msg += values.getUint8AsString(measNr);
    msg += "\n";
    ble.write(msg);
    Serial.print(msg.c_str());
    msg = "CO2: ";
    msg += values.getUint16AsString(values.getLastCO2());
    msg += "\n";
    ble.write(msg);
    Serial.print(msg.c_str());
    msg = "TVOC: ";
    msg += values.getUint8AsString(values.getLastVOC());
    msg += "\n";
    ble.write(msg);
    Serial.print(msg.c_str());
    msg = "UVI: ";
    msg += values.getUint8AsString(values.getLastUVI());
    msg += "\n";
    ble.write(msg);
    Serial.print(msg.c_str());
    msg = "Steps: ";
    msg += values.getUint16AsString(values.getLastStep());
    msg += "\n";
    msg += "\n";
    ble.write(msg);
    Serial.print(msg.c_str());

    if (values.warning) {
      ble.write("Thresholds exceeded. Check your values\n");
      Serial.print("Thresholds exceeded. Check your values\n");
    }
#endif    
}

void goLightSleepTimeout(uint64_t sleepMillis) {
  esp_sleep_enable_timer_wakeup(sleepMillis * 1000);
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
}

void goLightSleep() {
  ble.write("\nEnter sleep\n");
  Serial.println("\nEnter sleep\n");
  detachInterrupt(digitalPinToInterrupt(POWER_PIN));
  detachInterrupt(digitalPinToInterrupt(BLUETOOTH_PIN));
  detachInterrupt(digitalPinToInterrupt(WARNING_PIN));
  delay(2000);
  values.clearAllWarnings();
  vib.off();
  ledGreen.off();
  ledRed.off();
  ledBlue.off();
  sensors.off();
  gpio_wakeup_enable(GPIO_NUM_36, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_39, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_23, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  wakeUp();
}

void setTimeouts() {
  ms = millis();
  pedoTimeout = PEDO_FREQ + ms;
  uvTimeout = values.uvFreq + ms;
  airTimeout = values.aqFreq + ms;
  showTimeout = ms + values.showFreq;
}

void showMemoryStatus(void) {
  msg = "";
  uint32_t sec = (ms - wakeUpTime) / 1000;
  uint8_t runningSec = sec % 60;
  uint8_t runningMin = (sec / 60) % 60;
  uint16_t runningHrs = (ms - wakeUpTime) / 3600000;
  msg = values.getUint16AsString(runningHrs);
  msg += "h ";
  msg += values.getUint8AsString(runningMin);
  msg += "min ";
  msg += values.getUint8AsString(runningSec);
  msg += "s\n";
  
  ble.write("\n---Vitameter Status--- \n\n");
  ble.write("***Running Time: ");
  ble.write(msg);
  ble.write("\n");
  Serial.println("\n---Vitameter Status--- \n");
  Serial.print("***Running Time: ");
  Serial.println(msg.c_str());
  Serial.println("\n");
  
  ble.write("***Measurement Frequencies: \n");
  Serial.println("***Measurement Frequencies:");
  
  msg = "Air Quality: Every ";
  msg += values.getUint16AsString(values.aqFreq);
  msg += " ms\n";
  Serial.println(msg.c_str());
  ble.write(msg);
  msg = "UV Index: Every ";
  msg += values.getUint16AsString(values.uvFreq);
  msg += " ms\n";
  ble.write(msg);
  Serial.println(msg.c_str());  
  msg = "Show Measurement: Every ";
  msg += values.getUint16AsString(values.showFreq);
  msg += " ms\n";
  Serial.println(msg.c_str());
  ble.write(msg);
  ble.write("\n\n");  
  ble.write("***Flash Memory: \n");
  ble.write("Air Quality Data: ");
  Serial.println("\n");  
  Serial.println("***Flash Memory:");
  Serial.print("Air Quality Data: ");
  uint16_t currIdx = values.getCurrentVOCFlashIdx();
  ble.write(values.getUint16AsString(currIdx));
  ble.write("/1000\n");
  ble.write("UVI Data: ");
  Serial.print(values.getUint16AsString(currIdx).c_str());
  Serial.println("/1000");
  currIdx = values.getCurrentUVIFlashIdx();
  ble.write(values.getUint16AsString(currIdx));
  ble.write("/1000\n");
  ble.write("\n\n");
  ble.write("\n***Dynamic Memory: \n");
  ble.write("Air Quality Data: ");
  ble.write(values.getUint16AsString((values.co2_idx)).c_str());
  ble.write("/200\n");
  ble.write("UVI Data: ");
  ble.write(values.getUint16AsString((values.uvi_idx)).c_str());
  ble.write("/200\n\n\n");
  Serial.print("UVI Data: ");
  Serial.print(values.getUint16AsString(currIdx).c_str());
  Serial.println("/1000");
  Serial.println("\n");
  Serial.println("\n***Dynamic Memory:");
  Serial.print("Air Quality Data: ");
  Serial.print(values.getUint16AsString((values.co2_idx)).c_str());
  Serial.println("/200");
  Serial.print("UVI Data: ");
  Serial.print(values.getUint16AsString((values.uvi_idx)).c_str());
  Serial.println("/200\n\n");
  ble.write("***Thresholds: \n");
  Serial.println("***Thresholds: ");
  showThresholds();
}

void showThresholds(void) {
  msg = "";
  msg = "UVI thresh: ";
  msg += values.getUint8AsString(values.getUVIThresh());
  msg += "\n";
  ble.write(msg);
  Serial.print(msg.c_str());
  msg = "CO2 thresh: ";
  msg += values.getUint16AsString(values.getCO2Thresh());
  msg += "\n";
  ble.write(msg);
  Serial.print(msg.c_str());
  msg = "VOC thresh: ";
  msg += values.getUint8AsString(values.getVOCThresh());
  msg += "\n";
  ble.write(msg);
  Serial.print(msg.c_str());
  msg = "Step Goal: ";
  msg += values.getUint16AsString(values.getStepGoal());
  msg += "\n";
  ble.write(msg);
  Serial.print(msg.c_str());
}

void wakeUp() {
  // If button still pressed after 1 second
  delay(800);
  if (digitalRead(POWER_PIN) == PRESSED_BUTTON_LEVEL) {
    ble.write("\n WAKING UP");
    Serial.println("WAKING UP!");
    state = SENSORS_ACTIVE;
    showThresholds();
    ledGreen.on();
    sensorsInit();
    values.resetSteps();
    wakeUpTime = millis();
    setTimeouts();
  } else if (digitalRead(BLUETOOTH_PIN) == PRESSED_BUTTON_LEVEL) {
    // To clear memory: fist press BT. Then press WA for at least 3 sec.
    if (digitalRead(WARNING_PIN) == PRESSED_BUTTON_LEVEL) {
      ledBlue.on();
      ledRed.on();
      ledGreen.on();
      delay(3000);
      if (digitalRead(WARNING_PIN) == PRESSED_BUTTON_LEVEL) {
        values.clearMemory = 1;
        ledRed.off();
        ledBlue.off();
        ledGreen.off();
        checkBLE();
      } else {
        ledRed.off();
        ledBlue.off();
        ledGreen.off();
        delay(1000);
      }
    } else {
      sendDataOverUart();
    }
  } else if (digitalRead(WARNING_PIN) == PRESSED_BUTTON_LEVEL) {
    ledRed.on();
    delay(300);
    ledRed.off();
    showMemoryStatus();
    state = LIGHT_SLEEP;
    return;
  } else {
    // Ignore if button is pressed less than a second
    state = LIGHT_SLEEP;
    return;
  }
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
  attachInterrupt(digitalPinToInterrupt(POWER_PIN), pwButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BLUETOOTH_PIN), btButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(WARNING_PIN), waButtonISR, FALLING);
  ble.write("Good morning!\n");
}


void checkButtonState(void) {
  ms = millis();
  
  // Check Bluetooth button. 
  if (checkBT) {
    if (ms > btButtonPressed + 1200) {
      checkBT = 0;
      if (digitalRead(BLUETOOTH_PIN) == PRESSED_BUTTON_LEVEL) {
        // If pressed: start send over UART
        ledBlue.on();
        values.dataWanted_all = 1;
      }
    } else if (ms > btButtonPressed + 500) {
      if (digitalRead(BLUETOOTH_PIN) == !PRESSED_BUTTON_LEVEL) {
        checkBT = 0;
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
    } else if (ms > pwButtonPressed + 500) {
      if (digitalRead(POWER_PIN) == !PRESSED_BUTTON_LEVEL) {
        checkPW = 0;
        ble.write("Still alive!\n");
      }
    }
  }
  
  // Check Warning Button
  if (checkWA) {
    if (ms > waButtonPressed + 1200) {
      checkWA = 0;
      if (digitalRead(WARNING_PIN) == PRESSED_BUTTON_LEVEL) {
        ledRed.on();
        if (!ignoreWarning && values.warning) {
          vib.off();
          ignoreWarning = 1;
          dismissWarning();
          ble.write("Warnings deactivated\n");
          Serial.println("Warnings deactivated");
          
        } else if (ignoreWarning) {
          ignoreWarning = 0;
          ble.write("Warnings activated\n");
          Serial.println("Warnings activated");
        }
        ledRed.off();
      }    
    } else if (ms > waButtonPressed + 500) {
      if (digitalRead(WARNING_PIN) == !PRESSED_BUTTON_LEVEL) {
        ledRed.on();
        checkWA = 0;
        showMemoryStatus();
        delay(1000);
        ledRed.off();
      }
    }
  }  
}

void pwButtonISR() {
  ms = millis();
  if (pwDebounceTimer < ms) {
    pwDebounceTimer = ms + 100;
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

void checkBLE() {
  if (values.clearMemory) {
    Serial.println("Erase Memory!");
    ble.write("Erase Memory!\n");
    values.clearMemory = 0;
    for (int i=0; i<4; i++) {
      ledBlue.on();
      delay(500);
      ledBlue.off();
      delay(500);
    }
    values.clearAllMemory();
    if (values.getCurrentCO2FlashIdx() == 0) {
      Serial.println("Successfully cleared Memory!");
    } else {
      Serial.println("Failed.");
    }
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

    if (sent != oldSent) {
      /*
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
      */
      oldSent = sent;
      ble.write(processed);
    }
  }  
}

void sensorsInit() {
  msg = "";
  bool error = 0;
  sensors.on();
  delay(100); 
  pedo.calibrate();
  // UV
  
  if (!uv.begin()) {
    Serial.println("Failed to communicate with VEML6075 UV sensor! Please check your wiring.");
    msg = "UV ";
    error = 1;
  }
  else {
    Serial.println("Found VEML6075 (UV) sensor");
  }
  
  // Air Quality init
  if (!ccs.begin()) {
    Serial.println("Failed to start Air Quality sensor! Please check your wiring.");
    if (error) {
      msg += "and";
    }
    msg += " AQ ";
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
    ble.write("Error: ");
    ble.write(msg);
    ble.write(" failed. Plug Battery in and out\n\n");
    Serial.print(msg.c_str());
    Serial.println(" failed.");
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
  ms = millis();
  if (!values.warning) {
    ledRed.off();
  } else {
    ledRed.on();
    if (warningTimeout < ms) {
      if (!ignoreWarning && (warningVibTimeout < ms)) {
        warningVibCounter++;
        warningVibTimeout = ms + 500;
        vib.toggle();
      }
      if (warningVibCounter >= 4) {
        dismissWarning();
      }    
    }
  }    
}

void dismissWarning() {
  warningVibCounter = 0;
  warningTimeout = millis() + warningMs;
  ledRed.off();
  vib.off();
}
