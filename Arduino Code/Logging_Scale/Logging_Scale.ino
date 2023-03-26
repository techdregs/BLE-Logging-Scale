#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <ArduinoBLE.h>

// U8g2 Contructor and display variables
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2, /* reset=*/U8X8_PIN_NONE);
#define LCDWidth u8g2.getDisplayWidth()
#define ALIGN_CENTER(t) ((LCDWidth - (u8g2.getUTF8Width(t))) / 2)
#define ALIGN_RIGHT(t) (LCDWidth - u8g2.getUTF8Width(t))

//pins for HX711:
const int HX711_dout = 10;  //mcu > HX711 dout pin
const int HX711_sck = 8;    //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// Var for calibration and t for timer
const int calVal_eepromAdress = 0;
unsigned long t = 0;

//Buttons
#define Sleep_Button D0
#define Tare_Button D1
#define Units_Button D6

//Debounce settings for Units Button
int unitState = 0;
int unitButtonState = 0;
int lastUnitButtonState = 0;
unsigned long lastUnitDebounceTime = 0;
const int debounce = 50;

// UUIDs for scale service, and mass, buttons, and logging characteristics
BLEService scaleService("1eb9bb89-186e-4d2a-a204-346da73c061c");
BLEFloatCharacteristic massChar("5977b71a-a58c-40d2-85a4-34071043d9ca",  // this reads to the main display in app
                                BLERead | BLENotify);                    //remote clients can get notifications if this characteristic changes
BLEIntCharacteristic buttonChar("a8f2d9f3-c93a-4479-8208-7287262eacf6",  // this captures buttons pressed on the app
                                BLERead | BLEWrite | BLENotify);
BLEFloatCharacteristic loggingChar("9fdd73d8-77e8-4099-816f-a1619834c3f2",  // this sends timed data over for the data logging
                                   BLERead | BLENotify);                    //remote clients can get notifications if this characteristic changes
//Create variable to send to App for mass
float bleMass = 0.00;  // starting BLE mass value
bool logState = 0;
unsigned long lastLogTime = 0;  //var to capture log time
const int logInterval = 250;    //how long in milliseconds between log updates

void setup(void) {
  Serial.begin(57600);
  u8g2.begin();
  LoadCell.begin();

  //Buttons - Sleep Button has external pullup resistor
  pinMode(Tare_Button, INPUT_PULLUP);
  pinMode(Units_Button, INPUT_PULLUP);

  Serial.println();
  Serial.println("Starting...");

  //Load Cell Startup Sequence
  //LoadCell.setReverseOutput();  //uncomment to turn a negative output value to positive
  float calibrationValue;      // calibration value (see example file "Calibration.ino")
  calibrationValue = 2262.26;  // uncomment this if you want to set the calibration value in the sketch
#if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

  unsigned long stabilizingtime = 2000;  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_helvB08_tf);
      u8g2.drawStr(0, 20, "Timeout, check MCU>HX711 wiring and pin designations");
    } while (u8g2.nextPage());
    while (1)
      ;
  } else {
    LoadCell.setCalFactor(calibrationValue);  // set calibration value (float)
    Serial.println("Startup is complete");
    do {
      u8g2.setFont(u8g2_font_helvB08_tf);
      u8g2.drawStr(0, 20, "Load Cell Ok");
    } while (u8g2.nextPage());
  }
  //End of Loadcell Startup Sequence

  //Battery Monitoring pin - middle of voltage divider
  pinMode(A2, INPUT);  //ADC pin to read voltage

  // begin BLE initialization
  if (!BLE.begin()) {
    Serial.println("BLE failed to start!");
    do {
      u8g2.setFont(u8g2_font_helvB08_tf);
      u8g2.drawStr(0, 20, "BLE failed to start!");
    } while (u8g2.nextPage());
    while (1)
      ;
  }

  // Set BLE device name
  BLE.setLocalName("Xiao_Scale");
  BLE.setAdvertisedService(scaleService);       // add the service UUID
  scaleService.addCharacteristic(massChar);     // add the mass characteristic
  scaleService.addCharacteristic(buttonChar);   // add the buttons characteristic
  scaleService.addCharacteristic(loggingChar);  // add the logging characteristic
  BLE.addService(scaleService);                 // Add the service
  massChar.writeValue(bleMass);                 // set initial value for  mass

  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();
  Serial.println("Bluetooth® device active, waiting for connections...");
  do {
    u8g2.setFont(u8g2_font_helvB08_tf);
    u8g2.drawStr(0, 20, "Bluetooth® device active, waiting for connections...");
  } while (u8g2.nextPage());
}

void loop(void) {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  static boolean newDataReady = 0;     //Var for HX711 data check
  const int serialPrintInterval = 15;  // increase value to slow down serial print/BLE activity
  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  //Scale Read
  float newMass = 0;
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      newMass = i;

      // Serial.print("Load_cell output val: ");
      //  Serial.println(i);
      newDataReady = 0;
      t = millis();
      char weightChar[10];
      if (unitState == 0) {                  //if unitState = 0 then grams, otherwise ounzes
        dtostrf(newMass, 5, 1, weightChar);  //Convert Float to one decimal place
        strcat(weightChar, "g");
      } else {
        dtostrf(newMass / 28.3495, 5, 1, weightChar);  //Convert Float to one decimal place
        strcat(weightChar, "oz");
      }
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_helvB08_tf);
        char buffer[10];
        sprintf(buffer, "Batt: %i%%", battLevel());
        u8g2.drawStr(0, 16, buffer);
        if (central.connected()) {
          u8g2.drawStr(ALIGN_RIGHT("*BLE*"), 16, "*BLE*");
          if (newMass != bleMass) {
            bleMass = newMass;
            massChar.writeValue(bleMass);
          }
        } else {
          u8g2.drawStr(ALIGN_RIGHT("No BLE"), 16, "No BLE");
        }
        u8g2.setFont(u8g2_font_helvB24_tf);
        u8g2.drawStr(ALIGN_CENTER(weightChar), 50, weightChar);
      } while (u8g2.nextPage());
    }
  }

  if (digitalRead(Tare_Button) == LOW) {  // Check if Tare Button Pressed
    tare();
  }

  int readUnits = digitalRead(Units_Button);
  if (readUnits != lastUnitButtonState) {
    lastUnitDebounceTime = millis();
  }
  if (millis() - lastUnitDebounceTime > debounce) {
    if (readUnits != unitButtonState) {
      unitButtonState = readUnits;
      if (readUnits == LOW) {
        unitState = !unitState;
        Serial.println("Units ");
        Serial.print(unitState);
      }
    }
  }
  lastUnitButtonState = readUnits;

  if (digitalRead(Sleep_Button) == LOW) {  // Deep Sleep Routine
    Serial.println("Sleep Time.");
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_helvB08_tf);
      u8g2.drawStr(0, 16, "Sleep Time.");
    } while (u8g2.nextPage());
    esp_deep_sleep_enable_gpio_wakeup(1ULL << 2, ESP_GPIO_WAKEUP_GPIO_LOW);
    delay(1000);
    u8g2.setPowerSave(1);
    esp_deep_sleep_start();
  }

  // receive command from BT app, send '2' to initiate tare operation.
  if (buttonChar.written()) {
    int32_t k = 0;
    buttonChar.readValue(k);
    Serial.print("Received Button val: ");
    Serial.println(k);
    if (k == 2) {  // tare button pressed in app
      tare();      //tare the scale
    }
    if (k == 1) {            // start/stop loggin in app
      logState = !logState;  // flip logging state
    }
  }

  //Logging routine
  if (logState = 1) {                             // if logging is on
    if (millis() - lastLogTime >= logInterval) {  // if logInteval time has passed
      loggingChar.writeValue(bleMass);            // send new mass to logger
      lastLogTime = millis();                     // update timer for logging
    }
  }
}

int battLevel() {
  uint32_t Vbatt = 0;
  for (int i = 0; i < 16; i++) {
    Vbatt = Vbatt + analogReadMilliVolts(A2);
  }
  float Vbattf = 2 * Vbatt / 16 / 1000.0;
  int battLevel = round(-181.24 * pow(Vbattf, 3) + 2026.3 * pow(Vbattf, 2) - 7405.2 * Vbattf + 8885.8);
  //Serial.print(Vbattf, 1);
  //Serial.println(battLevel);
  return (battLevel);
}

void tare() {
  LoadCell.tareNoDelay();
  Serial.println("Tare");
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
}
