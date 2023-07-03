#include <esp32_smartdisplay.h>
#include <RTClib.h>
#include <ESP32Time.h>
#include <Timezone.h>
#include <Adafruit_BMP085.h>
#include <DHT.h>
#include <EEPROM.h>

#include <lvgl.h>
#include "ui/ui.h"

#define BERNA_FLY_CLOCK_VERSION "1.3"

#define I2C_SDA 27
#define I2C_SCL 22
#define DHT_PIN 4
#define THERMISTOR_PIN 35
#define DHTTYPE DHT21
#define EEPROM_SIZE 512

#define BMP_READ_INTERVAL 3000UL // 3 Seconds
#define THERMISTOR_READ_INTERVAL 3000UL // 3 Seconds

// I2C Setup
TwoWire I2CBME = TwoWire(0);

// Real Time Clock
RTC_DS3231 rtc;
ESP32Time esp32Rtc(3600);
// Italy Time Zone
TimeChangeRule myDST = {"CEST", Last, Sun, Mar, 2, 120}; //Daylight time = UTC - 2 hours
TimeChangeRule mySTD = {"CET", Last, Sun, Nov, 2, 60}; //Standard time = UTC - 1 hour
Timezone myTZ(myDST, mySTD);
TimeChangeRule *tcr;

// Pressure sensor (inside)
Adafruit_BMP085 bmp;

// Temperature sensor thermistor (inside)
double adcMax = 4095.0; // ADC resolution 12-bit (0-4095)
double R1 = 46400.0;    // Resistnce of the known resistor
double Ro = 37000.0;    // Resistance of Thermistor at 25 degree Celsius
double Beta = 3950.0;   // Beta value
double To = 298.15;     // Temperature in Kelvin for 25 degree Celsius

// Temperature sensor (outside)
DHT outsideTempSensor(DHT_PIN, DHTTYPE);

// System variables
time_t startRunTime;
bool tftInit, i2cserialInit, rtcInit, bmpInit, dhtInit, showRuntimeSeconds, showPressure;
int fuelTankChangeAlertMin, fuelTankChangeAlertMessageBoxCounter;
float insideTemperatureCorrection, outsideTemperatureCorrection;
unsigned long bmpPollingTarget = 0UL, thermistorPollingTarget = 0UL;
lv_obj_t * fuelTankChangeAlertMessageBox;

void setup()
{
  // Initialize serial
  Serial.begin(9600);
  Serial.println("Berna Fly Clock and Thermometer System Initialization...");

  // Initialize Display
  smartdisplay_init();

  // Load UI Data
  char errorBuffer[250] = "";
  ui_init();
  lv_label_set_text(ui_VersionLabel, "Version " BERNA_FLY_CLOCK_VERSION "\n" __DATE__);
  lv_timer_handler();

  EEPROM.begin(EEPROM_SIZE); // Initialize EEPROM
  i2cserialInit = I2CBME.begin(I2C_SDA, I2C_SCL, 100000); // Initialize I2C
  rtcInit = rtc.begin(&I2CBME); // Initialize RTC
  bmpInit = bmp.begin(BMP085_ULTRAHIGHRES, &I2CBME); // Initialize BMP Pressure Sensor
  outsideTempSensor.begin(); // Initialize DHT Temperature Sensor
  delay(700);
  dhtInit = !(isnan(outsideTempSensor.readHumidity()) || isnan(outsideTempSensor.readTemperature()));

  // Only for set RTC DateTime - (1m 30s for compile time and upload)
  //rtc.adjust(DateTime(2023, 6, 18, 13, 18, 25));

  // Check if I2C Connection is ok
  if (i2cserialInit) {
    Serial.println("I2C init OK");
  } else {
    Serial.println("I2C init ERROR!");
    sprintf(errorBuffer, "%sI2C init ERROR!\n", errorBuffer);
  }

  // Check if RTC Connection is ok
  // Save start run time for stopwatch
  if (rtcInit) {
    Serial.println("RTC init OK");
    startRunTime = rtc.now().unixtime();
  } else {
    Serial.println("RTC init ERROR!");
    sprintf(errorBuffer, "%sRTC init ERROR!\n", errorBuffer);
    // Use internal RTC for stopwatch
    esp32Rtc.setTime(00, 30, 7, 26, 2, 1993);
    startRunTime = esp32Rtc.getEpoch();
  }

  // Check if BMP (Pressure) Connection is ok
  if (bmpInit) {
    Serial.println("BMP init OK");
  } else{
    Serial.println("BMP init ERROR!");
    sprintf(errorBuffer, "%sBMP init ERROR!\n", errorBuffer);
  }

  // Check if DHT (Outside Temperature Sensor) Connection is ok
  if (dhtInit) {
    Serial.println("DHT init OK");
  } else {
    Serial.println("DHT init ERROR!");
    sprintf(errorBuffer, "%sDHT init ERROR!\n", errorBuffer);
  }
  
  // Initialize Thermistor for inside temperature
  pinMode(THERMISTOR_PIN, INPUT);

  // Load Fuel Tank Change Alert Setting from EEPROM address 0
  fuelTankChangeAlertMessageBoxCounter = 0;
  fuelTankChangeAlertMin = EEPROM.read(0);
  lv_dropdown_set_selected(ui_FuelTankChangeAlertDropdown, fuelTankChangeAlertMin);
  
   // Load Runtime Seconds Setting from EEPROM address 1
  showRuntimeSeconds = EEPROM.read(1) == 1;
  if(showRuntimeSeconds)
    lv_obj_add_state(ui_ShowRunTimeSecondsSwitch, LV_STATE_CHECKED);
  else
    lv_obj_clear_state(ui_ShowRunTimeSecondsSwitch, LV_STATE_CHECKED);
  
  // Load Internal Temperature Correction Setting from EEPROM address 10
  insideTemperatureCorrection = 0;
  insideTemperatureCorrection = EEPROM.readFloat(10);
  lv_dropdown_set_selected(ui_AdjustInsideTempDropdown, (50 + (insideTemperatureCorrection*10)));

  // Load External Temperature Correction Setting from EEPROM address 20
  outsideTemperatureCorrection = 0;
  outsideTemperatureCorrection = EEPROM.readFloat(20);
  lv_dropdown_set_selected(ui_AdjustOutsideTempDropdown, (50 + (outsideTemperatureCorrection*10)));

  // Set to show pressure value as default
  showPressure = true;

  Serial.println("Initialization complete!");

  // Show error
  if(strlen(errorBuffer) != 0) {
    lv_obj_set_style_text_color(ui_VersionLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text(ui_VersionLabel, errorBuffer);
    lv_timer_handler();
  }

  // Load main screen after splash
  delay(3500);
  lv_disp_load_scr(ui_Main);
}

void updateDatimeFromRTC() {
  char buf[25];

  //Get current date
  DateTime now = rtc.now();

  // Convert to local timezone
  time_t local = myTZ.toLocal(now.unixtime(), &tcr);

  // Print date
  sprintf(buf,"%02d/%02d/%02d", day(local), month(local), year(local)%100);
  lv_label_set_text(ui_DateText, buf);

  // Print time
  sprintf(buf,"%02d:%02d",hour(local),minute(local));
  lv_label_set_text(ui_HourText, buf);
}

void updatePressure() {
  char status, buf[25];
  double P = -1;

  P = (double) bmp.readSealevelPressure()/100;

  // Update pressure
  if (showPressure) {
    sprintf(buf,"%0.1f hPa", P);
    lv_label_set_text(ui_PressureText, buf);
  }
}

float thermistorAverageTemperature(float currentValueFromThermistor) {
  const int qtyReads = 25;
  static bool fistExecution = true;
  static float readings[qtyReads];
  static int readIndex = 0;
  static float total = 0;
  float average = 0;

  // If is the first execution initialize array
  if (fistExecution == true) {
    // Fill the entire array with current value
    for (int i = 0; i < qtyReads; i++) { 
      readings[i] = currentValueFromThermistor;
    }
    total = currentValueFromThermistor * qtyReads; // Average value at first execution
    fistExecution = false;
  }
  
  // Remove last value readed from the array
  total = total - readings[readIndex];

  // Set current value to active index
  readings[readIndex] = currentValueFromThermistor;

  // Add the current value readed from thermistor to total
  total = total + readings[readIndex];

  // Increment index
  readIndex++;

  // Reset index to 0 if the end of the array is reached
  if (readIndex >= qtyReads)
    readIndex = 0;

  // Calculate the average value
  average = total / qtyReads;

  return average;
}

float getInsideTemperature() {
  int Vo; // Holds the ADC Value
  float R2, tKelvin, tCelsius, tFahrenheit;
  char status, buf[25];

  Vo = analogRead(THERMISTOR_PIN);
  R2 = R1 * (adcMax / (float)Vo - 1.0); // Resistance of the Thermistor
  tKelvin = (Beta * To) / (Beta + (To * log(R2 / Ro)));
  tCelsius = tKelvin - 273.15;
  tFahrenheit = (tCelsius * 9.0) / 5.0 + 32.0;

  // Update average read from thermistor and return
  return thermistorAverageTemperature(tCelsius);
}

void updateInsideTemperature() {
  char buf[25];
  float tCelsius = getInsideTemperature() + insideTemperatureCorrection;

  // Update inside temperature
  if(tCelsius >= 0) {
    sprintf(buf,"+%0.1f °C",tCelsius);
  }
  else {
    sprintf(buf,"%0.1f °C",tCelsius);
  }
  lv_label_set_text(ui_InTempText, buf);
}

void updateOutsideTemperature() {
  char buf[25];

  double H = (double) outsideTempSensor.readHumidity();
  double T = (double) outsideTempSensor.readTemperature() + outsideTemperatureCorrection;

  // Update outside teperature
  if (T <= 3.5)
    lv_obj_set_style_bg_color(ui_OutTempText, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  else
    lv_obj_set_style_bg_color(ui_OutTempText, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);

  if(T >= 0) {
    sprintf(buf,"+%0.1f °C",T);
  }
  else {
    sprintf(buf,"%0.1f °C",T);
  }

  lv_label_set_text(ui_OutTempText, buf);

  // Update outside humidity
  if (!showPressure) {
    sprintf(buf,"%0.1f %%",H);
    lv_label_set_text(ui_PressureText, buf);
  }
}

void updateRunTime() {
  // Calculate run time
  time_t runTime;

  // Chech if external RTC is available, otherwise use internal ESP32 RTC
  if(rtcInit)
    runTime = rtc.now().unixtime() - startRunTime;
  else
    runTime = esp32Rtc.getEpoch() - startRunTime;
  //time_t runTime = rtc.now().unixtime() - DateTime(2023, 5, 27, 15, 10, 30).unixtime(); // TEST ONLY

  char buf[25];
  if (showRuntimeSeconds)
    sprintf(buf,"%02dh %02dm %02ds", hour(runTime), minute(runTime), second(runTime));
  else
    sprintf(buf,"%02dh %02dm", hour(runTime), minute(runTime));

  lv_label_set_text(ui_RuntimeText, buf);
}

static void FuelTankChangeAlertMessageBoxEventCb(lv_event_t * e)
{
  // Close Message Box, reset backlight and temp variables
  smartdisplay_tft_set_backlight((uint16_t) lv_slider_get_value(ui_BacklightSlider));
  fuelTankChangeAlertMessageBoxCounter = 0;
  lv_msgbox_close(fuelTankChangeAlertMessageBox);
}

void checkFuelTankChangeAlert() {
  // Calculate run time
  time_t runTime;

  // Check if external RTC is available, otherwise use internal ESP32 RTC
  if(rtcInit)
    runTime = rtc.now().unixtime() - startRunTime;
  else
    runTime = esp32Rtc.getEpoch() - startRunTime;

  if (!fuelTankChangeAlertMessageBoxCounter) {
    // Check every x min set
    if (runTime > 0 && runTime%(fuelTankChangeAlertMin*60) == 0) {
      char messageBuffer[50];
      fuelTankChangeAlertMessageBoxCounter = runTime;
      smartdisplay_tft_set_backlight(254);
      
      static const char * btns[] = {"Ok", ""};
      sprintf(messageBuffer, LV_SYMBOL_SHUFFLE " Switch fuel tank!\n%d Minutes are over", fuelTankChangeAlertMin);
      fuelTankChangeAlertMessageBox = lv_msgbox_create(NULL, NULL, messageBuffer, btns, false);
      lv_obj_add_event_cb(fuelTankChangeAlertMessageBox, FuelTankChangeAlertMessageBoxEventCb, LV_EVENT_VALUE_CHANGED, NULL);
      lv_obj_center(fuelTankChangeAlertMessageBox);
      lv_obj_set_style_text_font(fuelTankChangeAlertMessageBox, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_border_color(fuelTankChangeAlertMessageBox, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_bg_color(fuelTankChangeAlertMessageBox, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_bg_color(lv_msgbox_get_btns(fuelTankChangeAlertMessageBox), lv_color_hex(0x2095F6), LV_PART_ITEMS | LV_STATE_DEFAULT);
      lv_obj_set_size(lv_msgbox_get_btns(fuelTankChangeAlertMessageBox), 220, 50);
    }
  } else {
    if (runTime%2 == 0) {
      lv_obj_set_style_border_color(fuelTankChangeAlertMessageBox, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_bg_color(fuelTankChangeAlertMessageBox, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_bg_color(lv_msgbox_get_btns(fuelTankChangeAlertMessageBox), lv_color_hex(0xFF0000), LV_PART_ITEMS | LV_STATE_DEFAULT);
    } else {
      lv_obj_set_style_border_color(fuelTankChangeAlertMessageBox, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_bg_color(fuelTankChangeAlertMessageBox, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_bg_color(lv_msgbox_get_btns(fuelTankChangeAlertMessageBox), lv_color_hex(0x2095F6), LV_PART_ITEMS | LV_STATE_DEFAULT);
    }

    // Auto Close Fuel Tank Change Alert MessageBox after 120 seconds
    if(runTime - fuelTankChangeAlertMessageBoxCounter > 120) {
      FuelTankChangeAlertMessageBoxEventCb(NULL);
    }
  }
}

void loop()
{
  if(rtcInit)
    updateDatimeFromRTC();

  if(bmpInit)
    // Read value from BMP sensor every BMP_READ_INTERVAL or at startup
    if ((millis () - bmpPollingTarget >= BMP_READ_INTERVAL) || bmpPollingTarget == 0) {
      bmpPollingTarget = millis ();
      updatePressure();
    }
  
  // Read value from THERMISTOR sensor every THERMISTOR_READ_INTERVAL or at startup
  if ((millis () - thermistorPollingTarget >= THERMISTOR_READ_INTERVAL) || thermistorPollingTarget == 0) {
    thermistorPollingTarget = millis ();
    updateInsideTemperature();
  }
  getInsideTemperature();

  if(dhtInit)
    updateOutsideTemperature();
  
  updateRunTime();

  if (fuelTankChangeAlertMin)
    checkFuelTankChangeAlert();

  lv_timer_handler();
}
