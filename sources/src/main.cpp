#include <esp32_smartdisplay.h>
#include <RTClib.h>
#include <ESP32Time.h>
#include <Timezone.h>
#include <Adafruit_BMP085.h>
#include <DHT.h>
#include <EEPROM.h>

#include <lvgl.h>
#include "ui/ui.h"

#define BERNA_FLY_CLOCK_VERSION "1.1"

#define I2C_SDA 27
#define I2C_SCL 22
#define DHT_PIN 4
#define DHTTYPE DHT21
#define EEPROM_SIZE 512

// To compensate the internal BMP termhal sensor decrese the temperature measured by the sensor of BMP_COMPENSATION_TEMP °C in BMP_COMPENSATION_MIN minutes
#define BMP_COMPENSATION_TEMP -3.5
#define BMP_COMPENSATION_MIN 17.5

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

// Pressure and temperature sensor (inside)
Adafruit_BMP085 bmp;

// Temperature sensor (outside)
DHT outsideTempSensor(DHT_PIN, DHTTYPE);

// System variables
time_t startRunTime;
bool tftInit, i2cserialInit, rtcInit, bmpInit, dhtInit, bmpCompensationDone, showRuntimeSeconds, showPressure;
int fuelTankChangeAlertMin, fuelTankChangeAlertMessageBoxCounter;
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
  delay(500);
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

  // Check if BMP (Inside Temperature Sensor) Connection is ok
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
  
  // Set to show pressure value as default
  showPressure = true;

  // Set bool bmpCompensationDone to false to avoid duplicate in case of millis restart from 0
  bmpCompensationDone = false;

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

double getBMPCorrectionValue() {
  if (BMP_COMPENSATION_TEMP == 0 || BMP_COMPENSATION_MIN <= 0)
    return 0;
  
  long runtimeMillis = millis();

  // If total time is not over
  if (!bmpCompensationDone && runtimeMillis <= BMP_COMPENSATION_MIN*60000) {
    double multiplierMillis = BMP_COMPENSATION_TEMP/(BMP_COMPENSATION_MIN*60000);
    return runtimeMillis * multiplierMillis;
  } else {
    bmpCompensationDone = true;
    return BMP_COMPENSATION_TEMP;
  }
}

void updateInsideTemperatureAndPressure() {
  char status, buf[25];
  double T = -1, P = -1;

  T = (double) bmp.readTemperature() + getBMPCorrectionValue();
  P = (double) bmp.readSealevelPressure()/100;

  // Update inside temperature
  if(T >= 0) {
    sprintf(buf,"+%0.1f °C",T);
  }
  else {
    sprintf(buf,"%0.1f °C",T);
  }
  lv_label_set_text(ui_InTempText, buf);
  
  // Update pressure
  if (showPressure) {
    sprintf(buf,"%0.1f hPa", P);
    lv_label_set_text(ui_PressureText, buf);
  }
}

void updateOutsideTemperature() {
  char buf[25];

  double H = (double) outsideTempSensor.readHumidity();
  double T = (double) outsideTempSensor.readTemperature();

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
    updateInsideTemperatureAndPressure();

  if(dhtInit)
    updateOutsideTemperature();
  
  updateRunTime();

  if (fuelTankChangeAlertMin)
    checkFuelTankChangeAlert();

  lv_timer_handler();
}
