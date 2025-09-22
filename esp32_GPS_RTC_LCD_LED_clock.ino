//#define LCD_ATTACHED
#define LCD_I2C_ATTACHED
#define LED_DISPLAY_ATTACHED
#define LIGHT_SENSOR_ATTACHED
#define LCD_SPI_ATTACHED

#include <RTClib.h>
#ifdef LED_DISPLAY_ATTACHED
#include <TM1637Display.h>
#endif
#include <Time.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <TinyGPS.h>
#ifdef LCD_ATTACHED
#include <LiquidCrystal.h>
#endif
#ifdef LCD_I2C_ATTACHED
#include <LiquidCrystal_I2C.h>
#endif


//#################################################################################
#ifdef LIGHT_SENSOR_ATTACHED
// ALS-PT19 Adafruit light sensor

#define LIGHT_SENSOR_PIN 36
#endif
//#################################################################################

#define ESP32_SCL 22 //esp32 dev default SCL GPIO
#define ESP32_SDA 21 //esp32 dev default SDA GPIO

RTC_DS3231 rtc;

//#################################################################################

#define TM1637_DIO 25
#define TM1637_CLK 26

#ifdef LED_DISPLAY_ATTACHED
//TM1637 is not I2C (bit-bang protocol), so don't put it on WIRE lib and same pins;
TM1637Display display(TM1637_CLK, TM1637_DIO);
#endif

//#################################################################################

#define MAX_TZ_offset  14 * 2 // Kiribati: Line Islands
#define MIN_TZ_offset -12 * 2 // Baker Island; Howland Island

//#################################################################################

#define TOUCH_up 32
#define TOUCH_down 13
#define TOUCH_relay 14

//#################################################################################

#define SECONDS_RELAY 12

//#################################################################################

#ifdef LCD_ATTACHED

#define LCD_BACK_LIGHT 27

#define LCD_RS 19
#define LCD_EN 23
#define LCD_D4 18
#define LCD_D5 17
#define LCD_D6 16
#define LCD_D7 15

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

#endif

//#################################################################################

#ifdef LCD_I2C_ATTACHED

LiquidCrystal_I2C i2clcd(0x27, 16, 2);

#endif

//#################################################################################

 // Define the RX and TX pins for Serial 2
#define GPS_RXD2 5
#define GPS_TXD2 4

#define GPS_BAUD 9600

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);

// TinyGPS parser object
TinyGPS gps;

//#################################################################################

// define the number of bytes you want to access
#define EEPROM_SIZE 1
// timezone is incremented in 1/2 hour intervals
#define EEPROM_TZ_LOCATION 0

//#################################################################################
// Helper functions
void saveTZ(int val) {
  Serial.println("writing : " + String(val));
  int valToSave = 100 + val;
  EEPROM.write(EEPROM_TZ_LOCATION, (uint8_t) valToSave);
  EEPROM.commit();
}

int readTZ() {
  int retval = (int) EEPROM.read(EEPROM_TZ_LOCATION) - 100;
  // Serial.println("read back : " + String(retval));
  return retval;
}

String twoDigitFormat(const uint8_t n) {
  String sn = String(n);
  if (n < 10) {
    return String("0" + sn);
  }
  return sn;
}

void writeLCD(String topLine, String bottomLine) {
#ifdef LCD_ATTACHED
  lcd.clear();
  lcd.print(topLine);
  // go to row 1 column 0, note that this is indexed at 0
  lcd.setCursor(0,1); 
  lcd.print (bottomLine);
#endif
#ifdef LCD_I2C_ATTACHED
  i2clcd.clear();
  i2clcd.print(topLine);
  // go to row 1 column 0, note that this is indexed at 0
  i2clcd.setCursor(0,1); 
  i2clcd.print (bottomLine);
#endif
}

String tzOffsetFormat(int offset) {
  String ret = String(" " + String(offset / 2) + ":");
  if (offset % 2) {
    return String(ret + "30");
  } else {
    return String(ret + "00");
  } 
}

void turnLCDBackLightOn() {
#ifdef LCD_ATTACHED
  digitalWrite(LCD_BACK_LIGHT, HIGH);
#endif
#ifdef LCD_I2C_ATTACHED
  i2clcd.backlight();
#endif 
}

void turnLCDBackLightOff() {
#ifdef LCD_ATTACHED
  digitalWrite(LCD_BACK_LIGHT, LOW);
#endif
#ifdef LCD_I2C_ATTACHED
  i2clcd.noBacklight();
#endif
}

int upPressed() {
  return (touchRead(TOUCH_up) < 50) && (touchRead(TOUCH_down) > 50) && (touchRead(TOUCH_relay) > 50);
}

int downPressed() {
  return (touchRead(TOUCH_down) < 50) && (touchRead(TOUCH_up) > 50) && (touchRead(TOUCH_relay) > 50);
}

int relayPressed() {
  return (touchRead(TOUCH_relay) < 50) && (touchRead(TOUCH_up) > 50) && (touchRead(TOUCH_down) > 50);
}

void digitalClockDisplay(int offset, bool relayEnabledLocal) {

  DateTime now = rtc.now();
  // set the Time to the latest GPS reading
  setTime(now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
  // apply TZ offset
  adjustTime(offset * SECS_PER_MIN * 30);

  String datestr = String("      " + String(year()) + "/" + twoDigitFormat(month()) + "/" + twoDigitFormat(day()));
  String ampm = " AM";
  uint8_t rawh = hour();
  uint8_t h = rawh;

  if (h > 11) {
    ampm = " PM";
    if (h > 12) {
      h -= 12;
    }
  }

  byte secdot = B00000000;
  if (second() % 2) {
    secdot = B10000000; 
    if(relayEnabledLocal) {
      digitalWrite(SECONDS_RELAY, HIGH);
    } else {
      digitalWrite(SECONDS_RELAY, LOW);
    }
  } else {
    digitalWrite(SECONDS_RELAY, LOW);
  }
#ifdef LED_DISPLAY_ATTACHED
#ifdef LIGHT_SENSOR_ATTACHED
  if (rawh < 8 || rawh > 8) {
    int lightValue = analogRead(LIGHT_SENSOR_PIN);
    Serial.print("Raw Light Reading: ");
    Serial.print(lightValue);
    if (lightValue > 2048) {
      lightValue = 2047;
    }
    Serial.print(" -> ");
    Serial.println((int)(lightValue / 256));
    display.setBrightness((int)(lightValue / 256)); // Sets the brightness level to 3
  }
#endif

  uint8_t data[] = {(uint8_t (h / 10)) ? display.encodeDigit(uint8_t (h / 10)) : B00000000, display.encodeDigit(uint8_t (h % 10)) | secdot, display.encodeDigit(uint8_t (minute() / 10)), display.encodeDigit(uint8_t (minute() % 10))};
	display.setSegments(data);
  //display.setBrightness(second() % 8); //brightness in rotation seven steps
#endif
  String timestr = String("     " + twoDigitFormat(h) + ":" + twoDigitFormat(minute()) + ":" + twoDigitFormat(second()) + ampm);
  writeLCD(datestr, timestr);
}


//#################################################################################
// Global veriables
uint32_t prevDisplay = 0; // when the digital clock was displayed
int tz_offset = 0; // timezone offset
time_t lcdBackLightOn = 0; // when the digital clock was displayed
uint32_t prevGPSSync = 0; // when the RTC was synced with GPS
bool relayEnabled = false; // operate relay when on
//#################################################################################

void setup() {

#ifdef LIGHT_SENSOR_ATTACHED
  // Light sensor ALS-PT19
  pinMode(LIGHT_SENSOR_PIN, INPUT);
#endif

#ifdef LCD_ATTACHED
  // LCD backlight
  pinMode(LCD_BACK_LIGHT, OUTPUT);
#endif

  // Seconds relay
  pinMode(SECONDS_RELAY, OUTPUT);
  
#ifdef LCD_ATTACHED
  // initialize LCD display
  lcd.begin(16, 2);
#endif

  Wire.begin(ESP32_SDA, ESP32_SCL);

#ifdef LCD_I2C_ATTACHED
  // initialize LCD display
  i2clcd.init();
#endif

  // Serial Monitor
  Serial.begin(115200);
  delay(1000);

  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RXD2, GPS_TXD2);

  // EEPROM TZ data read
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);

  // read the last TZ offset from flash memory
  tz_offset = readTZ();

#ifdef LED_DISPLAY_ATTACHED
  // Set brightness controll for the LCD controller
  display.setBrightness(7); // Sets the brightness level to 3
  display.clear();
#endif

  if (!rtc.begin()) {
    Serial.println("RTC not detected");
  } else {
    //Serial.println("RTC detected");
  }
}

void loop() {
  // Display time from RTC
  DateTime now = rtc.now();
  uint32_t nowsecs = now.secondstime();
  if (nowsecs != prevDisplay) { //update the display only if the time has changed
    prevDisplay = nowsecs;
    digitalClockDisplay(tz_offset, relayEnabled);  
  }

  // Check if timezone offset is being incremented or decremented
  int saved_tz_offset = tz_offset;
  if(upPressed()) {
    delay(100);
    lcdBackLightOn = prevDisplay;
    turnLCDBackLightOn();
    while(upPressed()) {
      delay(750);
      tz_offset += 1;
      if (tz_offset > MAX_TZ_offset) tz_offset = MIN_TZ_offset;
      writeLCD("Setting TZ:", "UTC + " + tzOffsetFormat(tz_offset));
      Serial.println("Setting TZ: UTC + " + tzOffsetFormat(tz_offset));
    }
  } else if(downPressed()) {
    delay(100);
    lcdBackLightOn = prevDisplay;
    turnLCDBackLightOn();
    while(downPressed()) {
      delay(750);
      tz_offset -= 1;
      if (tz_offset < MIN_TZ_offset) tz_offset = MAX_TZ_offset;
      writeLCD("Setting TZ:", "UTC+ " + tzOffsetFormat(tz_offset));
      Serial.println("Setting TZ: UTC+ " + tzOffsetFormat(tz_offset));
    }
  } else if(relayPressed()) {
    delay(100);
    bool previousRelayEnabled = relayEnabled;
    lcdBackLightOn = prevDisplay;
    turnLCDBackLightOn();
    while(relayPressed()) {
      delay(300);
      relayEnabled = !previousRelayEnabled;
    }
  }
  if (saved_tz_offset != tz_offset) {
    saveTZ(tz_offset);
  }
  if (prevDisplay == lcdBackLightOn) {
    lcdBackLightOn = rtc.now().secondstime();
  } else if (rtc.now().secondstime() > (lcdBackLightOn + 4)) {
    lcdBackLightOn = 0;
    turnLCDBackLightOff();
  }


  // Update RTC from GPS if needed
  while (gpsSerial.available() > 0) {
    // get the byte data from the GPS
    char gpsData = gpsSerial.read();
    // Serial.println(gpsData);
    if (gps.encode(gpsData)) {
      //processGPSData(gpsData);
      unsigned long age;
      int Year;
      byte Month, Day, Hour, Minute, Second;
      gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
      DateTime now = rtc.now();
      if ((!prevGPSSync) || ((age < 500) && (now.secondstime() >= (prevGPSSync + 300)))) {
        prevGPSSync = now.secondstime();
        // set the Time to the latest GPS reading
        Serial.println( "Setting time from gps : " + String(prevGPSSync));
        //TODO:
        //setTime(Hour, Minute, Second, Day, Month, Year);
        rtc.adjust(DateTime(Year, Month, Day, Hour, Minute, Second));
      }
    }
  }
}
