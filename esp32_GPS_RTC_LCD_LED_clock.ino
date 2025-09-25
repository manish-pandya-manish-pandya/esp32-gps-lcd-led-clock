#define LCD_I2C_ATTACHED
#define LED_DISPLAY_ATTACHED
#define LIGHT_SENSOR_ATTACHED
#define RTC_ATTACHED
#define LCD_ILI9488_SPI_ATTACHED

#ifdef RTC_ATTACHED
#include <RTClib.h>
#endif

#ifdef LED_DISPLAY_ATTACHED
#include <TM1637Display.h>
#endif

#include <Time.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <TinyGPS.h>

#ifdef LCD_I2C_ATTACHED
#include <LiquidCrystal_I2C.h>
#endif

#ifdef LCD_ILI9488_SPI_ATTACHED
#include <TFT_eSPI.h>

#include <SD.h>
#include <sd_defines.h>
#include <sd_diskio.h>
#include <SPI.h>

#include <FS.h>
#include <FSImpl.h>
#include <vfs_api.h>

#include <TJpg_Decoder.h>
#endif

#ifdef LCD_ILI9488_SPI_ATTACHED
//#################################################################################
// MUST match in Setup21_ILI9488.h
#define TFT_MISO 19 // (leave TFT SDO disconnected if other SPI devices share MISO)
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15  // Chip select control pin
#define TFT_DC    2  // Data Command control pin
#define TFT_RST  -1  // Reset pin (could connect to RST pin)

//#define TOUCH_CS 3 // could not get the touch sensor to work

#define SD_SCS 4
#define SPI_SD_FREQUENCY 40000000 // 40MHz

TFT_eSPI tft = TFT_eSPI();
//#################################################################################
#endif



//#################################################################################
#ifdef LIGHT_SENSOR_ATTACHED
// ALS-PT19 Adafruit light sensor

#define LIGHT_SENSOR_PIN 36
#endif
//#################################################################################

#define ESP32_SCL 22 //esp32 dev default SCL GPIO
#define ESP32_SDA 21 //esp32 dev default SDA GPIO

#ifdef RTC_ATTACHED
RTC_DS3231 rtc;
#endif

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
#ifdef LCD_I2C_ATTACHED
    i2clcd.backlight();
#endif 
}

void turnLCDBackLightOff() {
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
#ifdef RTC_ATTACHED
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
    int lightValue = analogRead(LIGHT_SENSOR_PIN);
    Serial.print("Raw Light Reading: ");
    Serial.print(lightValue);
    if (lightValue > 79) {
        lightValue = 79;
    }
    Serial.print(" -> ");
    Serial.println((int)(lightValue / 10));
    display.setBrightness((int)(lightValue / 10)); // Sets the brightness level to 3
#endif

    uint8_t data[] = {
        (uint8_t (h / 10)) ? display.encodeDigit(uint8_t (h / 10)) : B00000000,
        display.encodeDigit(uint8_t (h % 10)) | secdot,
        display.encodeDigit(uint8_t (minute() / 10)),
        display.encodeDigit(uint8_t (minute() % 10))
    };
	display.setSegments(data);
    //display.setBrightness(second() % 8); //brightness in rotation seven steps
#endif
    String timestr = String(
        "     " + twoDigitFormat(h) +
        ":" + twoDigitFormat(minute()) +
        ":" + twoDigitFormat(second()) + ampm
    );
    writeLCD(datestr, timestr);
#endif
}

#ifdef LCD_ILI9488_SPI_ATTACHED

String file_list[40];
int file_num = 0;
int file_index = 0;

void setupLCDandSD(void) {
    pinMode(SD_SCS, OUTPUT);
    tft.init();
    tft.fillScreen(random(0xFFFF));
    tft.setSwapBytes(true); // We need to swap the colour bytes (endianess)
    yield();


    //SD(HSPI) init
    if (!SD.begin(SD_SCS, tft.getSPIinstance(), SPI_SD_FREQUENCY)) {
        Serial.println("Card Mount Failed");
        while (1) delay(0);
    } else {
        Serial.println("Card Mount Successeded");
    }
    //sd_test();
    file_num = get_pic_list(SD, "/", 0, file_list);
    Serial.print("jpg file count:");
    Serial.println(file_num);
    Serial.println("All jpg:");
    for (int i = 0; i < file_num; i++) {
        Serial.println(file_list[i]);
    }
    //SPI_OFF_SD;

    // The jpeg image can be scaled by a factor of 1, 2, 4, or 8
    TJpgDec.setJpgScale(1);

    // The decoder must be given the exact name of the rendering function above
    TJpgDec.setCallback(tft_output);
}

// This next function will be called during decoding of the jpeg file to
// render each block to the TFT. If you use a different TFT library
// you will need to adapt this function to suit.
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
     // Stop further decoding as image is running off bottom of screen
    if ( y >= tft.height() ) {
        Serial.println ("image is larger than viewport");
        return 0;
    }

    // This function will clip the image block rendering automatically at the TFT boundaries
    tft.pushImage(x, y, w, h, bitmap);

    // Return 1 to decode next block
    return 1;
}

//Gets all image files in the SD card root directory
int get_pic_list(fs::FS &fs, const char *dirname, uint8_t levels, String wavlist[30]) {
    Serial.printf("Listing directory: %s\n", dirname);
    int i = 0;

    File root = fs.open(dirname);
    if (!root) {
        Serial.println("Failed to open directory");
        return i;
    }
    if (!root.isDirectory()) {
        Serial.println("Not a directory");
        return i;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
        } else {
            String temp = file.name();
            String lowerTemp = file.name();
            lowerTemp.toLowerCase();
            if (lowerTemp.endsWith(".jpg") || lowerTemp.endsWith(".jpeg")) {
                wavlist[i] = temp;
                i++;
            }
        }
        file = root.openNextFile();
    }
    return i;
}
#endif



//#################################################################################
// Global variables
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

    // Seconds relay
    pinMode(SECONDS_RELAY, OUTPUT);

#if defined(LCD_I2C_ATTACHED) || defined(LED_DISPLAY_ATTACHED)
    Wire.begin(ESP32_SDA, ESP32_SCL);
#endif

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

#ifdef RTC_ATTACHED
    if (!rtc.begin()) {
        Serial.println("RTC not detected");
    } else {
        //Serial.println("RTC detected");
    }
#endif

#ifdef LCD_ILI9488_SPI_ATTACHED
    setupLCDandSD();
#endif
}


long int runtime = millis();
int flag = 1;


void loop() {
    if ((millis() - runtime > 10000) || flag == 1) {
        Serial.print("Displaying -> ");
        Serial.println(file_list[file_index].c_str());
        String filename = String("/") + file_list[file_index].c_str();
        TJpgDec.drawSdJpg(0, 0, filename);
        file_index++;
        if (file_index >= file_num) {
            file_index = 0;
        }
        runtime = millis();
        flag = 0;
    }


#ifdef RTC_ATTACHED
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
#endif

}
