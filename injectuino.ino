#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <TinyGPS.h>

#include "config.h"
#include "injection.c"

#ifdef USE_FAT16
#include <SPI.h>
#include <SdCard.h>
#include <Fat16.h>
#else
#include <SD.h>
#endif

// -- GPS ---------------------
TinyGPS gps;
unsigned long fix_age = TinyGPS::GPS_INVALID_AGE;
float lat = 0.0, lon = 0.0;
float curSpeed = 0.0;
char fix_count = 0;
int year = 1987;
byte month = 7;
byte day = 24;
byte hour = 0;
byte minute = 0;
byte second = 0;

// -- Display -----------------
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
byte oldButton = 0;
char mode = 0;

// -- SD Card -----------------
boolean sdEnabled = false;
#ifdef USE_FAT16
SdCard card;
#endif
struct daily {
  unsigned long distance;
  float liters;
} daily = {0, 0.0};

// -- Injection ---------------
int rpm = 0;
float duty = 0.0;
float consPerHour = 0.0;
float instantCons = 0.0;
float voltage = 0.0;
float lastLiters = 0.0;
float dailyCons = 10.0;

// -- Timing ------------------
unsigned long lastRefreshTime = 0;
byte refreshStep = 0;

// -- Configuration -----------
int eepromOffset = 0;
struct MyConfig {
  // Whole traveled distance, in meters
  unsigned long distanceM;
  // Last latitude and longitude
  float lastLat, lastLon;
  // The flow of the injectors, in cm3 per minute
  float injectorFlow;
  // Number of time EEPROM has been written
  int writeCount;
  // PWM value for backlight
  int backlight;
} configuration;

/* Save configuration to EEPROM */
void backup(boolean stopBacklight) {
  unsigned long start, end;
  int realOffset = 1;
  if (stopBacklight) {
    // Save power by shutting down LCD backlight
    digitalWrite(BACKLIGHT_PIN, LOW);
  } else {
    /*
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Saving...");
    */
    start = millis();
  }
  realOffset += eepromOffset * sizeof(MyConfig);
  configuration.writeCount += 1;
  EEPROM_writeAnything(realOffset, configuration);
  if (!stopBacklight) {
    end = millis();
    lcd.setCursor(0, 0);
    lcd.print("Saved off. ");
    lcd.print(eepromOffset);
    /*
    lcd.setCursor(0, 1);
    lcd.print(end - start);
    lcd.print("ms");
    */
    delay(1000);
  }
}

/* ISR called when pin 2 falls down */
void backupIsr() {
  backup(true);
}

/* Load configuration from EEPROM */
void load() {
  int realOffset = 1;
  eepromOffset = EEPROM.read(0);
  realOffset += eepromOffset * sizeof(MyConfig);
  EEPROM_readAnything(realOffset, configuration);
  // Save configuration at new place now
  if (configuration.writeCount >= 1000) {
    cli();
    eepromOffset += 1;
    EEPROM.write(0, eepromOffset);
    configuration.writeCount = 0;
    backup(false);
    sei();
  }
}

void readButtons() {
  byte button = 1 << (analogRead(0) >> 7);
  button = button & ~oldButton;
  switch (button) {
    case BTN_RIGHT:
      switch (mode) {
        case MODE_DISTANCE:
          configuration.distanceM++;
          break;
        case MODE_BACKLIGHT:
          configuration.backlight += 4;
          break;
        case MODE_INJFLOW:
          configuration.injectorFlow += 10.0;
          break;
      }
      break;
    case BTN_UP:
      mode = max(mode - 1, 0);
      lcd.clear();
      break;
    case BTN_DOWN:
      mode = min(mode + 1, MODE_INJFLOW);
      lcd.clear();
      break;
    case BTN_LEFT:
      switch (mode) {
        case MODE_DISTANCE:
          configuration.distanceM--;
          break;
        case MODE_BACKLIGHT:
          configuration.backlight -= 4;
          break;
        case MODE_INJFLOW:
          configuration.injectorFlow -= 10.0;
          break;
      }
      break;
    case BTN_SELECT:
      if (mode == MODE_DAILY)
        newDaily();
      else
        backup(false);
      break;
    default:
      break;
  }
  oldButton = button;
}

void dateTime(uint16_t* date, uint16_t* time) {
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year, month, day);
  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour, minute, second);
}

void readVoltage() {
  // val / resolution * vref * divisor
  //  x  /    1024    *  5   *  3
  voltage = analogRead(VOLTAGE_PIN) * 0.0146484375;
}

void writeLog() {
  if (!sdEnabled)
    return;
#ifdef USE_FAT16
  Fat16 dataFile;
  if (dataFile.open(FILENAME_LOG, O_CREAT|O_WRITE|O_APPEND)) {
#else
  File dataFile = SD.open(FILENAME_LOG, FILE_WRITE);
  if (dataFile) {
#endif
    dataFile.print(year);
    dataFile.write('-');
    if (month < 10)
      dataFile.write('0');
    dataFile.print(month);
    dataFile.write('-');
    if (day < 10)
      dataFile.write('0');
    dataFile.print(day);
    dataFile.write('T');
    if (hour < 10)
      dataFile.write('0');
    dataFile.print(hour);
    dataFile.write(':');
    if (minute < 10)
      dataFile.write('0');
    dataFile.print(minute);
    dataFile.write(':');
    if (second < 10)
      dataFile.write('0');
    dataFile.print(second);
    dataFile.write('Z');
    dataFile.write(',');
    dataFile.print(long(lat * 1000000L));
    dataFile.write(',');
    dataFile.print(long(lon * 1000000L));
    dataFile.write(',');
    dataFile.print(int(curSpeed));
    dataFile.write(',');
    dataFile.print(int(duty * 100));
    dataFile.write(',');
    dataFile.print(rpm);
    dataFile.write(',');
    dataFile.print(configuration.distanceM);
    dataFile.write('\n');
    dataFile.close();
  }
}

void writeDaily(const char *fileName, bool append) {
  if (!sdEnabled)
    return;
#ifdef USE_FAT16
  Fat16 dataFile;
  if (dataFile.open(FILENAME_DAILY, O_CREAT|O_WRITE)) {
    if (!append)
      dataFile.seekSet(0);
#else
  File dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    if (!append)
      dataFile.seek(0);
#endif
    dataFile.write((byte*)&daily, sizeof(daily));
    dataFile.close();
  }
}

void readDaily() {
  if (!sdEnabled)
    return;
#ifdef USE_FAT16
  Fat16 dataFile;
  if (dataFile.open(FILENAME_LOG, O_CREAT|O_WRITE|O_APPEND)) {
#else
  File dataFile = SD.open(FILENAME_DAILY, FILE_READ);
  if (dataFile) {
#endif
    int val = 0;
    byte *ptr = (byte*)&daily;
    while ((val = dataFile.read()) >= 0) {
      *ptr = (byte)val;
      ptr++;
    }
    dataFile.close();
  }
}

void newDaily() {
  // Write daily data to another file
  writeDaily(FILENAME_OLDDAILY, true);
  // Reset the main file
#ifdef USE_FAT16
  Fat16::remove(FILENAME_DAILY);
#else
  SD.remove(FILENAME_DAILY);
#endif
  daily.distance = 0;
  daily.liters = 0.0;
}

void readGps() {
  int incomingByte = 0;

  while (Serial.available() > 0) {
      incomingByte = Serial.read();

      if (gps.encode(incomingByte)) {
        gps.f_get_position(&lat, &lon, &fix_age);
        curSpeed = gps.f_speed_kmph();
        fix_count = (fix_count + 1) % 10;
      }
  }

  if (fix_count == 0) {
    float delta = TinyGPS::distance_between(configuration.lastLat,
        configuration.lastLon, lat, lon);
    if (delta >= 5.0) {
      configuration.lastLat = lat;
      configuration.lastLon = lon;
      configuration.distanceM += delta;
      daily.distance += delta;
      writeLog();
      writeDaily(FILENAME_DAILY, false);
    }
  }
}

void padPrintFloat(float num, float max_, char precision) {
  int num2 = abs(num);
  max_ /= 10.0;
  if (num < 0.0) {
    max_ /= 100.0;
  }
  while (num2 <= max_ && 0.999 <= max_) {
    lcd.write(' ');
    max_ /= 10.0;
  }
  lcd.print(num, precision);
}

void padPrintLong(long num, long max_, char pad) {
  max_ /= 10;
  while (num <= max_ && 0 < max_) {
    lcd.write(pad);
    max_ /= 10;
  }
  lcd.print(num);
}

void printGps() {
  lcd.setCursor(0, 1);
  if (fix_age > 5000) {
    lcd.print("No GPS ");
  } else {
    padPrintLong(long(curSpeed), 999L, ' ');
    lcd.print("kmh ");
  }
}

void printConsumption() {
  lcd.setCursor(7, 1);
  if (curSpeed < 10.1) {
    lcd.write(' ');
    padPrintFloat(consPerHour, 99.9, 1);
    lcd.print("L/h ");
  } else {
    padPrintFloat(instantCons, 999.9, 1);
    lcd.print("L100");
  }
}

void printMenu() {
  lcd.setCursor(0, 0);
  switch (mode) {
    case MODE_RPM_DUTY:
    {
      padPrintLong(rpm, 9999, ' ');
      lcd.print("RPM   ");
      padPrintFloat(duty * 100.0, 999.0, 1);
      lcd.print("%");
      break;
    }
    case MODE_DAILY:
    {
      padPrintLong(daily.distance, 9999999L, ' ');
      lcd.print("m ");
      padPrintFloat(daily.liters, 99.0, 3);
      lcd.print("L ");
      break;
    }
    case MODE_DISTANCE:
    {
      padPrintLong((configuration.distanceM)/1000, 999999L, ' ');
      lcd.print("km ");
      padPrintFloat(voltage, 99.0, 1);
//      lcd.setCursor(15, 0);
      lcd.write('V');
      break;
    }
    case MODE_CONS_DTE:
    {
      lcd.print("DTE:");
      float dte = (68.0 - daily.liters) * 100.0;
      if (curSpeed > 10.0)
        dte /= (dailyCons * 9 + instantCons) / 10.0;
      else
        dte /= dailyCons;
      padPrintLong(long(dte), 9999, ' ');
      lcd.print("km ");
      padPrintFloat(dailyCons, 99.0, 1);
      lcd.write('L');
      break;
    }
    case MODE_POSITION:
    {
      padPrintFloat(lat, 99.9, 4);
      lcd.setCursor(8, 0);
      padPrintFloat(lon, 999.9, 4);
      break;
    }
    case MODE_BACKLIGHT:
    {
      configuration.backlight = constrain(configuration.backlight, 0, 255);
      analogWrite (BACKLIGHT_PIN, configuration.backlight);
      lcd.print("Backlight: ");
      padPrintLong(configuration.backlight, 999, ' ');
      break;
    }
    case MODE_INJFLOW:
    {
      lcd.print("Inj flow: ");
      padPrintFloat(configuration.injectorFlow, 9999.9, 1);
      break;
    }
    case MODE_LOGGING:
    {
      if (sdEnabled) {
        unsigned long fileSize = 0;
#ifdef USE_FAT16
        Fat16 dataFile;
        if (dataFile.open(FILENAME_LOG, O_CREAT|O_WRITE|O_APPEND)) {
          fileSize = dataFile.fileSize();
#else
        File dataFile = SD.open(FILENAME_LOG, FILE_READ);
        if (dataFile) {
          fileSize = dataFile.size();
#endif
          dataFile.close();
        }
        lcd.print("Log: ");
        lcd.print(fileSize);
        lcd.write('b');
      } else {
#ifdef USE_FAT16
        lcd.print("SD error ");
        lcd.print(int(card.errorCode));
#else
        lcd.print("No SD card");
#endif
      }
      break;
    }
    case MODE_TIME:
    {
      lcd.print("Time: ");
      padPrintLong((hour + 2) % 24, 99, '0');
      lcd.write(':');
      padPrintLong(minute, 99, '0');
      lcd.write(':');
      padPrintLong(second, 99, '0');
      break;
    }
    default:
      break;
  }
  printGps();
  printConsumption();
}

void setup() {
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.clear();
//  lcd.print("Hello, world!");

  pinMode(BACKLIGHT_PIN, OUTPUT);
//  pinMode(CHIPSELECT_PIN, OUTPUT);
  pinMode(INJ_READ_PIN, INPUT);
  pinMode(KEYPAD_PIN, INPUT_PULLUP);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(POWER_PIN, INPUT);

  // load configuration from EEPROM
  load();

  // load daily data from SD card
#ifdef USE_FAT16
  Fat16::dateTimeCallback(dateTime);
  if (card.init(0, CHIPSELECT_PIN)) {
    lcd.print("SD card ");
    lcd.print(card.cardSize());
    lcd.write('b');
    delay(500);
    sdEnabled = Fat16::init(&card);
  }
#else
  sdEnabled = SD.begin(CHIPSELECT_PIN);
  SdFile::dateTimeCallback(dateTime);
#endif
  readDaily();

  // initialize with safe defaults
  if (configuration.lastLat == 0.0) {
    configuration.lastLat = 50.6065753;
    configuration.lastLon = 3.0758584;
    configuration.distanceM = 0;
    /* 216-260 per injector, 4 injectors. */
    configuration.injectorFlow = 1040.0;
    configuration.backlight = 80;
  }
  lat = configuration.lastLat;
  lon = configuration.lastLon;

  // set LCD backlight
  analogWrite (BACKLIGHT_PIN, configuration.backlight);

  // start listening to GPS
  Serial.begin(9600);

  // attach powerdown interrupt to backup
  attachInterrupt(0, backupIsr, FALLING);
  // start measuring injection
  attachInterrupt(1, injInterrupt, CHANGE);
}

void loop() {
  boolean refreshNow = false;
  unsigned long now = millis();
  float liters;

  if (now - lastRefreshTime > 50) {
    printMenu();
    lastRefreshTime = now;
    refreshStep++;
    refreshNow = true;
  }

  injTakeSample();
  readGps();

  injCompute(configuration.injectorFlow, &duty, &consPerHour, &rpm);
  if (curSpeed > 0.0) {
    instantCons = consPerHour * 100.0 / curSpeed;
  }
  injTakeSample();

  if (refreshNow && !(refreshStep % 10)) {
    injGetTotalLiters(configuration.injectorFlow, &liters);
    daily.liters += liters - lastLiters;
    lastLiters = liters;
    if (daily.distance > 0.0)
      dailyCons = daily.liters / float(daily.distance) * 100000.0;
  }
  if (refreshNow && (refreshStep % 10) == 1) {
    readVoltage();
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, NULL, &fix_age);
  }
  if (refreshNow && (refreshStep % 2)) {
    readButtons();
  }
}

