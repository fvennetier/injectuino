/* vim: tw=80 ts=2 sw=2 */
#include "config.h"

#include <Wire.h>
#include <LiquidCrystal_SR2W.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <TinyGPS.h>

#include "injection.c"

#include <SPI.h>

// -- GPS ---------------------
#define PMTK_API_SET_FIX_CTL_2HZ  "$PMTK300,500,0,0,0,0*28"
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_38400 "$PMTK251,38400*27"
#define PMTK_SET_BAUD_19200 "$PMTK251,19200*22"
#define PMTK_SET_BAUD_14400 "$PMTK251,14400*29"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_Q_RELEASE "$PMTK605*31"
TinyGPS gps;
unsigned long fix_age = TinyGPS::GPS_INVALID_AGE;

float curSpeed = 0.0;
float maxSpeed = 0.0;
int year = 1987;
byte month = 7;
byte day = 24;
byte hour = 0;
byte minute = 0;
byte second = 0;

// -- Display -----------------
//                      addr,en,rw,rs,d4,d5,d6,d7,bl,blpol
//LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
LiquidCrystal_SR2W lcd(A4, A5);

byte oldButton = 0;
byte resetAsked = 0;
char mode = 0;

byte k10[8] = {
  0b10010,
  0b10101,
  0b10101,
  0b10010,
  0b00100,
  0b00101,
  0b00110,
  0b00101
};
byte m0[8] = {
  0b01000,
  0b10100,
  0b10100,
  0b01000,
  0b00000,
  0b11010,
  0b10101,
  0b10001
};

// -- Injection ---------------
short rpm = 0;
short maxRpm = 0;
short duty = 0;
short maxDuty = 0;
short maxDutyRpm = 0;
short consPerHour = 0;
short instantCons = 0;
short voltage = 0;
short tripCons = 100;

// -- Timing ------------------
unsigned long lastRefreshTime = 0;
byte injRefreshMod = 4;
byte refreshStep = 0;
byte backupTimer = 0;

unsigned long distAtStart = 0;
float litersAtStart = 0;

// -- Configuration -----------
byte eepromOffset = 0;
struct PersistentData {
  // Whole traveled distance, in meters
  unsigned long distTot;
  // Trip distance, in meters
  unsigned long distTrip;
  // Trip liters
  float liters;
  // Last latitude and longitude
  float lastLat, lastLon;
  // Number of times EEPROM has been written
  int writeCount;
  // PWM value for backlight
  int backlight;
  // Padding bytes to align on 32 bytes (and allow evolutions)
  char padding[8];
} pData;

void dumpEeprom() {
  for (int16_t offset = 0; offset < 1024; offset++) {
    byte val = EEPROM.read(offset);
    //Serial.print(val, HEX);
    Serial.write(val);
    lcd.setCursor(0, 0);
    lcd.print(offset);
    delay(1);
  }
  lcd.setCursor(0, 0);
  lcd.print(F("Done!"));
  delay(1000);
}

/* Save configuration to EEPROM */
void backup(boolean noDisplay) {
  unsigned long start, end;
  int realOffset = 1;
  realOffset += eepromOffset * sizeof(PersistentData);
  pData.writeCount += 1;
  EEPROM_writeAnything(realOffset, pData);
  if (!noDisplay) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("off "));
    lcd.print(eepromOffset);
    lcd.setCursor(0, 1);
    lcd.print(F("wc "));
    lcd.print(pData.writeCount);

    delay(2000);
  }
}

/* ISR called when pin 2 falls down */
void backupIsr() {
  backup(true);
  digitalWrite(13, HIGH);
}

/* Load configuration from EEPROM */
void load() {
  int realOffset = 1;
  eepromOffset = EEPROM.read(0);
  realOffset += eepromOffset * sizeof(pData);
  EEPROM_readAnything(realOffset, pData);
  // Save configuration at new place now
  if (pData.writeCount >= 25000) {
    cli();
    eepromOffset += 1;
    EEPROM.write(0, eepromOffset);
    pData.writeCount = 0;
    backup(false);
    sei();
  }
}

byte readButtons() {
  byte value = 0;
  byte rawValue = (byte)(analogRead(0) / 8);
  switch (rawValue) {
    case 0:
    case 1:
    case 2:
      value = BTN_TOP;
      break;
    case 31:
    case 32:
    case 33:
      value = BTN_MIDDLE1;
      break;
    case 50:
    case 51:
    case 52:
      value = BTN_MIDDLE2;
      break;
    case 63:
    case 64:
    case 65:
      value = BTN_BOTTOM;
      break;
  }
  return value;
}

void reactButtons() {
  byte button = readButtons();
  button = button & ~oldButton;
  switch (button) {
    case BTN_TOP:
      mode = (mode + 1) % MODE_COUNT;
      lcd.clear();
      break;
    case BTN_MIDDLE1:
      if (mode == MODE_ACTION) {
        dumpEeprom();
      } else {
        pData.backlight = !pData.backlight;
        lcd.setBacklight(pData.backlight);
      }
      break;
    case BTN_MIDDLE2:
      backup(false);
      break;
    case BTN_BOTTOM:
      if (mode == MODE_ACTION) {
        // ~2s to reach 20
        if (resetAsked > 20) {
          newTrip();
        } else {
          resetAsked++;
          // Skip button debounce
          return;
        }
      } else {
        injRefreshMod = max(2, 2 * (injRefreshMod % INJ_MEAN_SAMPLES));
      }
      break;
    default:
      break;
  }
  resetAsked = 0;
  oldButton = button;
}

void readVoltage() {
  // val / resolution * vref * divisor
  //  x  /    1023    *  5   *  3
  short s_voltage = (analogRead(VOLTAGE_PIN) + analogRead(VOLTAGE_PIN)) / 2;
  float f_voltage = s_voltage * 0.0146627566 + VOLTAGE_OFFSET;
  voltage = (short)(f_voltage * 10.0);
}

float readVcc() {
  // http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  return 1125.3 / float(result); // Calculate Vcc (in V); 1125.3 = 1.1*1023
}

// This code may be used to log to something else (bluetooth?)
/*
void writeLog() {
  if (year <= 2000) {
    return;
  }
  File dataFile = SD.open(FILENAME_LOG, FILE_WRITE);
  if (dataFile) {
    // Datetime
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
    // Latitude and longitude
    dataFile.print(pData.lastLat, 6);
    dataFile.write(',');
    dataFile.print(pData.lastLon, 6);
    dataFile.write(',');
    // Current speed
    dataFile.print(int(curSpeed));
    dataFile.write(',');
    // Current duty cycle
    dataFile.print(duty);
    dataFile.write(',');
    // Current RPM
    dataFile.print(rpm);
    dataFile.write(',');
    // Current total distance
    dataFile.print(pData.distTot);
    dataFile.write('\n');
    dataFile.close();
  }
}
*/

void newTrip() {
  pData.distTrip = 0;
  pData.liters = 0.0;
  tripCons = 100;
  injSetTotalLiters(pData.liters);
  distAtStart = pData.distTot;
  litersAtStart = pData.liters;
}

void readGps() {
  int incomingByte = 0;
  float lat = pData.lastLat, lon = pData.lastLon;
  byte loop = 0;

  while (Serial.available() > 0) {
    incomingByte = Serial.read();

/*
    if (mode == MODE_STATS && incomingByte >= 32) {
      lcd.setCursor(8 + (loop++ % 4), 3);
      lcd.write((byte)incomingByte);
    }
*/

    if (gps.encode(incomingByte)) {
      gps.f_get_position(&lat, &lon, &fix_age);
      curSpeed = gps.f_speed_kmph();
      float delta = TinyGPS::distance_between(pData.lastLat,
          pData.lastLon, lat, lon);
      // 100 <= hdop <= 100000 --> delta >= 10m
      if (delta >= float(gps.hdop()) / 10.0 && fix_age < 2000) {
        unsigned short uDelta = (unsigned short)delta;
        pData.lastLat = lat;
        pData.lastLon = lon;
        if (uDelta < 20000u) {
          pData.distTot += uDelta;
          pData.distTrip += uDelta;
        }
      }
    }
  }

  gps.get_position(NULL, NULL, &fix_age);
}

void padPrintFloat2(float num, char units, char decis) {
  float num2 = abs(num);
  byte digits = 0;
  do {
    digits++;
    num2 /= 10.0;
  } while (num2 >= 1.0);
  if (num < 0.0)
    digits++;
  while (units - digits > 0) {
    lcd.write(' ');
    digits++;
  }
  lcd.print(num, decis);
}

void padPrintLong(long num, char units, char pad) {
  long num2 = abs(num);
  byte digits = 0;
  do {
    digits++;
    num2 /= 10;
  } while (num2 > 0);
  if (num < 0)
    digits++;
  while (units - digits > 0) {
    lcd.write(pad);
    digits++;
  }
  lcd.print(num);
}

void padPrintFloatShort(short num, char units, byte div) {
  short pref = num / div;
  short pref2 = pref;
  short suf = num % div;
  byte digits = 0;
  do {
    digits++;
    pref2 /= 10;
  } while (pref2 > 0);
  while (units - digits > 0) {
    lcd.write(' ');
    digits++;
  }
  lcd.print(pref);
  lcd.write('.');
  lcd.print(suf);
}

void printSpeed() {
  lcd.setCursor(0, 3);
  if (fix_age > 5000) {
    lcd.print(F("No GPS "));
  } else {
    padPrintLong(long(curSpeed), 3, ' ');
    lcd.print(F("kmh "));
  }
}

void printConsumption() {
  lcd.setCursor(12, 3);
  if (curSpeed < 15.0) {
    lcd.write(' ');
    padPrintFloatShort(consPerHour, 2, 10);
    lcd.print(F("L/h"));
  } else {
    padPrintFloatShort(instantCons, 3, 10);
    lcd.print(F("L\x01\x02"));
  }
}

void printStatFromStart() {
  short distHm = (pData.distTot - distAtStart) / 100;
  float liters = pData.liters - litersAtStart;
  lcd.setCursor(14, 2);
  padPrintLong(distHm / 10, 3, ' ');
  lcd.print("km");
  lcd.setCursor(12, 3);
  if (distHm > 0) {
    padPrintFloat2((liters * 1000.0)/(float)distHm, 3, 1);
  } else {
    lcd.print(" ----");
  }
  lcd.print(F("L\x01\x02"));
}

float computeDte() {
  float dte = (TANK_VOL - pData.liters) * 1000.0;
  if (curSpeed > 15.0)
    dte /= (tripCons * 19 + instantCons) / 20.0;
  else
    dte /= float(tripCons);
  return dte;
}

void printTimeDate() {
  lcd.setCursor(0, 1);
  padPrintLong((hour + 1 + (month > 3 && month < 11)) % 24, 2, '0');
  lcd.write(':');
  padPrintLong(minute, 2, '0');
  lcd.write(':');
  padPrintLong(second, 2, '0');
  lcd.write(' ');
  lcd.write(' ');
  padPrintLong(day, 2, '0');
  lcd.write('/');
  padPrintLong(month, 2, '0');
  lcd.write('/');
  padPrintLong(year, 4, '0');
}

void printMenu() {
  float dte, hdop;
  lcd.setCursor(0, 0);
  switch (mode) {
    case MODE_NORMAL: // -------------------------
      padPrintLong(rpm, 4, ' ');
      lcd.print(F("RPM"));
      padPrintLong(lastInjMicros, 5, ' ');
      lcd.print(F("us"));
      padPrintFloatShort(voltage, 3, 10);
      lcd.write('V');
      lcd.setCursor(0, 1); // --------------------
      lcd.print("Auto:");
      if (tripCons == 0) {
        lcd.print(" ----");
      } else {
        dte = computeDte();
        padPrintLong(long(dte), 5, ' ');
      }
      lcd.print("km");
      padPrintFloat2(TANK_VOL - pData.liters, 3, 3);
      lcd.write('L');
      lcd.setCursor(0, 2); // --------------------
      padPrintFloat2(float(pData.distTrip)/1000.0, 4, 2);
      lcd.print("km   ");
      padPrintFloatShort(tripCons, 3, 10);
      lcd.print(F("L\x01\x02"));
      printConsumption();
      break; // ----------------------------------
    case MODE_EXPERT:
      lcd.print(F("Total: "));
      padPrintFloat2(float(pData.distTot)/1000.0, 3, 3);
      lcd.print("km");
      printTimeDate(); // ------------------------
      lcd.setCursor(0, 2); // --------------------
      padPrintFloat2(abs(pData.lastLat), 2, 5);
      lcd.write((pData.lastLat>0)?'N':'S');
      lcd.write(' ');
      padPrintFloat2(abs(pData.lastLon), 3, 5);
      lcd.write((pData.lastLon>0)?'E':'W');
      lcd.setCursor(7, 3);
      lcd.print(F("HDOP:"));
      hdop = min(gps.hdop(), 999999);
      padPrintFloat2(float(hdop)/100.0, 4, 2);
      break; // ----------------------------------
    case MODE_ACTION:
      lcd.print(F("Total: "));
      padPrintFloat2(float(pData.distTot)/1000.0, 3, 3);
      lcd.print("km");
      lcd.setCursor(0, 1);
      lcd.print(F("Fill"));
      padPrintFloat2(pData.liters, 3, 1);
      lcd.print(F("L B2: dump"));
      lcd.setCursor(0, 2);
      padPrintFloat2(float(pData.distTrip)/1000.0, 4, 2);
      lcd.print(F("km  B3: save"));
      lcd.setCursor(11, 3);
      lcd.print(F("B4: reset"));
      break;

    case MODE_STATS:
      lcd.print(F("Max:"));
      padPrintFloatShort(maxDuty, 3, 10);
      lcd.print("%@");
      padPrintLong(maxDutyRpm, 4, ' ');
      lcd.print(F("RPM"));

      lcd.setCursor(0, 1);
      lcd.print(F("Inj:"));
      padPrintFloatShort(duty, 3, 10);
      lcd.print("% mean: ");
      lcd.print(injRefreshMod);

      lcd.setCursor(0, 2);
      lcd.print(F("Max:"));
      padPrintFloat2(maxSpeed, 3, 1);
      lcd.print(F("kmh"));

      printStatFromStart();
      break;
  }
}

void setBacklight() {
  lcd.setBacklight(pData.backlight);
}

void lowPowerLoop() {
  float vcc = readVcc();
  while (vcc < 4.92f) {
    digitalWrite(13, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(F("USB?"));
    delay(400);
    vcc = readVcc();
  }
}

void setup() {
  pinMode(13, OUTPUT);
  lcd.begin(20, 4);
  lcd.clear();
  lowPowerLoop();
  digitalWrite(13, LOW);
  lcd.createChar(1, k10);
  lcd.createChar(2, m0);

  pinMode(CHIPSELECT_PIN, OUTPUT);
  pinMode(INJ_READ_PIN, INPUT);
  pinMode(KEYPAD_PIN, INPUT_PULLUP);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(POWER_PIN, INPUT);

  // load configuration from EEPROM
  load();

  // initialize with safe defaults
  if (pData.lastLat == 0.0) {
    pData.lastLat = 50.6065753;
    pData.lastLon = 3.0758584;
    pData.distTot = 0;
    pData.distTrip = 0;
    pData.liters = 0.0;
    pData.backlight = 80;
  }

  distAtStart = pData.distTot;
  litersAtStart = pData.liters;

  Serial.begin(9600);

  // Ask the GPS module to increase serial bitrate
  if (readButtons() == BTN_TOP) {
    lcd.setCursor(0, 0);
    lcd.print(F("Serial"));
    while (Serial.available() > 0)
      Serial.read();
    Serial.println(F(SERIAL_SET_SPEED));
    Serial.flush();
    delay(10);
    Serial.begin(SERIAL_SPEED);
    delay(500);
  }

  // start listening to GPS
  lcd.setCursor(0, 0);
  lcd.print(F("GPS   "));
  delay(1000);

  // If we are lucky enough, we can get a time fix now
  readGps();
  delay(1000);
  readGps();

  unsigned long age = 0;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, NULL, &age);
  if (minute < 96) {
    printTimeDate();
  } else {
    lcd.print(F("not ready"));
  }
  delay(1000);

  setBacklight();

  // start measuring injection
  injSetTotalLiters(pData.liters);
  attachInterrupt(1, injInterrupt, CHANGE);
  // attach powerdown interrupt to backup
  attachInterrupt(0, backupIsr, FALLING);
}

void loop() {
  unsigned long now = millis();
  readGps();
  // We must print something to make the screen flicker
  // fast enough so we don't see it flicker
  printSpeed();

  if (now - lastRefreshTime < 50) {
    // Less than 50ms were elapsed, don't do anything
    return;
  }
  lastRefreshTime = now;
  refreshStep++;

  // Probe injection each 4 loops (200ms)
  if ((refreshStep % 4) == 0) {
    injTakeSample(voltage);
    injGetRpm(&rpm);
  }

  // Refresh instant consumption
  if ((refreshStep % injRefreshMod) == 0) {
    injCompute(&duty, &consPerHour, injRefreshMod);
    if (curSpeed > 0.0) {
      instantCons = consPerHour * 100 / curSpeed;
    }
  }

  // Refresh consumption every 64 loops (3.2s), not at the same time as injection
  if ((refreshStep % 64) == 2) {
    injGetTotalLiters(&(pData.liters));
    if (pData.distTrip > 0)
      tripCons = short(pData.liters / float(pData.distTrip) * 1000000.0);
  }
  // React to user key presses each odd loop (100ms), and refresh time each even
  if (refreshStep % 2) {
    reactButtons();
    readVoltage();
    if (curSpeed > maxSpeed)
      maxSpeed = curSpeed;
    if (rpm > maxRpm && rpm < maxRpm + 1500)
      maxRpm = rpm;
    if (duty > maxDuty) {
      maxDuty = duty;
      maxDutyRpm = rpm;
    }
  } else {
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, NULL, NULL);
    // Write log entry every 12.8s
    if (refreshStep == 126) { // %16 -> 14
      //writeLog();
      backupTimer++;
      // Security backup every 13 minutes
      if ((backupTimer % 64) == 0) {
        backup(true);
      }
    }
  }
  printMenu();
}

