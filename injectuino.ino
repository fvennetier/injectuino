// vim: tw=80 ts=2 sw=2 expandtab
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
uint16_t rpm = 0;
uint16_t maxRpm = 0;
uint16_t duty = 0;
uint16_t maxDuty = 0;
uint16_t maxDutyMicros = 0;
uint16_t maxDutyRpm = 0;
uint16_t consPerHour = 0;
uint16_t instantCons = 0;
uint16_t voltage = 0;
uint16_t tripCons = 100;
uint16_t curInjMicros = 0;
uint16_t maxInjMicros = 0;
uint16_t maxInjMicrosRpm = 0;

// -- Timing ------------------
uint32_t lastRefreshTime = 0;
byte injRefreshMod = 4;
byte refreshStep = 0;
byte backupTimer = 0;

uint32_t distAtStart = 0;
float litersAtStart = 0;

// -- Configuration -----------
byte eepromOffset = 0;
struct PersistentData {
  // Whole traveled distance, in meters
  uint32_t distTot;
  // Trip distance, in meters
  uint32_t distTrip;
  // Trip liters
  float liters;
  // Last latitude and longitude
  float lastLat, lastLon;
  // Number of times EEPROM has been written
  int16_t writeCount;
  // PWM value for backlight
  int16_t backlight;
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
      if (mode == MODE_ACTION || mode == MODE_STATS) {
        // ~2s to reach 20
        if (resetAsked > 20) {
          if (mode == MODE_ACTION)
            newTrip();
          else
            resetMax();
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
  voltage = (uint16_t)(f_voltage * 10.0);
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

void newTrip() {
  pData.distTrip = 0;
  pData.liters = 0.0;
  tripCons = 100;
  injSetTotalLiters(pData.liters);
  distAtStart = pData.distTot;
  litersAtStart = pData.liters;
}

void resetMax() {
  maxSpeed = 0.0;
  maxDuty = 0;
  maxDutyMicros = 0;
  maxDutyRpm = 0;
  maxInjMicros = 0;
  maxInjMicrosRpm = 0;
}

void readGps() {
  int incomingByte = 0;
  float lat = pData.lastLat, lon = pData.lastLon;

  while (Serial.available() > 0) {
    incomingByte = Serial.read();

    if (gps.encode(incomingByte)) {
      gps.f_get_position(&lat, &lon, &fix_age);
      curSpeed = gps.f_speed_kmph();
      float delta = TinyGPS::distance_between(pData.lastLat,
          pData.lastLon, lat, lon);
      // 100 <= hdop <= 100000 --> delta >= 10m
      if (delta >= float(gps.hdop()) / 10.0 && fix_age < 2000) {
        uint16_t uDelta = (uint16_t)delta;
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

/* Print a (positive) 16bit number at the current screen position.
 * The number will be left-padded with `pad` until it fills `units`
 * characters on screen. */
void padPrintShort(uint16_t num, char units, char pad) {
  uint16_t num2 = num;
  byte digits = 0;
  do {
    digits++;
    num2 /= 10;
  } while (num2 > 0);
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
    padPrintShort(uint16_t(curSpeed), 3, ' ');
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
  lcd.setCursor(7, 3);
  padPrintShort(distHm / 10, 3, ' ');
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
  // DST from april to october
  padPrintShort((hour + 1 + (month > 3 && month < 11)) % 24, 2, '0');
  lcd.write(':');
  padPrintShort(minute, 2, '0');
  lcd.write(':');
  padPrintShort(second, 2, '0');
  lcd.write(' ');
  lcd.write(' ');
  padPrintShort(day, 2, '0');
  lcd.write('/');
  padPrintShort(month, 2, '0');
  lcd.write('/');
  padPrintShort(year, 4, '0');
}

void printMenu() {
  float dte;
  short hdop;
  lcd.setCursor(0, 0);
  switch (mode) {
    case MODE_NORMAL: // -------------------------
      padPrintShort(rpm, 4, ' ');
      lcd.print(F("RPM"));
      padPrintShort(curInjMicros, 5, ' ');
      lcd.print(F("us"));
      padPrintFloatShort(voltage, 3, 10);
      lcd.write('V');
      lcd.setCursor(0, 1); // --------------------
      lcd.print("Auto:");
      if (tripCons == 0) {
        lcd.print(" ----");
      } else {
        dte = computeDte();
        padPrintShort(uint16_t(dte), 5, ' ');
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
      hdop = min(gps.hdop(), 9999);
      padPrintFloatShort(hdop, 2, 100);
      lcd.write('/');
      padPrintShort(gps.satellites(), 2, '0');
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
      lcd.print(F("Max: "));
      padPrintShort(maxInjMicros, 5, ' ');
      lcd.print(F("us@"));
      padPrintShort(maxInjMicrosRpm, 4, ' ');
      lcd.print(F("RPM"));

      lcd.setCursor(0, 1);
      padPrintShort(injRefreshMod, 2, '0');

      lcd.setCursor(5, 1);
      padPrintShort(maxDutyMicros, 5, ' ');
      lcd.print(F("us@"));
      padPrintShort(maxDutyRpm, 4, ' ');
      lcd.print(F("RPM"));

      lcd.setCursor(0, 2);
      lcd.print(F("Max:"));
      padPrintFloat2(maxSpeed, 3, 1);
      lcd.print(F("kmh"));
      lcd.setCursor(13, 2);
      padPrintShort(maxRpm, 4, ' ');
      lcd.print(F("RPM"));

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
  uint32_t now = millis();
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
    injGetSmoothRpm(&rpm);
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
      tripCons = (uint16_t)(pData.liters / float(pData.distTrip) * 1000000.0);
  }
  // React to user key presses each odd loop (100ms), and refresh time each even
  if (refreshStep % 2) {
    reactButtons();
    readVoltage();
    uint16_t realRpm;
    injGetRpm(&realRpm);
    curInjMicros = SAFE_COPY(uint16_t, lastInjMicros);
    if (curSpeed > maxSpeed)
      maxSpeed = curSpeed;
    if (realRpm > maxRpm && realRpm < maxRpm + 1500)
      maxRpm = realRpm;
    if (curInjMicros > maxInjMicros && realRpm > 1500) {
      maxInjMicros = curInjMicros;
      maxInjMicrosRpm = realRpm;
    }
    if (duty > maxDuty) {
      maxDuty = duty;
      maxDutyMicros = curInjMicros;
      maxDutyRpm = realRpm;
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
