#define INJ_MEAN_SAMPLES 8
#define INJ_OFFSET_MICROS 1000
#define FILENAME_LOG "logging.txt"
#define FILENAME_DAILY "daily.dat"
#define FILENAME_OLDDAILY "history.dat"
#define FILENAME_EEPROM "eeprom.dat"

#define LCD20x4

#define BACKLIGHT_PIN 10
#define CHIPSELECT_PIN 8
#define INJ_READ_PIN 3
#define POWER_PIN 2
#define KEYPAD_PIN A0
#define VOLTAGE_PIN A1

#define BTN_RIGHT   1
#define BTN_UP      2
#define BTN_DOWN    4
#define BTN_LEFT    8
#define BTN_SELECT 32

//#ifdef LCD20x4
#define MODE_CONS_DTE    0
#define MODE_DAILY       1
#define MODE_RPM_DUTY    2
#define MODE_DISTANCE    3
#define MODE_TIME        4
#define MODE_POSITION    5
#define MODE_LOGGING     6
#define MODE_BACKLIGHT   7
#define MODE_INJFLOW     8
//#else
#define MODE_NORMAL 0
#define MODE_EXPERT 1
//#endif

//#define USE_FAT16 1
//#define SOFTWARE_SPI 1
