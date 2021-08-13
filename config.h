#define INJ_MEAN_SAMPLES 16
#define VOLTAGE_OFFSET 1.1
// Lucas 5720A510
//#define INJ_CC_BY_MIN 1040L
//#define INJ_OFFSET_MICROS 640
// Bosch 0280150431
//#define INJ_CC_BY_MIN 1400L
//#define INJ_OFFSET_MICROS 720
// Bosch 0280155968
#define INJ_CC_BY_MIN 1660L
#define INJ_OFFSET_MICROS 700

// Voltage source is low, compensate with increased
// resistor value.
//#define TEMP_SENSOR_SERIES_RES 1000.0
#define TEMP_SENSOR_SERIES_RES 1100.0
#define TEMP_SENSOR_NOM_RES 36.51
#define TEMP_SENSOR_NOM_TEMP 120.0
//#define TEMP_SENSOR_COEF 4013.0
#define TEMP_SENSOR_COEF 4130.0
#define KELVIN 273.15

#define SERIAL_SPEED 19200
#define SERIAL_SET_SPEED "$PMTK251,19200*22"

#define FILENAME_LOG "logging.txt"
#define FILENAME_PDATA "persist.dat"
#define FILENAME_PDATA_HIST "history.dat"
#define FILENAME_EEPROM "backup.rom"
#define FILENAME_LOADME "loadme.rom"

#define BACKLIGHT_PIN 10
#define INJ_READ_PIN 3
#define POWER_PIN 2
#define KEYPAD_PIN A0
#define VOLTAGE_PIN A1
#define TEMP_PIN A3

# define BTN_TOP     1
# define BTN_MIDDLE1 2
# define BTN_MIDDLE2 4
# define BTN_BOTTOM  8

# define MODE_NORMAL 0
# define MODE_EXPERT 1
# define MODE_ACTION 2
# define MODE_STATS  3
# define MODE_COUNT  4

#define TANK_VOL 68.0

#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 128
