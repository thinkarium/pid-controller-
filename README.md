# pid-controller-

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <max6675.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// =====================================================
// PINS
// =====================================================

#define BTN_UP       2
#define BTN_DOWN     3
#define BTN_SET      4

#define RELAY_PIN    5

#define MAX6675_SO   6
#define MAX6675_CS   7
#define MAX6675_SCK  8

// =====================================================
// RELAY LOGIC
// ACTIVE high 
// =====================================================

#define RELAY_ON     HIGH
#define RELAY_OFF    LOW

// =====================================================
// LCD & THERMOCOUPLE
// =====================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);

MAX6675 thermocouple(
  MAX6675_SCK,
  MAX6675_CS,
  MAX6675_SO
);

// =====================================================
// TEMPERATURE LIMITS
// =====================================================

const double MIN_SAFE_TEMP = 0.0;
const double MAX_SAFE_TEMP = 1000.0;

const double MIN_SETPOINT = 30.0;
const double MAX_SETPOINT = 1000.0;

// =====================================================
// CONTROL BAND
// =====================================================

const double CONTROL_BAND = 4.0;

// =====================================================
// PID
// =====================================================

double setpoint = 70.0;
double currentTemp = 0.0;
double pidOutput = 0.0;

// Starting values for low-temperature testing.
// These MUST be tuned experimentally for your heater.
double Kp = 2.0;
double Ki = 0.20;
double Kd = 2.0;

double calOffset = 0.0;

PID myPID(
  &currentTemp,
  &pidOutput,
  &setpoint,
  Kp,
  Ki,
  Kd,
  DIRECT
);

// =====================================================
// RELAY TIME PROPORTIONAL CONTROL
// =====================================================

const unsigned long WindowSize = 20000UL; // 20 seconds

const unsigned long MIN_RELAY_ON_TIME  = 5000UL;
const unsigned long MIN_RELAY_OFF_TIME = 5000UL;

unsigned long windowStartTime = 0;
unsigned long relayChangeTime = 0;

bool relayState = false;

// =====================================================
// EEPROM
// =====================================================

#define EEPROM_MAGIC_ADDR  100
#define EEPROM_MAGIC       0x55AA

#define ADDR_SETPOINT      0
#define ADDR_KP            8
#define ADDR_KI            16
#define ADDR_KD            24
#define ADDR_CAL_OFFSET    32

// =====================================================
// MENU
// =====================================================

enum State {
  DISPLAY_MODE,
  MENU_NAV,
  EDIT_ITEM,
  ERROR_MODE
};

State currentState = DISPLAY_MODE;

const char* menuItems[] = {
  "1.Setpoint",
  "2.Kp",
  "3.Ki",
  "4.Kd",
  "5.Cal Offset",
  "6.Exit"
};

const int NUM_MENU_ITEMS = 6;

int currentMenuIdx = 0;
int editValue = 0;

// =====================================================
// TIMING
// =====================================================

unsigned long lastButtonPress = 0;
unsigned long lastTempRead = 0;
unsigned long lastDisplayUpdate = 0;

const unsigned long DEBOUNCE_TIME = 200;
const unsigned long TEMP_INTERVAL = 500;
const unsigned long DISPLAY_INTERVAL = 200;

// =====================================================
// FUNCTION DECLARATIONS
// =====================================================

void handleButtons();
void updateDisplay();

void saveSettings();
void loadSettings();

void relayOn();
void relayOff();

void updateRelayControl();

bool temperatureIsValid(double temp);

// =====================================================
// SETUP
// =====================================================

void setup() {

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SET, INPUT_PULLUP);

  pinMode(RELAY_PIN, OUTPUT);

  relayOff();

  wdt_enable(WDTO_2S);

  lcd.init();
  lcd.backlight();

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("PRO PID SYSTEM");

  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  wdt_reset();

  delay(1200);

  wdt_reset();

  loadSettings();

  myPID.SetTunings(Kp, Ki, Kd);

  myPID.SetOutputLimits(0, WindowSize);

  myPID.SetSampleTime(1000);

  myPID.SetMode(AUTOMATIC);

  windowStartTime = millis();
  relayChangeTime = millis();

  lcd.clear();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  wdt_reset();

  unsigned long now = millis();

  // ===================================================
  // TEMPERATURE READING
  // ===================================================

  if (now - lastTempRead >= TEMP_INTERVAL) {

    lastTempRead = now;

    double rawTemp = thermocouple.readCelsius();

    if (!temperatureIsValid(rawTemp)) {

      currentState = ERROR_MODE;

      relayOff();

      pidOutput = 0;
    }

    else {

      currentTemp = rawTemp + calOffset;

      if (currentTemp < MIN_SAFE_TEMP ||
          currentTemp > MAX_SAFE_TEMP) {

        currentState = ERROR_MODE;

        relayOff();

        pidOutput = 0;
      }

      else {

        if (currentState == ERROR_MODE) {
          currentState = DISPLAY_MODE;
        }
      }
    }
  }

  // ===================================================
  // NORMAL OPERATION
  // ===================================================

  if (currentState != ERROR_MODE) {

    handleButtons();

    // =================================================
    // HARD SAFETY CUTOFF
    // =================================================

    if (currentTemp >= MAX_SAFE_TEMP) {

      pidOutput = 0;

      relayOff();
    }

    // =================================================
    // PID CONTROL
    // =================================================

    else {

      myPID.Compute();

      updateRelayControl();
    }
  }

  // ===================================================
  // LCD
  // ===================================================

  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {

    lastDisplayUpdate = now;

    updateDisplay();
  }
}

// =====================================================
// TEMPERATURE VALIDATION
// =====================================================

bool temperatureIsValid(double temp) {

  if (isnan(temp))
    return false;

  if (temp < MIN_SAFE_TEMP)
    return false;

  if (temp > MAX_SAFE_TEMP)
    return false;

  return true;
}

// =====================================================
// RELAY ON
// =====================================================

void relayOn() {

  if (!relayState) {

    relayState = true;

    digitalWrite(RELAY_PIN, RELAY_ON);

    relayChangeTime = millis();
  }
}

// =====================================================
// RELAY OFF
// =====================================================

void relayOff() {

  relayState = false;

  digitalWrite(RELAY_PIN, RELAY_OFF);

  relayChangeTime = millis();
}

// =====================================================
// RELAY CONTROL
// =====================================================

void updateRelayControl() {

  unsigned long now = millis();

  // ---------------------------------------------------
  // RESET 20 SECOND WINDOW
  // ---------------------------------------------------

  if (now - windowStartTime >= WindowSize) {

    windowStartTime = now;
  }

  unsigned long elapsed =
    now - windowStartTime;

  // ---------------------------------------------------
  // ABOVE SETPOINT
  // ---------------------------------------------------

  if (currentTemp >= setpoint) {

    relayOff();

    return;
  }

  // ---------------------------------------------------
  // VERY CLOSE TO SETPOINT
  // ---------------------------------------------------

  if (currentTemp >= setpoint - CONTROL_BAND) {

    // PID is already reducing output here.
    // Do not force full heating.

    if (pidOutput <= 0) {

      relayOff();

      return;
    }
  }

  // ---------------------------------------------------
  // PID OUTPUT ZERO
  // ---------------------------------------------------

  if (pidOutput <= 0) {

    relayOff();

    return;
  }

  // ---------------------------------------------------
  // FULL PID OUTPUT
  // ---------------------------------------------------

  if (pidOutput >= WindowSize) {

    if (!relayState &&
        now - relayChangeTime >= MIN_RELAY_OFF_TIME) {

      relayOn();
    }

    return;
  }

  // ---------------------------------------------------
  // NORMAL TIME PROPORTIONAL CONTROL
  // ---------------------------------------------------

  bool demandON = (pidOutput > elapsed);

  if (demandON) {

    if (!relayState &&
        now - relayChangeTime >= MIN_RELAY_OFF_TIME) {

      relayOn();
    }
  }

  else {

    if (relayState &&
        now - relayChangeTime >= MIN_RELAY_ON_TIME) {

      relayOff();
    }
  }
}

// =====================================================
// BUTTON HANDLING
// =====================================================

void handleButtons() {

  unsigned long now = millis();

  if (now - lastButtonPress < DEBOUNCE_TIME)
    return;

  bool up   = digitalRead(BTN_UP) == LOW;
  bool down = digitalRead(BTN_DOWN) == LOW;
  bool set  = digitalRead(BTN_SET) == LOW;

  if (!up && !down && !set)
    return;

  lastButtonPress = now;

  // ===================================================
  // HOME SCREEN
  // ===================================================

  if (currentState == DISPLAY_MODE) {

    if (set) {

      currentState = MENU_NAV;
      currentMenuIdx = 0;

      lcd.clear();

      return;
    }

    if (up) {

      setpoint += 10;

      if (setpoint > MAX_SETPOINT)
        setpoint = MAX_SETPOINT;

      return;
    }

    if (down) {

      setpoint -= 10;

      if (setpoint < MIN_SETPOINT)
        setpoint = MIN_SETPOINT;

      return;
    }
  }

  // ===================================================
  // MENU
  // ===================================================

  if (currentState == MENU_NAV) {

    if (up) {

      currentMenuIdx++;

      if (currentMenuIdx >= NUM_MENU_ITEMS)
        currentMenuIdx = 0;

      return;
    }

    if (down) {

      currentMenuIdx--;

      if (currentMenuIdx < 0)
        currentMenuIdx = NUM_MENU_ITEMS - 1;

      return;
    }

    if (set) {

      if (currentMenuIdx == 5) {

        saveSettings();

        currentState = DISPLAY_MODE;

        lcd.clear();

        return;
      }

      currentState = EDIT_ITEM;

      if (currentMenuIdx == 0)
        editValue = (int)setpoint;

      else if (currentMenuIdx == 1)
        editValue = (int)(Kp * 10);

      else if (currentMenuIdx == 2)
        editValue = (int)(Ki * 100);

      else if (currentMenuIdx == 3)
        editValue = (int)(Kd * 10);

      else if (currentMenuIdx == 4)
        editValue = (int)calOffset;

      lcd.clear();

      return;
    }
  }

  // ===================================================
  // EDIT MODE
  // ===================================================

  if (currentState == EDIT_ITEM) {

    if (up) {

      if (currentMenuIdx == 0)
        editValue += 10;
      else
        editValue++;

      return;
    }

    if (down) {

      if (currentMenuIdx == 0)
        editValue -= 10;
      else
        editValue--;

      return;
    }

    if (set) {

      if (currentMenuIdx == 0) {

        editValue =
          constrain(
            editValue,
            (int)MIN_SETPOINT,
            (int)MAX_SETPOINT
          );

        setpoint = editValue;
      }

      else if (currentMenuIdx == 1) {

        editValue = constrain(editValue, 0, 1000);

        Kp = editValue / 10.0;

        myPID.SetTunings(Kp, Ki, Kd);
      }

      else if (currentMenuIdx == 2) {

        editValue = constrain(editValue, 0, 1000);

        Ki = editValue / 100.0;

        myPID.SetTunings(Kp, Ki, Kd);
      }

      else if (currentMenuIdx == 3) {

        editValue = constrain(editValue, 0, 1000);

        Kd = editValue / 10.0;

        myPID.SetTunings(Kp, Ki, Kd);
      }

      else if (currentMenuIdx == 4) {

        editValue = constrain(editValue, -20, 20);

        calOffset = editValue;
      }

      saveSettings();

      currentState = MENU_NAV;

      lcd.clear();

      return;
    }
  }
}

// =====================================================
// LCD DISPLAY
// =====================================================

void updateDisplay() {

  if (currentState == DISPLAY_MODE) {

    lcd.setCursor(0, 0);
    lcd.print("PV:");
    lcd.setCursor(3, 0);
    lcd.print("       ");
    lcd.setCursor(3, 0);
    lcd.print(currentTemp, 1);
    lcd.print((char)223);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("SV:");
    lcd.setCursor(3, 1);
    lcd.print("       ");
    lcd.setCursor(3, 1);
    lcd.print(setpoint, 0);
    lcd.print((char)223);
    lcd.print("C");

    return;
  }

  if (currentState == MENU_NAV) {

    lcd.setCursor(0, 0);
    lcd.print("--- CONFIG ---  ");

    lcd.setCursor(0, 1);
    lcd.print(menuItems[currentMenuIdx]);

    int len = strlen(menuItems[currentMenuIdx]);

    for (int i = len; i < 16; i++)
      lcd.print(" ");

    return;
  }

  if (currentState == EDIT_ITEM) {

    lcd.setCursor(0, 0);

    lcd.print("EDITING: ");
    lcd.print(currentMenuIdx + 1);
    lcd.print("       ");

    lcd.setCursor(0, 1);

    lcd.print("Val: > ");

    if (currentMenuIdx == 0 ||
        currentMenuIdx == 4) {

      lcd.print(editValue);
    }

    else if (currentMenuIdx == 1 ||
             currentMenuIdx == 3) {

      lcd.print(editValue / 10.0, 1);
    }

    else if (currentMenuIdx == 2) {

      lcd.print(editValue / 100.0, 2);
    }

    lcd.print("    ");

    return;
  }

  if (currentState == ERROR_MODE) {

    lcd.setCursor(0, 0);
    lcd.print("!! TC ERROR !! ");

    lcd.setCursor(0, 1);
    lcd.print("RELAY OFF      ");

    return;
  }
}

// =====================================================
// EEPROM SAVE
// =====================================================

void saveSettings() {

  EEPROM.put(ADDR_SETPOINT, setpoint);
  EEPROM.put(ADDR_KP, Kp);
  EEPROM.put(ADDR_KI, Ki);
  EEPROM.put(ADDR_KD, Kd);
  EEPROM.put(ADDR_CAL_OFFSET, calOffset);

  unsigned int magic = EEPROM_MAGIC;

  EEPROM.put(EEPROM_MAGIC_ADDR, magic);
}

// =====================================================
// EEPROM LOAD
// =====================================================

void loadSettings() {

  unsigned int magic;

  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  if (magic != EEPROM_MAGIC) {

    setpoint = 70.0;

    Kp = 2.0;
    Ki = 0.20;
    Kd = 2.0;

    calOffset = 0.0;

    saveSettings();

    return;
  }

  EEPROM.get(ADDR_SETPOINT, setpoint);
  EEPROM.get(ADDR_KP, Kp);
  EEPROM.get(ADDR_KI, Ki);
  EEPROM.get(ADDR_KD, Kd);
  EEPROM.get(ADDR_CAL_OFFSET, calOffset);

  if (isnan(setpoint) ||
      setpoint < MIN_SETPOINT ||
      setpoint > MAX_SETPOINT)
    setpoint = 70.0;

  if (isnan(Kp) || Kp < 0 || Kp > 100)
    Kp = 2.0;

  if (isnan(Ki) || Ki < 0 || Ki > 10)
    Ki = 0.20;

  if (isnan(Kd) || Kd < 0 || Kd > 100)
    Kd = 2.0;

  if (isnan(calOffset) ||
      calOffset < -20 ||
      calOffset > 20)
    calOffset = 0.0;
}