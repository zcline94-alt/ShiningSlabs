/*
=========================================================
PREMIUM POKEMON SLAB DISPLAY version 1.4
ESP32-S2 + 66 WS2812 LEDs

v1.4: Brightness button changes now ramp smoothly (2 steps per loop,
  roughly 0.5-0.8 s) instead of jumping. The instant jump caused a large
  current step on all 66 LEDs, and the resulting supply/ground bounce could
  corrupt one frame sent to the status LED on D12. Because WS2812 data is
  GRB, a corrupted frame showed up as a brief green flash. Everything else
  runs exactly as in v1.3.

v1.3: Built on v1.1 (v1.2's shutdown state machine is removed). The only
  addition: whenever the firmware drops HOLD_PIN to power off, the status
  LED is blanked and held dark, so it can't be redrawn green while power
  collapses. Everything else runs exactly as in v1.1.

v1.1 status LED (D12) changes:
  - Status LED is frozen while the power button is pressed and for a short
    time after release, so button presses can't flash green or red.
  - "Full" (green) requires the ST pin to be high continuously for 3 s.
  - Low battery (red) uses a filtered voltage, a lower threshold suited to a
    1S LiPo under LED load, hysteresis, and a 10 s confirmation.
  - Any AN1 reading near GND (button pressed / recovering) is ignored as a
    battery measurement. No red at boot before a valid reading exists.
  - Full charge is latched (and saved to flash) once ST reports full, so a
    quick unplug/replug still shows green. The latch clears only once the
    battery has actually been drawn down on battery power.
=========================================================
*/

#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ADS1X15.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

#define LED_PIN   13
#define LED_COUNT 66
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define STATUS_LED_PIN 12
Adafruit_NeoPixel statusLED(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

#define BTN_MODE        18
#define BTN_COLOR       17
#define BTN_BRIGHTNESS  16

#define HOLD_PIN 5
#define IO14_PIN 14

Adafruit_ADS1015 ads;
Preferences prefs;

int currentMode = 0;
int paletteIndex = 0;
int staticColorIndex = 0;

const int NUM_MODES = 9;

const uint8_t brightnessLevels[] = {70, 115, 145};
const int numBrightnessLevels = sizeof(brightnessLevels) / sizeof(brightnessLevels[0]);
int brightnessIndex = 0;
uint8_t brightness = brightnessLevels[brightnessIndex];        // level currently applied to the strip
uint8_t targetBrightness = brightnessLevels[brightnessIndex];  // level the ramp is heading toward

// Brightness change per loop pass (~20 ms). A full change from 145 to 70
// takes about 38 passes (~0.75 s), so the LED current never steps sharply.
const int BRIGHTNESS_RAMP_STEP = 2;

const unsigned long BUTTON_DEBOUNCE_MS = 180;

bool lastModeButtonState = HIGH;
bool lastColorButtonState = HIGH;
bool lastBrightnessButtonState = HIGH;

unsigned long lastModePressTime = 0;
unsigned long lastColorPressTime = 0;
unsigned long lastBrightnessPressTime = 0;

float chargerVoltage = 0.0;       // AN0: USB/charger-present voltage
float batteryVoltage = 0.0;       // AN1: filtered battery voltage (valid readings only)
float chargeStatusVoltage = 0.0;  // AN2: charge controller ST pin, high when battery is full

bool batteryVoltageValid = false; // false until the first real battery reading

const float CHARGER_PRESENT_MIN_VOLTAGE = 4.4;
const float CHARGE_COMPLETE_MIN_VOLTAGE = 4.0;  // AN2 above this = charge complete

// ---------------- Battery measurement (AN1) ----------------
// These assume AN1 sees the battery directly (no divider). If there is a
// divider, scale these thresholds by the divider ratio.

// Any AN1 reading below this is the power button pulling the node toward
// GND (or the node recovering afterward). A 1S LiPo is never this low in
// normal use, so it is never treated as a battery reading.
const float BATTERY_VALID_MIN_VOLTAGE = 2.80;

// After AN1 has been disturbed, wait this long before trusting it again.
const unsigned long BATTERY_SETTLE_AFTER_RELEASE_MS = 750;

// Exponential filter on the battery reading. Readings arrive every 20 ms,
// so 0.02 gives roughly a 1 s time constant and smooths LED-load ripple.
const float BATTERY_FILTER_ALPHA = 0.02;

// Low battery with hysteresis. With 66 LEDs running the cell sags noticeably,
// so 3.35 V under load is roughly the last 5-10% of a typical LiPo.
const float LOW_BATTERY_ENTER_VOLTAGE = 3.35;
const float LOW_BATTERY_EXIT_VOLTAGE  = 3.55;
const unsigned long LOW_BATTERY_CONFIRM_MS = 10000;

bool lowBatteryActive = false;
bool lowBatteryTimerRunning = false;
unsigned long lowBatteryTimerStart = 0;

// ---------------- Charge-complete tracking (AN2) ----------------
// ST must stay high this long (with the charger present and the button
// untouched) before it counts as "full".
const unsigned long CHARGE_COMPLETE_CONFIRM_MS = 3000;

bool chargeCompleteConfirmed = false;
bool stHighTimerRunning = false;
unsigned long stHighTimerStart = 0;

// Full-charge latch: survives unplug/replug and power cycles (stored in
// flash). It clears only after the battery has been used down below this
// voltage on battery power for a sustained period.
const float FULL_LATCH_CLEAR_VOLTAGE = 4.00;
const unsigned long FULL_LATCH_CLEAR_CONFIRM_MS = 30000;

bool fullChargeLatched = false;
bool fullLatchClearTimerRunning = false;
unsigned long fullLatchClearTimerStart = 0;

// ---------------- Status LED ----------------
enum BatteryStatusState {
  STATUS_OFF,
  STATUS_CHARGING,
  STATUS_FULL,
  STATUS_LOW_BATTERY
};

BatteryStatusState batteryStatus = STATUS_OFF;
BatteryStatusState pendingBatteryStatus = STATUS_OFF;
unsigned long batteryStatusChangeStart = 0;
const unsigned long BATTERY_STATUS_DEBOUNCE_MS = 1200;

// The status LED holds its current color while the power button is pressed
// and for this long after AN1 last dipped toward GND.
const unsigned long STATUS_FREEZE_AFTER_BUTTON_MS = 1500;

// Last time AN1 was below BATTERY_VALID_MIN_VOLTAGE (button activity).
unsigned long lastAn1DisturbedTime = 0;

// ---------------- Power-off blanking ----------------
// Set whenever the firmware drops HOLD_PIN. While set, the status LED is
// kept dark. It clears automatically once no power-off has been requested
// for this long (and the power button is released), which only happens if
// the board is still powered, e.g. by USB.
bool statusBlankActive = false;
unsigned long statusBlankRequestTime = 0;
const unsigned long STATUS_BLANK_RELEASE_MS = 2000;

enum ButtonPowerState {
  WAIT_FOR_STARTUP_RELEASE,
  SYSTEM_READY,
  BUTTON_HOLDING,
  WAIT_FOR_TOGGLE_RELEASE
};

ButtonPowerState buttonState = WAIT_FOR_STARTUP_RELEASE;

bool ledsEnabled = true;
unsigned long buttonPressStart = 0;

const float BUTTON_PRESSED_MAX_VOLTAGE = 1.0;
const float BUTTON_RELEASED_MIN_VOLTAGE = 2.0;
const unsigned long HOLD_TIME_MS = 80;

// AN1 soft-latch debounce. The input must remain steadily pressed or released
// before the power-button state machine is allowed to act on it.
const unsigned long POWER_BUTTON_DEBOUNCE_MS = 60;
const unsigned long POWER_BUTTON_LOCKOUT_MS = 350;

bool powerButtonCandidatePressed = true;
bool powerButtonStablePressed = true;
unsigned long powerButtonCandidateChangeTime = 0;
unsigned long lastPowerToggleTime = 0;

uint32_t paletteRedOrange[] = {
  0x660000, 0x990000, 0xCC0000, 0xFF3300, 0xFF6600
};

uint32_t paletteBlueCyan[] = {
  0x0000FF, 0x0033FF, 0x00CCFF, 0x00FFFF, 0x3399FF
};

uint32_t paletteMagentaPurple[] = {
  0xFF00FF, 0xCC00FF, 0x9900FF, 0x6600FF, 0x3300FF
};

uint32_t paletteGreen[] = {
  0xCCFF00, 0x006600, 0x009900, 0x00CC00, 0x00FF00
};

uint32_t paletteLightning[] = {
  0xFCAE1E, 0xFA8128, 0xFFCC33, 0xFFF700, 0xFFC30B
};

// Additional premium backlight palettes
uint32_t paletteIce[] = {
  0x001A66, 0x0055FF, 0x00AFFF, 0x33DDFF, 0x00EEFF
};

uint32_t paletteSunset[] = {
  0x99004D, 0xE60073, 0xFF3366, 0xFF6600, 0xFFB000
};

uint32_t paletteGold[] = {
  0x663300, 0x995C00, 0xCC8800, 0xFFB800, 0xFFE066
};

uint32_t paletteMystic[] = {
  0x190033, 0x3D007A, 0x7300CC, 0x0066FF, 0x00E0CC
};

const int NUM_PALETTES = 9;

// Holo Glint (Mode 7) colors: {halo color, deep halo color, glint color}.
// The glint is drawn at full strength over a slightly dimmed halo, so it
// reads as a moving reflection without using any white.
const uint32_t holoGlintPalettes[NUM_PALETTES][3] = {
  {0xFF1A00, 0x4A0000, 0xFFD000},  // Fire: hot red, deep red, gold
  {0x005BFF, 0x000040, 0x00FFFF},  // Water: electric blue, navy, cyan
  {0xFF00CC, 0x26004D, 0x0080FF},  // Psychic: magenta, deep violet, azure
  {0x00FF55, 0x002000, 0xDFFF00},  // Forest: bright green, deep green, lime
  {0xFF7A00, 0x5A1800, 0xFFFF40},  // Lightning: orange, dark amber, yellow
  {0x0077FF, 0x000033, 0x00FFD0},  // Ice: blue, navy, aqua
  {0xFF007A, 0x4D003D, 0xFFB000},  // Sunset: pink, plum, orange-gold
  {0xFF8A00, 0x3D1700, 0xFFE000},  // Gold: orange-gold, brown, bright gold
  {0x7A00FF, 0x200040, 0x00FFD5}   // Mystic: purple, violet, teal
};

// Higher-contrast color pairs used only by the Dual-Tone mode (Mode 8).
const uint32_t dualToneContrastPairs[NUM_PALETTES][2] = {
  {0xB00000, 0xFFD000},  // Red vs gold
  {0x0033FF, 0x00FFFF},  // Blue vs cyan
  {0xFF00CC, 0x3300FF},  // Magenta vs electric blue
  {0x00A83A, 0xDFFF00},  // Green vs lime
  {0xFF5A00, 0xFFFF55},  // Orange vs bright yellow
  {0x1A00FF, 0x00E5FF},  // Indigo vs glacier cyan
  {0xE00070, 0xFF9A00},  // Pink vs orange
  {0xA84A00, 0xFFC800},  // Amber vs bright gold
  {0x7A00FF, 0x00E8C8}   // Purple vs teal
};

uint32_t staticColors[] = {
  0xFF0000,
  0xFF6600,
  0xFFFF00,
  0x00FF00,
  0x00FFFF,
  0x0000FF,
  0x8000FF,
  0xFF00FF
};

const int numStaticColors = sizeof(staticColors) / sizeof(staticColors[0]);

const int TETRIS_COMBO_COUNT = 9;
const int TETRIS_COLORS_PER_COMBO = 5;

uint32_t tetrisCombos[TETRIS_COMBO_COUNT][TETRIS_COLORS_PER_COMBO] = {
  {0xFF0000, 0xFF6600, 0xFFFF00, 0xFF00FF, 0xFF0066},
  {0x0000FF, 0x00CCFF, 0x00FFFF, 0x8000FF, 0x00FFAA},
  {0x006600, 0x00CC00, 0xCCFF00, 0xFFFF00, 0x66FF00},
  {0x6600FF, 0x9900FF, 0xFF00FF, 0xFF66CC, 0xFF0099},
  {0xFF3300, 0x00FFFF, 0x8000FF, 0xFFFF00, 0x00FF44},
  {0x001A66, 0x0055FF, 0x00AFFF, 0x33DDFF, 0x00EEFF},
  {0x99004D, 0xE60073, 0xFF3366, 0xFF6600, 0xFFB000},
  {0x663300, 0x995C00, 0xCC8800, 0xFFB800, 0xFFE066},
  {0x190033, 0x3D007A, 0x7300CC, 0x0066FF, 0x00E0CC}
};

/*
  Physical LED rows from the supplied PCB layout.

  LED data starts at the upper-left, snakes through the three top rows,
  travels down the right side, crosses the bottom, and returns up the left.
  Grouping the LEDs by their physical height lets the new effects move from
  the actual top of the display to the actual bottom instead of following
  the electrical daisy-chain path.
*/
const int PHYSICAL_ROW_COUNT = 13;

const uint8_t physicalRowSizes[PHYSICAL_ROW_COUNT] = {
  8, 8, 8,
  4, 4, 4, 4, 4, 4, 4, 4, 4,
  6
};

const uint8_t physicalRows[PHYSICAL_ROW_COUNT][8] = {
  { 0,  1,  2,  3,  4,  5,  6,  7},
  { 8,  9, 10, 11, 12, 13, 14, 15},
  {16, 17, 18, 19, 20, 21, 22, 23},
  {24, 25, 64, 65,  0,  0,  0,  0},
  {26, 27, 62, 63,  0,  0,  0,  0},
  {28, 29, 60, 61,  0,  0,  0,  0},
  {30, 31, 58, 59,  0,  0,  0,  0},
  {32, 33, 56, 57,  0,  0,  0,  0},
  {34, 35, 54, 55,  0,  0,  0,  0},
  {36, 37, 52, 53,  0,  0,  0,  0},
  {38, 39, 50, 51,  0,  0,  0,  0},
  {40, 41, 48, 49,  0,  0,  0,  0},
  {42, 43, 44, 45, 46, 47,  0,  0}
};

// Forward declarations keep the sketch valid as either an Arduino .ino or .cpp file.
void handleButtons();
void updateBrightnessRamp();
void readADSInputs();
void updateChargeTracking();
bool isPowerButtonDisturbed();
void setFullChargeLatch(bool latched);
void handleUsbDisconnectShutdown();
void requestStatusLedBlank();
void turnAllLEDsOff();
void updateStatusLED();
void runCurrentMode();
uint32_t* getPalette();
void rainbowMode();
void staticColorMode();
void topBottomGradientMode();
void starFieldMode();
void tetrisMode();
void splitSideMode();
void pokemonEnergyMode();
void setPhysicalRowColor(int row, uint32_t color);
float getPhysicalX(int led);
float getPhysicalY(int led);
void holoGlintMode();
void dualToneBacklightMode();
void playStartupAnimation();
void startLedOutputWithAnimation();
uint32_t scaleColor(uint32_t c, float scale);
uint32_t complementaryColor(uint32_t c);
uint32_t blendColors(uint32_t c1, uint32_t c2, float t);
void rgbToHsv(uint32_t c, uint16_t &hue, uint8_t &sat, uint8_t &val);
uint32_t blendHue(uint32_t c1, uint32_t c2, float t);
void fadeAll(uint8_t amount);
uint32_t Wheel(byte pos);

void setup()
{

  pinMode(HOLD_PIN, OUTPUT);
  digitalWrite(HOLD_PIN, HIGH);

  
  pinMode(IO14_PIN, OUTPUT);
  digitalWrite(IO14_PIN, HIGH);

  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_COLOR, INPUT_PULLUP);
  pinMode(BTN_BRIGHTNESS, INPUT_PULLUP);

  strip.begin();
  strip.setBrightness(brightness);
  strip.show();

  statusLED.begin();
  statusLED.setBrightness(150);
  statusLED.show();

  // Restore the full-charge latch so a replug after power-down still
  // shows green when the battery hasn't been used since it filled.
  prefs.begin("slabdisp", false);
  fullChargeLatched = prefs.getBool("fullLatch", false);

  Wire.begin();
  Wire.setTimeOut(50);

  ads.begin(0x48);
  ads.setGain(GAIN_TWOTHIRDS);

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 10000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };

  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  randomSeed(analogRead(1));

  // Run once after the ESP32, watchdog, and LED driver are initialized.
  startLedOutputWithAnimation();
}

void loop()
{
  esp_task_wdt_reset();

  handleButtons();
  readADSInputs();
  updateChargeTracking();
  updateStatusLED();
  handleUsbDisconnectShutdown();

  if (ledsEnabled)
  {
    updateBrightnessRamp();
    runCurrentMode();
  }

  delay(15);
}

void updateBrightnessRamp()
{
  if (brightness == targetBrightness)
    return;

  if (brightness < targetBrightness)
    brightness = (uint8_t)min((int)brightness + BRIGHTNESS_RAMP_STEP, (int)targetBrightness);
  else
    brightness = (uint8_t)max((int)brightness - BRIGHTNESS_RAMP_STEP, (int)targetBrightness);

  strip.setBrightness(brightness);
}

void handleButtons()
{
  if (!ledsEnabled)
    return;

  bool modeButtonState = digitalRead(BTN_MODE);
  bool colorButtonState = digitalRead(BTN_COLOR);
  bool brightnessButtonState = digitalRead(BTN_BRIGHTNESS);

  if (lastModeButtonState == HIGH && modeButtonState == LOW)
  {
    if (millis() - lastModePressTime > BUTTON_DEBOUNCE_MS)
    {
      currentMode++;
      if (currentMode >= NUM_MODES) currentMode = 0;
      lastModePressTime = millis();
    }
  }

  if (lastColorButtonState == HIGH && colorButtonState == LOW)
  {
    if (millis() - lastColorPressTime > BUTTON_DEBOUNCE_MS)
    {
      if (currentMode == 1 || currentMode == 5 || currentMode == 6)
      {
        staticColorIndex++;
        if (staticColorIndex >= numStaticColors)
          staticColorIndex = 0;
      }
      else
      {
        paletteIndex++;
        if (paletteIndex >= NUM_PALETTES)
          paletteIndex = 0;
      }

      lastColorPressTime = millis();
    }
  }

  if (lastBrightnessButtonState == HIGH && brightnessButtonState == LOW)
  {
    if (millis() - lastBrightnessPressTime > BUTTON_DEBOUNCE_MS)
    {
      brightnessIndex++;
      if (brightnessIndex >= numBrightnessLevels)
        brightnessIndex = 0;

      // Only set the target here. updateBrightnessRamp() moves the strip
      // toward it gradually so the LED current doesn't jump.
      targetBrightness = brightnessLevels[brightnessIndex];

      lastBrightnessPressTime = millis();
    }
  }

  lastModeButtonState = modeButtonState;
  lastColorButtonState = colorButtonState;
  lastBrightnessButtonState = brightnessButtonState;
}

void readADSInputs()
{
  static unsigned long lastADSRead = 0;

  if (millis() - lastADSRead < 20)
    return;

  int16_t an0 = ads.readADC_SingleEnded(0);
  int16_t an1 = ads.readADC_SingleEnded(1);
  int16_t an2 = ads.readADC_SingleEnded(2);

  float an0Voltage = ads.computeVolts(an0);
  float an1Voltage = ads.computeVolts(an1);
  float an2Voltage = ads.computeVolts(an2);

  // AN0 is dedicated to charger detection.
  chargerVoltage = an0Voltage;

  // AN2 is the charge controller's ST output. It is pulled down while
  // charging and goes high once the battery is fully charged.
  chargeStatusVoltage = an2Voltage;

  // Anything on AN1 below the battery-valid floor is the power button
  // pulling the node toward GND, or the node recovering after a press.
  // Record it so the battery reading and the status LED ignore it.
  if (an1Voltage < BATTERY_VALID_MIN_VOLTAGE)
  {
    lastAn1DisturbedTime = millis();
  }

  // Apply hysteresis first: below 1.0 V is pressed, above 2.0 V is released,
  // and the previous candidate state is retained in the undefined middle band.
  bool rawButtonPressed = powerButtonCandidatePressed;

  if (an1Voltage < BUTTON_PRESSED_MAX_VOLTAGE)
  {
    rawButtonPressed = true;
  }
  else if (an1Voltage > BUTTON_RELEASED_MIN_VOLTAGE)
  {
    rawButtonPressed = false;
  }

  if (rawButtonPressed != powerButtonCandidatePressed)
  {
    powerButtonCandidatePressed = rawButtonPressed;
    powerButtonCandidateChangeTime = millis();
  }

  if ((powerButtonStablePressed != powerButtonCandidatePressed) &&
      (millis() - powerButtonCandidateChangeTime >= POWER_BUTTON_DEBOUNCE_MS))
  {
    powerButtonStablePressed = powerButtonCandidatePressed;
  }

  bool buttonPressed = powerButtonStablePressed;
  bool buttonReleased = !powerButtonStablePressed;

  // AN1 is shared with the power-button sensing circuit. Only accept it as a
  // battery measurement when the button is released, the reading is a
  // plausible LiPo voltage, and AN1 has had time to settle after the last
  // press. Otherwise keep the last good (filtered) value.
  if (buttonReleased &&
      an1Voltage >= BATTERY_VALID_MIN_VOLTAGE &&
      (millis() - lastAn1DisturbedTime >= BATTERY_SETTLE_AFTER_RELEASE_MS))
  {
    if (!batteryVoltageValid)
    {
      batteryVoltage = an1Voltage;
      batteryVoltageValid = true;
    }
    else
    {
      batteryVoltage += BATTERY_FILTER_ALPHA * (an1Voltage - batteryVoltage);
    }
  }

  switch(buttonState)
  {
    case WAIT_FOR_STARTUP_RELEASE:
      if (buttonReleased)
      {
        buttonState = SYSTEM_READY;
      }
      break;

    case SYSTEM_READY:
      if (buttonPressed)
      {
        buttonPressStart = millis();
        buttonState = BUTTON_HOLDING;
      }
      break;

    case BUTTON_HOLDING:
      if (buttonReleased)
      {
        buttonState = SYSTEM_READY;
      }
      else if (buttonPressed &&
               (millis() - buttonPressStart >= HOLD_TIME_MS) &&
               (millis() - lastPowerToggleTime >= POWER_BUTTON_LOCKOUT_MS))
      {
        if (an0Voltage > 4.3)
        {
          ledsEnabled = !ledsEnabled;

          pinMode(HOLD_PIN, OUTPUT);
          digitalWrite(HOLD_PIN, HIGH);

          if (ledsEnabled)
          {
            // Reinitialize the WS2812 data line, send a known all-off frame,
            // and then run the same animation used at first startup.
            startLedOutputWithAnimation();
            updateStatusLED();
          }
          else
          {
            strip.setBrightness(0);

            for(int i = 0; i < LED_COUNT; i++)
            {
              strip.setPixelColor(i, 0);
            }

            strip.show();

            pinMode(LED_PIN, OUTPUT);
            digitalWrite(LED_PIN, LOW);

            updateStatusLED();
          }
        }
        else
        {
          // On battery, this press is the power-off. Blank the status LED
          // first so it can't be redrawn while the supply collapses.
          requestStatusLedBlank();

          pinMode(HOLD_PIN, OUTPUT);
          digitalWrite(HOLD_PIN, !digitalRead(HOLD_PIN));
        }

        lastPowerToggleTime = millis();
        buttonState = WAIT_FOR_TOGGLE_RELEASE;
      }
      break;

    case WAIT_FOR_TOGGLE_RELEASE:
      if (buttonReleased)
      {
        buttonState = SYSTEM_READY;
      }
      break;
  }

  lastADSRead = millis();
}

bool isPowerButtonDisturbed()
{
  return (millis() - lastAn1DisturbedTime) < STATUS_FREEZE_AFTER_BUTTON_MS;
}

void setFullChargeLatch(bool latched)
{
  if (fullChargeLatched == latched)
    return;

  fullChargeLatched = latched;

  // Written only when the latch changes, so flash wear is negligible.
  prefs.putBool("fullLatch", latched);
}

void updateChargeTracking()
{
  bool chargerPresent = chargerVoltage > CHARGER_PRESENT_MIN_VOLTAGE;
  bool buttonDisturbed = isPowerButtonDisturbed();
  unsigned long now = millis();

  // ---- Charge complete: ST must be high continuously for 3 s ----
  bool stHigh = chargeStatusVoltage > CHARGE_COMPLETE_MIN_VOLTAGE;

  if (chargerPresent && stHigh && !buttonDisturbed)
  {
    if (!stHighTimerRunning)
    {
      stHighTimerRunning = true;
      stHighTimerStart = now;
    }
    else if (now - stHighTimerStart >= CHARGE_COMPLETE_CONFIRM_MS)
    {
      chargeCompleteConfirmed = true;
    }
  }
  else
  {
    stHighTimerRunning = false;
    chargeCompleteConfirmed = false;
  }

  // ---- Full-charge latch ----
  if (chargerPresent)
  {
    if (chargeCompleteConfirmed)
    {
      setFullChargeLatch(true);
    }

    fullLatchClearTimerRunning = false;
  }
  else if (fullChargeLatched && batteryVoltageValid && !buttonDisturbed)
  {
    // On battery power, clear the latch once the cell has genuinely been
    // used down, not on a momentary sag.
    if (batteryVoltage < FULL_LATCH_CLEAR_VOLTAGE)
    {
      if (!fullLatchClearTimerRunning)
      {
        fullLatchClearTimerRunning = true;
        fullLatchClearTimerStart = now;
      }
      else if (now - fullLatchClearTimerStart >= FULL_LATCH_CLEAR_CONFIRM_MS)
      {
        setFullChargeLatch(false);
        fullLatchClearTimerRunning = false;
      }
    }
    else
    {
      fullLatchClearTimerRunning = false;
    }
  }

  // ---- Low battery with hysteresis and a sustained-low requirement ----
  if (chargerPresent || !batteryVoltageValid)
  {
    // While charging, AN1 reads the charger's output, not the resting cell.
    lowBatteryActive = false;
    lowBatteryTimerRunning = false;
  }
  else if (!buttonDisturbed)
  {
    if (!lowBatteryActive)
    {
      if (batteryVoltage < LOW_BATTERY_ENTER_VOLTAGE)
      {
        if (!lowBatteryTimerRunning)
        {
          lowBatteryTimerRunning = true;
          lowBatteryTimerStart = now;
        }
        else if (now - lowBatteryTimerStart >= LOW_BATTERY_CONFIRM_MS)
        {
          lowBatteryActive = true;
        }
      }
      else
      {
        lowBatteryTimerRunning = false;
      }
    }
    else if (batteryVoltage > LOW_BATTERY_EXIT_VOLTAGE)
    {
      lowBatteryActive = false;
      lowBatteryTimerRunning = false;
    }
  }
}

void handleUsbDisconnectShutdown()
{
  if (chargerVoltage < 4.6 && ledsEnabled == false)
  {
    requestStatusLedBlank();
    turnAllLEDsOff();
    digitalWrite(HOLD_PIN, LOW);
  }
}

void requestStatusLedBlank()
{
  statusBlankActive = true;
  statusBlankRequestTime = millis();

  // Push the off state out right away, before HOLD_PIN drops.
  statusLED.setPixelColor(0, 0);
  statusLED.show();
}

void turnAllLEDsOff()
{
  strip.setBrightness(0);

  for(int i = 0; i < LED_COUNT; i++)
  {
    strip.setPixelColor(i, 0);
  }

  strip.show();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  updateStatusLED();
}

void updateStatusLED()
{
  // After a power-off request, keep the status LED dark. If the board is
  // still alive once requests stop (USB is powering it), resume normally.
  if (statusBlankActive)
  {
    if ((millis() - statusBlankRequestTime < STATUS_BLANK_RELEASE_MS) ||
        isPowerButtonDisturbed())
    {
      statusLED.setPixelColor(0, 0);
      statusLED.show();
      return;
    }

    statusBlankActive = false;

    // Any color change after blanking goes through the full debounce.
    pendingBatteryStatus = batteryStatus;
    batteryStatusChangeStart = millis();
  }

  if (!isPowerButtonDisturbed())
  {
    BatteryStatusState requestedStatus;
    bool chargerPresent = chargerVoltage > CHARGER_PRESENT_MIN_VOLTAGE;

    if (chargerPresent)
    {
      // Green: ST has confirmed full, or the battery was already full and
      // hasn't been used since (quick unplug/replug restarts the charger
      // and pulls ST low again, but the cell is still full).
      if (chargeCompleteConfirmed || fullChargeLatched)
      {
        requestedStatus = STATUS_FULL;
      }
      else
      {
        // Yellow: charger present, charge not yet complete.
        requestedStatus = STATUS_CHARGING;
      }
    }
    else if (lowBatteryActive)
    {
      requestedStatus = STATUS_LOW_BATTERY;
    }
    else
    {
      requestedStatus = STATUS_OFF;
    }

    if (requestedStatus != pendingBatteryStatus)
    {
      pendingBatteryStatus = requestedStatus;
      batteryStatusChangeStart = millis();
    }

    if ((batteryStatus != pendingBatteryStatus) &&
        (millis() - batteryStatusChangeStart >= BATTERY_STATUS_DEBOUNCE_MS))
    {
      batteryStatus = pendingBatteryStatus;
    }
  }
  else
  {
    // Power button active: hold the current color, and make any change that
    // follows the press go through the full debounce from scratch.
    pendingBatteryStatus = batteryStatus;
    batteryStatusChangeStart = millis();
  }

  if (batteryStatus == STATUS_CHARGING)
  {
    // Yellow: charger present, charge not complete.
    statusLED.setPixelColor(0, statusLED.Color(255, 180, 0));
  }
  else if (batteryStatus == STATUS_FULL)
  {
    statusLED.setPixelColor(0, statusLED.Color(0, 255, 0));
  }
  else if (batteryStatus == STATUS_LOW_BATTERY)
  {
    statusLED.setPixelColor(0, statusLED.Color(255, 0, 0));
  }
  else
  {
    statusLED.setPixelColor(0, 0);
  }

  statusLED.show();
}

void runCurrentMode()
{
  switch(currentMode)
  {
    case 0: rainbowMode(); break;
    case 1: staticColorMode(); break;
    case 2: topBottomGradientMode(); break;
    case 3: starFieldMode(); break;
    case 4: tetrisMode(); break;
    case 5: splitSideMode(); break;
    case 6: pokemonEnergyMode(); break;
    case 7: holoGlintMode(); break;
    case 8: dualToneBacklightMode(); break;
  }
}

uint32_t* getPalette()
{
  switch(paletteIndex)
  {
    case 0: return paletteRedOrange;
    case 1: return paletteBlueCyan;
    case 2: return paletteMagentaPurple;
    case 3: return paletteGreen;
    case 4: return paletteLightning;
    case 5: return paletteIce;
    case 6: return paletteSunset;
    case 7: return paletteGold;
    default: return paletteMystic;
  }
}

void rainbowMode()
{
  static float shift = 0;

  for(int i = 0; i < LED_COUNT; i++)
  {
    uint8_t hue = sin((i * 0.06) + shift) * 127 + 128;
    strip.setPixelColor(i, Wheel(hue));
  }

  strip.show();
  shift += 0.025;
}

void staticColorMode()
{
  uint32_t c = staticColors[staticColorIndex];

  for(int i = 0; i < LED_COUNT; i++)
  {
    strip.setPixelColor(i, c);
  }

  strip.show();
}

void topBottomGradientMode()
{
  static float offset = 0;
  uint32_t* palette = getPalette();
  const int paletteSize = 5;

  for(int i = 0; i < LED_COUNT; i++)
  {
    float pos = ((float)i / LED_COUNT) * (paletteSize - 1) + offset;

    while(pos >= paletteSize) pos -= paletteSize;

    int idx1 = (int)pos;
    int idx2 = (idx1 + 1) % paletteSize;
    float t = pos - idx1;

    strip.setPixelColor(i, blendColors(palette[idx1], palette[idx2], t));
  }

  strip.show();
  offset += 0.025;
}

void starFieldMode()
{
  fadeAll(18);

  uint32_t* palette = getPalette();

  for(int i = 0; i < 4; i++)
  {
    strip.setPixelColor(random(LED_COUNT), palette[random(5)]);
  }

  strip.show();
}

void tetrisMode()
{
  static int pos = 0;
  static uint8_t speedAccumulator = 0;

  int combo = paletteIndex % TETRIS_COMBO_COUNT;

  for(int i = 0; i < LED_COUNT; i++)
  {
    int colorIndex = ((i / 4) + (pos / 8)) % (TETRIS_COLORS_PER_COMBO - 1);
    int nextColorIndex = (colorIndex + 1) % (TETRIS_COLORS_PER_COMBO - 1);

    float blendPos = ((i % 4) / 4.0);
    uint32_t baseColor = blendHue(
      tetrisCombos[combo][colorIndex],
      tetrisCombos[combo][nextColorIndex],
      blendPos
    );

    float breathe = (sin((i * 0.18) + (pos * 0.10)) + 1.0) * 0.5;
    strip.setPixelColor(i, scaleColor(baseColor, 0.10 + (breathe * 0.10)));
  }

  for(int i = 0; i < 12; i++)
  {
    float tail = 1.0 - ((float)i / 12.0);
    int led = (pos - i + LED_COUNT) % LED_COUNT;

    uint32_t c1 = tetrisCombos[combo][i % TETRIS_COLORS_PER_COMBO];
    uint32_t c2 = tetrisCombos[combo][(i + 1) % TETRIS_COLORS_PER_COMBO];
    uint32_t trailColor = blendHue(c1, c2, tail);

    strip.setPixelColor(led, scaleColor(trailColor, 0.25 + (tail * 0.75)));
  }

  int secondPos = (pos + 31) % LED_COUNT;

  for(int i = 0; i < 6; i++)
  {
    float tail = 1.0 - ((float)i / 6.0);
    int led = (secondPos - i + LED_COUNT) % LED_COUNT;
    uint32_t trailColor = tetrisCombos[combo][(i + 2) % TETRIS_COLORS_PER_COMBO];
    strip.setPixelColor(led, scaleColor(trailColor, 0.15 + (tail * 0.55)));
  }

  strip.show();

  // Advance one position every frame and one extra position every third
  // frame. This makes the mode approximately 33% faster without affecting
  // the timing of the other LED modes.
  pos++;
  speedAccumulator++;

  if (speedAccumulator >= 3)
  {
    pos++;
    speedAccumulator = 0;
  }

  while (pos >= LED_COUNT)
    pos -= LED_COUNT;
}

void splitSideMode()
{
  static float verticalOffset = 0.0;

  uint32_t c1 = staticColors[staticColorIndex];
  uint32_t c2 = staticColors[(staticColorIndex + 3) % numStaticColors];

  const float blendWidth = 14.0;
  const float half = LED_COUNT / 2.0;

  for(int i = 0; i < LED_COUNT; i++)
  {
    float pos = (float)i + verticalOffset;

    while(pos >= LED_COUNT) pos -= LED_COUNT;
    while(pos < 0) pos += LED_COUNT;

    uint32_t color;

    if (pos < half - blendWidth)
    {
      color = c1;
    }
    else if (pos < half + blendWidth)
    {
      float t = (pos - (half - blendWidth)) / (blendWidth * 2.0);
      t = t * t * (3.0 - 2.0 * t);
      color = blendHue(c1, c2, t);
    }
    else if (pos < LED_COUNT - blendWidth)
    {
      color = c2;
    }
    else
    {
      float t = (pos - (LED_COUNT - blendWidth)) / blendWidth;
      t = constrain(t, 0.0, 1.0);
      t = t * t * (3.0 - 2.0 * t);
      color = blendHue(c2, c1, t);
    }

    float shimmer = (sin((i * 0.10) + (verticalOffset * 0.20)) + 1.0) * 0.5;
    color = scaleColor(color, 0.86 + (shimmer * 0.14));

    strip.setPixelColor(i, color);
  }

  strip.show();

  verticalOffset += 0.035;
  if (verticalOffset >= LED_COUNT) verticalOffset -= LED_COUNT;
}

void pokemonEnergyMode()
{
  uint32_t colorA = staticColors[staticColorIndex];
  uint32_t colorB = staticColors[(staticColorIndex + 3) % numStaticColors];
  uint32_t midpointColor = blendHue(colorA, colorB, 0.5);

  for(int i = 0; i < LED_COUNT; i++)
  {
    if (i < 16)
    {
      strip.setPixelColor(i, colorA);
    }
    else if (i >= 16 && i < 24)
    {
      strip.setPixelColor(i, midpointColor);
    }
    else
    {
      strip.setPixelColor(i, colorB);
    }
  }

  strip.show();
}

void setPhysicalRowColor(int row, uint32_t color)
{
  if (row < 0 || row >= PHYSICAL_ROW_COUNT)
    return;

  for(int i = 0; i < physicalRowSizes[row]; i++)
  {
    strip.setPixelColor(physicalRows[row][i], color);
  }
}

float getPhysicalX(int led)
{
  // Three top rows: left-to-right, right-to-left, left-to-right.
  if (led >= 0 && led < 8)
    return led / 7.0;

  if (led >= 8 && led < 16)
    return (15 - led) / 7.0;

  if (led >= 16 && led < 24)
    return (led - 16) / 7.0;

  // Two distinct columns on the right side.
  if (led >= 24 && led < 42)
    return (((led - 24) % 2) == 0) ? 0.84 : 1.0;

  // Bottom row runs electrically from right to left.
  if (led >= 42 && led < 48)
    return (47 - led) / 5.0;

  // Two distinct columns on the left side.
  return ((led % 2) == 0) ? 0.0 : 0.16;
}

float getPhysicalY(int led)
{
  for(int row = 0; row < PHYSICAL_ROW_COUNT; row++)
  {
    for(int i = 0; i < physicalRowSizes[row]; i++)
    {
      if (physicalRows[row][i] == led)
      {
        return (float)row / (float)(PHYSICAL_ROW_COUNT - 1);
      }
    }
  }

  return 0.0;
}

void holoGlintMode()
{
  // Showcase effect for graded slabs. A rich, steady halo frames the card,
  // and every few seconds a bright diagonal glint in a contrasting color
  // sweeps across the display like light catching holo foil, leaving a short
  // trail of sparkles. The
  // quiet time between sweeps keeps attention on the card itself.
  const unsigned long GLINT_PERIOD_MS = 5200;   // one sweep every 5.2 s
  const unsigned long GLINT_SWEEP_MS = 1500;    // time for one sweep to cross
  const unsigned long BREATH_PERIOD_MS = 7000;  // slow halo breathing
  const float CORE_HALF_WIDTH = 0.10;           // bright center of the glint
  const float BLOOM_HALF_WIDTH = 0.30;          // soft glow around the glint
  const float BLOOM_LEVEL = 0.35;

  static uint8_t sparkle[LED_COUNT] = {0};

  const uint32_t* colors = holoGlintPalettes[paletteIndex];
  uint32_t glintColor = colors[2];

  unsigned long now = millis();

  // Gentle breathing keeps the halo alive without pulling focus. The halo
  // stays below full brightness so the colored glint still stands out.
  float breathPhase = (float)(now % BREATH_PERIOD_MS) / (float)BREATH_PERIOD_MS;
  float breath = 0.70 + (0.15 * ((sin(breathPhase * 2.0 * PI) + 1.0) * 0.5));

  // Glint travels from the upper-left corner to the lower-right corner.
  // Between sweeps it is parked far off the display so nothing is drawn.
  unsigned long cycle = now % GLINT_PERIOD_MS;
  float glintCenter = -10.0;

  if (cycle < GLINT_SWEEP_MS)
  {
    float progress = (float)cycle / (float)GLINT_SWEEP_MS;
    glintCenter = -BLOOM_HALF_WIDTH + (progress * (1.0 + (2.0 * BLOOM_HALF_WIDTH)));
  }

  for(int i = 0; i < LED_COUNT; i++)
  {
    float x = getPhysicalX(i);
    float y = getPhysicalY(i);

    // Base halo: vivid at the top, deepening slightly toward the bottom.
    uint32_t color = blendColors(colors[0], colors[1], y * 0.55);
    color = scaleColor(color, breath);

    // Diagonal coordinate: 0 at the upper-left, 1 at the lower-right.
    float diagonal = (x * 0.45) + (y * 0.55);
    float distance = fabs(diagonal - glintCenter);

    float core = constrain(1.0 - (distance / CORE_HALF_WIDTH), 0.0, 1.0);
    core = core * core * (3.0 - (2.0 * core));

    float bloom = constrain(1.0 - (distance / BLOOM_HALF_WIDTH), 0.0, 1.0);
    bloom = bloom * bloom * BLOOM_LEVEL;

    float glint = (core > bloom) ? core : bloom;
    color = blendColors(color, glintColor, glint * 0.95);

    // The bright core occasionally leaves a sparkle behind as it passes.
    if (core > 0.55 && random(1000) < 35)
    {
      sparkle[i] = 255;
    }

    if (sparkle[i] > 0)
    {
      color = blendColors(color, glintColor, sparkle[i] / 255.0);
      sparkle[i] = (uint8_t)(((uint16_t)sparkle[i] * 215) >> 8);
    }

    strip.setPixelColor(i, color);
  }

  strip.show();
}

void dualToneBacklightMode()
{
  static float shimmerPhase = 0.0;

  uint32_t leftColor = dualToneContrastPairs[paletteIndex][0];
  uint32_t rightColor = dualToneContrastPairs[paletteIndex][1];

  for(int i = 0; i < LED_COUNT; i++)
  {
    float x = getPhysicalX(i);
    float y = getPhysicalY(i);

    // Hold each side near its full color and restrict blending to the center.
    // This produces a stronger left-versus-right contrast than a full-width blend.
    float blend = constrain((x - 0.36) / 0.28, 0.0, 1.0);
    blend = blend * blend * (3.0 - (2.0 * blend));

    uint32_t color = blendColors(leftColor, rightColor, blend);

    float leftPulse = (sin(shimmerPhase - (y * 2.5)) + 1.0) * 0.5;
    float rightPulse = (sin(shimmerPhase + PI + (y * 2.5)) + 1.0) * 0.5;
    float pulse = (1.0 - blend) * leftPulse + blend * rightPulse;
    float intensity = 0.78 + (pulse * 0.22);

    strip.setPixelColor(i, scaleColor(color, intensity));
  }

  strip.show();

  shimmerPhase += 0.030;
  if (shimmerPhase >= (2.0 * PI))
    shimmerPhase -= (2.0 * PI);
}


void playStartupAnimation()
{
  uint32_t* palette = getPalette();

  // Start from a guaranteed dark frame.
  strip.setBrightness(brightness);
  strip.clear();
  strip.show();
  delay(3);

  // A soft top-to-bottom wave follows the actual physical LED rows.
  // Only a small number of rows are bright at once to avoid a large
  // startup-current step.
  const int trailLength = 5;

  for(int frame = 0; frame < PHYSICAL_ROW_COUNT + trailLength; frame++)
  {
    strip.clear();

    for(int row = 0; row < PHYSICAL_ROW_COUNT; row++)
    {
      int distanceBehindHead = frame - row;

      if (distanceBehindHead >= 0 && distanceBehindHead < trailLength)
      {
        float intensity = 1.0 -
          ((float)distanceBehindHead / (float)trailLength);

        int colorIndex = (row + (frame / 2)) % 5;
        uint32_t color = scaleColor(
          palette[colorIndex],
          0.20 + (intensity * 0.80)
        );

        setPhysicalRowColor(row, color);
      }
    }

    strip.show();
    esp_task_wdt_reset();
    delay(35);
  }

  // Leave a clean buffer so the selected operating mode starts normally
  // on the following loop iteration.
  strip.clear();
  strip.show();
  delay(3);
}

void startLedOutputWithAnimation()
{
  // The normal off routine holds the WS2812 data pin low. Reinitialize it,
  // keep it low long enough to reset the LED chain, and send an all-off
  // frame before any visible animation data. This prevents random pixels
  // from flashing when the LEDs are enabled again.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(2);

  strip.begin();
  strip.setBrightness(brightness);
  strip.clear();
  strip.show();
  delay(2);

  playStartupAnimation();
}

uint32_t scaleColor(uint32_t c, float scale)
{
  scale = constrain(scale, 0.0, 1.0);

  uint8_t r = (c >> 16) & 0xFF;
  uint8_t g = (c >> 8) & 0xFF;
  uint8_t b = c & 0xFF;

  return strip.Color(r * scale, g * scale, b * scale);
}

uint32_t complementaryColor(uint32_t c)
{
  uint8_t r = (c >> 16) & 0xFF;
  uint8_t g = (c >> 8) & 0xFF;
  uint8_t b = c & 0xFF;

  return strip.Color(255 - r, 255 - g, 255 - b);
}

uint32_t blendColors(uint32_t c1, uint32_t c2, float t)
{
  uint8_t r1 = (c1 >> 16) & 0xFF;
  uint8_t g1 = (c1 >> 8) & 0xFF;
  uint8_t b1 = c1 & 0xFF;

  uint8_t r2 = (c2 >> 16) & 0xFF;
  uint8_t g2 = (c2 >> 8) & 0xFF;
  uint8_t b2 = c2 & 0xFF;

  return strip.Color(
    r1 * (1 - t) + r2 * t,
    g1 * (1 - t) + g2 * t,
    b1 * (1 - t) + b2 * t
  );
}

void rgbToHsv(uint32_t c, uint16_t &hue, uint8_t &sat, uint8_t &val)
{
  int r = (c >> 16) & 0xFF;
  int g = (c >> 8) & 0xFF;
  int b = c & 0xFF;

  int maxC = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
  int minC = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
  int delta = maxC - minC;

  val = maxC;
  sat = (maxC == 0) ? 0 : (uint8_t)((delta * 255) / maxC);

  if (delta == 0)
  {
    hue = 0;
    return;
  }

  // Hue uses the same 0-65535 scale as Adafruit_NeoPixel::ColorHSV().
  float h;
  if (maxC == r)
    h = (float)(g - b) / delta;
  else if (maxC == g)
    h = 2.0 + ((float)(b - r) / delta);
  else
    h = 4.0 + ((float)(r - g) / delta);

  if (h < 0.0)
    h += 6.0;

  hue = (uint16_t)(h * (65536.0 / 6.0));
}

uint32_t blendHue(uint32_t c1, uint32_t c2, float t)
{
  // A straight RGB blend between opposite colors (yellow and blue, or red
  // and cyan) passes through gray, which reads as white on WS2812s. Blending
  // around the color wheel instead keeps every in-between color saturated.
  uint16_t h1, h2;
  uint8_t s1, s2, v1, v2;

  rgbToHsv(c1, h1, s1, v1);
  rgbToHsv(c2, h2, s2, v2);

  // Take the shorter way around the color wheel.
  int32_t dh = (int32_t)h2 - (int32_t)h1;
  if (dh > 32768)
    dh -= 65536;
  else if (dh < -32768)
    dh += 65536;

  uint16_t h = (uint16_t)((int32_t)h1 + (int32_t)(dh * t));
  uint8_t sat = (uint8_t)(s1 + ((int)s2 - (int)s1) * t);
  uint8_t val = (uint8_t)(v1 + ((int)v2 - (int)v1) * t);

  return strip.ColorHSV(h, sat, val);
}

void fadeAll(uint8_t amount)
{
  for(int i = 0; i < LED_COUNT; i++)
  {
    uint32_t c = strip.getPixelColor(i);

    int r = (c >> 16 & 0xFF) - amount;
    int g = (c >> 8 & 0xFF) - amount;
    int b = (c & 0xFF) - amount;

    strip.setPixelColor(
      i,
      max(0, r),
      max(0, g),
      max(0, b)
    );
  }
}

uint32_t Wheel(byte pos)
{
  if(pos < 85)
  {
    return strip.Color(
      pos * 3,
      255 - pos * 3,
      0
    );
  }
  else if(pos < 170)
  {
    pos -= 85;

    return strip.Color(
      255 - pos * 3,
      0,
      pos * 3
    );
  }
  else
  {
    pos -= 170;

    return strip.Color(
      0,
      pos * 3,
      255 - pos * 3
    );
  }
}
