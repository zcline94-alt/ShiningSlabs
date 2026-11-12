/*
=========================================================
PREMIUM POKEMON SLAB DISPLAY version 2.2
ESP32-S2 + 66 WS2812 LEDs

v2.2: Wi-Fi transmit power lowered from 8.5 dBm to 2 dBm to reduce the
  current bursts that were disturbing LEDs. Range is shorter: keep the
  phone within a few meters of the display.

v2.1: The status LED on D12 shows solid blue while Wi-Fi is on (the ripple
  animation is removed). Wi-Fi timeout is now 2 min.

v2.0: Wi-Fi is off unless the user asks for it.
  - Wi-Fi no longer starts at power-up. Hold the Mode button for 3 s to turn
    it on.
  - Wi-Fi turns itself off after WIFI_TIMEOUT_MS (2 min) with no requests
    from the phone app. The open app checks in every 2.5 s, so Wi-Fi stays
    on while the page is open and times out 2 min after it is closed or the
    phone is locked. Holding Mode for 3 s again restarts the 2 min.
  - A short Mode press still changes the mode, but now on release, so a
    3 s hold doesn't also change the mode. Releasing before 3 s = next mode.
  - Everything else is unchanged from the code this was built on.

v1.8: Old-PCB support via BOARD_VERSION.
  - On the old PCB, AN0 is a regulated rail, not raw USB: it reads about
    4.50 V on battery and 4.81 V on USB. v1.7's single 4.4 V threshold saw
    USB as always present, so the power button only toggled the LEDs and
    never cut power.
  - Charger detection now uses hysteresis per board: USB counts as present
    above CHARGER_ON_VOLTAGE and as gone below CHARGER_OFF_VOLTAGE, still
    with the 150 ms debounce.
      New PCB: on > 4.40 V, off < 4.40 V (same as v1.7)
      Old PCB: on > 4.70 V, off < 4.62 V
  - BOARD_VERSION 1 also sets HAS_CHARGE_STATUS_PIN to 0 (no ST on AN2;
    AN2 floats around 0.6 V on that board).

v1.7: One firmware for both board versions.
  - Set HAS_CHARGE_STATUS_PIN to 0 for boards where the charge controller's
    ST pin is NOT wired to AN2. Those boards decide "full" (green) from the
    battery voltage instead: AN1 must stay at or above 4.15 V on USB for
    40 minutes, which covers most of the charger's constant-voltage taper.
    AN2 is then ignored completely, so a floating input can't cause a
    random green.
  - With HAS_CHARGE_STATUS_PIN 1 (boards with ST on AN2) nothing changes.

v1.6: Status LED fix and "My Design" mode.
  - Status LED frames are now sent only when the color changes, plus a
    1 s refresh, instead of on every loop pass (~50 per second). With Wi-Fi
    running, each TX burst is a current step on the shared rail, the same
    mechanism behind the v1.4 green flash; 50 frames a second gave those
    bursts constant chances to corrupt a frame.
  - Charger detection is one debounced decision (AN0 must sit on one side of
    4.4 V for 150 ms) used everywhere. v1.5 compared raw AN0 against three
    different thresholds (4.3 / 4.4 / 4.6). With Wi-Fi adding load on USB,
    AN0 could sag into the 4.4-4.6 V band, where the shutdown routine
    blanked the status LED over and over while the charger logic still saw
    USB as present.
  - New mode 9, "My Design": every one of the 66 LEDs gets its own color,
    painted on a map of the display in the phone app. The design is saved
    to flash with the other settings.
  - /api/state now also returns raw AN0/AN1/AN2 readings, shown under
    Diagnostics in the app.
  - Set ENABLE_WEB_CONTROL to 0 to build without Wi-Fi (useful to A/B test
    any remaining status LED behavior).

v1.5: Phone control. The display starts its own Wi-Fi network
  ("ShiningGuard-XXXX"). Join it from a phone and the control page opens
  (captive portal), or browse to http://192.168.4.1. From there you can
  pick the LED mode, palette or color (including two custom colors from a
  color picker), set brightness on a slider, and turn the display off.
  - HTTP API (form-encoded, JSON replies):
      GET  /api/info   mode / palette / color names and swatches
      GET  /api/state  current settings, charger and battery status
      POST /api/set    any of: mode, palette, color, custom, custom2,
                       brightness, leds  (see handleApiSet())
  - Mode, palette, color, custom colors and brightness are saved to flash
    5 s after the last change (from the app or the buttons) and restored at
    power-up. They are also flushed right before any power-off.
  - App brightness changes go through the same ramp as the button, and are
    clamped to the same 145 ceiling.
  - Turning the display off from the app behaves like the power button: on
    USB the LEDs go dark and Wi-Fi stays up; on battery the device powers
    down (HOLD_PIN low) and the button turns it back on.
  - Wi-Fi starts after the startup animation and runs at reduced TX power
    to limit supply droop and battery draw.
  - The physical buttons and status LED behave exactly as in v1.4.

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
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

// 0 = no Wi-Fi or phone control at all (buttons only, like v1.4).
#define ENABLE_WEB_CONTROL 1

// Which PCB this build is for.
//   2 = current PCB (AN0 is the raw USB line, ST wired to AN2)
//   1 = old PCB     (AN0 is a regulated rail: ~4.50 V on battery, ~4.81 V
//                    on USB; ST not wired to AN2)
#define BOARD_VERSION 2

#if BOARD_VERSION == 1
  // 0 = no ST connection; "full" is judged from battery voltage and time.
  #define HAS_CHARGE_STATUS_PIN 0
#else
  // 1 = charge controller ST pin wired to AN2 (green when ST reports full).
  #define HAS_CHARGE_STATUS_PIN 1
#endif

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

const int NUM_MODES = 10;
const int MODE_MY_DESIGN = 9;

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

// Debounced charger decision. Every part of the firmware uses this one
// value (isChargerPresent()) instead of comparing raw AN0 readings.
const unsigned long CHARGER_DEBOUNCE_MS = 150;
bool chargerPresentStable = false;
bool chargerCandidate = false;
bool chargerStateInitialized = false;
unsigned long chargerCandidateTime = 0;

// Last raw readings, reported in /api/state for diagnostics.
float rawAn0Voltage = 0.0;
float rawAn1Voltage = 0.0;
float rawAn2Voltage = 0.0;
float batteryVoltage = 0.0;       // AN1: filtered battery voltage (valid readings only)
float chargeStatusVoltage = 0.0;  // AN2: charge controller ST pin, high when battery is full

bool batteryVoltageValid = false; // false until the first real battery reading

// Charger detection on AN0, with hysteresis. USB is "present" once AN0
// rises above CHARGER_ON_VOLTAGE and "gone" once it falls below
// CHARGER_OFF_VOLTAGE; in between the previous state is kept.
#if BOARD_VERSION == 1
  // Old PCB: battery ~4.50 V, USB ~4.81 V. Off sits 0.12 V above the
  // battery reading; on sits 0.11 V below the USB reading so Wi-Fi/LED
  // load sag on USB doesn't drop it.
  const float CHARGER_ON_VOLTAGE  = 4.70;
  const float CHARGER_OFF_VOLTAGE = 4.62;
#else
  const float CHARGER_ON_VOLTAGE  = 4.40;
  const float CHARGER_OFF_VOLTAGE = 4.40;
#endif
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

// Used only when HAS_CHARGE_STATUS_PIN is 0. A Li-ion charger holds the cell
// near 4.2 V while current tapers, so voltage alone can't see "done"; holding
// above this level for a long time is the next best sign. The 50 mV band
// keeps filter ripple from restarting the 40-minute timer.
const float FULL_BY_VOLTAGE_MIN = 4.15;
const float FULL_BY_VOLTAGE_RESET = 4.10;
const unsigned long FULL_BY_VOLTAGE_HOLD_MS = 40UL * 60UL * 1000UL;

bool fullByVoltageTimerRunning = false;
unsigned long fullByVoltageTimerStart = 0;

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

// ---------------- Custom colors (set from the phone) ----------------
// staticColorIndex == CUSTOM_COLOR_INDEX selects the custom slot. Static
// Color mode uses customColorA; Split Side and Energy use A and B. The
// color button cycles presets only, so pressing it from the custom slot
// wraps back to the first preset.
const int CUSTOM_COLOR_INDEX = numStaticColors;

// ---------------- My Design (mode 9) ----------------
// One color per LED, in strip (electrical) order. Painted from the app.
uint32_t designPixels[LED_COUNT];
bool designDirty = false;
uint32_t customColorA = 0xFF00AA;
uint32_t customColorB = 0x00E0FF;

// ---------------- Phone control (Wi-Fi access point + web API) ----------------
// The SSID gets the last two MAC bytes appended, e.g. "ShiningGuard-3F9A".
// Change the password before shipping (WPA2 needs 8+ characters).
const char* WIFI_AP_SSID_PREFIX = "ShiningGuard-";
const char* WIFI_AP_PASSWORD    = "shiningguard";
const IPAddress WIFI_AP_IP(192, 168, 4, 1);
const IPAddress WIFI_AP_MASK(255, 255, 255, 0);

WebServer server(80);
DNSServer dnsServer;
bool webControlStarted = false;
bool webRoutesRegistered = false;

// Hold the Mode button this long to turn Wi-Fi on.
const unsigned long WIFI_HOLD_MS = 3000;

// Wi-Fi turns off after this long with no requests from the phone app.
const unsigned long WIFI_TIMEOUT_MS = 2UL * 60UL * 1000UL;
unsigned long lastWebActivityTime = 0;

// Mode button: debounced level, plus press tracking for short vs. long press.
const unsigned long MODE_BUTTON_STABLE_MS = 30;
bool modeButtonCandidate = HIGH;
bool modeButtonStable = HIGH;
unsigned long modeButtonCandidateTime = 0;
bool modePressActive = false;
bool modeLongPressFired = false;
unsigned long modePressStart = 0;
char wifiSsid[24] = "";

// Brightness range accepted from the app. The ceiling matches the highest
// button level so the app can't push LED current past what v1.4 allowed.
const uint8_t BRIGHTNESS_MIN = 10;
const uint8_t BRIGHTNESS_MAX = 145;

// Settings are written to flash this long after the last change, so a
// slider drag or a run of button presses becomes a single write.
const unsigned long SETTINGS_SAVE_DELAY_MS = 5000;
bool settingsDirty = false;
unsigned long settingsDirtyTime = 0;

const char* const modeNames[] = {
  "Rainbow", "Static Color", "Gradient Flow", "Star Field", "Tetris",
  "Split Side", "Energy", "Holo Glint", "Dual Tone", "My Design"
};

const char* const paletteNames[] = {
  "Fire", "Water", "Psychic", "Forest", "Lightning",
  "Ice", "Sunset", "Gold", "Mystic"
};

const char* const staticColorNames[] = {
  "Red", "Orange", "Yellow", "Green", "Cyan", "Blue", "Violet", "Magenta"
};

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
uint32_t getStaticColorA();
void myDesignMode();
void setDefaultDesign();
void writeStatusLed(uint32_t color, bool force);
void handleApiDesign();
uint32_t getStaticColorB();
bool modeUsesStaticColors(int mode);
bool isChargerPresent();
void setLedsEnabled(bool enabled);
void powerOffNow();
int clampInt(int value, int low, int high);
int nearestBrightnessIndex(uint8_t level);
void loadUserSettings();
void markSettingsDirty();
void saveUserSettingsNow();
void saveUserSettingsIfDue();
void startWebControl();
void stopWebControl();
void handleWebControl();
void enableWifiFromButton();
void noteWebActivity();
void handleApiInfo();
void handleApiState();
void handleApiSet();
void handleWebRoot();
void handleWebNotFound();
String buildStateJson();
String colorToHex(uint32_t c);
bool parseHexColor(const String &text, uint32_t &out);

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

  // Restore the last mode, colors and brightness chosen by app or buttons.
  loadUserSettings();
  strip.setBrightness(brightness);

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

  // Wi-Fi stays off at power-up. Hold the Mode button for 3 s to turn it on.
}

void loop()
{
  esp_task_wdt_reset();

  handleButtons();
  readADSInputs();
  updateChargeTracking();
  updateStatusLED();
  handleUsbDisconnectShutdown();
  handleWebControl();
  saveUserSettingsIfDue();

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
  {
    modePressActive = false;
    return;
  }

  bool modeButtonState = digitalRead(BTN_MODE);
  bool colorButtonState = digitalRead(BTN_COLOR);
  bool brightnessButtonState = digitalRead(BTN_BRIGHTNESS);

  // ---- Mode button ----
  // The level must hold for MODE_BUTTON_STABLE_MS to count. A press that is
  // released before WIFI_HOLD_MS changes the mode (on release). Holding past
  // WIFI_HOLD_MS turns on Wi-Fi instead, and that release changes nothing.
  if (modeButtonState != modeButtonCandidate)
  {
    modeButtonCandidate = modeButtonState;
    modeButtonCandidateTime = millis();
  }

  if (modeButtonStable != modeButtonCandidate &&
      millis() - modeButtonCandidateTime >= MODE_BUTTON_STABLE_MS)
  {
    modeButtonStable = modeButtonCandidate;

    if (modeButtonStable == LOW)
    {
      // Pressed.
      modePressActive = true;
      modeLongPressFired = false;
      modePressStart = millis();
    }
    else if (modePressActive)
    {
      // Released.
      modePressActive = false;

      if (!modeLongPressFired)
      {
        currentMode++;
        if (currentMode >= NUM_MODES) currentMode = 0;
        lastModePressTime = millis();
        markSettingsDirty();
      }
    }
  }

  if (modePressActive && !modeLongPressFired &&
      modeButtonStable == LOW &&
      millis() - modePressStart >= WIFI_HOLD_MS)
  {
    modeLongPressFired = true;
    enableWifiFromButton();
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
      markSettingsDirty();
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
      markSettingsDirty();
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

  rawAn0Voltage = an0Voltage;
  rawAn1Voltage = an1Voltage;
  rawAn2Voltage = an2Voltage;

  // AN0 is dedicated to charger detection. The decision must hold for
  // CHARGER_DEBOUNCE_MS so supply ripple (LED or Wi-Fi bursts) can't flip it.
  chargerVoltage = an0Voltage;

  bool rawCharger = chargerCandidate;

  if (!chargerStateInitialized)
  {
    // First reading: split the hysteresis band down the middle.
    rawCharger = an0Voltage > ((CHARGER_ON_VOLTAGE + CHARGER_OFF_VOLTAGE) * 0.5);
  }
  else if (an0Voltage > CHARGER_ON_VOLTAGE)
  {
    rawCharger = true;
  }
  else if (an0Voltage < CHARGER_OFF_VOLTAGE)
  {
    rawCharger = false;
  }

  if (!chargerStateInitialized)
  {
    chargerPresentStable = rawCharger;
    chargerCandidate = rawCharger;
    chargerCandidateTime = millis();
    chargerStateInitialized = true;
  }
  else if (rawCharger != chargerCandidate)
  {
    chargerCandidate = rawCharger;
    chargerCandidateTime = millis();
  }
  else if (chargerPresentStable != chargerCandidate &&
           millis() - chargerCandidateTime >= CHARGER_DEBOUNCE_MS)
  {
    chargerPresentStable = chargerCandidate;
  }

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
        if (isChargerPresent())
        {
          // On USB the button toggles the LEDs; the board stays powered.
          setLedsEnabled(!ledsEnabled);
        }
        else
        {
          // On battery, this press is the power-off.
          powerOffNow();
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
  bool chargerPresent = isChargerPresent();
  bool buttonDisturbed = isPowerButtonDisturbed();
  unsigned long now = millis();

#if HAS_CHARGE_STATUS_PIN
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

#else
  // ---- Charge complete without an ST pin: voltage held high on USB ----
  // Readings taken during a button press are already excluded from
  // batteryVoltage, so a press doesn't restart the timer.
  if (!chargerPresent || !batteryVoltageValid ||
      batteryVoltage < FULL_BY_VOLTAGE_RESET)
  {
    fullByVoltageTimerRunning = false;
    chargeCompleteConfirmed = false;
  }
  else if (batteryVoltage >= FULL_BY_VOLTAGE_MIN)
  {
    if (!fullByVoltageTimerRunning)
    {
      fullByVoltageTimerRunning = true;
      fullByVoltageTimerStart = now;
    }
    else if (now - fullByVoltageTimerStart >= FULL_BY_VOLTAGE_HOLD_MS)
    {
      chargeCompleteConfirmed = true;
    }
  }
#endif

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
  if (!isChargerPresent() && ledsEnabled == false)
  {
    saveUserSettingsNow();
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
  writeStatusLed(0, true);
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
      writeStatusLed(0, false);
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
    bool chargerPresent = isChargerPresent();

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

  uint32_t color = 0;

  if (webControlStarted)
  {
    // Blue: Wi-Fi is on. Battery tracking keeps running underneath, and the
    // battery color comes back as soon as Wi-Fi turns off.
    color = statusLED.Color(0, 0, 255);
  }
  else if (batteryStatus == STATUS_CHARGING)
  {
    // Yellow: charger present, charge not complete.
    color = statusLED.Color(255, 180, 0);
  }
  else if (batteryStatus == STATUS_FULL)
  {
    color = statusLED.Color(0, 255, 0);
  }
  else if (batteryStatus == STATUS_LOW_BATTERY)
  {
    color = statusLED.Color(255, 0, 0);
  }

  writeStatusLed(color, false);
}

// Sends a frame to the status LED only when its color changes, plus a slow
// refresh so a frame corrupted by supply noise is corrected within a second.
// v1.5 sent a frame on every loop pass; with Wi-Fi bursting on the rail that
// meant ~50 chances a second to latch a bad color.
const unsigned long STATUS_LED_REFRESH_MS = 1000;

void writeStatusLed(uint32_t color, bool force)
{
  static uint32_t lastColor = 0xFFFFFFFF;
  static unsigned long lastWrite = 0;

  if (!force && color == lastColor && (millis() - lastWrite < STATUS_LED_REFRESH_MS))
    return;

  statusLED.setPixelColor(0, color);
  statusLED.show();

  lastColor = color;
  lastWrite = millis();
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
    case 9: myDesignMode(); break;
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
  uint32_t c = getStaticColorA();

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

  uint32_t c1 = getStaticColorA();
  uint32_t c2 = getStaticColorB();

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
  uint32_t colorA = getStaticColorA();
  uint32_t colorB = getStaticColorB();
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

void myDesignMode()
{
  for (int i = 0; i < LED_COUNT; i++)
  {
    strip.setPixelColor(i, designPixels[i]);
  }

  strip.show();
}

void setDefaultDesign()
{
  // Starter design: magenta at the top fading to cyan at the bottom.
  for (int i = 0; i < LED_COUNT; i++)
  {
    designPixels[i] = blendColors(0xFF00AA, 0x00E0FF, getPhysicalY(i)) & 0xFFFFFF;
  }
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

/*
=========================================================
v1.5 additions: shared helpers, saved settings, phone control
=========================================================
*/

// ---------------- Shared helpers ----------------

int clampInt(int value, int low, int high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

bool isChargerPresent()
{
  return chargerPresentStable;
}

bool modeUsesStaticColors(int mode)
{
  // Same set of modes the color button treats as static-color modes.
  return (mode == 1 || mode == 5 || mode == 6);
}

uint32_t getStaticColorA()
{
  if (staticColorIndex >= CUSTOM_COLOR_INDEX)
    return customColorA;

  return staticColors[staticColorIndex];
}

uint32_t getStaticColorB()
{
  if (staticColorIndex >= CUSTOM_COLOR_INDEX)
    return customColorB;

  return staticColors[(staticColorIndex + 3) % numStaticColors];
}

int nearestBrightnessIndex(uint8_t level)
{
  int best = 0;

  for (int i = 1; i < numBrightnessLevels; i++)
  {
    if (abs((int)brightnessLevels[i] - (int)level) <
        abs((int)brightnessLevels[best] - (int)level))
    {
      best = i;
    }
  }

  return best;
}

// Turns the LED strip on or off without cutting power (USB present).
// Used by both the power button and the app.
void setLedsEnabled(bool enabled)
{
  if (enabled == ledsEnabled)
    return;

  ledsEnabled = enabled;

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

// Battery power-off: same sequence the button used in v1.4, plus a flush
// of any unsaved settings first.
void powerOffNow()
{
  saveUserSettingsNow();

  // Blank the status LED first so it can't be redrawn while the supply
  // collapses.
  requestStatusLedBlank();

  pinMode(HOLD_PIN, OUTPUT);
  digitalWrite(HOLD_PIN, LOW);
}

// ---------------- Saved settings ----------------

void loadUserSettings()
{
  currentMode      = clampInt(prefs.getUChar("mode", 0), 0, NUM_MODES - 1);
  paletteIndex     = clampInt(prefs.getUChar("palette", 0), 0, NUM_PALETTES - 1);
  staticColorIndex = clampInt(prefs.getUChar("color", 0), 0, CUSTOM_COLOR_INDEX);
  customColorA     = prefs.getUInt("customA", customColorA) & 0xFFFFFF;
  customColorB     = prefs.getUInt("customB", customColorB) & 0xFFFFFF;

  if (prefs.getBytesLength("design") == sizeof(designPixels))
    prefs.getBytes("design", designPixels, sizeof(designPixels));
  else
    setDefaultDesign();

  uint8_t saved = prefs.getUChar("bright", brightnessLevels[0]);
  saved = (uint8_t)clampInt(saved, BRIGHTNESS_MIN, BRIGHTNESS_MAX);

  brightness = saved;
  targetBrightness = saved;
  brightnessIndex = nearestBrightnessIndex(saved);
}

void markSettingsDirty()
{
  settingsDirty = true;
  settingsDirtyTime = millis();
}

void saveUserSettingsNow()
{
  if (!settingsDirty)
    return;

  // NVS skips the write when a value hasn't changed, so this is cheap.
  prefs.putUChar("mode", (uint8_t)currentMode);
  prefs.putUChar("palette", (uint8_t)paletteIndex);
  prefs.putUChar("color", (uint8_t)staticColorIndex);
  prefs.putUInt("customA", customColorA);
  prefs.putUInt("customB", customColorB);
  prefs.putUChar("bright", targetBrightness);

  if (designDirty)
  {
    prefs.putBytes("design", designPixels, sizeof(designPixels));
    designDirty = false;
  }

  settingsDirty = false;
}

void saveUserSettingsIfDue()
{
  if (settingsDirty && (millis() - settingsDirtyTime >= SETTINGS_SAVE_DELAY_MS))
    saveUserSettingsNow();
}

// ---------------- Phone control: Wi-Fi + web server ----------------

extern const char INDEX_HTML[];

void startWebControl()
{
#if !ENABLE_WEB_CONTROL
  return;
#endif

  WiFi.mode(WIFI_AP);

  uint8_t mac[6];
  WiFi.softAPmacAddress(mac);
  snprintf(wifiSsid, sizeof(wifiSsid), "%s%02X%02X",
           WIFI_AP_SSID_PREFIX, mac[4], mac[5]);

  WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_IP, WIFI_AP_MASK);

  if (!WiFi.softAP(wifiSsid, WIFI_AP_PASSWORD, 6, 0, 2))  // channel 6, visible, 2 clients
  {
    WiFi.mode(WIFI_OFF);
    return;
  }

  // The phone is used right next to the display, so minimal TX power is
  // enough. Lower power means smaller current bursts on the rail the WS2812s
  // and status LED share. If the phone can't connect reliably, step up to
  // WIFI_POWER_5dBm.
  WiFi.setTxPower(WIFI_POWER_2dBm);

  // Answer every DNS lookup with our own address so phones show the
  // control page as a captive portal right after joining.
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", WIFI_AP_IP);

  // Routes are registered once; Wi-Fi can be turned on and off many times.
  if (!webRoutesRegistered)
  {
    server.on("/", HTTP_GET, handleWebRoot);
    server.on("/api/info", HTTP_GET, handleApiInfo);
    server.on("/api/state", HTTP_GET, handleApiState);
    server.on("/api/set", HTTP_POST, handleApiSet);
    server.on("/api/set", HTTP_GET, handleApiSet);  // handy for testing in a browser
    server.on("/api/design", HTTP_GET, handleApiDesign);
    server.on("/api/design", HTTP_POST, handleApiDesign);
    server.onNotFound(handleWebNotFound);
    webRoutesRegistered = true;
  }

  server.begin();

  webControlStarted = true;
  noteWebActivity();
}

void stopWebControl()
{
  if (!webControlStarted)
    return;

  dnsServer.stop();
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  webControlStarted = false;
}

// Any request from the phone app restarts the Wi-Fi timeout. Captive-portal
// probes from the phone's OS (redirected in handleWebNotFound) don't count,
// so a phone that is merely still joined doesn't keep Wi-Fi on forever.
void noteWebActivity()
{
  lastWebActivityTime = millis();
}

// Called when the Mode button has been held for WIFI_HOLD_MS.
void enableWifiFromButton()
{
#if ENABLE_WEB_CONTROL
  if (!webControlStarted)
    startWebControl();

  if (webControlStarted)
    noteWebActivity();  // a repeat hold restarts the timeout
#endif
}

void handleWebControl()
{
  if (!webControlStarted)
    return;

  dnsServer.processNextRequest();
  server.handleClient();

  if (millis() - lastWebActivityTime >= WIFI_TIMEOUT_MS)
    stopWebControl();
}

String colorToHex(uint32_t c)
{
  char buf[8];
  snprintf(buf, sizeof(buf), "#%06lX", (unsigned long)(c & 0xFFFFFF));
  return String(buf);
}

bool parseHexColor(const String &text, uint32_t &out)
{
  String s = text;
  s.trim();
  if (s.startsWith("#")) s.remove(0, 1);
  if (s.length() != 6) return false;

  for (unsigned int i = 0; i < s.length(); i++)
  {
    if (!isHexadecimalDigit(s[i])) return false;
  }

  out = strtoul(s.c_str(), nullptr, 16) & 0xFFFFFF;
  return true;
}

void sendJson(int code, const String &body)
{
  noteWebActivity();
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

void sendJsonError(const String &message)
{
  sendJson(400, "{\"ok\":false,\"error\":\"" + message + "\"}");
}

void handleWebRoot()
{
  noteWebActivity();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", INDEX_HTML);
}

void handleWebNotFound()
{
  // Captive portal: anything aimed at another host goes to the control page.
  if (server.hostHeader() != WIFI_AP_IP.toString())
  {
    server.sendHeader("Location", String("http://") + WIFI_AP_IP.toString() + "/");
    server.send(302, "text/plain", "");
    return;
  }

  sendJson(404, "{\"ok\":false,\"error\":\"not found\"}");
}

void handleApiInfo()
{
  String json;
  json.reserve(3200);

  json += "{\"modes\":[";
  for (int i = 0; i < NUM_MODES; i++)
  {
    const char* uses = (i == 0) ? "none" :
                       (i == MODE_MY_DESIGN) ? "design" :
                       (modeUsesStaticColors(i) ? "color" : "palette");
    if (i) json += ",";
    json += "{\"name\":\""; json += modeNames[i];
    json += "\",\"uses\":\""; json += uses;
    json += "\",\"two\":"; json += (i == 5 || i == 6) ? "true" : "false";
    json += "}";
  }

  json += "],\"palettes\":[";
  int savedPalette = paletteIndex;
  for (int p = 0; p < NUM_PALETTES; p++)
  {
    paletteIndex = p;
    uint32_t* pal = getPalette();
    if (p) json += ",";
    json += "{\"name\":\""; json += paletteNames[p]; json += "\",\"colors\":[";
    for (int c = 0; c < 5; c++)
    {
      if (c) json += ",";
      json += "\""; json += colorToHex(pal[c]); json += "\"";
    }
    json += "]}";
  }
  paletteIndex = savedPalette;

  json += "],\"colors\":[";
  for (int i = 0; i < numStaticColors; i++)
  {
    if (i) json += ",";
    json += "{\"name\":\""; json += staticColorNames[i];
    json += "\",\"hex\":\""; json += colorToHex(staticColors[i]); json += "\"}";
  }

  // Physical position of every LED (strip order), so the app can draw the
  // display the way it is actually laid out on the PCB.
  json += "],\"leds\":[";
  for (int i = 0; i < LED_COUNT; i++)
  {
    if (i) json += ",";
    json += "["; json += String(getPhysicalX(i), 3);
    json += ","; json += String(getPhysicalY(i), 3); json += "]";
  }

  json += "],\"custom_index\":"; json += CUSTOM_COLOR_INDEX;
  json += ",\"design_mode\":"; json += MODE_MY_DESIGN;
  json += ",\"brightness_min\":"; json += BRIGHTNESS_MIN;
  json += ",\"brightness_max\":"; json += BRIGHTNESS_MAX;
  json += ",\"ssid\":\""; json += wifiSsid;
  json += "\",\"firmware\":\"1.8\",\"board\":" + String(BOARD_VERSION) + ",\"has_st\":" + String(HAS_CHARGE_STATUS_PIN ? "true" : "false") + "}";

  sendJson(200, json);
}

String buildStateJson()
{
  bool charger = isChargerPresent();
  const char* power = "battery";

  if (charger)
    power = (chargeCompleteConfirmed || fullChargeLatched) ? "full" : "charging";
  else if (lowBatteryActive)
    power = "low";

  String json;
  json.reserve(300);
  json += "{\"ok\":true,\"leds\":"; json += ledsEnabled ? "true" : "false";
  json += ",\"mode\":"; json += currentMode;
  json += ",\"palette\":"; json += paletteIndex;
  json += ",\"color\":"; json += staticColorIndex;
  json += ",\"custom\":\""; json += colorToHex(customColorA);
  json += "\",\"custom2\":\""; json += colorToHex(customColorB);
  json += "\",\"brightness\":"; json += targetBrightness;
  json += ",\"charger\":"; json += charger ? "true" : "false";
  json += ",\"power\":\""; json += power;
  json += "\",\"battery_valid\":"; json += batteryVoltageValid ? "true" : "false";
  json += ",\"battery_v\":"; json += String(batteryVoltage, 2);
  json += ",\"an0\":"; json += String(rawAn0Voltage, 3);
  json += ",\"an1\":"; json += String(rawAn1Voltage, 3);
  json += ",\"an2\":"; json += String(rawAn2Voltage, 3);
  json += ",\"status_led\":"; json += (int)batteryStatus;
  json += "}";
  return json;
}

void handleApiState()
{
  sendJson(200, buildStateJson());
}

/*
  POST /api/set  (form-encoded or query string). Every field is optional:
    mode=0..9          LED mode (9 = My Design)
    palette=0..8       palette for palette-based modes
    color=0..8         preset static color, or 8 for the custom slot
    custom=RRGGBB      custom color A (also selects the custom slot)
    custom2=RRGGBB     custom color B (Split Side / Energy)
    brightness=10..145 target brightness (ramped)
    leds=0|1           0 = off (on battery this powers the device down)
  All fields are validated before anything is applied, so a bad request
  changes nothing. Replies with the same JSON as /api/state.
*/
void handleApiSet()
{
  int newMode = currentMode;
  int newPalette = paletteIndex;
  int newColor = staticColorIndex;
  uint32_t newCustomA = customColorA;
  uint32_t newCustomB = customColorB;
  int newBrightness = targetBrightness;
  int newLeds = -1;

  if (server.hasArg("mode"))
  {
    newMode = server.arg("mode").toInt();
    if (newMode < 0 || newMode >= NUM_MODES) { sendJsonError("mode out of range"); return; }
  }

  if (server.hasArg("palette"))
  {
    newPalette = server.arg("palette").toInt();
    if (newPalette < 0 || newPalette >= NUM_PALETTES) { sendJsonError("palette out of range"); return; }
  }

  if (server.hasArg("color"))
  {
    newColor = server.arg("color").toInt();
    if (newColor < 0 || newColor > CUSTOM_COLOR_INDEX) { sendJsonError("color out of range"); return; }
  }

  if (server.hasArg("custom"))
  {
    if (!parseHexColor(server.arg("custom"), newCustomA)) { sendJsonError("custom must be RRGGBB"); return; }
    newColor = CUSTOM_COLOR_INDEX;
  }

  if (server.hasArg("custom2"))
  {
    if (!parseHexColor(server.arg("custom2"), newCustomB)) { sendJsonError("custom2 must be RRGGBB"); return; }
    newColor = CUSTOM_COLOR_INDEX;
  }

  if (server.hasArg("brightness"))
  {
    String b = server.arg("brightness");
    if (b.length() == 0 || !isDigit(b[0])) { sendJsonError("brightness must be a number"); return; }
    newBrightness = clampInt(b.toInt(), BRIGHTNESS_MIN, BRIGHTNESS_MAX);
  }

  if (server.hasArg("leds"))
  {
    String l = server.arg("leds");
    if (l == "1" || l == "on" || l == "true") newLeds = 1;
    else if (l == "0" || l == "off" || l == "false") newLeds = 0;
    else { sendJsonError("leds must be 0 or 1"); return; }
  }

  // Everything validated; apply.
  bool changed = (newMode != currentMode) || (newPalette != paletteIndex) ||
                 (newColor != staticColorIndex) || (newCustomA != customColorA) ||
                 (newCustomB != customColorB) || (newBrightness != targetBrightness);

  currentMode = newMode;
  paletteIndex = newPalette;
  staticColorIndex = newColor;
  customColorA = newCustomA;
  customColorB = newCustomB;

  // Only the target moves here; updateBrightnessRamp() eases the strip
  // toward it so the LED current never steps. Keep the button's index in
  // step so the next button press continues from the nearest level.
  targetBrightness = (uint8_t)newBrightness;
  brightnessIndex = nearestBrightnessIndex(targetBrightness);

  if (changed)
    markSettingsDirty();

  if (newLeds == 0 && ledsEnabled && !isChargerPresent())
  {
    // On battery, "off" is a full power-down. Reply first so the phone
    // gets an answer, then drop power the same way the button does.
    String reply = buildStateJson();
    reply.replace("\"leds\":true", "\"leds\":false");
    reply.replace("{\"ok\":true", "{\"ok\":true,\"powering_off\":true");
    sendJson(200, reply);
    delay(150);
    powerOffNow();
    return;
  }

  if (newLeds == 0)
    setLedsEnabled(false);
  else if (newLeds == 1)
    setLedsEnabled(true);

  sendJson(200, buildStateJson());
}

/*
  GET  /api/design  -> {"ok":true,"pixels":"RRGGBB..."}  (66 colors, strip order)
  POST /api/design  pixels=RRGGBB... (exactly 66 x 6 hex characters)
  A successful POST switches the display to My Design (mode 9) and saves the
  design to flash with the other settings.
*/
void handleApiDesign()
{
  if (server.method() == HTTP_POST || server.hasArg("pixels"))
  {
    String hex = server.arg("pixels");

    if (hex.length() != (unsigned int)(LED_COUNT * 6))
    {
      sendJsonError("pixels must be 66 colors as RRGGBB");
      return;
    }

    for (unsigned int i = 0; i < hex.length(); i++)
    {
      if (!isHexadecimalDigit(hex[i]))
      {
        sendJsonError("pixels must be hex");
        return;
      }
    }

    char chunk[7];
    chunk[6] = 0;

    for (int i = 0; i < LED_COUNT; i++)
    {
      for (int k = 0; k < 6; k++) chunk[k] = hex[i * 6 + k];
      designPixels[i] = strtoul(chunk, nullptr, 16) & 0xFFFFFF;
    }

    currentMode = MODE_MY_DESIGN;
    designDirty = true;
    markSettingsDirty();
  }

  String json;
  json.reserve(LED_COUNT * 6 + 32);
  json += "{\"ok\":true,\"mode\":"; json += currentMode;
  json += ",\"pixels\":\"";

  char buf[7];
  for (int i = 0; i < LED_COUNT; i++)
  {
    snprintf(buf, sizeof(buf), "%06lX", (unsigned long)(designPixels[i] & 0xFFFFFF));
    json += buf;
  }

  json += "\"}";
  sendJson(200, json);
}

// ---------------- Phone control page (served from flash) ----------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0d0f14">
<title>Shining Guard</title>
<style>
:root{--bg:#0d0f14;--card:#171a22;--line:#262b36;--text:#eef1f6;--dim:#8a93a6;--accent:#f5c542;--ok:#38d47a;--warn:#ffb020;--bad:#ff5a5a}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--text);font:15px/1.4 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;padding:16px 16px 40px;max-width:520px;margin:auto}
header{display:flex;align-items:center;justify-content:space-between;gap:12px;margin:4px 0 16px}
h1{font-size:20px;margin:0;letter-spacing:.2px}
h1 span{color:var(--accent)}
.chip{font-size:12px;color:var(--dim);border:1px solid var(--line);border-radius:999px;padding:4px 10px;white-space:nowrap}
.chip b{font-weight:600}
.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:14px;margin-bottom:12px}
.card h2{font-size:12px;text-transform:uppercase;letter-spacing:1px;color:var(--dim);margin:0 0 10px;font-weight:600}
.row{display:flex;align-items:center;justify-content:space-between;gap:12px}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
button{font:inherit;color:var(--text);background:#1f2430;border:1px solid var(--line);border-radius:12px;padding:12px 6px;cursor:pointer}
button.on{border-color:var(--accent);box-shadow:0 0 0 1px var(--accent) inset;background:#262116}
.pal{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.pal button{padding:8px 6px 10px;font-size:13px}
.bar{height:14px;border-radius:7px;margin-bottom:6px}
.dots{display:grid;grid-template-columns:repeat(5,1fr);gap:10px}
.dot{aspect-ratio:1;border-radius:50%;padding:0;border:2px solid transparent}
.dot.on{border-color:#fff;box-shadow:0 0 0 2px var(--bg) inset}
.custom{display:flex;gap:10px;margin-top:12px;align-items:center;flex-wrap:wrap}
.custom label{display:flex;align-items:center;gap:8px;font-size:13px;color:var(--dim)}
input[type=color]{width:44px;height:44px;border:none;border-radius:50%;padding:0;background:none}
input[type=color]::-webkit-color-swatch-wrapper{padding:0}
input[type=color]::-webkit-color-swatch{border:2px solid var(--line);border-radius:50%}
input[type=range]{width:100%;accent-color:var(--accent);height:32px}
.power{width:64px;height:36px;border-radius:18px;position:relative;padding:0}
.power::after{content:"";position:absolute;top:4px;left:4px;width:26px;height:26px;border-radius:50%;background:var(--dim);transition:.2s}
.power.on{background:#2d2614}
.power.on::after{left:32px;background:var(--accent)}
.note{color:var(--dim);font-size:13px;margin:8px 0 0}
.hidden{display:none}
#toast{position:fixed;left:50%;bottom:20px;transform:translateX(-50%);background:#2a1d1d;color:#ffd6d6;border:1px solid #5a2b2b;padding:10px 14px;border-radius:12px;font-size:14px;opacity:0;transition:.25s;pointer-events:none}
#toast.show{opacity:1}
.off .dim-when-off{opacity:.45;pointer-events:none}
.grid button.wide{grid-column:1/-1}
.board{position:relative;width:min(100%,300px);aspect-ratio:2/3;margin:4px auto 14px;background:#090b10;border:1px solid var(--line);border-radius:14px;touch-action:none;user-select:none;-webkit-user-select:none}
.slab{position:absolute;left:27%;right:27%;top:27%;bottom:11%;border:1px dashed #303747;border-radius:6px;display:grid;place-items:center;color:#4a5263;font-size:11px;letter-spacing:1px;text-transform:uppercase;pointer-events:none}
.px{position:absolute;width:8.5%;aspect-ratio:1;border-radius:50%;transform:translate(-50%,-50%);border:1px solid #ffffff1f}
.px.dark{background:#141821!important;box-shadow:none!important}
.tools{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
.tools .sw{width:30px;height:30px;border-radius:50%;padding:0;border:2px solid transparent}
.tools .sw.on{border-color:#fff;box-shadow:0 0 0 2px var(--bg) inset}
.tools .sw.offsw{background:#141821;border-color:#3a4252}
.actions{display:flex;gap:8px;margin-top:12px}
.actions button{flex:1;padding:10px 6px;font-size:14px}
details.card summary{cursor:pointer;color:var(--dim);font-size:12px;text-transform:uppercase;letter-spacing:1px;font-weight:600}
.diag{display:grid;grid-template-columns:auto 1fr;gap:4px 14px;font:13px ui-monospace,Menlo,monospace;margin-top:10px}
.diag span:nth-child(odd){color:var(--dim)}
</style></head>
<body>
<header>
  <h1>Shining <span>Guard</span></h1>
  <div class="chip" id="status">Connecting…</div>
</header>

<div class="card row">
  <div><div style="font-weight:600">Display</div><div class="note" id="powerNote" style="margin:2px 0 0"></div></div>
  <button class="power" id="power" aria-label="Display on or off"></button>
</div>

<div class="dim-when-off">
  <div class="card">
    <h2>Mode</h2>
    <div class="grid" id="modes"></div>
  </div>

  <div class="card" id="paletteCard">
    <h2>Palette</h2>
    <div class="pal" id="palettes"></div>
  </div>

  <div class="card hidden" id="colorCard">
    <h2>Color</h2>
    <div class="dots" id="colors"></div>
    <div class="custom">
      <label><input type="color" id="customA" value="#ff00aa">Custom</label>
      <label id="customBWrap"><input type="color" id="customB" value="#00e0ff">Second color</label>
    </div>
  </div>

  <div class="card hidden" id="designCard">
    <h2>My Design</h2>
    <div class="board" id="board"><div class="slab">Card</div></div>
    <div class="tools" id="brushes"></div>
    <div class="actions">
      <button id="fillAll">Fill all</button>
      <button id="clearAll">Clear</button>
    </div>
    <p class="note">Pick a color, then tap or drag across the LEDs. Changes show on the display right away and are saved automatically.</p>
  </div>

  <div class="card hidden" id="rainbowCard">
    <h2>Color</h2>
    <p class="note" style="margin:0">Rainbow cycles through every hue on its own.</p>
  </div>

  <div class="card">
    <div class="row"><h2 style="margin:0">Brightness</h2><span class="chip" id="bval">–</span></div>
    <input type="range" id="bright" min="10" max="145" step="1">
  </div>
</div>

<details class="card" id="diagCard">
  <summary>Diagnostics</summary>
  <div class="diag" id="diag"></div>
</details>

<div id="toast"></div>

<script>
const $=id=>document.getElementById(id);
let info=null,state=null,dragging=false,lastSend=0,sendTimer=null,offline=false;
let design=null,brush='ff00aa',painting=false,designTimer=null,designLast=0;

function toast(msg){const t=$('toast');t.textContent=msg;t.classList.add('show');clearTimeout(t._h);t._h=setTimeout(()=>t.classList.remove('show'),2600)}

async function getJson(path,opts){
  const r=await fetch(path,Object.assign({cache:'no-store'},opts||{}));
  const j=await r.json();
  if(!r.ok) throw new Error(j.error||('HTTP '+r.status));
  return j;
}

async function send(params){
  try{
    const j=await getJson('/api/set',{method:'POST',body:new URLSearchParams(params)});
    state=j;offline=false;render();
    if(j.powering_off) toast('Powering off. Use the button on the display to turn it back on.');
  }catch(e){toast(e.message==='Failed to fetch'?'Lost connection to the display':e.message)}
}

function pct(v){return Math.round((v-info.brightness_min)/(info.brightness_max-info.brightness_min)*100)}

function build(){
  const m=$('modes');m.innerHTML='';
  info.modes.forEach((mode,i)=>{const b=document.createElement('button');b.textContent=mode.name;if(mode.uses==='design')b.className='wide';b.onclick=()=>send({mode:i});m.appendChild(b)});

  const board=$('board');board.querySelectorAll('.px').forEach(n=>n.remove());
  info.leds.forEach((xy,i)=>{const d=document.createElement('div');d.className='px dark';d.dataset.i=i;d.style.left=(7+xy[0]*86)+'%';d.style.top=(5+xy[1]*90)+'%';board.appendChild(d)});

  const br=$('brushes');br.innerHTML='';
  const pick=document.createElement('input');pick.type='color';pick.id='brushPick';pick.value='#'+brush;
  pick.addEventListener('input',e=>{brush=e.target.value.slice(1).toUpperCase();markBrush()});
  br.appendChild(pick);
  info.colors.concat([{name:'Off',hex:'#000000'}]).forEach(col=>{
    const b=document.createElement('button');b.className='sw'+(col.hex==='#000000'?' offsw':'');
    if(col.hex!=='#000000')b.style.background=col.hex;
    b.title=col.name;b.setAttribute('aria-label',col.name);b.dataset.hex=col.hex.slice(1);
    b.onclick=()=>{brush=col.hex.slice(1);if(brush!=='000000')$('brushPick').value='#'+brush.toLowerCase();markBrush()};
    br.appendChild(b);
  });
  markBrush();

  const p=$('palettes');p.innerHTML='';
  info.palettes.forEach((pal,i)=>{
    const b=document.createElement('button');
    const bar=document.createElement('div');bar.className='bar';bar.style.background='linear-gradient(90deg,'+pal.colors.join(',')+')';
    b.appendChild(bar);b.appendChild(document.createTextNode(pal.name));
    b.onclick=()=>send({palette:i});p.appendChild(b);
  });

  const c=$('colors');c.innerHTML='';
  info.colors.forEach((col,i)=>{const b=document.createElement('button');b.className='dot';b.style.background=col.hex;b.title=col.name;b.setAttribute('aria-label',col.name);b.onclick=()=>send({color:i});c.appendChild(b)});

  const r=$('bright');r.min=info.brightness_min;r.max=info.brightness_max;
}

function render(){
  if(!info||!state) return;
  const mode=info.modes[state.mode];
  document.body.classList.toggle('off',!state.leds);
  $('power').classList.toggle('on',state.leds);
  $('powerNote').textContent=state.leds?mode.name:(state.charger?'Off (on USB)':'Off');

  [...$('modes').children].forEach((b,i)=>b.classList.toggle('on',i===state.mode));
  [...$('palettes').children].forEach((b,i)=>b.classList.toggle('on',i===state.palette));
  [...$('colors').children].forEach((b,i)=>b.classList.toggle('on',i===state.color));

  $('paletteCard').classList.toggle('hidden',mode.uses!=='palette');
  $('colorCard').classList.toggle('hidden',mode.uses!=='color');
  $('rainbowCard').classList.toggle('hidden',mode.uses!=='none');
  $('designCard').classList.toggle('hidden',mode.uses!=='design');
  if(mode.uses==='design'&&!design) loadDesign();
  renderDiag();
  $('customBWrap').classList.toggle('hidden',!mode.two);

  const custom=state.color===info.custom_index;
  $('customA').style.outline=custom?'2px solid #fff':'none';
  $('customA').style.borderRadius='50%';
  if(document.activeElement!==$('customA')) $('customA').value=state.custom.toLowerCase();
  if(document.activeElement!==$('customB')) $('customB').value=state.custom2.toLowerCase();

  if(!dragging){$('bright').value=state.brightness;$('bval').textContent=pct(state.brightness)+'%'}

  let s;
  if(state.power==='full') s='<b style="color:var(--ok)">Charged</b>';
  else if(state.power==='charging') s='<b style="color:var(--warn)">Charging</b>';
  else if(state.power==='low') s='<b style="color:var(--bad)">Battery low</b>';
  else s='Battery'+(state.battery_valid?' '+state.battery_v.toFixed(2)+' V':'');
  $('status').innerHTML=s;
}

function markBrush(){[...$('brushes').querySelectorAll('.sw')].forEach(b=>b.classList.toggle('on',b.dataset.hex.toUpperCase()===brush.toUpperCase()))}

function drawDesign(){
  if(!design) return;
  $('board').querySelectorAll('.px').forEach(d=>{
    const c=design[+d.dataset.i];const off=c==='000000';
    d.classList.toggle('dark',off);
    if(!off){d.style.background='#'+c;d.style.boxShadow='0 0 10px #'+c+'aa'}
  });
}

async function loadDesign(){
  try{const j=await getJson('/api/design');design=j.pixels.match(/.{6}/g);drawDesign()}
  catch(e){toast('Could not load your design')}
}

function sendDesign(){
  designLast=Date.now();
  getJson('/api/design',{method:'POST',body:new URLSearchParams({pixels:design.join('')})})
    .then(j=>{if(state&&state.mode!==j.mode){state.mode=j.mode;render()}})
    .catch(e=>toast(e.message==='Failed to fetch'?'Lost connection to the display':e.message));
}

function queueDesign(){
  clearTimeout(designTimer);
  const wait=Math.max(0,180-(Date.now()-designLast));
  designTimer=setTimeout(sendDesign,wait);
}

function paintAt(x,y){
  const el=document.elementFromPoint(x,y);
  const d=el&&el.closest?el.closest('.px'):null;
  if(!d||!design) return;
  const i=+d.dataset.i;
  if(design[i]===brush.toUpperCase()) return;
  design[i]=brush.toUpperCase();drawDesign();queueDesign();
}

const board=$('board');
board.addEventListener('pointerdown',e=>{painting=true;board.setPointerCapture(e.pointerId);paintAt(e.clientX,e.clientY)});
board.addEventListener('pointermove',e=>{if(painting)paintAt(e.clientX,e.clientY)});
['pointerup','pointercancel'].forEach(t=>board.addEventListener(t,()=>{painting=false}));
$('fillAll').onclick=()=>{if(!design)return;design=design.map(()=>brush.toUpperCase());drawDesign();queueDesign()};
$('clearAll').onclick=()=>{if(!design)return;design=design.map(()=>'000000');drawDesign();queueDesign()};

function renderDiag(){
  if(!$('diagCard').open) return;
  const names=['off','charging (yellow)','full (green)','low (red)'];
  const rows=[['AN0 charger',state.an0.toFixed(3)+' V'],['AN1 battery',state.an1.toFixed(3)+' V'],['AN2 ST',info.has_st?state.an2.toFixed(3)+' V':'not used on this board'],
    ['Battery (filtered)',state.battery_valid?state.battery_v.toFixed(2)+' V':'no valid reading yet'],
    ['Charger',state.charger?'present':'absent'],['Status LED',names[state.status_led]||state.status_led],['Firmware',info.firmware]];
  $('diag').innerHTML=rows.map(r=>'<span>'+r[0]+'</span><span>'+r[1]+'</span>').join('');
}
$('diagCard').addEventListener('toggle',renderDiag);

$('power').onclick=()=>{
  if(!state) return;
  if(state.leds&&!state.charger&&!confirm('This powers the display off. Use the button on the display to turn it back on.')) return;
  send({leds:state.leds?0:1});
};

const slider=$('bright');
slider.addEventListener('input',()=>{
  dragging=true;$('bval').textContent=pct(+slider.value)+'%';
  const now=Date.now();clearTimeout(sendTimer);
  if(now-lastSend>200){lastSend=now;send({brightness:slider.value})}
  else sendTimer=setTimeout(()=>{lastSend=Date.now();send({brightness:slider.value})},200);
});
slider.addEventListener('change',()=>{clearTimeout(sendTimer);send({brightness:slider.value}).then(()=>{dragging=false;render()})});

$('customA').addEventListener('change',e=>send({custom:e.target.value.slice(1)}));
$('customB').addEventListener('change',e=>send({custom2:e.target.value.slice(1)}));

async function refresh(){
  try{state=await getJson('/api/state');offline=false;render()}
  catch(e){if(!offline){offline=true;$('status').textContent='Not connected'}}
}

(async()=>{
  try{info=await getJson('/api/info');build();await refresh()}
  catch(e){$('status').textContent='Not connected'}
  setInterval(()=>{if(!info){getJson('/api/info').then(j=>{info=j;build();refresh()}).catch(()=>{})}else if(!dragging)refresh()},2500);
})();
</script>
</body></html>)rawliteral";
