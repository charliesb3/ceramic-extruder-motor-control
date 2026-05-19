/*
 * ============================================================
 * motor_control.ino — 3-Motor Ratio Speed Controller
 * Target: Arduino Mega 2560
 * ============================================================
 *
 * Required Libraries (install via Arduino IDE > Sketch > Include Library
 *                      > Manage Libraries):
 *
 *   1. AccelStepper  — by Mike McCauley
 *      Search "AccelStepper" in Library Manager
 *
 *   2. LiquidCrystal_I2C  — by Frank de Brabander
 *      Search "LiquidCrystal I2C" in Library Manager
 *
 * WIRING SUMMARY
 * ============================================================
 *
 * LCD (HD44780 with PCF8574 I2C backpack, 20×4):
 *   VCC -> 5V          GND -> GND
 *   SDA -> Mega pin 20 (hardware I2C SDA)
 *   SCL -> Mega pin 21 (hardware I2C SCL)
 *
 * Potentiometers (all 10 kΩ linear taper):
 *   One outer leg -> 5V,  other outer leg -> GND
 *   Master knob wiper   -> A0
 *   Motor 1 ratio wiper -> A1
 *   Motor 2 ratio wiper -> A2
 *   Motor 3 ratio wiper -> A3
 *
 * DM556T Stepper Drivers:
 *   Motor 1:  STEP -> pin 2,  DIR -> pin 3
 *   Motor 2:  STEP -> pin 4,  DIR -> pin 5
 *   Motor 3:  STEP -> pin 6,  DIR -> pin 7
 *
 * DM556T Wiring Options (STEP and DIR are differential inputs):
 *
 *   Common-cathode (recommended for Arduino Mega 5V logic):
 *     Connect STEP- and DIR- to GND.
 *     Drive STEP+ and DIR+ directly from the Arduino output pins.
 *     Signals are non-inverted — this is what this firmware assumes.
 *
 *   Common-anode:
 *     Connect STEP+ and DIR+ to +5V.
 *     Drive STEP- and DIR- from the Arduino output pins.
 *     Signals are logically inverted; motors may run in reverse or not spin
 *     at all without additional invert logic or DIR wiring changes.
 *
 * DM556T Logic Level Note (Arduino Mega drives 5V):
 *   The DM556T opto-isolated inputs are compatible with 5V logic in
 *   common-cathode wiring. The driver's internal resistor limits opto-
 *   coupler current to roughly 10–15 mA at 5V, within the Arduino Mega's
 *   40 mA per-pin maximum.
 *
 *   Optional protection: add a 100–220 Ω series resistor between each
 *   Arduino output pin and the corresponding STEP+/DIR+ terminal. This
 *   keeps opto current above the reliable-trigger threshold (~7 mA) while
 *   providing some protection against wiring faults. Many builders omit
 *   these resistors without issue; they are low-cost insurance.
 *
 *   Minimum STEP pulse width required by the DM556T: 2.5 μs.
 *   AccelStepper generates pulses well above this threshold on a 16 MHz
 *   Mega at all speeds this firmware produces.
 *
 *   DIR setup time: DIR must be stable ≥5 μs before the STEP rising edge.
 *   AccelStepper always writes DIR before toggling STEP, satisfying this.
 *
 *   If motors run in the wrong direction, set REVERSE_MOTOR_X to true or
 *   physically swap two wires on one motor winding (A+ ↔ A−).
 *
 * Enable Pins (defined for future use, not driven by this firmware):
 *   Motor 1 ENA -> pin 8    Motor 2 ENA -> pin 9    Motor 3 ENA -> pin 10
 *   Leave ENA terminals disconnected unless you need software enable/disable.
 *   Enable polarity varies between driver revisions — consult your exact
 *   DM556T manual before wiring ENA to avoid accidentally disabling the driver.
 *
 * Power:
 *   Main PSU (24 V or 36 V) -> DM556T V+ / GND terminals
 *   5 V buck converter output -> Arduino 5V pin and LCD VCC
 *     CAUTION: Verify the buck output is exactly 5.0 V with a multimeter
 *     before connecting to the Arduino 5V pin. Do NOT feed a 5 V supply
 *     into the Arduino VIN pin — VIN expects roughly 7–9 V and routes
 *     through the onboard regulator; 5 V on VIN may not power the board.
 *
 * ============================================================
 */

// ============================================================
// CONFIGURATION — Edit these to match your hardware
// ============================================================

// LCD I2C address. 0x27 is most common with PCF8574 backpacks.
// If the display stays blank, try 0x3F.
// Run an I2C scanner sketch to find the address if unsure.
#define LCD_I2C_ADDRESS  0x27

// Steps per full mechanical revolution.
// 1.8° motors = 200 (most common), 0.9° motors = 400.
#define MOTOR_FULL_STEPS_PER_REV  200

// Microstepping divisor — MUST match the DM556T DIP switch setting (SW5–SW8).
// Common values: 1 (full step), 2, 4, 8, 16, 32.
// See the calibration notes at the bottom of this file.
#define MICROSTEPS  8

// Master speed range in RPM (MIN should normally stay 0).
#define MIN_MASTER_RPM    0.0f
#define MAX_MASTER_RPM   60.0f  // conservative for initial testing; increase once everything runs correctly

// Ratio knob range — each motor's RPM = masterRPM × ratio.
#define MIN_RATIO  0.0f
#define MAX_RATIO  2.0f  // conservative for initial testing; increase once everything runs correctly

// Master dead zone: raw ADC counts (0–1023) at or below this value are
// treated as zero speed. Prevents unintended creep when the knob is at
// its minimum. Adjust if your pot doesn't quite reach 0.
#define MASTER_DEAD_ZONE_ADC  30

// Ratio dead zone: ratio values below this are clamped to 0, fully
// stopping that motor. Lets you park individual motors at knob minimum.
#define RATIO_DEAD_ZONE  0.02f

// Potentiometer read interval in milliseconds (~100 Hz).
// Each analogRead() takes ~112 μs; four reads take ~450 μs total.
// Scheduling reads on this timer keeps the main loop fast between reads
// so motor stepping is minimally interrupted.
#define POT_READ_INTERVAL_MS   10UL

// LCD update interval in milliseconds (one row per interval).
// Four rows × 62 ms = ~250 ms for a full display refresh (~4 Hz).
// Writing one I2C row takes ~6 ms at 400 kHz; refreshing one row per
// interval caps the stepping gap to that duration.
#define LCD_LINE_INTERVAL_MS   62UL

// Analog smoothing strength (exponential moving average alpha).
// Range: 0.0 (no response) to 1.0 (no smoothing).
// 0.08 gives gentle filtering suitable for hand-turned potentiometers.
// Increase toward 0.3 for faster response; decrease toward 0.03 for
// heavier filtering if readings are noisy.
#define SMOOTH_ALPHA  0.08f

// Motor direction reversal — index 0 = Motor 1, 1 = Motor 2, 2 = Motor 3.
// Set an entry to true to invert that motor's direction.
// Flip if a motor runs backwards relative to your mechanical requirement.
// constexpr lets the compiler evaluate REVERSE[i] ? -1.0f : 1.0f at
// compile time when the index is constant, eliminating the dead branch.
constexpr bool REVERSE[3] = { false, false, false };

// Set to 1 to enable Serial debug output at 115200 baud. Set to 0 to disable.
// When enabled, prints master RPM and each motor's RPM and ratio at ~4 Hz.
#define DEBUG  0

// ============================================================
// PIN ASSIGNMENTS — Remap here if you change your wiring
// ============================================================

#define STEP_PIN_1   2
#define DIR_PIN_1    3
#define STEP_PIN_2   4
#define DIR_PIN_2    5
#define STEP_PIN_3   6
#define DIR_PIN_3    7

#define ENA_PIN_1    8   // Enable pins — defined for future use, not driven
#define ENA_PIN_2    9
#define ENA_PIN_3   10

#define POT_MASTER   A0
#define POT_RATIO_1  A1
#define POT_RATIO_2  A2
#define POT_RATIO_3  A3

// Pot pin lookup — index 0 = master, 1–3 = motor ratio knobs.
const uint8_t POT_PINS[4] = { POT_MASTER, POT_RATIO_1, POT_RATIO_2, POT_RATIO_3 };

// ============================================================
// INCLUDES
// ============================================================

#include <Wire.h>
#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>

// ============================================================
// ANALOG SMOOTHER
// ============================================================

// Exponential moving average (EMA) filter for potentiometer readings.
// A bool flag detects the first call and seeds the filter from the raw
// sample directly, avoiding any startup transient.
class AnalogSmoother {
public:
    explicit AnalogSmoother(float alpha = SMOOTH_ALPHA)
        : _alpha(alpha), _value(0.0f), _seeded(false) {}

    // Pass the raw analogRead() result; returns the smoothed float value.
    float update(int raw) {
        if (!_seeded) {
            _seeded = true;
            _value  = static_cast<float>(raw);
        } else {
            _value += _alpha * (static_cast<float>(raw) - _value);
        }
        return _value;
    }

    float value() const { return _value; }

private:
    float _alpha;
    float _value;
    bool  _seeded;
};

// ============================================================
// OBJECTS
// ============================================================

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 20, 4);

// AccelStepper::DRIVER mode uses a dedicated STEP pin and DIR pin.
// setSpeed() + runSpeed() gives constant velocity with no acceleration ramp.
AccelStepper motors[3] = {
    AccelStepper(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1),
    AccelStepper(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2),
    AccelStepper(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3)
};

// Index 0 = master knob, 1–3 = motor ratio knobs.
AnalogSmoother smoothers[4];

// ============================================================
// STATE
// ============================================================

static float masterRPM = 0.0f;
static float ratios[3] = {};
static float rpms[3]   = {};

static unsigned long lastPotRead   = 0;
static unsigned long lastLcdUpdate = 0;
static uint8_t       lcdLine       = 0;  // next row to refresh (cycles 0–3)

// ============================================================
// HELPERS
// ============================================================

// Float-compatible map() equivalent.
static float mapf(float x, float inLo, float inHi, float outLo, float outHi) {
    return (x - inLo) / (inHi - inLo) * (outHi - outLo) + outLo;
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Precomputed scale factor: steps per second per RPM.
// Collapses MOTOR_FULL_STEPS_PER_REV * MICROSTEPS / 60.0 into a single
// compile-time constant so rpmToStepsPerSec() is one multiply at runtime.
static constexpr float STEPS_PER_RPM =
    static_cast<float>(MOTOR_FULL_STEPS_PER_REV) * MICROSTEPS / 60.0f;

static float rpmToStepsPerSec(float rpm) {
    return rpm * STEPS_PER_RPM;
}

// ── LCD formatting ─────────────────────────────────────────────────────────
//
// Each row is exactly 20 characters. dtostrf() handles float-to-string
// conversion safely on AVR (unlike %f in snprintf, which is unreliable).
// It pads with leading spaces to totalWidth, keeping the layout stable as
// values change.
//
// Motor row layout (20 chars):
//   "M1:  xxx.x RPM  x.xx"
//    ^^^  ^^^^^  ^^^  ^^^^
//    3+2  5      6    4    = 20
//
// Master row layout (20 chars):
//   "Master:  xxx.x RPM  "
//    ^^^^^^^  ^^^^^  ^^^^
//    9        5      6    = 20

static void lcdWriteMotorRow(uint8_t row, uint8_t motorNum,
                             float rpm, float ratio) {
    char rpmBuf[8];
    char ratioBuf[6];
    char line[21];
    dtostrf(rpm,   5, 1, rpmBuf);   // e.g. "300.0" or "  0.0"
    dtostrf(ratio, 4, 2, ratioBuf); // e.g. "2.00"  or "0.00"
    snprintf(line, sizeof(line), "M%d:  %s RPM  %s", motorNum, rpmBuf, ratioBuf);
    lcd.setCursor(0, row);
    lcd.print(line);
}

static void lcdWriteMasterRow(float rpm) {
    char rpmBuf[8];
    char line[21];
    dtostrf(rpm, 5, 1, rpmBuf);
    snprintf(line, sizeof(line), "Master:  %s RPM  ", rpmBuf);
    lcd.setCursor(0, 3);
    lcd.print(line);
}

// ============================================================
// SCHEDULED TASKS
// ============================================================

// Called every POT_READ_INTERVAL_MS.
// Reads all four potentiometers, applies smoothing and dead zones, computes
// commanded RPM per motor, and calls setSpeed() on each AccelStepper.
// Keeping speed updates here (rather than every loop iteration) makes the
// hot path — the three runSpeed() calls — as short as possible.
static void readAndUpdateSpeeds() {
    float rawMaster = smoothers[0].update(analogRead(POT_PINS[0]));

    // Master RPM with dead zone
    if (rawMaster < static_cast<float>(MASTER_DEAD_ZONE_ADC)) {
        masterRPM = 0.0f;
    } else {
        masterRPM = mapf(rawMaster,
                         static_cast<float>(MASTER_DEAD_ZONE_ADC), 1023.0f,
                         MIN_MASTER_RPM, MAX_MASTER_RPM);
        masterRPM = clampf(masterRPM, MIN_MASTER_RPM, MAX_MASTER_RPM);
    }

    // Formula: motorRPM = masterRPM × ratio
    // Example: master=60, ratios=1.0/0.5/2.0 → M1=60, M2=30, M3=120 RPM.
    // Turning master down preserves relative ratios across all three motors.
    for (uint8_t i = 0; i < 3; i++) {
        float raw  = smoothers[i + 1].update(analogRead(POT_PINS[i + 1]));
        ratios[i]  = clampf(mapf(raw, 0.0f, 1023.0f, MIN_RATIO, MAX_RATIO), MIN_RATIO, MAX_RATIO);
        if (ratios[i] < RATIO_DEAD_ZONE) ratios[i] = 0.0f;
        rpms[i]    = masterRPM * ratios[i];
        float speed = rpmToStepsPerSec(rpms[i]) * (REVERSE[i] ? -1.0f : 1.0f);
        motors[i].setSpeed(speed);
    }

#if DEBUG
    // Rate-limit serial output to ~4 Hz (every 25 calls × 10 ms = 250 ms).
    static uint8_t debugTick = 0;
    if (++debugTick >= 25) {
        debugTick = 0;
        Serial.print(F("Master=")); Serial.print(masterRPM, 1); Serial.print(F(" RPM"));
        for (uint8_t i = 0; i < 3; i++) {
            Serial.print(F("  M")); Serial.print(i + 1); Serial.print('=');
            Serial.print(rpms[i], 1);
            Serial.print(F(" (x")); Serial.print(ratios[i], 2); Serial.print(F(")"));
        }
        Serial.println();
    }
#endif
}

// Called every LCD_LINE_INTERVAL_MS.
// Rows 0–2 display per-motor data indexed directly from the arrays;
// row 3 displays the master RPM. Four calls × 62 ms = ~250 ms full refresh.
static void updateLcdRow() {
    if (lcdLine < 3) {
        lcdWriteMotorRow(lcdLine, lcdLine + 1, rpms[lcdLine], ratios[lcdLine]);
    } else {
        lcdWriteMasterRow(masterRPM);
    }
    lcdLine = (lcdLine + 1) & 3;  // 0 → 1 → 2 → 3 → 0
}

// ============================================================
// SETUP
// ============================================================

void setup() {
#if DEBUG
    Serial.begin(115200);
    Serial.println(F("Motor Control — starting"));
#endif

    // 400 kHz fast-mode I2C reduces each LCD row write from ~25 ms to ~6 ms,
    // shortening the stepping gap during each display update.
    Wire.begin();
    Wire.setClock(400000UL);

    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("  Motor Controller  "));
    lcd.setCursor(0, 1);
    lcd.print(F("   Initializing...  "));

    // setMaxSpeed() must be called before setSpeed().
    // Calculate the theoretical maximum steps/sec with 10 % headroom so
    // AccelStepper never silently clips a requested speed.
    const float maxSpd = rpmToStepsPerSec(MAX_MASTER_RPM * MAX_RATIO) * 1.1f;
    for (uint8_t i = 0; i < 3; i++) {
        motors[i].setMaxSpeed(maxSpd);
        motors[i].setSpeed(0.0f);
    }

    delay(1500);
    lcd.clear();
}

// ============================================================
// LOOP
//
// Priority order, top to bottom:
//   1. runSpeed() × 3  — every iteration, no conditions
//   2. readAndUpdateSpeeds() — every 10 ms (~100 Hz)
//   3. updateLcdRow()  — every 62 ms (~4 Hz full refresh)
//
// runSpeed() is non-blocking: it checks micros() internally and pulses
// STEP only when a step is due, returning in a few microseconds either
// way. Calling it unconditionally at the top of every iteration gives
// the tightest achievable step timing in a cooperative loop.
//
// The millis() checks below are two integer subtractions and comparisons —
// effectively free. Heavy work (ADC reads, I2C writes) only runs when its
// timer fires, so the hot path stays as short as possible.
// ============================================================

void loop() {
    // ── TOP PRIORITY ─────────────────────────────────────────
    motors[0].runSpeed();
    motors[1].runSpeed();
    motors[2].runSpeed();

    unsigned long now = millis();

    // ── MEDIUM PRIORITY: read pots, recalculate, update speeds ─
    // ~450 μs of analogRead() work every 10 ms. At 60 RPM × ratio 2
    // (the default ceiling), the step period is 625 μs, so the brief
    // read pause is comparable to one step interval and recovers quickly.
    if (now - lastPotRead >= POT_READ_INTERVAL_MS) {
        lastPotRead = now;
        readAndUpdateSpeeds();
    }

    // ── LOW PRIORITY: refresh one LCD row ────────────────────
    // ~6 ms of I2C write work every 62 ms. Four rows cycle in ~250 ms.
    if (now - lastLcdUpdate >= LCD_LINE_INTERVAL_MS) {
        lastLcdUpdate = now;
        updateLcdRow();
    }
}

// ============================================================
// LCD DISPLAY FORMAT REFERENCE
// ============================================================
//
//  Row 0: "M1:  xxx.x RPM  x.xx"   (motor 1 commanded RPM and ratio)
//  Row 1: "M2:  xxx.x RPM  x.xx"   (motor 2)
//  Row 2: "M3:  xxx.x RPM  x.xx"   (motor 3)
//  Row 3: "Master:  xxx.x RPM  "   (master speed setting)
//
//  Each row is exactly 20 characters. RPM is 1 decimal place; ratio is 2.
//
// ============================================================
// CALIBRATION & FIRST-RUN NOTES
// ============================================================
//
// ── 1. CONFIRM LCD I2C ADDRESS ──────────────────────────────
//    If the display stays blank or shows random blocks on startup:
//    a) Upload the I2C scanner sketch (search "Arduino I2C scanner"
//       for a ready-made example).
//    b) Open Serial Monitor at 115200 baud and note the address found.
//       Common values are 0x27 and 0x3F.
//    c) Change LCD_I2C_ADDRESS at the top of this file to match.
//
// ── 2. SET DM556T MICROSTEP DIP SWITCHES ────────────────────
//    The MICROSTEPS constant in this firmware MUST match your DM556T
//    DIP switch setting. A mismatch causes wrong RPM or stalling.
//
//    The table below is an example only. Actual switch assignments vary
//    between DM556T revisions — always verify against the label printed
//    on your specific driver or its accompanying manual.
//
//    Example microstep table (SW5 SW6 SW7 SW8):
//      OFF OFF OFF OFF → 1  (full step)
//      ON  OFF OFF OFF → 2
//      OFF ON  OFF OFF → 4
//      ON  ON  OFF OFF → 8   ← matches firmware default
//      OFF OFF ON  OFF → 16
//      ON  OFF ON  OFF → 32
//
// ── 3. SET DRIVER PEAK CURRENT (SW1–SW3) ───────────────────
//    Set the driver current to approximately 70–80 % of your motor's
//    rated peak current to balance torque against heat.
//    Example: 3.0 A motor → set driver to ~2.4 A.
//    Running too high causes overheating; too low causes stalling.
//
// ── 4. FIRST-POWER-ON PROCEDURE ─────────────────────────────
//    Step 1 — USB only, no motor PSU connected:
//    a) Upload this sketch via USB. Confirm the LCD shows the startup
//       message, then transitions to four data rows.
//    b) Confirm all four rows show 0.0 RPM with all knobs at minimum.
//    c) Turn each knob individually and confirm its value changes on
//       the LCD.
//    d) Turn the master knob fully to minimum and confirm all RPM
//       rows return to 0.0 — this verifies the dead zone is working.
//    e) Set DEBUG to 1, re-upload, open Serial Monitor at 115200 baud.
//       Confirm RPM and ratio values in the serial output match the LCD.
//       Set DEBUG back to 0 when satisfied.
//
//    Step 2 — Connect and test one motor and driver only:
//    f) Wire and power only Motor 1 and its DM556T. Leave Motors 2
//       and 3 and their drivers unpowered and disconnected.
//    g) Set ratio knobs 2 and 3 to minimum. Slowly raise the master
//       knob and Motor 1 ratio knob. Confirm Motor 1 spins and the
//       LCD shows a non-zero RPM for M1.
//    h) Verify direction, smooth speed change, and clean stop before
//       connecting additional motors.
//
//    Step 3 — Add the remaining motors one at a time:
//    i) Connect and test Motor 2 alone, repeating steps g–h.
//    j) Connect and test Motor 3 alone, repeating steps g–h.
//    k) Finally test all three motors together at low speed.
//
// ── 5. CORRECT MOTOR DIRECTION ──────────────────────────────
//    If a motor spins the wrong way:
//    Option A: Set REVERSE_MOTOR_1 (or 2 or 3) to true and re-upload.
//    Option B: Physically swap two wires on one motor winding
//              (A+ ↔ A−, or B+ ↔ B−, but not both pairs).
//    Do not swap both winding pairs simultaneously — the motor will
//    spin in the same direction but with reduced torque.
//
// ── 6. ADJUSTING SPEED AND RATIO RANGE ──────────────────────
//    — Change MAX_MASTER_RPM for a higher or lower top speed.
//    — Change MAX_RATIO to allow faster individual motor ratios.
//    — After changing either, verify the maximum steps/sec stays
//      within AccelStepper's practical limit on a 16 MHz Mega (~50 000 Hz):
//
//        max steps/sec = MAX_MASTER_RPM × MAX_RATIO
//                        × MOTOR_FULL_STEPS_PER_REV × MICROSTEPS / 60
//        Default settings: 60 × 2 × 200 × 8 / 60 = 3 200 steps/sec   ← safe.
//        At 300 RPM / ratio 3: 300 × 3 × 200 × 8 / 60 = 24 000 steps/sec ← also safe.
//
//      If your calculation exceeds ~40 000 steps/sec, reduce MICROSTEPS
//      or lower MAX_MASTER_RPM / MAX_RATIO accordingly.
//
// ── 7. SMOOTHING ADJUSTMENT ─────────────────────────────────
//    SMOOTH_ALPHA = 0.08 suits most hand-turned pots.
//    — Increase toward 0.3 for faster (less filtered) knob response.
//    — Decrease toward 0.02 for heavier filtering on noisy readings.
//
// ── 8. HIGH-SPEED STEPPING NOTE ─────────────────────────────
//    AccelStepper's runSpeed() is software-timed via micros(). At speeds
//    above ~20 000 steps/sec, or if the main loop has other blocking work,
//    step timing may become irregular. For very high speeds or timing-
//    critical applications, consider hardware timer interrupts (TimerOne
//    library or direct AVR timer registers) to generate step pulses.
//
// ============================================================
