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
 * DM556T Signal Wiring Note:
 *   The DM556T uses differential STEP/DIR inputs (STEP+/STEP- and DIR+/DIR-).
 *   Two common single-ended wiring options:
 *
 *     Common-cathode (recommended):
 *       Connect STEP- and DIR- to GND.
 *       Drive STEP+ and DIR+ from the Arduino pins.
 *       Signals are non-inverted — this is what this firmware assumes.
 *
 *     Common-anode:
 *       Connect STEP+ and DIR+ to +5V.
 *       Drive STEP- and DIR- from the Arduino pins.
 *       Signals are logically inverted; motors may run backwards or not at all
 *       unless you add invert logic or swap DIR wiring.
 *
 *   If motors run in the wrong direction, set the REVERSE_MOTOR_X constant
 *   to true, or physically swap two motor phase wires (A+ ↔ A−).
 *
 * Enable Pins (defined for future use, not driven by this firmware):
 *   Motor 1 ENA -> pin 8    Motor 2 ENA -> pin 9    Motor 3 ENA -> pin 10
 *   Leave the ENA terminals disconnected unless you need software enable/disable
 *   control. Enable polarity varies between driver revisions — consult your exact
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
// See the DM556T manual or the calibration notes at the bottom of this file.
#define MICROSTEPS  8

// Master speed range in RPM (min should normally stay 0)
#define MIN_MASTER_RPM    0.0f
#define MAX_MASTER_RPM   60.0f  // conservative for initial testing; increase once everything runs correctly

// Ratio knob range — each motor RPM = masterRPM × ratio
#define MIN_RATIO  0.0f
#define MAX_RATIO  2.0f  // conservative for initial testing; increase once everything runs correctly

// Master dead zone: raw ADC counts (0–1023) at or below this value are
// treated as zero speed. Prevents unintended creep when the knob is
// at its minimum position. Adjust if your pot doesn't quite reach 0.
#define MASTER_DEAD_ZONE_ADC  30

// Ratio dead zone: ratio values below this are clamped to 0, fully stopping
// the motor. Lets you park individual motors by turning their knob to minimum.
#define RATIO_DEAD_ZONE  0.02f

// LCD update strategy: one row is refreshed every LCD_LINE_INTERVAL_MS ms.
// All four rows cycle at this rate → full display refresh every 4× this value.
// Default 62 ms per row → full refresh ≈ 4 Hz.
// Spreading updates across time keeps each I2C write brief and avoids
// long gaps in motor stepping.
#define LCD_LINE_INTERVAL_MS  62UL

// Analog smoothing strength (exponential moving average alpha).
// Range: 0.0 (no response) to 1.0 (no smoothing).
// 0.08 gives gentle filtering suitable for hand-turned potentiometers.
// Increase toward 0.3 for faster response; decrease toward 0.03 for
// heavier filtering if your readings are very noisy.
#define SMOOTH_ALPHA  0.08f

// Motor direction reversal. Set to true to invert a motor's direction.
// Flip if a motor runs backwards relative to your mechanical requirement.
const bool REVERSE_MOTOR_1 = false;
const bool REVERSE_MOTOR_2 = false;
const bool REVERSE_MOTOR_3 = false;

// Set to 1 to enable Serial debug output at 115200 baud. Set to 0 to disable.
// Prints master RPM and each motor's RPM and ratio once per display refresh cycle.
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
// Initialises to the first sample so there is no startup transient.
class AnalogSmoother {
public:
    explicit AnalogSmoother(float alpha = SMOOTH_ALPHA)
        : _alpha(alpha), _value(-1.0f) {}

    // Pass the raw analogRead() result; returns the smoothed float value.
    float update(int raw) {
        if (_value < 0.0f)
            _value = static_cast<float>(raw);  // seed on first call
        else
            _value += _alpha * (static_cast<float>(raw) - _value);
        return _value;
    }

    float value() const { return _value; }

private:
    float _alpha;
    float _value;
};

// ============================================================
// OBJECTS
// ============================================================

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 20, 4);

// AccelStepper::DRIVER mode uses a dedicated STEP pin and DIR pin.
// setSpeed() + runSpeed() gives constant velocity with no acceleration ramp.
AccelStepper motor1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper motor2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
AccelStepper motor3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);

AnalogSmoother smoothMaster;
AnalogSmoother smoothRatio1;
AnalogSmoother smoothRatio2;
AnalogSmoother smoothRatio3;

// ============================================================
// STATE
// ============================================================

static float masterRPM = 0.0f;
static float ratio1 = 0.0f, ratio2 = 0.0f, ratio3 = 0.0f;
static float rpm1   = 0.0f, rpm2   = 0.0f, rpm3   = 0.0f;

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

// RPM → steps per second using the compile-time motor constants.
static float rpmToStepsPerSec(float rpm) {
    return rpm * (static_cast<float>(MOTOR_FULL_STEPS_PER_REV) * MICROSTEPS) / 60.0f;
}

// ── LCD formatting ─────────────────────────────────────────────────────────
//
// Each row is exactly 20 characters. We use dtostrf() for float-to-string
// conversion (safe on AVR) and then snprintf() to assemble the fixed-width
// line. dtostrf(value, totalWidth, decimals, buf) right-pads with spaces if
// the number is shorter than totalWidth.
//
// Motor row layout (20 chars):
//   "M1:  300.0 RPM  3.00"
//    ^^^  ^^^^^  ^^^  ^^^^
//    3+2  5      6    4    = 20
//
// Master row layout (20 chars):
//   "Master:  300.0 RPM  "
//    ^^^^^^^  ^^^^^  ^^^^
//    9        5      6    = 20

static void lcdWriteMotorRow(uint8_t row, uint8_t motorNum,
                             float rpm, float ratio) {
    char rpmBuf[8];
    char ratioBuf[6];
    char line[21];
    dtostrf(rpm,   5, 1, rpmBuf);   // e.g. "300.0" or "  0.0"
    dtostrf(ratio, 4, 2, ratioBuf); // e.g. "3.00"  or "0.00"
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
// SETUP
// ============================================================

void setup() {
#if DEBUG
    Serial.begin(115200);
    Serial.println(F("Motor Control — starting"));
#endif

    // 400 kHz fast-mode I2C reduces each LCD row write from ~25 ms to ~6 ms,
    // which shortens the gap in motor stepping during display updates.
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
    // Calculate the theoretical maximum steps/sec across all possible settings,
    // with 10 % headroom so AccelStepper never clips the requested speed.
    const float maxSpd = rpmToStepsPerSec(MAX_MASTER_RPM * MAX_RATIO) * 1.1f;
    motor1.setMaxSpeed(maxSpd);
    motor2.setMaxSpeed(maxSpd);
    motor3.setMaxSpeed(maxSpd);

    motor1.setSpeed(0.0f);
    motor2.setSpeed(0.0f);
    motor3.setSpeed(0.0f);

    delay(1500);
    lcd.clear();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
    // ── 1. Read and smooth all potentiometers ─────────────────
    float rawMaster = smoothMaster.update(analogRead(POT_MASTER));
    float rawR1     = smoothRatio1.update(analogRead(POT_RATIO_1));
    float rawR2     = smoothRatio2.update(analogRead(POT_RATIO_2));
    float rawR3     = smoothRatio3.update(analogRead(POT_RATIO_3));

    // ── 2. Master RPM with dead zone ──────────────────────────
    // Below MASTER_DEAD_ZONE_ADC the master is treated as fully off,
    // guaranteeing a clean stop even if the pot doesn't quite hit 0.
    if (rawMaster < static_cast<float>(MASTER_DEAD_ZONE_ADC)) {
        masterRPM = 0.0f;
    } else {
        masterRPM = mapf(rawMaster,
                         static_cast<float>(MASTER_DEAD_ZONE_ADC), 1023.0f,
                         MIN_MASTER_RPM, MAX_MASTER_RPM);
        masterRPM = clampf(masterRPM, MIN_MASTER_RPM, MAX_MASTER_RPM);
    }

    // ── 3. Ratio multipliers with dead zone ───────────────────
    ratio1 = clampf(mapf(rawR1, 0.0f, 1023.0f, MIN_RATIO, MAX_RATIO), MIN_RATIO, MAX_RATIO);
    ratio2 = clampf(mapf(rawR2, 0.0f, 1023.0f, MIN_RATIO, MAX_RATIO), MIN_RATIO, MAX_RATIO);
    ratio3 = clampf(mapf(rawR3, 0.0f, 1023.0f, MIN_RATIO, MAX_RATIO), MIN_RATIO, MAX_RATIO);

    if (ratio1 < RATIO_DEAD_ZONE) ratio1 = 0.0f;
    if (ratio2 < RATIO_DEAD_ZONE) ratio2 = 0.0f;
    if (ratio3 < RATIO_DEAD_ZONE) ratio3 = 0.0f;

    // ── 4. Commanded RPM: motorRPM = masterRPM × ratio ────────
    //
    // Example:  master = 100 RPM, ratios = 1.0 / 0.5 / 2.0
    //   → M1 = 100 RPM,  M2 = 50 RPM,  M3 = 200 RPM
    // Turn master down to 50 RPM — relative ratios are preserved:
    //   → M1 = 50 RPM,   M2 = 25 RPM,  M3 = 100 RPM
    rpm1 = masterRPM * ratio1;
    rpm2 = masterRPM * ratio2;
    rpm3 = masterRPM * ratio3;

    // ── 5. Update motor speeds ────────────────────────────────
    // Positive speed → forward, negative speed → reverse.
    // REVERSE_MOTOR_X flips the sign for motors that are mechanically inverted.
    float speed1 = rpmToStepsPerSec(rpm1) * (REVERSE_MOTOR_1 ? -1.0f : 1.0f);
    float speed2 = rpmToStepsPerSec(rpm2) * (REVERSE_MOTOR_2 ? -1.0f : 1.0f);
    float speed3 = rpmToStepsPerSec(rpm3) * (REVERSE_MOTOR_3 ? -1.0f : 1.0f);

    motor1.setSpeed(speed1);
    motor2.setSpeed(speed2);
    motor3.setSpeed(speed3);

    // ── 6. Step motors — non-blocking ────────────────────────
    // runSpeed() checks micros() internally and pulses the STEP pin only
    // when a step is due. No delay(), no blocking.
    motor1.runSpeed();
    motor2.runSpeed();
    motor3.runSpeed();

    // ── 7. Refresh one LCD row per interval (~4 Hz full cycle) ─
    // Writing all four rows at once could block for ~25 ms (even at 400 kHz I2C),
    // causing missed steps. Spreading updates one row per interval limits each
    // write to ~6 ms.
    unsigned long now = millis();
    if (now - lastLcdUpdate >= LCD_LINE_INTERVAL_MS) {
        lastLcdUpdate = now;

        switch (lcdLine) {
            case 0: lcdWriteMotorRow(0, 1, rpm1, ratio1); break;
            case 1: lcdWriteMotorRow(1, 2, rpm2, ratio2); break;
            case 2: lcdWriteMotorRow(2, 3, rpm3, ratio3); break;
            case 3: lcdWriteMasterRow(masterRPM);          break;
        }
        lcdLine = (lcdLine + 1) & 3;  // cycle 0 → 1 → 2 → 3 → 0

#if DEBUG
        if (lcdLine == 0) {  // print once per complete display refresh cycle
            Serial.print(F("Master="));  Serial.print(masterRPM, 1);
            Serial.print(F(" RPM  |  M1="));
            Serial.print(rpm1, 1);  Serial.print(F(" (x")); Serial.print(ratio1, 2); Serial.print(F(")"));
            Serial.print(F("  M2="));
            Serial.print(rpm2, 1);  Serial.print(F(" (x")); Serial.print(ratio2, 2); Serial.print(F(")"));
            Serial.print(F("  M3="));
            Serial.print(rpm3, 1);  Serial.print(F(" (x")); Serial.print(ratio3, 2); Serial.println(F(")"));
        }
#endif
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
//  Each row is exactly 20 characters. RPM is always 1 decimal place.
//  Ratio is always 2 decimal places.
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
//      or reduce MAX_MASTER_RPM / MAX_RATIO accordingly.
//
// ── 7. SMOOTHING ADJUSTMENT ─────────────────────────────────
//    SMOOTH_ALPHA = 0.08 suits most hand-turned pots.
//    — Increase toward 0.3 for faster (less filtered) knob response.
//    — Decrease toward 0.02 for heavier filtering on noisy readings.
//
// ── 8. HIGH-SPEED STEPPING NOTE ─────────────────────────────
//    AccelStepper's runSpeed() is software-timed via micros(). At speeds
//    above ~20 000 steps/sec, or if the main loop has other blocking work,
//    step timing may become irregular. For very high speeds or
//    timing-critical applications, consider hardware timer interrupts
//    (TimerOne library or direct AVR timer registers) to generate step pulses.
//
// ============================================================
