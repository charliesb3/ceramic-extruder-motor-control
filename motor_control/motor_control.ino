/*
 * ============================================================
 * motor_control.ino — 3-Motor Base/Multiplier Speed Controller
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
 *   Master knob wiper       -> A0   (global multiplier: 0%=full left, 100%=centre, 200%=full right)
 *     NOTE: If the master knob reads high (~1000) when turned fully left and low
 *     (~0) when turned fully right (i.e. the multiplier increases as you turn
 *     right instead of left), the outer pot wires are reversed. Swap the two
 *     outer wires on the master pot to correct the direction.
 *   Motor 1 base RPM wiper  -> A1   (0–200 RPM, full left to full right)
 *   Motor 2 base RPM wiper  -> A2   (0–200 RPM, full left to full right)
 *   Motor 3 base RPM wiper  -> A3   (0–200 RPM, full left to full right)
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
 *   If motors run in the wrong direction, set that motor's REVERSE entry to
 *   true or physically swap two wires on one motor winding (A+ ↔ A−).
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
#define LCD_I2C_ADDRESS          0x27

// Steps per full mechanical revolution.
// 1.8° motors = 200 (most common), 0.9° motors = 400.
#define MOTOR_FULL_STEPS_PER_REV 200

// Microstepping divisor — MUST match the DM556T DIP switch setting (SW5–SW8).
// Common values: 1 (full step), 2, 4, 8, 16, 32.
// See calibration notes at the bottom of this file.
#define MICROSTEPS               8

// Per-motor base speed range. Each motor's knob maps full-left to BASE_MIN_RPM
// and full-right to BASE_MAX_RPM, independently of the master knob.
#define BASE_MIN_RPM             0.0f
#define BASE_MAX_RPM             200.0f

// Master multiplier range.
//   Full left  (ADC=0)    → 0.0× →   0% — all motors stopped
//   Centre     (ADC≈512)  → 1.0× → 100% — motors run at their base RPM
//   Full right (ADC=1023) → 2.0× → 200% — motors run at twice their base RPM
#define MASTER_MIN_MULTIPLIER    0.0f
#define MASTER_MAX_MULTIPLIER    2.0f

// Absolute ceiling on final RPM = BASE_MAX_RPM × MASTER_MAX_MULTIPLIER.
// Update this if either constant above changes.
#define FINAL_MAX_RPM            400.0f

// How often (ms) to read all four potentiometers and recalculate speeds.
// 16-sample averaging at 112 μs/read takes ~3.6 ms per pot, so the read
// block itself is ~14 ms regardless of this interval.
#define POT_READ_INTERVAL_MS     250UL

// How often (ms) to refresh one LCD row.
// Four rows × this value = full display refresh period.
// At 100 kHz I2C, writing one row takes roughly 9 ms; spacing rows 100 ms
// apart caps the stepping gap to ~9 ms per interval.
// 4 rows × 100 ms = 400 ms per full refresh ≈ 2.5 Hz.
#define LCD_LINE_INTERVAL_MS     100UL

// Number of real ADC samples averaged per pot reading.
// Each sample is preceded by a discarded dummy read (double-read pattern).
// Total analogRead() calls per pot per interval: MULTI_SAMPLE_COUNT × 2.
// 16 samples reduces random noise by a factor of √16 = 4 compared to one read.
#define MULTI_SAMPLE_COUNT       16

// Low-end dead zone for all potentiometers, as a fraction of each pot's ADC
// range (ADC_MAX − ADC_MIN). While the raw reading sits within this band the
// output is forced to zero — fully left acts as an off switch.
// 0.04 = 4% ≈ the bottom ~41 counts of a 0–1022 range.
// Raise to 0.06–0.08 if the motor still creeps at the fully-left stop.
#define POT_DEADZONE_FRACTION    0.04f

// Minimum change in quantized final RPM before setSpeed() is called.
// Prevents redundant driver commands when the reading hovers at a boundary.
// 1 RPM is the coarsest reasonable step given whole-RPM quantization.
#define SPEED_UPDATE_THRESHOLD   1.0f

// Motor direction reversal — index 0 = Motor 1, 1 = Motor 2, 2 = Motor 3.
// Set an entry to true to reverse that motor's direction.
constexpr bool REVERSE[3] = { false, false, false };

// Set to 1 to enable Serial debug output (computed speeds) at 115200 baud.
#define DEBUG          0

// Set to 1 to print raw and smoothed ADC counts for all four pots on every
// pot-read cycle (~every 250 ms). Use this to find the calibration endpoints
// for each pot. See calibration note 4 at the bottom for the procedure.
// Set back to 0 after calibrating.
#define DEBUG_RAW_POTS 0

// ============================================================
// POT CALIBRATION — measured ADC endpoints per potentiometer
// ============================================================
//
// Most potentiometers do not reach exactly 0 at full-left or exactly 1023
// at full-right. Entering the actual measured endpoints here maps the full
// physical knob travel to the full intended output range, so a motor knob
// turned hard left truly reads 0 RPM and hard right truly reads 200 RPM.
//
// HOW TO CALIBRATE (do once per pot, with motors powered off):
//   Step 1. Set DEBUG_RAW_POTS to 1 above and re-upload the sketch.
//   Step 2. Open Serial Monitor at 115200 baud.
//   Step 3. Turn the MASTER knob fully left (minimum position) and hold it.
//           Note the "MST raw=xxx" value printed each line.
//           Enter that number as MASTER_ADC_MIN below.
//   Step 4. Turn the MASTER knob fully right (maximum position) and hold it.
//           Note the "MST raw=xxx" value.
//           Enter that number as MASTER_ADC_MAX below.
//   Step 5. Repeat steps 3–4 for each motor knob, reading "M1 raw=", "M2 raw=",
//           "M3 raw=" and filling in M1_ADC_MIN / M1_ADC_MAX, etc.
//   Step 6. Set DEBUG_RAW_POTS back to 0 and re-upload.
//
// Uncalibrated safe defaults: 0 and 1023.
// These work without calibration but the knob endpoints may fall slightly
// short of 0 RPM / 200 RPM / 0% / 200%.

#define MASTER_ADC_MIN      0  // raw ADC when master knob is at full-left  (measured 0)
#define MASTER_ADC_MAX   1022  // raw ADC when master knob is at full-right (measured 1022)

#define M1_ADC_MIN          0  // raw ADC when motor 1 knob is at full-left  (measured 0)
#define M1_ADC_MAX       1022  // raw ADC when motor 1 knob is at full-right (measured 1022)

#define M2_ADC_MIN          0  // raw ADC when motor 2 knob is at full-left  (measured 0)
#define M2_ADC_MAX       1022  // raw ADC when motor 2 knob is at full-right (measured 1022)

#define M3_ADC_MIN          0  // raw ADC when motor 3 knob is at full-left  (measured 0)
#define M3_ADC_MAX       1022  // raw ADC when motor 3 knob is at full-right (measured 1022)

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

#define POT_MASTER   A0  // master multiplier knob
#define POT_BASE_1   A1  // motor 1 base RPM knob
#define POT_BASE_2   A2  // motor 2 base RPM knob
#define POT_BASE_3   A3  // motor 3 base RPM knob

// Index 0 = master knob, 1–3 = motor base RPM knobs.
const uint8_t POT_PINS[4] = { POT_MASTER, POT_BASE_1, POT_BASE_2, POT_BASE_3 };

// Calibrated ADC endpoint ranges, indexed the same as POT_PINS.
// Built from the per-pot _ADC_MIN / _ADC_MAX constants in the section above.
const int POT_ADC_MIN[4] = { MASTER_ADC_MIN, M1_ADC_MIN, M2_ADC_MIN, M3_ADC_MIN };
const int POT_ADC_MAX[4] = { MASTER_ADC_MAX, M1_ADC_MAX, M2_ADC_MAX, M3_ADC_MAX };

// ============================================================
// INCLUDES
// ============================================================

#include <Wire.h>
#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>

// ============================================================
// ANALOG SMOOTHER — multi-sample average → hysteresis gate → EMA
// If motor speed jitter is noticeable at steady knob positions, re-enable EMA
// smoothing by restoring AnalogSmoother and replacing the deadzoneMapf() calls
// in readAndUpdateSpeeds() with:
//   float ema = smoother.update(raw);
//   output = deadzoneMapf(ema, adcMin, adcMax, outMin, outMax);
// A starting alpha of 0.2 (at 250 ms interval) gives ~1.25 s settling time.

// ============================================================
// OBJECTS
// ============================================================

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 20, 4);

// AccelStepper::DRIVER mode: separate STEP and DIR pins.
// setSpeed() + runSpeed() → constant velocity, no acceleration ramp.
AccelStepper motors[3] = {
    AccelStepper(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1),
    AccelStepper(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2),
    AccelStepper(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3)
};


// ============================================================
// STATE
// ============================================================

static float masterMultiplier = 1.0f;              // clamped 0.0–2.0
static float baseRpms[3]      = {};                // per-motor base RPM (EMA output, 0–200)
static float finalRpms[3]     = {};                // quantized final RPM (0–400), drives motors

// Last RPM value actually sent to setSpeed(). Initialised to an impossible
// value so the first readAndUpdateSpeeds() call always issues setSpeed().
static float lastSetRpms[3]   = { -1.0f, -1.0f, -1.0f };

static unsigned long lastPotRead   = 0;
static unsigned long lastLcdUpdate = 0;
static uint8_t       lcdLine       = 0;  // next row to refresh (cycles 0–3)

// ============================================================
// HELPERS
// ============================================================

static float mapf(float x, float inLo, float inHi, float outLo, float outHi) {
    return (x - inLo) / (inHi - inLo) * (outHi - outLo) + outLo;
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Map an EMA value to [outLo, outHi] with a low-end dead zone.
// Values within POT_DEADZONE_FRACTION of adcMin return outLo (zero).
// Above the threshold the remaining ADC range is remapped to the full output
// range, so the knob still sweeps the complete output span outside the zone.
static float deadzoneMapf(float value, float adcMin, float adcMax,
                           float outLo, float outHi) {
    float threshold = adcMin + (adcMax - adcMin) * POT_DEADZONE_FRACTION;
    if (value <= threshold) return outLo;
    return clampf(mapf(value, threshold, adcMax, outLo, outHi), outLo, outHi);
}

// Take MULTI_SAMPLE_COUNT samples from pin using the double-read pattern:
// each real reading is preceded by a dummy read that is discarded, allowing
// the ADC sample-and-hold capacitor to fully settle after the mux switches
// channels. Returns the integer average, clamped to 0–1023.
static int readMultiSample(uint8_t pin) {
    int32_t sum = 0;
    for (uint8_t s = 0; s < MULTI_SAMPLE_COUNT; s++) {
        analogRead(pin);         // dummy — lets mux settle, result discarded
        sum += analogRead(pin);  // real sample
    }
    int avg = (int)(sum / (int32_t)MULTI_SAMPLE_COUNT);
    return avg < 0 ? 0 : (avg > 1023 ? 1023 : avg);
}

// Compile-time scale factor: steps per second per RPM.
static constexpr float STEPS_PER_RPM =
    static_cast<float>(MOTOR_FULL_STEPS_PER_REV) * MICROSTEPS / 60.0f;

static float rpmToStepsPerSec(float rpm) {
    return rpm * STEPS_PER_RPM;
}

// ── LCD row formatting ──────────────────────────────────────────────────────
//
// Every row is exactly 20 characters. Integer fields use snprintf %d with
// explicit width so old digits are always overwritten. dtostrf() is used for
// the float multiplier because avr-libc snprintf does not support %f.
//
// Motor row layout (20 chars):
//   "M1:  400RPM B:200   "
//    M   = 1
//    %d  = 1  (motor number 1–3)
//    :   = 1
//    (2 spaces) = 2
//    %3d = 3  (final RPM 0–400, right-justified)
//    RPM B: = 6
//    %3d = 3  (base RPM 0–200, right-justified)
//    (3 spaces) = 3
//    total = 1+1+1+2+3+6+3+3 = 20
//
// Master row layout (20 chars):
//   "Master:200% x2.00   "
//    Master: = 7
//    %3d     = 3  (master percent 0–200, right-justified)
//    %%      = 1  (literal %)
//    (space)x = 2
//    %s      = 4  (dtostrf width 4, 2 decimals: "0.00"–"2.00")
//    (3 spaces) = 3
//    total = 7+3+1+2+4+3 = 20

static void lcdWriteMotorRow(uint8_t row, uint8_t motorNum,
                             int finalRpm, int baseRpm) {
    char line[21];
    snprintf(line, sizeof(line), "M%d:  %3dRPM B:%3d   ",
             motorNum, finalRpm, baseRpm);
    lcd.setCursor(0, row);
    lcd.print(line);
}

static void lcdWriteMasterRow(float multiplier, int pct) {
    char multBuf[6];
    char line[21];
    dtostrf(multiplier, 4, 2, multBuf);  // e.g. "0.00", "1.00", "2.00"
    snprintf(line, sizeof(line), "Master:%3d%% x%s   ", pct, multBuf);
    lcd.setCursor(0, 3);
    lcd.print(line);
}

// ============================================================
// SCHEDULED TASKS
// ============================================================

// Called every POT_READ_INTERVAL_MS.
// Full pipeline: multi-sample read → constrain to calibrated range →
// dead-zone + map to physical units → quantize to whole RPM →
// update setSpeed() only if value changed.
static void readAndUpdateSpeeds() {

    // ── Master multiplier ────────────────────────────────────────────────
    // Constrain the raw reading to the calibrated endpoint range so that
    // mapf() always receives inputs within its domain.
    // ADC_MIN → 0.0× (0%)   midpoint → 1.0× (100%)   ADC_MAX → 2.0× (200%)
    int rawMaster = readMultiSample(POT_PINS[0]);
    rawMaster     = constrain(rawMaster, POT_ADC_MIN[0], POT_ADC_MAX[0]);
    masterMultiplier = deadzoneMapf((float)rawMaster,
        (float)POT_ADC_MIN[0], (float)POT_ADC_MAX[0],
        MASTER_MIN_MULTIPLIER, MASTER_MAX_MULTIPLIER);

#if DEBUG_RAW_POTS
    // Declare storage here so the for-loop blocks below can fill it.
    int dbgRaw[4];
    dbgRaw[0] = rawMaster;
#endif

    // ── Per-motor base RPM and final speed ──────────────────────────────
    // Formula: finalRPM[i] = baseRPM[i] × masterMultiplier
    // Examples (bases = 50 / 100 / 200):
    //   Master  0% → finals =   0 /   0 /   0 RPM
    //   Master 100% → finals =  50 / 100 / 200 RPM
    //   Master 200% → finals = 100 / 200 / 400 RPM
    for (uint8_t i = 0; i < 3; i++) {
        int rawBase = readMultiSample(POT_PINS[i + 1]);
        rawBase     = constrain(rawBase, POT_ADC_MIN[i + 1], POT_ADC_MAX[i + 1]);

#if DEBUG_RAW_POTS
        dbgRaw[i + 1] = rawBase;
#endif

        baseRpms[i] = deadzoneMapf((float)rawBase,
            (float)POT_ADC_MIN[i + 1], (float)POT_ADC_MAX[i + 1],
            BASE_MIN_RPM, BASE_MAX_RPM);

        // Clamp product, then quantize to the nearest whole RPM.
        // Whole-RPM quantization prevents sub-1-RPM noise from reaching setSpeed().
        float rawFinal = clampf(baseRpms[i] * masterMultiplier, 0.0f, FINAL_MAX_RPM);
        finalRpms[i]   = roundf(rawFinal);  // nearest whole RPM, stored as float

        // Push a new speed command only when the quantized value moved by
        // at least SPEED_UPDATE_THRESHOLD RPM from what was last sent.
        if (fabsf(finalRpms[i] - lastSetRpms[i]) >= SPEED_UPDATE_THRESHOLD) {
            lastSetRpms[i] = finalRpms[i];
            float speed    = rpmToStepsPerSec(finalRpms[i])
                             * (REVERSE[i] ? -1.0f : 1.0f);
            motors[i].setSpeed(speed);
        }
    }

#if DEBUG_RAW_POTS
    // One line per read cycle showing the 16-sample average (constrained to
    // calibrated range) for every pot.
    // Use these values to determine _ADC_MIN / _ADC_MAX for each pot:
    //   turn a knob fully left → record "raw=" → enter as that pot's _ADC_MIN
    //   turn a knob fully right → record "raw=" → enter as that pot's _ADC_MAX
    Serial.print(F("[CAL] MST raw="));  Serial.print(dbgRaw[0]);
    for (uint8_t i = 0; i < 3; i++) {
        Serial.print(F(" | M")); Serial.print(i + 1);
        Serial.print(F(" raw="));  Serial.print(dbgRaw[i + 1]);
    }
    Serial.println();
#endif

#if DEBUG
    // Rate-limit serial output to once per second (4 calls × 250 ms).
    static uint8_t dbTick = 0;
    if (++dbTick >= 4) {
        dbTick = 0;
        int pct = (int)roundf(clampf(masterMultiplier * 100.0f, 0.0f, 200.0f));
        Serial.print(F("Master x")); Serial.print(masterMultiplier, 2);
        Serial.print(F(" (")); Serial.print(pct); Serial.print(F("%)"));
        for (uint8_t i = 0; i < 3; i++) {
            Serial.print(F("  M")); Serial.print(i + 1);
            Serial.print(F(" base=")); Serial.print((int)roundf(baseRpms[i]));
            Serial.print(F(" final=")); Serial.print((int)finalRpms[i]);
        }
        Serial.println();
    }
#endif
}

// Called every LCD_LINE_INTERVAL_MS.
// Refreshes one row per call; all four rows cycle in 4 × LCD_LINE_INTERVAL_MS.
static void updateLcdRow() {
    if (lcdLine < 3) {
        // finalRpms[i] is already quantized to a whole RPM; cast is exact.
        int dispFinal = (int)clampf(finalRpms[lcdLine], 0.0f, FINAL_MAX_RPM);
        int dispBase  = (int)roundf(clampf(baseRpms[lcdLine], BASE_MIN_RPM, BASE_MAX_RPM));
        lcdWriteMotorRow(lcdLine, lcdLine + 1, dispFinal, dispBase);
    } else {
        float cMult = clampf(masterMultiplier, MASTER_MIN_MULTIPLIER, MASTER_MAX_MULTIPLIER);
        int   pct   = (int)roundf(clampf(cMult * 100.0f, 0.0f, 200.0f));
        lcdWriteMasterRow(cMult, pct);
    }
    lcdLine = (lcdLine + 1) & 3;  // 0 → 1 → 2 → 3 → 0
}

// ============================================================
// SETUP
// ============================================================

void setup() {
#if DEBUG || DEBUG_RAW_POTS
    Serial.begin(115200);
    Serial.println(F("Motor Control — starting"));
#endif

    Wire.begin();
    // 100 kHz standard-mode I2C. 400 kHz fast-mode caused display glitches
    // on this hardware; 100 kHz is more reliable over longer wire runs.
    Wire.setClock(100000UL);

    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("  Motor Controller  "));
    lcd.setCursor(0, 1);
    lcd.print(F("   Initializing...  "));

    // setMaxSpeed() must be called before setSpeed().
    // 10 % headroom ensures AccelStepper never silently clips a requested speed.
    const float maxSpd = rpmToStepsPerSec(FINAL_MAX_RPM) * 1.1f;
    for (uint8_t i = 0; i < 3; i++) {
        motors[i].setMaxSpeed(maxSpd);
        motors[i].setSpeed(0.0f);
    }

    delay(1500);
    lcd.clear();
}

// ============================================================
// LOOP — three-tier cooperative scheduling
//
// Priority 1 — every iteration, no conditions:
//   runSpeed() × 3  (non-blocking; returns in a few microseconds)
//
// Priority 2 — every 250 ms:
//   readAndUpdateSpeeds()  (~14 ms of analogRead work)
//
// Priority 3 — every 100 ms:
//   updateLcdRow()  (~9 ms of I2C write work for one row)
//
// runSpeed() is at the unconditional top so it gets called as often as
// possible. The millis() comparisons that gate priorities 2 and 3 are two
// integer subtractions — effectively free in the hot path.
// ============================================================

void loop() {
    unsigned long now = millis();

#if DEBUG
    // ── STEP PIN TEST MODE ───────────────────────────────────
    // Sends one 10 µs pulse to every STEP pin once per second so you can
    // probe each pin with a multimeter (DC voltage will read low but flick
    // briefly) or an oscilloscope/logic analyser to confirm the signal.
    // Serial Monitor at 115200 will print a confirmation line each pulse.
    // Set DEBUG back to 0 and re-upload to return to normal operation.
    static unsigned long lastPulse = 0;
    if (now - lastPulse >= 1000UL) {
        lastPulse = now;
        digitalWrite(STEP_PIN_1, HIGH);
        digitalWrite(STEP_PIN_2, HIGH);
        digitalWrite(STEP_PIN_3, HIGH);
        delayMicroseconds(10);          // 10 µs — above DM556T 2.5 µs minimum
        digitalWrite(STEP_PIN_1, LOW);
        digitalWrite(STEP_PIN_2, LOW);
        digitalWrite(STEP_PIN_3, LOW);
        Serial.println(F("[TEST] pulsed STEP: M1=pin2  M2=pin4  M3=pin6"));
    }
    return;
#endif

    // ── TOP PRIORITY: step all motors ────────────────────────
    motors[0].runSpeed();
    motors[1].runSpeed();
    motors[2].runSpeed();

    // ── MEDIUM PRIORITY: read pots and recalculate ───────────
    if (now - lastPotRead >= POT_READ_INTERVAL_MS) {
        lastPotRead = now;
        readAndUpdateSpeeds();
    }

    // ── LOW PRIORITY: refresh one LCD row ────────────────────
    if (now - lastLcdUpdate >= LCD_LINE_INTERVAL_MS) {
        lastLcdUpdate = now;
        updateLcdRow();
    }
}

// ============================================================
// LCD DISPLAY FORMAT REFERENCE
// ============================================================
//
//  Row 0: "M1:  xxx RPM B:xxx   "  — motor 1 final RPM and base RPM
//  Row 1: "M2:  xxx RPM B:xxx   "  — motor 2
//  Row 2: "M3:  xxx RPM B:xxx   "  — motor 3
//  Row 3: "Master:xxx% x X.XX   "  — master percent (0–200) and multiplier
//
//  Each row is exactly 20 characters with fixed-width integer fields.
//  Final and base RPM are displayed in whole RPM. Master percent is an
//  integer 0–200; 100 means motors run at their base speed.
//
//  Example readings (bases = 50 / 100 / 200 RPM, master at centre = 100%):
//    "M1:   50RPM B: 50   "
//    "M2:  100RPM B:100   "
//    "M3:  200RPM B:200   "
//    "Master:100% x1.00   "
//
//  With master at full right (200%):
//    "M1:  100RPM B: 50   "
//    "M2:  200RPM B:100   "
//    "M3:  400RPM B:200   "
//    "Master:200% x2.00   "
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
//    Set driver current to approximately 70–80 % of your motor's rated
//    peak current to balance torque against heat.
//    Example: 3.0 A motor → set driver to ~2.4 A.
//    Running too high causes overheating; too low causes stalling.
//
// ── 4. CALIBRATE POTENTIOMETERS ─────────────────────────────
//    Do this once after wiring, before the first motor test.
//    a) Set DEBUG_RAW_POTS to 1 and re-upload. Open Serial Monitor at 115200.
//    b) Turn the MASTER knob fully left (minimum). The Serial Monitor prints
//       a line every 250 ms:
//         [CAL] MST raw=NNN ema=NNN.N | M1 raw=NNN ...
//       Note the "MST raw=" value and enter it as MASTER_ADC_MIN.
//    c) Turn MASTER fully right. Note "MST raw=" → enter as MASTER_ADC_MAX.
//    d) Repeat for each motor knob: full-left reading → M1_ADC_MIN (etc.),
//       full-right reading → M1_ADC_MAX (etc.).
//    e) Set DEBUG_RAW_POTS back to 0 and re-upload.
//    The knobs now span their full physical travel across the full speed range.
//
// ── 5. FIRST-POWER-ON PROCEDURE ─────────────────────────────
//    Step 1 — USB only, no motor PSU connected:
//    a) Upload this sketch via USB. Confirm the LCD shows the startup
//       message, then transitions to four data rows.
//    b) With all motor knobs and master knob at full left, confirm all
//       rows show 0 RPM and "Master:  0% x0.00".
//    c) Turn master to centre — confirm it shows "Master:100% x1.00".
//       Note: response is intentionally slow (~1 s); this is normal.
//    d) Raise a motor knob with master at centre and confirm its base
//       RPM and final RPM change together (1:1 at 100%).
//    e) With motor knobs set, sweep master left-to-right and confirm
//       final RPMs scale from 0 to 2× base while base RPMs stay fixed.
//    f) Confirm all values are bounded: base RPM never exceeds 200,
//       final RPM never exceeds 400, master never exceeds 200%.
//    g) Set DEBUG to 1, re-upload, open Serial Monitor at 115200 baud.
//       Confirm serial output matches the LCD. Set DEBUG back to 0.
//
//    Step 2 — Connect and test one motor and driver only:
//    h) Wire and power only Motor 1 and its DM556T driver. Leave Motors
//       2 and 3 unpowered and disconnected.
//    i) Set master to centre (100%). Slowly raise Motor 1 base RPM knob
//       and confirm the motor spins with increasing speed.
//    j) Move master above and below centre; confirm speed scales up and
//       down. Master at full left must completely stop the motor.
//    k) Verify direction and smooth speed change before continuing.
//
//    Step 3 — Add remaining motors one at a time:
//    l) Connect and test Motor 2 alone, repeating steps i–k.
//    m) Connect and test Motor 3 alone, repeating steps i–k.
//    n) Test all three together at various master and base RPM settings.
//
// ── 6. CORRECT MOTOR DIRECTION ──────────────────────────────
//    If a motor spins the wrong way:
//    Option A: Set that motor's REVERSE entry to true and re-upload.
//    Option B: Physically swap two wires on one motor winding
//              (A+ ↔ A−, or B+ ↔ B−, but not both pairs simultaneously).
//
// ── 7. ADJUSTING SPEED RANGE ────────────────────────────────
//    — Change BASE_MAX_RPM to raise or lower the per-motor ceiling.
//    — Change MASTER_MAX_MULTIPLIER to change the master's boost range.
//    — Update FINAL_MAX_RPM = BASE_MAX_RPM × MASTER_MAX_MULTIPLIER.
//    — After any change, verify max steps/sec stays within AccelStepper's
//      practical limit on a 16 MHz Mega (~50 000 steps/sec):
//
//        max steps/sec = FINAL_MAX_RPM × MOTOR_FULL_STEPS_PER_REV × MICROSTEPS / 60
//        Default: 400 × 200 × 8 / 60 = 10 667 steps/sec  ← well within limit.
//
//      If your calculation exceeds ~40 000 steps/sec, reduce MICROSTEPS
//      or lower FINAL_MAX_RPM accordingly.
//
// ── 8. TUNING STABILITY vs. RESPONSE ────────────────────────
//    The firmware has three knobs for trading responsiveness against stability.
//    Adjust them in this order if the display is still noisy or too sluggish:
//
//    a) POT_READ_INTERVAL_MS (default 250):
//       Raise to 500 for very slow, very stable hand control.
//       Lower to 100 for snappier (but noisier) response.
//
//    d) MULTI_SAMPLE_COUNT (default 16):
//       Raise to 32 for maximum noise rejection (doubles read time to ~7 ms/pot).
//       Lower to 8 if read time must be shorter.
//
// ── 9. HIGH-SPEED STEPPING NOTE ─────────────────────────────
//    AccelStepper's runSpeed() is software-timed via micros(). At speeds
//    above ~20 000 steps/sec, or if the main loop has other blocking work,
//    step timing may become irregular. For very high speeds or timing-
//    critical applications, consider hardware timer interrupts (TimerOne
//    library or direct AVR timer registers) to generate step pulses.
//
// ============================================================
