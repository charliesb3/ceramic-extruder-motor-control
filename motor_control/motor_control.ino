/*
 * ============================================================
 * motor_control.ino — 3-Motor Base/Multiplier Speed Controller
 * Target: Arduino Mega 2560
 * ============================================================
 *
 * Architecture: cooperative state-machine scheduler
 *   Every loop() call unconditionally invokes runSpeed() for all three
 *   motors first. All background work (ADC reads, LCD updates) is broken
 *   into individual sub-operations that each block for at most a few
 *   hundred microseconds, so step generation is never starved.
 *
 *   Pot reads:  one double-read (dummy + real, ~208 µs) per loop iteration,
 *               cycling Master → M1 → M2 → M3 → repeat. Speeds are
 *               recalculated immediately when each pot completes its
 *               MULTI_SAMPLE_COUNT samples.
 *
 *   LCD writes: one I2C operation (~450 µs) per loop iteration when a row
 *               write is in progress. A row is only queued when its
 *               formatted content has actually changed. Rows are checked
 *               at most once every LCD_LINE_INTERVAL_MS milliseconds.
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
 * Motor 1 — HR4988 stepper driver:
 *   Arduino pin 2 -> STEP
 *   Arduino pin 3 -> DIR
 *   Arduino 5V    -> VDD (logic supply)
 *   Arduino/common ground -> logic GND
 *   Motor PSU positive -> VMOT
 *   Motor PSU negative -> motor-power GND
 *   RESET and SLEEP tied HIGH in hardware
 *   EN tied LOW or otherwise permanently enabled in hardware
 *   MS1/MS2/MS3 set in hardware to match MICROSTEPS_M1
 *   Motor coils connected to 1A/1B and 2A/2B
 *   Note: EN, RESET, SLEEP, and MS1/MS2/MS3 are handled entirely in
 *   hardware — this firmware does not drive those pins.
 *
 * Motors 2 and 3 — DM556T stepper drivers:
 *   Motor 2:  Arduino pin 4 -> PUL+/STEP+,  Arduino pin 5 -> DIR+
 *   Motor 3:  Arduino pin 6 -> PUL+/STEP+,  Arduino pin 7 -> DIR+
 *   PUL-/STEP- and DIR- -> Arduino/common logic ground
 *   ENA disconnected
 *
 * DM556T Wiring Options for Motors 2 and 3 (STEP and DIR are differential
 * inputs):
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
 * DM556T Logic Level Note for Motors 2 and 3 (Arduino Mega drives 5V):
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
 *   Motor 1 EN is tied LOW (or permanently enabled) in hardware — no pin needed.
 *   Motor 2 ENA -> pin 9    Motor 3 ENA -> pin 10
 *   Leave ENA terminals disconnected unless you need software enable/disable.
 *   Enable polarity varies between driver revisions — consult your exact
 *   DM556T manual before wiring ENA to avoid accidentally disabling the driver.
 *
 * Power:
 *   Main PSU (24 V or 36 V) -> DM556T V+ / GND terminals and HR4988 VMOT / GND
 *   5 V buck converter output -> Arduino 5V pin, LCD VCC, and HR4988 VDD
 *     CAUTION: Verify the buck output is exactly 5.0 V with a multimeter
 *     before connecting to the Arduino 5V pin. Do NOT feed a 5 V supply
 *     into the Arduino VIN pin — VIN expects roughly 7–9 V and routes
 *     through the onboard regulator; 5 V on VIN may not power the board.
 *
 * Universal Extruder Interface (UEI):
 *   H11L1 logic output -> Arduino Mega D18 (INT5)
 *   H11L1 ground       -> Arduino GND
 *
 *   Current implementation:
 *     Diagnostic input only.
 *
 *   Future implementation:
 *     Interrupt-driven RUN/PAUSE control.
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

// Per-motor microstepping divisors.
// Each constant MUST match that driver's actual microstep wiring or DIP-switch setting.
//
// Motor 1 — HR4988:
//   MS1/MS2/MS3 currently disconnected → full-step mode (1 microstep).
//   MICROSTEPS_M1 = 1
//   To switch to 1/8 microstepping later, wire MS1=HIGH, MS2=HIGH, MS3=LOW
//   and change MICROSTEPS_M1 to 8 — no other code changes are required.
//
// Motor 2 — DM556T:
//   1/8 microstepping set via SW5–SW8 DIP switches.
//   MICROSTEPS_M2 = 8
//
// Motor 3 — DM556T:
//   1/8 microstepping set via SW5–SW8 DIP switches.
//   MICROSTEPS_M3 = 8
#define MICROSTEPS_M1            1  // HR4988 — MS1/MS2/MS3 disconnected, full-step mode
#define MICROSTEPS_M2            8  // DM556T — must match SW5–SW8 DIP switch setting
#define MICROSTEPS_M3            8  // DM556T — must match SW5–SW8 DIP switch setting

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

// Minimum interval (ms) between checks of each LCD row.
// A row is only rewritten when its displayed value has actually changed, so
// this controls how quickly a change appears on the display, not how often
// the display physically updates.
// 4 rows × 250 ms = up to 1 s for a full display cycle when all rows change.
#define LCD_LINE_INTERVAL_MS     250UL

// Number of real ADC samples averaged per pot reading.
// Each sample pair consists of one dummy read (mux settling) and one real read.
// 16 samples reduces random noise by a factor of √16 = 4.
// The MULTI_SAMPLE_COUNT samples for each pot are spread across consecutive
// loop() calls — each call contributes exactly one sample pair (~208 µs).
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

// Set to 1 to print raw ADC counts for all four pots once per ~250 ms.
// Use this to find the calibration endpoints for each pot.
// See calibration note 4 at the bottom for the procedure.
// Set back to 0 after calibrating.
#define DEBUG_RAW_POTS 0

// Set to 1 to skip normal motor control and instead pulse all three STEP pins
// once per second. The pulse is 10 µs wide — long enough for the DM556T and
// HR4988 but slow enough to detect with a multimeter in DC mode (the reading
// will flick briefly on each pulse) or confirm with a logic analyser/oscilloscope.
// Serial Monitor at 115200 prints a confirmation line on every pulse.
// Set back to 0 and re-upload to return to normal operation.
#define DEBUG_STEP_PINS 0

// Set to 1 to enable Serial diagnostic output for the UEI extrusion sensor.
// Prints the initial pin state once at startup, then a rate-limited summary
// approximately every 250 ms: "UEI STATE: HIGH | TRANSITIONS: N".
// Individual edges at high speed are counted but not printed individually —
// the transition counter is the authoritative diagnostic for rapid activity.
// Serial Monitor at 115200 baud. Set back to 0 after verification.
#define DEBUG_EXTRUSION_SENSOR 1

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

#define ENA_PIN_2    9   // Enable pins — defined for future use, not driven
#define ENA_PIN_3   10   // (Motor 1 EN is permanently enabled in hardware)

// Universal Extruder Interface (UEI) — H11L1 extrusion detector
#define EXTRUSION_SENSOR_PIN  18  // Arduino Mega D18 (INT5)

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
#if DEBUG_EXTRUSION_SENSOR
#include <util/atomic.h>   // ATOMIC_BLOCK / ATOMIC_RESTORESTATE for 4-byte ISR-shared variable
#endif

// ============================================================
// ANALOG SMOOTHER — multi-sample average → dead-zone → physical units
// If motor speed jitter is noticeable at steady knob positions, EMA smoothing
// can be layered on top by storing a per-pot running average and passing it
// through deadzoneMapf() instead of the raw average.
// A starting alpha of 0.2 gives ~1.25 s settling time at a ~13 ms pot cycle.

// ============================================================
// OBJECTS
// ============================================================

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 20, 4);

// AccelStepper::DRIVER mode: separate STEP and DIR pins.
// setSpeed() + runSpeed() → constant velocity, no acceleration ramp.
// All three drivers (HR4988 for Motor 1, DM556T for Motors 2 and 3) use the
// same STEP/DIR interface — AccelStepper::DRIVER is compatible with all.
AccelStepper motors[3] = {
    AccelStepper(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1),
    AccelStepper(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2),
    AccelStepper(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3)
};

// ============================================================
// STATE
// ============================================================

static float masterMultiplier = 1.0f;   // clamped 0.0–2.0
static float baseRpms[3]      = {};     // per-motor base RPM (0–200)
static float finalRpms[3]     = {};     // quantized final RPM (0–400), drives motors

// Last RPM value sent to setSpeed(). Initialised to an impossible value so
// the first pot completion always issues setSpeed().
static float lastSetRpms[3] = { -1.0f, -1.0f, -1.0f };

// ── Pot state machine ─────────────────────────────────────────────────────
// One sample pair (dummy + real analogRead) is taken per loop() call.
// potIdx cycles 0→1→2→3→0; potSmpl counts samples within the current pot.
static uint8_t  potIdx  = 0;   // which pot is being sampled (0–3)
static uint8_t  potSmpl = 0;   // samples collected so far for this pot (0–15)
static int32_t  potSum  = 0;   // running sum for current pot

// ── UEI extrusion sensor (D18 / INT5) ─────────────────────────────────────
static volatile bool     extrusionNewData        = false; // set by ISR, cleared in loop()
static volatile uint8_t  extrusionPinState       = 0;     // pin state latched by ISR
static volatile uint32_t extrusionTransitionCount = 0;    // edge count; zeroed after each report

// ── LCD state machine ─────────────────────────────────────────────────────
// lcdCache[row] holds the 20-character string last written to that display row.
// Writing is spread one I2C operation per loop() call:
//   lcdPendCol == 20   → nothing in progress
//   lcdPendCol == 0xFF → setCursor command needed before first character
//   lcdPendCol 0–19   → writing character at that column index
static char     lcdCache[4][21]  = {};    // content currently shown on each row
static uint8_t  lcdPendRow       = 0;    // display row being written
static uint8_t  lcdPendCol       = 20;   // write-state sentinel (see above)
static uint8_t  lcdCheckRow      = 0;    // next row to check for changes (0–3)
static unsigned long lastLcdCheck = 0;   // millis() when tryStartLcdRow() last ran

// ============================================================
// HELPERS
// ============================================================

static float mapf(float x, float inLo, float inHi, float outLo, float outHi) {
    return (x - inLo) / (inHi - inLo) * (outHi - outLo) + outLo;
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Map a pot value to [outLo, outHi] with a low-end dead zone.
// Values within POT_DEADZONE_FRACTION of adcMin return outLo (zero).
// Above the threshold the remaining ADC range is remapped to the full output
// range, so the knob still sweeps the complete output span outside the zone.
static float deadzoneMapf(float value, float adcMin, float adcMax,
                           float outLo, float outHi) {
    float threshold = adcMin + (adcMax - adcMin) * POT_DEADZONE_FRACTION;
    if (value <= threshold) return outLo;
    return clampf(mapf(value, threshold, adcMax, outLo, outHi), outLo, outHi);
}

// Per-motor compile-time scale factors: steps per second per RPM.
// Index 0 = Motor 1 (HR4988, MICROSTEPS_M1).
// Index 1 = Motor 2 (DM556T, MICROSTEPS_M2).
// Index 2 = Motor 3 (DM556T, MICROSTEPS_M3).
static constexpr float STEPS_PER_RPM[3] = {
    (float)MOTOR_FULL_STEPS_PER_REV * MICROSTEPS_M1 / 60.0f,
    (float)MOTOR_FULL_STEPS_PER_REV * MICROSTEPS_M2 / 60.0f,
    (float)MOTOR_FULL_STEPS_PER_REV * MICROSTEPS_M3 / 60.0f
};

static float rpmToStepsPerSec(uint8_t motorIndex, float rpm) {
    return rpm * STEPS_PER_RPM[motorIndex];
}

// ── Per-motor speed recalculation ─────────────────────────────────────────
// Recomputes finalRpms[m] from baseRpms[m] × masterMultiplier and calls
// setSpeed() only when the quantized result differs by at least
// SPEED_UPDATE_THRESHOLD from the last sent value.
static void recalcMotorSpeed(uint8_t m) {
    float rawFinal  = clampf(baseRpms[m] * masterMultiplier, 0.0f, FINAL_MAX_RPM);
    finalRpms[m]    = roundf(rawFinal);
    if (fabsf(finalRpms[m] - lastSetRpms[m]) < SPEED_UPDATE_THRESHOLD) return;
    lastSetRpms[m]  = finalRpms[m];
    float speed     = rpmToStepsPerSec(m, finalRpms[m]) * (REVERSE[m] ? -1.0f : 1.0f);
    motors[m].setSpeed(speed);
}

// ── Pot state machine ──────────────────────────────────────────────────────
// Called once per loop(). Takes ONE sample pair (dummy + real analogRead) for
// the current pot and advances the state machine. When MULTI_SAMPLE_COUNT
// samples have been collected the average is computed, speeds are updated, and
// the machine advances to the next pot.
//
// Pot cycle time: 4 pots × 16 samples × ~208 µs ≈ 13 ms end-to-end, spread
// across individual loop() calls with runSpeed() executing between each.
// Worst-case blocking per call: two analogRead() ≈ ~208 µs on a 16 MHz Mega.
static void doOnePotSample() {
    uint8_t pin = POT_PINS[potIdx];
    analogRead(pin);                  // dummy — lets ADC mux settle after channel switch
    int sample = analogRead(pin);     // real sample
    potSum += sample;
    potSmpl++;
    if (potSmpl < MULTI_SAMPLE_COUNT) return;

    // All samples collected — compute constrained average.
    int avg = (int)(potSum / (int32_t)MULTI_SAMPLE_COUNT);
    if (avg < 0)    avg = 0;
    if (avg > 1023) avg = 1023;
    int raw = constrain(avg, POT_ADC_MIN[potIdx], POT_ADC_MAX[potIdx]);

    // Map to physical quantity and recalculate affected motor speeds.
    float fraw = (float)raw;
    float flo  = (float)POT_ADC_MIN[potIdx];
    float fhi  = (float)POT_ADC_MAX[potIdx];

    if (potIdx == 0) {
        // Master multiplier — affects all three motors.
        masterMultiplier = deadzoneMapf(fraw, flo, fhi,
                                        MASTER_MIN_MULTIPLIER, MASTER_MAX_MULTIPLIER);
        recalcMotorSpeed(0);
        recalcMotorSpeed(1);
        recalcMotorSpeed(2);
    } else {
        // Per-motor base RPM.
        uint8_t m = potIdx - 1;
        baseRpms[m] = deadzoneMapf(fraw, flo, fhi, BASE_MIN_RPM, BASE_MAX_RPM);
        recalcMotorSpeed(m);
    }

#if DEBUG_RAW_POTS
    // Accumulate raw values; print one line per complete 4-pot cycle, rate-
    // limited to ~250 ms so the Serial Monitor stays readable.
    static int           dbgRaw[4] = {};
    static unsigned long dbgLast   = 0;
    dbgRaw[potIdx] = raw;
    if (potIdx == 3) {
        unsigned long n = millis();
        if (n - dbgLast >= 250UL) {
            dbgLast = n;
            Serial.print(F("[CAL] MST raw="));  Serial.print(dbgRaw[0]);
            for (uint8_t i = 0; i < 3; i++) {
                Serial.print(F(" | M")); Serial.print(i + 1);
                Serial.print(F(" raw="));  Serial.print(dbgRaw[i + 1]);
            }
            Serial.println();
        }
    }
#endif

    // Advance to the next pot.
    potSum  = 0;
    potSmpl = 0;
    potIdx  = (potIdx + 1) & 3;
}

// ── LCD state machine ──────────────────────────────────────────────────────
//
// Row format reference (every row is exactly 20 characters):
//
//   Motor rows (rows 0–2):
//     "M1:  400RPM B:200   "
//      M%d = 2, ": " = 2, %3d = 3, "RPM B:" = 6, %3d = 3, "   " = 3 → 20
//
//   Master row (row 3):
//     "Master:200% x2.00   "
//      "Master:" = 7, %3d = 3, "% x" = 3, dtostrf(4,2) = 4, "   " = 3 → 20

// Format current values for display row 'row' into buf (must be ≥21 bytes).
static void formatLcdRow(uint8_t row, char *buf) {
    if (row < 3) {
        int f = (int)clampf(finalRpms[row], 0.0f, FINAL_MAX_RPM);
        int b = (int)roundf(clampf(baseRpms[row], BASE_MIN_RPM, BASE_MAX_RPM));
        snprintf(buf, 21, "M%d:  %3dRPM B:%3d   ", row + 1, f, b);
    } else {
        float cMult = clampf(masterMultiplier, MASTER_MIN_MULTIPLIER, MASTER_MAX_MULTIPLIER);
        int   pct   = (int)roundf(clampf(cMult * 100.0f, 0.0f, 200.0f));
        char  mBuf[6];
        dtostrf(cMult, 4, 2, mBuf);   // e.g. "0.00", "1.00", "2.00"
        snprintf(buf, 21, "Master:%3d%% x%s   ", pct, mBuf);
    }
}

// Called once per loop() when a row write is in progress.
// Issues exactly ONE I2C operation (setCursor or one character write).
// Worst-case blocking: ~450 µs at 100 kHz I2C.
static void stepLcd() {
    if (lcdPendCol == 0xFF) {
        lcd.setCursor(0, lcdPendRow);
        lcdPendCol = 0;
    } else if (lcdPendCol < 20) {
        lcd.write((uint8_t)lcdCache[lcdPendRow][lcdPendCol]);
        lcdPendCol++;
    }
}

// Called every LCD_LINE_INTERVAL_MS when no row write is in progress.
// Checks the next display row. If its formatted string differs from the cached
// (last-written) content a write is queued; otherwise the row is skipped.
// Cycles through rows 0–3.
static void tryStartLcdRow() {
    uint8_t row = lcdCheckRow;
    lcdCheckRow = (lcdCheckRow + 1) & 3;

    char newLine[21];
    formatLcdRow(row, newLine);
    if (memcmp(newLine, lcdCache[row], 20) == 0) return;  // no change — skip

    memcpy(lcdCache[row], newLine, 21);  // update cache to new content
    lcdPendRow = row;
    lcdPendCol = 0xFF;                   // signal: send setCursor first
}

// ============================================================
// UEI EXTRUSION SENSOR ISR
// ============================================================
// Fires on every CHANGE transition of D18 (INT5).
// Latches pin state, increments the interval counter, signals loop().
// No Serial, LCD, motor, float, timing, or other work allowed here.
static void extrusionISR() {
    extrusionPinState        = (uint8_t)digitalRead(EXTRUSION_SENSOR_PIN);
    extrusionTransitionCount++;
    extrusionNewData         = true;
}

// ============================================================
// SETUP
// ============================================================

void setup() {
#if DEBUG || DEBUG_RAW_POTS || DEBUG_STEP_PINS || DEBUG_EXTRUSION_SENSOR
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

    // Calculate maximum step rate independently for each motor so that
    // different MICROSTEPS values per driver are handled correctly.
    // 10% headroom ensures AccelStepper never silently clips a requested speed.
    for (uint8_t i = 0; i < 3; i++) {
        float maxSpd = FINAL_MAX_RPM * STEPS_PER_RPM[i] * 1.1f;
        motors[i].setMaxSpeed(maxSpd);
        motors[i].setSpeed(0.0f);
    }

#if DEBUG_EXTRUSION_SENSOR
    // INPUT (no pull-up): H11L1 provides a driven logic output.
    // If the idle state is unstable or transitions occur randomly, the input
    // may be floating — investigate whether an external pull-up is required.
    pinMode(EXTRUSION_SENSOR_PIN, INPUT);
    extrusionPinState = (uint8_t)digitalRead(EXTRUSION_SENSOR_PIN);
    attachInterrupt(digitalPinToInterrupt(EXTRUSION_SENSOR_PIN), extrusionISR, CHANGE);
    Serial.print(F("EXTRUSION SENSOR: "));
    Serial.println(digitalRead(EXTRUSION_SENSOR_PIN) ? F("HIGH") : F("LOW"));
#endif

    delay(1500);
    lcd.clear();
}

// ============================================================
// LOOP — cooperative state-machine scheduler
//
// Every iteration, unconditionally:
//   runSpeed() × 3        (~4 µs total — always first, never skipped)
//
// Every iteration, one pot sample:
//   doOnePotSample()      (~208 µs — one dummy + one real analogRead)
//   Speeds recalculate automatically when each pot's 16 samples complete.
//
// Every iteration, one LCD operation (when a row write is in progress):
//   stepLcd()             (~450 µs — one I2C setCursor or character write)
//
// Every LCD_LINE_INTERVAL_MS (when no row write is in progress):
//   tryStartLcdRow()      (pure computation, ~0 µs if no change; queues a
//                          write if the next row's content has changed)
//
// Maximum gap between consecutive runSpeed() calls:
//   Normal (pot sample only):  ~212 µs
//   During LCD row write:      ~454 µs  (one pot sample + one I2C op)
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
        delayMicroseconds(10);          // 10 µs — above DM556T 2.5 µs and HR4988 minimums
        digitalWrite(STEP_PIN_1, LOW);
        digitalWrite(STEP_PIN_2, LOW);
        digitalWrite(STEP_PIN_3, LOW);
        Serial.println(F("[TEST] pulsed STEP: M1=pin2  M2=pin4  M3=pin6"));
    }
    return;
#endif

#if DEBUG_STEP_PINS
    // ── STEP PIN DIAGNOSTIC MODE ─────────────────────────────
    // Pulses all three STEP pins once per second so each pin can be verified
    // individually with a multimeter in DC mode — the reading will flick on
    // each pulse. Probe pin 2 (M1), pin 4 (M2), pin 6 (M3) one at a time.
    // Serial Monitor at 115200 prints a line on every pulse cycle.
    // Normal motor control and LCD updates are suspended while active.
    // Cycles through M1→M2→M3, holding each STEP pin HIGH for 1 s then LOW
    // for 1 s before moving to the next motor. Only one pin is HIGH at a time.
    // 6 states total: motor0-HIGH, motor0-LOW, motor1-HIGH, motor1-LOW, ...
    static unsigned long lastStepPinsPulse = 0;
    static uint8_t       stepPinState      = 0;  // 0–5

    if (now - lastStepPinsPulse >= 1000UL) {
        lastStepPinsPulse = now;

        const uint8_t pins[3]    = { STEP_PIN_1, STEP_PIN_2, STEP_PIN_3 };
        const uint8_t pinNums[3] = { 2, 4, 6 };

        uint8_t motor  = stepPinState / 2;        // 0, 0, 1, 1, 2, 2
        bool    isHigh = (stepPinState % 2) == 0; // HIGH on even states

        // Drive all pins LOW first so only the active motor is ever HIGH.
        digitalWrite(STEP_PIN_1, LOW);
        digitalWrite(STEP_PIN_2, LOW);
        digitalWrite(STEP_PIN_3, LOW);
        if (isHigh) digitalWrite(pins[motor], HIGH);

        Serial.print(F("[STEP] M")); Serial.print(motor + 1);
        Serial.print(F(" pin"));    Serial.print(pinNums[motor]);
        Serial.println(isHigh ? F(" -> HIGH") : F(" -> LOW"));

        stepPinState = (stepPinState + 1) % 6;
    }
    return;
#endif

    // ── TOP PRIORITY: step all motors (unconditional) ────────
    motors[0].runSpeed();
    motors[1].runSpeed();
    motors[2].runSpeed();

    // ── ONE pot sample per iteration ─────────────────────────
    doOnePotSample();

    // ── ONE LCD I2C operation per iteration (if in progress) ─
    stepLcd();

    // ── Start a new LCD row check every LCD_LINE_INTERVAL_MS ─
    // Only runs when no row write is in progress (lcdPendCol == 20).
    if (lcdPendCol == 20 && now - lastLcdCheck >= LCD_LINE_INTERVAL_MS) {
        lastLcdCheck = now;
        tryStartLcdRow();
    }

#if DEBUG_EXTRUSION_SENSOR
    {
        static unsigned long lastUeiReport = 0;
        if (now - lastUeiReport >= 250UL) {
            lastUeiReport = now;

            uint8_t  pinState;
            uint32_t transitions;
            bool     newData;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                newData                  = extrusionNewData;
                extrusionNewData         = false;
                pinState                 = extrusionPinState;
                transitions              = extrusionTransitionCount;
                extrusionTransitionCount = 0;
            }
            (void)newData;

            Serial.print(F("UEI STATE: "));
            Serial.print(pinState ? F("HIGH") : F("LOW"));
            Serial.print(F(" | TRANSITIONS: "));
            Serial.println(transitions);
        }
    }
#endif
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
// ── 2. SET MICROSTEP LEVELS ─────────────────────────────────
//    Motor 1 (HR4988): MICROSTEPS_M1 must match the MS1/MS2/MS3 pin
//    wiring set in hardware. The HR4988 truth table:
//      MS1 MS2 MS3 → microsteps
//       L   L   L  → full step  (1)   ← current setting (pins disconnected)
//       H   L   L  → half step  (2)
//       L   H   L  → quarter    (4)
//       H   H   L  → eighth     (8)
//       H   H   H  → sixteenth  (16)
//
//    Motors 2 and 3 (DM556T): MICROSTEPS_M2/M3 must match the SW5–SW8
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
//      ON  ON  OFF OFF → 8   ← matches MICROSTEPS_M2 / M3 default
//      OFF OFF ON  OFF → 16
//      ON  OFF ON  OFF → 32
//
// ── 3. SET DRIVER PEAK CURRENT ──────────────────────────────
//    Motor 1 (HR4988): Set the onboard current-limiting trimpot to
//    approximately 70–80% of the motor's rated peak current.
//    Formula: Vref = (rated peak current × 0.7) × 8 × Rsense
//    Use a multimeter to measure Vref at the trimpot wiper vs. GND.
//
//    Motors 2 and 3 (DM556T): Set driver current (SW1–SW3) to
//    approximately 70–80% of the motor's rated peak current to balance
//    torque against heat.
//    Example: 3.0 A motor → set driver to ~2.4 A.
//    Running too high causes overheating; too low causes stalling.
//
// ── 4. CALIBRATE POTENTIOMETERS ─────────────────────────────
//    Do this once after wiring, before the first motor test.
//    a) Set DEBUG_RAW_POTS to 1 and re-upload. Open Serial Monitor at 115200.
//    b) Turn the MASTER knob fully left (minimum). The Serial Monitor prints
//       a line roughly every 250 ms:
//         [CAL] MST raw=NNN | M1 raw=NNN | M2 raw=NNN | M3 raw=NNN
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
//    h) Wire and power only Motor 1 and its HR4988 driver. Leave Motors
//       2 and 3 unpowered and disconnected.
//    i) Set master to centre (100%). Slowly raise Motor 1 base RPM knob
//       and confirm the motor spins with increasing speed.
//    j) Move master above and below centre; confirm speed scales up and
//       down. Master at full left must completely stop the motor.
//    k) Verify direction and smooth speed change before continuing.
//
//    Step 3 — Add remaining motors one at a time:
//    l) Connect and test Motor 2 (DM556T) alone, repeating steps i–k.
//    m) Connect and test Motor 3 (DM556T) alone, repeating steps i–k.
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
//        max steps/sec = FINAL_MAX_RPM × MOTOR_FULL_STEPS_PER_REV × MICROSTEPS_Mn / 60
//        Default M1: 400 × 200 × 1 / 60 =  1 333 steps/sec  ← full-step HR4988
//        Default M2/M3: 400 × 200 × 8 / 60 = 10 667 steps/sec  ← 1/8 DM556T
//
//      If your calculation exceeds ~40 000 steps/sec, reduce the relevant
//      MICROSTEPS_Mn constant or lower FINAL_MAX_RPM accordingly.
//
// ── 8. TUNING STABILITY vs. RESPONSE ────────────────────────
//    In the cooperative scheduler, pots are sampled continuously and speeds
//    update within ~13 ms of a knob movement (4 pots × 16 samples × ~208 µs).
//    Increasing MULTI_SAMPLE_COUNT slows response but improves noise rejection:
//
//      MULTI_SAMPLE_COUNT 8:   cycle ~6.5 ms,  noise rejection √8  ≈ 2.8×
//      MULTI_SAMPLE_COUNT 16:  cycle ~13 ms,   noise rejection √16 = 4.0×  ← default
//      MULTI_SAMPLE_COUNT 32:  cycle ~26 ms,   noise rejection √32 ≈ 5.7×
//
//    LCD_LINE_INTERVAL_MS controls how quickly a changed value appears on the
//    display. With 4 rows and 250 ms per check, the worst-case display lag
//    for a single changing row is 250 ms; all four rows changing simultaneously
//    take up to 1 s to fully refresh. Reduce this constant if you need faster
//    display response (at the cost of slightly more LCD overhead in the loop).
//
// ── UEI EXTRUSION SENSOR VERIFICATION ───────────────────────
//    The diagnostic prints approximately four lines per second:
//      UEI STATE: HIGH | TRANSITIONS: N
//    Individual edges are not printed one per line. All detected edges
//    are counted. The H11L1 produces a pulse stream during extrusion —
//    the steady HIGH or LOW state alone does not identify activity;
//    the transition count is the authoritative diagnostic.
//
//    1.  Upload the sketch with DEBUG_EXTRUSION_SENSOR 1.
//    2.  Open Serial Monitor at 115200 baud.
//    3.  Verify the startup state printed (HIGH or LOW at rest).
//    4.  At idle, confirm TRANSITIONS reports 0 each ~250 ms interval.
//    5.  Confirm no random transitions appear while the printer is idle.
//        Random transitions indicate noise, a grounding problem, or a
//        floating input — investigate before proceeding.
//    6.  Trigger slow extrusion; verify TRANSITIONS shows a nonzero count.
//    7.  Trigger rapid extrusion; verify TRANSITIONS shows a higher count.
//    8.  Stop extrusion; verify TRANSITIONS returns to 0.
//    9.  Record the idle logic level. Note: the steady state alone does
//        not identify extrusion — only nonzero TRANSITIONS confirm the
//        pulse stream is being detected correctly.
//   Set DEBUG_EXTRUSION_SENSOR back to 0 after verification.
//
// ── 9. STEPPING SMOOTHNESS NOTE ─────────────────────────────
//    AccelStepper's runSpeed() is software-timed via micros(). The cooperative
//    scheduler limits the worst-case gap between consecutive runSpeed() calls to
//    approximately:
//      ~212 µs during pot sampling alone (no LCD write in progress)
//      ~454 µs during an active LCD row write (pot sample + one I2C op)
//    At Motor 1's maximum (400 RPM full-step = 1 333 steps/sec, one step every
//    750 µs), even the worst-case 454 µs gap is well within one step period.
//    For very high microstepped speeds (e.g. 10 667 steps/sec at 93 µs/step),
//    hardware timer interrupts (TimerOne library or direct AVR timer registers)
//    would eliminate jitter entirely.
//
// ============================================================
