/*
 * ============================================================
 *  Camera Slider Controller  —  12864 Screen Mod  v1.1
 *  Hardware : Arduino Uno
 *             BIGTREETECH MINI12864 V2.0  (SPI, UC1701)
 *             TMC2209 + NEMA 17
 *             2 endstops
 *             2 external buttons: LEFT, RIGHT
 *             1 external E-STOP panel button
  *             WS2812 RGB backlight (on Mini12864 board)
 *
 *  PIN MAP
 *  -------
 *  MINI12864 EXP1 connector:
 *    EXP1-1  BEEPER    → (not connected)
 *    EXP1-2  BTN_ENC   → D4   encoder push / select
 *    EXP1-3  LCD_RST   → D8   display reset
 *    EXP1-4  LCD_A0/RS → A3   data/command select
 *    EXP1-5  BTN_BACK  → D5   back button
 *    EXP1-6  BTN_CONF  → D6   confirm button  (KILL on BTT label)
 *    EXP1-7  NEOPIXEL  → A4   WS2812 RGB data
 *    EXP1-8  GND       → GND
 *    EXP1-9  KILL      → (not connected — E-Stop is external)
 *    EXP1-10 5V        → 5V
 *
 *  MINI12864 EXP2 connector:
 *    EXP2-1  MISO      → D12  (SPI — not used but wired for completeness)
 *    EXP2-2  SCK       → D13  SPI clock
 *    EXP2-3  ENC_CLK   → D2   encoder A (hardware interrupt)
 *    EXP2-4  SD_CS     → (not connected)
 *    EXP2-5  ENC_DT    → D3   encoder B
 *    EXP2-6  MOSI      → D11  SPI data  ← NOTE: shares EN_PIN — see below
 *    EXP2-7  LCD_CS    → A5   display chip select
 *    EXP2-8  GND       → GND
 *    EXP2-9  NC        → (not connected)
 *    EXP2-10 5V        → 5V (or NC)
 *
 *  ⚠  PIN CONFLICT NOTE:
 *    EXP2-6 (MOSI) uses D11 — the same pin previously used for TMC2209 EN.
 *    TMC2209 EN is moved to A2 in this fork.
 *    The external E-STOP button moves to A0 (previously BTN_LEFT).
 *    BTN_LEFT moves to A1, BTN_RIGHT moves to A2... see full map below.
 *
 *  REVISED PIN ASSIGNMENTS:
 *    ENC CLK (EXP2-3) → D2   (interrupt)
 *    ENC DT  (EXP2-5) → D3
 *    ENC SW  (EXP1-2) → D4
 *    BTN BACK(EXP1-5) → D5
 *    BTN CONF(EXP1-6) → D6
  *    LCD RST (EXP1-3) → D8
 *    STEP PIN         → D9
 *    DIR  PIN         → D10
 *    MOSI (EXP2-6)    → D11  (SPI — shared with former EN_PIN)
 *    MISO (EXP2-1)    → D12  (SPI)
 *    SCK  (EXP2-2)    → D13  (SPI)
 *    BTN  LEFT        → A0
 *    BTN  RIGHT       → A1
 *    TMC2209 EN       → A2   (moved from D11)
 *    LCD A0  (EXP1-4) → A3
 *    NEOPIXEL(EXP1-7) → A4
 *    LCD CS  (EXP2-7) → A5
 *    Endstop LEFT     → (use external interrupt or polling — moved to software)
 *
 *  ⚠  UNO SPI CONFLICT:
 *    Hardware SPI on Uno uses D11(MOSI), D12(MISO), D13(SCK).
 *    Endstops must move off D12/D13. Wire endstops to free analog pins:
 *    Endstop LEFT  → A6  (if Uno has it) — or use software SPI instead.
 *    For simplicity this fork uses U8g2 SOFTWARE SPI to free D11–D13
 *    for endstops, keeping the original D12/D13 endstop wiring.
 *    Software SPI pins chosen to avoid all conflicts:
 *      SCK  → D13   CLK
 *      MOSI → D11   data  (same physical pins, but driven by software)
 *      CS   → A5
 *      A0   → A3
 *      RST  → D8
 *
 *  FINAL CLEAN PIN MAP (this fork):
 *    D2   ENC CLK (interrupt)     D3   ENC DT
 *    D4   ENC SW (select)         D5   BTN BACK
  *    D6   BTN CONFIRM             D7   (free)
 *    D8   LCD RESET               D9   STEP
 *    D10  DIR                     D11  LCD MOSI (software SPI)
 *    D12  Endstop LEFT            D13  LCD SCK  (software SPI)
 *    A0   BTN LEFT                A1   BTN RIGHT
 *    A2   TMC2209 EN              A3   LCD A0/RS
 *    A4   NEOPIXEL data           A5   LCD CS
 *    BTN ESTOP (panel) → external wire to A0 shared with... 
 *    *** E-STOP is its own dedicated panel button wired to:
 *    Use D0 is risky (serial). Add a small PCB jumper or use 
 *    the KILL pin on EXP1-9 → wire to A0, move BTN_LEFT to a
 *    dedicated screw terminal on the PCB.
 *
 *  SIMPLIFIED FINAL ASSIGNMENTS USED IN CODE:
 *    E-STOP panel button → EXP1 KILL pin → connected to A0
 *    BTN LEFT  → dedicated screw terminal → A1 (was A0)
 *    BTN RIGHT → dedicated screw terminal → (share with confirm? No.)
 *    *** For this fork, BTN_RIGHT is removed from the pin map and
 *    jog right is handled by encoder CW in jog mode, jog left by CCW.
 *    Physical LEFT/RIGHT buttons: LEFT=A1, ESTOP=A0, RIGHT omitted
  *
  *  Libraries: U8g2, AccelStepper,
 *             Adafruit NeoPixel  (Library Manager)
 *
 *  --------
 *  1. Upload IR_Learn.ino to discover your remote's button codes.
 *  2. Open Serial Monitor at 115200 baud and press each button.
 * ============================================================
 */

#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>
#include <AccelStepper.h>
#include <EEPROM.h>

// ─────────────────────────────────────────────

// ─────────────────────────────────────────────

// ─────────────────────────────────────────────

//  Replace 0x00000000 with your remote's codes.
// ─────────────────────────────────────────────

// ─────────────────────────────────────────────
//  DISPLAY  —  Mini12864 UC1701, software SPI
//  Software SPI avoids hardware SPI conflicts
//  with endstop pins D12/D13.
//  U8g2 constructor: (rotation, cs, a0, reset, clock, data)
// ─────────────────────────────────────────────
#define LCD_CS    A5
#define LCD_A0    A3
#define LCD_RST   8
#define LCD_SCK   13
#define LCD_MOSI  11

U8G2_UC1701_MINI12864_1_4W_SW_SPI u8g2(
  U8G2_R0,   // no rotation
  LCD_SCK,   // clock
  LCD_MOSI,  // data
  LCD_CS,    // chip select
  LCD_A0,    // data/command
  LCD_RST    // reset
);

// ─────────────────────────────────────────────
//  NEOPIXEL BACKLIGHT
//  Mini12864 V2.0 has 2 WS2812 LEDs on EXP1-7
// ─────────────────────────────────────────────
#define NEO_PIN    A4
#define NEO_COUNT  2
Adafruit_NeoPixel neo(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

// Backlight state colours
#define NEO_IDLE    neo.Color( 0, 180,   0)  // green
#define NEO_RUNNING neo.Color( 0,   0, 180)  // blue
#define NEO_PAUSED  neo.Color(200, 120,   0)  // amber
#define NEO_ESTOP   neo.Color(255,   0,   0)  // red
#define NEO_HOMING  neo.Color(200, 200, 200)  // white
#define NEO_OFF     neo.Color(  0,   0,   0)

void setBacklight(uint32_t colour) {
  neo.fill(colour);
  neo.show();
}

// ─────────────────────────────────────────────
//  PINS
// ─────────────────────────────────────────────
// ─────────────────────────────────────────────
//  PINS
// ─────────────────────────────────────────────
#define ENC_CLK      2     // EXP2-3  hardware interrupt
#define ENC_DT       3     // EXP2-5
#define ENC_SW       4     // EXP1-2  short=confirm  long=E-Stop
#define BTN_RIGHT    5     // EXP1-5  short=slow right  long=fast right
#define BTN_PLAYPAUSE 6    // EXP1-6  short=start/pause  long=cancel
#define STEP_PIN     9
#define DIR_PIN      10
//      EN_PIN       —     // TMC2209 EN tied permanently to GND (always enabled)
#define ENDSTOP_PIN  12    // both endstops wired NC in parallel to one pin
//      D13          —     // SPI SCK (LCD only — no endstop)
#define BTN_LEFT     A0    // short=slow left  long=fast left
//      A1           —     // free
//      A2           —     // free (EN moved to GND)
#define LCD_A0       A3    // EXP1-4  display data/command
#define NEO_PIN      A4    // EXP1-7  WS2812 backlight
#define LCD_CS       A5    // EXP2-7  display chip select

// ─────────────────────────────────────────────
//  MOTOR CONFIG
// ─────────────────────────────────────────────
#define STEPS_PER_MM       80
#define JOG_SLOW_STEPS     (STEPS_PER_MM * 1)
#define JOG_FAST_SPEED     (STEPS_PER_MM * 30)
#define JOG_SLOW_SPEED     (STEPS_PER_MM * 5)
#define HOME_SPEED         (STEPS_PER_MM * 5)
#define DEFAULT_SPEED_MMS  10
#define DEFAULT_LOOP_COUNT 0
#define BTN_HOLD_MS        400
#define MAX_TRAVEL_STEPS   (STEPS_PER_MM * 200)   // physical rail length

// ─────────────────────────────────────────────
//  DATA STRUCTURES
// ─────────────────────────────────────────────
#define MAX_STOPS  10
#define NUM_SLOTS  4

struct StopPoint {
  long     position;    // steps from home
  uint16_t timerSec;   // dwell in seconds
};

struct ProgramSlot {
  bool      valid;
  uint16_t  speedMms;   // travel speed mm/s
  uint16_t  loopCount;  // 0 = continuous
  long      startPos;
  long      endPos;
  uint8_t   stopCount;
  StopPoint stops[MAX_STOPS];
};

struct FullLoopSettings {
  uint16_t speedMms;
  uint16_t loopCount;
};

// ─────────────────────────────────────────────
//  RUNTIME STATE
// ─────────────────────────────────────────────
ProgramSlot      slots[NUM_SLOTS];
FullLoopSettings fullLoop = { DEFAULT_SPEED_MMS, DEFAULT_LOOP_COUNT };

// Active program selection: 0-3=slots, 4=FullLoop, 5=MoveToEnd, -1=none
int  activeSlot  = -1;
bool isHomed     = false;

// ── Program execution state machine ──
enum RunState {
  RUN_IDLE,
  RUN_MOVING_TO_START,
  RUN_MOVING_TO_STOP,
  RUN_DWELLING,
  RUN_MOVING_TO_END,
  RUN_RETURNING,
  RUN_PAUSED,
  RUN_DONE,
  // Full loop specific
  RUN_FL_FORWARD,
  RUN_FL_BACKWARD,
  RUN_FL_DONE,
  // Move to endpoint
  RUN_MTE_MOVING
};

RunState runState      = RUN_IDLE;
bool     motorRunning  = false;
bool     programActive = false;

// Execution tracking
uint16_t  currentLoop   = 0;    // which loop iteration we're on
uint8_t   currentStop   = 0;    // which stop point we're heading to
unsigned long dwellStart = 0;   // millis() when dwell began
long      dwellDuration  = 0;   // ms to dwell

// Program-building workspace
ProgramSlot wip;
int         wipSlot     = -1;
bool        wipHasStart = false;
bool        wipHasEnd   = false;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ─────────────────────────────────────────────
//  ENCODER ISR
// ─────────────────────────────────────────────
volatile int encDelta = 0;
void encoderISR() {
  bool clk = digitalRead(ENC_CLK);
  bool dt  = digitalRead(ENC_DT);
  if (!clk) encDelta += (dt != clk) ? +1 : -1;
}

// ─────────────────────────────────────────────
//  BUTTON STRUCT
// ─────────────────────────────────────────────
struct Button {
  uint8_t       pin;
  bool          last      = HIGH;
  bool          pressed   = false;
  unsigned long downAt    = 0;
  bool          held      = false;
};

Button btnEnc      = { ENC_SW       };   // short=confirm  long=E-Stop
Button btnRight    = { BTN_RIGHT    };   // short=slow right  long=fast right
Button btnPlayPause= { BTN_PLAYPAUSE};   // short=start/pause  long=cancel
Button btnLeft     = { BTN_LEFT     };   // short=slow left   long=fast left

// Track encoder long-press separately so we can fire E-Stop once on hold
bool encLongFired = false;

void updateButton(Button &b) {
  bool cur = digitalRead(b.pin);
  b.pressed = false;
  if (cur == LOW && b.last == HIGH) { b.downAt = millis(); b.held = false; }
  if (cur == LOW && !b.held && millis() - b.downAt >= BTN_HOLD_MS) b.held = true;
  if (cur == HIGH && b.last == LOW) { if (!b.held) b.pressed = true; b.held = false; }
  b.last = cur;
}

void updateAllButtons() {
  updateButton(btnEnc);
  updateButton(btnRight);
  updateButton(btnPlayPause);
  updateButton(btnLeft);
}

// ─────────────────────────────────────────────
//  MENU IDs
// ─────────────────────────────────────────────
enum MenuID {
  MNU_ROOT, MNU_SELECT_PROGRAM, MNU_CLEAR_PROGRAM,
  MNU_FULL_LOOP_SETTINGS, MNU_MOTOR_CONTROL, MNU_PROG_MOVEMENT,
  MNU_PROG_SELECT_SLOT, MNU_CONFIRM_CLEAR, MNU_CONFIRM_CLEAR_ALL,
  MNU_CONFIRM_DISABLE, MNU_EDIT_VALUE, MNU_ADD_STOP_TIMER,
  MNU_RUN_SCREEN, MNU_PAUSE_SCREEN, MNU_ESTOP_SCREEN,
  MNU_HOME_REQUIRED   // prompt before starting when not homed
};

MenuID  currentMenu = MNU_ROOT;
int     menuCursor  = 0;
int     menuScroll  = 0;
#define VISIBLE_ROWS 4

// ─────────────────────────────────────────────
//  VALUE EDITOR
// ─────────────────────────────────────────────
struct ValueEditor {
  const char* label;
  long value, minVal, maxVal, step;
  MenuID onDone;
  void*  target;
};
ValueEditor ved;

int clearSlotTarget = -1;

// ─────────────────────────────────────────────
//  MENU ITEM ARRAYS
// ─────────────────────────────────────────────
const char* rootItems[] = {
  "Home All Motors","Start / Stop","Select Program",
  "Clear Program","Full Loop Settings","Motor Control","Program Movement"
};
const uint8_t rootCount = 7;

const char* fullLoopItems[] = {
  "Set Speed","Set Loop Count","Reset to Default","< Back"
};
const uint8_t fullLoopCount = 4;

const char* motorCtrlItems[] = {
  "Home All Motors","Disable Motors","< Back"
};
const uint8_t motorCtrlCount = 3;

// Program Movement now includes Set Speed
const char* progMovItems[] = {
  "Select Slot","Home","Set Speed","Set Start",
  "Add Stop Point","Set End Point","Set Loop Count","Save Program","< Back"
};
const uint8_t progMovCount = 9;

const char* progSlotItems[] = {
  "Slot 1","Slot 2","Slot 3","Slot 4","< Back"
};
const uint8_t progSlotCount = 5;

// ─────────────────────────────────────────────
//  MOTOR HELPERS
// ─────────────────────────────────────────────
// EN is tied permanently LOW — motor always enabled.
// Call disableMotor() only in software by stopping steps;
// the coils stay energized (holding torque maintained).
// To fully de-energize, a relay or manual EN wire cut is needed.
void enableMotor(bool) {}   // no-op — EN hardwired LOW

// Single endstop pin — both switches in parallel (NC)
// Direction of travel tells us which end was hit.
bool endstopTriggered()  { return digitalRead(ENDSTOP_PIN) == LOW; }
bool endstopLeft()       { return endstopTriggered() && stepper.speed() <= 0; }
bool endstopRight()      { return endstopTriggered() && stepper.speed() >= 0; }

void setRunSpeed(float sps) {
  stepper.setMaxSpeed(sps);
  stepper.setAcceleration(sps * 2.0);
}

void displayOn()  { u8g2.setPowerSave(0); }
void displayOff() { u8g2.setPowerSave(1); }

void stopMotorHard() {
  stepper.stop();
  while (stepper.isRunning()) stepper.run();
  stepper.setSpeed(0);
  enableMotor(false);
  motorRunning  = false;
  programActive = false;
  runState      = RUN_IDLE;
  displayOn();
  setBacklight(NEO_IDLE);  // green — back to idle
  openMenu(MNU_ROOT);
}

void decelStop() {
  stepper.stop();
  while (stepper.isRunning()) stepper.run();
  stepper.setSpeed(0);
}

void homeMotor() {
  isHomed = false;
  setBacklight(NEO_HOMING);
  setRunSpeed(HOME_SPEED);
  stepper.setSpeed(-HOME_SPEED);
  while (!endstopTriggered()) {
    stepper.runSpeed();
    // Allow encoder long-press E-Stop even during homing
    if (btnEnc.held && !encLongFired) break;
  }
  stepper.setCurrentPosition(0);
  stepper.stop();
  isHomed = true;
  setBacklight(NEO_IDLE);
}

// ─────────────────────────────────────────────
//  JOG  (locked out during active run)
//  LEFT button:  hold=fast left,  tap=1mm left
//  RIGHT button: hold=fast right, tap=1mm right
// ─────────────────────────────────────────────
void handleJog() {
  if (programActive && runState != RUN_PAUSED) return;

  bool lHeld = (digitalRead(BTN_LEFT)  == LOW);
  bool rHeld = (digitalRead(BTN_RIGHT) == LOW);

  if (lHeld && !endstopLeft()) {
    stepper.setMaxSpeed(JOG_FAST_SPEED);
    stepper.setSpeed(-JOG_FAST_SPEED);
    stepper.runSpeed();
    motorRunning = true;
  } else if (rHeld && !endstopRight()) {
    stepper.setMaxSpeed(JOG_FAST_SPEED);
    stepper.setSpeed(JOG_FAST_SPEED);
    stepper.runSpeed();
    motorRunning = true;
  } else if (btnLeft.pressed && !endstopLeft()) {
    setRunSpeed(JOG_SLOW_SPEED);
    stepper.move(-JOG_SLOW_STEPS);
    while (stepper.distanceToGo() != 0) stepper.run();
    motorRunning = false;
  } else if (btnRight.pressed && !endstopRight()) {
    setRunSpeed(JOG_SLOW_SPEED);
    stepper.move(JOG_SLOW_STEPS);
    while (stepper.distanceToGo() != 0) stepper.run();
    motorRunning = false;
  } else {
    if (motorRunning && !programActive) {
      stepper.stop();
      motorRunning = false;
    }
  }
}

// ─────────────────────────────────────────────
//  E-STOP  (triggered by encoder long press)
// ─────────────────────────────────────────────
void handleEstop() {
  decelStop();
  motorRunning  = false;
  programActive = false;
  runState      = RUN_IDLE;
  isHomed       = false;   // position unknown after e-stop
  encLongFired  = true;    // prevent re-trigger while still held
  displayOn();
  setBacklight(NEO_ESTOP);
  currentMenu   = MNU_ESTOP_SCREEN;
  menuCursor    = 0;
  menuScroll    = 0;
}

// ─────────────────────────────────────────────
//  PAUSE / RESUME
// ─────────────────────────────────────────────
void pauseProgram() {
  decelStop();
  enableMotor(true);
  motorRunning = false;
  runState     = RUN_PAUSED;
  displayOn();
  setBacklight(NEO_PAUSED);
  currentMenu  = MNU_PAUSE_SCREEN;
  menuCursor   = 0;   // always start on Resume, not wherever cursor was
  menuScroll   = 0;
}

void resumeProgram() {
  motorRunning = true;
  currentMenu  = MNU_RUN_SCREEN;
  setBacklight(NEO_RUNNING);

  // Re-issue the correct moveTo target so AccelStepper
  // resumes motion immediately from current position.
  if (activeSlot == 4) {
    // Full loop — determine direction from runState preserved at pause
    float spd = fullLoop.speedMms * STEPS_PER_MM;
    setRunSpeed(spd);
    if (runState == RUN_FL_BACKWARD)
      stepper.moveTo(0);
    else {
      runState = RUN_FL_FORWARD;
      stepper.moveTo(MAX_TRAVEL_STEPS);
    }

  } else if (activeSlot == 5) {
    float spd = fullLoop.speedMms * STEPS_PER_MM;
    setRunSpeed(spd);
    stepper.moveTo(MAX_TRAVEL_STEPS);
    runState = RUN_MTE_MOVING;

  } else if (activeSlot >= 0 && activeSlot < NUM_SLOTS) {
    ProgramSlot &p = slots[activeSlot];
    float spd = p.speedMms * STEPS_PER_MM;
    setRunSpeed(spd);

    // If paused during a dwell, resume the dwell (dwellStart already set)
    if (runState == RUN_DWELLING) {
      // No moveTo needed — tickProgram will handle the dwell timer
      return;
    }
    // If paused mid-move to a stop point
    if (currentStop < p.stopCount) {
      stepper.moveTo(p.stops[currentStop].position);
      runState = RUN_MOVING_TO_STOP;
    } else {
      // Past all stops — heading to end point or returning
      if (runState == RUN_RETURNING) {
        setRunSpeed(HOME_SPEED);
        stepper.moveTo(p.startPos);
      } else {
        stepper.moveTo(p.endPos);
        runState = RUN_MOVING_TO_END;
      }
    }
  }
}

void cancelProgram() {
  // Decel already done (paused state); return to start then disable
  runState = RUN_RETURNING;
  currentMenu = MNU_RUN_SCREEN;
  motorRunning = true;
  long returnPos = 0;
  if (activeSlot >= 0 && activeSlot < NUM_SLOTS)
    returnPos = slots[activeSlot].startPos;
  setRunSpeed(HOME_SPEED);
  stepper.moveTo(returnPos);
}

// ─────────────────────────────────────────────
//  SLOT SUMMARY
// ─────────────────────────────────────────────
void slotSummary(int idx, char* buf, uint8_t len) {
  if (!slots[idx].valid) { snprintf(buf, len, "Empty"); return; }
  ProgramSlot &s = slots[idx];
  uint16_t totalSec = 0;
  for (uint8_t i = 0; i < s.stopCount; i++) totalSec += s.stops[i].timerSec;
  snprintf(buf, len, "%dsp %ds %s",
    s.stopCount, totalSec, s.loopCount == 0 ? "lp" : "x%d");
}

// ─────────────────────────────────────────────
//  PROGRAM EXECUTION STATE MACHINE
// ─────────────────────────────────────────────
void tickProgram() {
  if (!programActive) return;

  // ── Slot program ─────────────────────────────
  if (activeSlot >= 0 && activeSlot < NUM_SLOTS) {
    ProgramSlot &p = slots[activeSlot];
    float spd = p.speedMms * STEPS_PER_MM;

    switch (runState) {

      case RUN_MOVING_TO_START:
        if (!stepper.isRunning()) {
          // Arrived at start — begin first stop or end
          currentStop = 0;
          currentLoop = 0;
          if (p.stopCount > 0) {
            setRunSpeed(spd);
            stepper.moveTo(p.stops[0].position);
            runState = RUN_MOVING_TO_STOP;
          } else {
            setRunSpeed(spd);
            stepper.moveTo(p.endPos);
            runState = RUN_MOVING_TO_END;
          }
        }
        break;

      case RUN_MOVING_TO_STOP:
        if (!stepper.isRunning()) {
          // Arrived at stop — begin dwell
          dwellStart    = millis();
          dwellDuration = (long)p.stops[currentStop].timerSec * 1000UL;
          runState      = RUN_DWELLING;
        }
        break;

      case RUN_DWELLING:
        if (millis() - dwellStart >= (unsigned long)dwellDuration) {
          currentStop++;
          if (currentStop < p.stopCount) {
            // More stops
            setRunSpeed(spd);
            stepper.moveTo(p.stops[currentStop].position);
            runState = RUN_MOVING_TO_STOP;
          } else {
            // All stops done — move to end
            setRunSpeed(spd);
            stepper.moveTo(p.endPos);
            runState = RUN_MOVING_TO_END;
          }
        }
        break;

      case RUN_MOVING_TO_END:
        if (!stepper.isRunning()) {
          currentLoop++;
          bool loopForever = (p.loopCount == 0);
          bool loopDone    = (!loopForever && currentLoop >= p.loopCount);

          if (!loopDone) {
            // Start next loop — return to start
            currentStop = 0;
            setRunSpeed(spd);
            stepper.moveTo(p.startPos);
            runState = RUN_MOVING_TO_START;
          } else {
            // All loops done — return to start then finish
            setRunSpeed(HOME_SPEED);
            stepper.moveTo(p.startPos);
            runState = RUN_RETURNING;
          }
        }
        break;

      case RUN_RETURNING:
        if (!stepper.isRunning()) {
          runState = RUN_DONE;
        }
        break;

      case RUN_DONE:
        stopMotorHard();
        break;

      default: break;
    }

    if (motorRunning && runState != RUN_PAUSED && runState != RUN_DONE)
      stepper.run();
  }

  // ── Full Loop ────────────────────────────────
  else if (activeSlot == 4) {
    float spd = fullLoop.speedMms * STEPS_PER_MM;

    switch (runState) {
      case RUN_FL_FORWARD:
        if (!stepper.isRunning()) {
          // Reached right end — go back
          setRunSpeed(spd);
          stepper.moveTo(0);
          runState = RUN_FL_BACKWARD;
        }
        break;

      case RUN_FL_BACKWARD:
        if (!stepper.isRunning()) {
          currentLoop++;
          bool loopForever = (fullLoop.loopCount == 0);
          bool loopDone    = (!loopForever && currentLoop >= fullLoop.loopCount);
          if (!loopDone) {
            setRunSpeed(spd);
            stepper.moveTo(MAX_TRAVEL_STEPS);
            runState = RUN_FL_FORWARD;
          } else {
            runState = RUN_FL_DONE;
          }
        }
        break;

      case RUN_FL_DONE:
        stopMotorHard();
        break;

      default: break;
    }

    if (motorRunning && runState != RUN_PAUSED)
      stepper.run();
  }

  // ── Move to Endpoint ─────────────────────────
  else if (activeSlot == 5) {
    switch (runState) {
      case RUN_MTE_MOVING:
        if (!stepper.isRunning()) {
          stopMotorHard();
        }
        break;
      default: break;
    }
    if (motorRunning) stepper.run();
  }

  // ── Endstop safety — reverse if loops remain ─
  if (motorRunning) {
    bool hitLeft  = endstopLeft();   // already checks speed direction internally
    bool hitRight = endstopRight();

    if (hitLeft || hitRight) {
      decelStop();

      // Determine if more loops remain
      bool loopForever = false;
      bool hasLoops    = false;

      if (activeSlot >= 0 && activeSlot < NUM_SLOTS) {
        loopForever = (slots[activeSlot].loopCount == 0);
        hasLoops    = loopForever || (currentLoop + 1 < slots[activeSlot].loopCount);
      } else if (activeSlot == 4) {
        loopForever = (fullLoop.loopCount == 0);
        hasLoops    = loopForever || (currentLoop + 1 < fullLoop.loopCount);
      }
      // Move to endpoint (slot 5) never reverses — stop on endstop

      if (hasLoops && activeSlot != 5) {
        // Reverse direction — move to opposite end
        float spd = (activeSlot == 4)
          ? fullLoop.speedMms * STEPS_PER_MM
          : slots[activeSlot].speedMms * STEPS_PER_MM;
        setRunSpeed(spd);
        if (hitLeft) {
          // Was going left, hit left endstop — reverse right
          stepper.moveTo(MAX_TRAVEL_STEPS);
          runState = (activeSlot == 4) ? RUN_FL_FORWARD : RUN_MOVING_TO_END;
        } else {
          // Was going right, hit right endstop — reverse left
          long returnPos = (activeSlot >= 0 && activeSlot < NUM_SLOTS)
            ? slots[activeSlot].startPos : 0;
          stepper.moveTo(returnPos);
          currentLoop++;
          runState = (activeSlot == 4) ? RUN_FL_BACKWARD : RUN_MOVING_TO_START;
        }
      } else {
        // No loops remain — end program
        stopMotorHard();
      }
    }
  }
}

// ─────────────────────────────────────────────
//  PROGRAM LAUNCH
//  Call this to attempt to start the selected
//  program. If not homed, opens the home prompt
//  instead and returns false.
//  launchProgram() does the actual motor start
//  and is called after homing is confirmed.
// ─────────────────────────────────────────────
bool startProgram() {
  // Guard: no program selected
  if (activeSlot < 0) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(4, 28, "No program selected");
      u8g2.drawStr(4, 44, "Use Select Program");
    } while (u8g2.nextPage());
    delay(1500);
    return false;
  }

  // Guard: slot selected but empty
  if (activeSlot < NUM_SLOTS && !slots[activeSlot].valid) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(4, 28, "Slot is empty.");
      u8g2.drawStr(4, 44, "Build a program first");
    } while (u8g2.nextPage());
    delay(1500);
    return false;
  }

  // Must be homed before running any program
  if (!isHomed) {
    openMenu(MNU_HOME_REQUIRED);
    return false;
  }

  return launchProgram();
}

// Actually starts the motor — called after home confirmed
bool launchProgram() {
  if (activeSlot >= 0 && activeSlot < NUM_SLOTS) {
    ProgramSlot &p = slots[activeSlot];
    float spd = p.speedMms * STEPS_PER_MM;
    setRunSpeed(spd);
    stepper.moveTo(p.startPos);
    currentLoop   = 0;
    currentStop   = 0;
    runState      = RUN_MOVING_TO_START;
    motorRunning  = true;
    programActive = true;
    displayOff();
    setBacklight(NEO_RUNNING);

  } else if (activeSlot == 4) {
    float spd = fullLoop.speedMms * STEPS_PER_MM;
    setRunSpeed(spd);
    stepper.moveTo(MAX_TRAVEL_STEPS);
    currentLoop   = 0;
    runState      = RUN_FL_FORWARD;
    motorRunning  = true;
    programActive = true;
    displayOff();
    setBacklight(NEO_RUNNING);

  } else if (activeSlot == 5) {
    float spd = fullLoop.speedMms * STEPS_PER_MM;
    setRunSpeed(spd);
    stepper.moveTo(MAX_TRAVEL_STEPS);
    runState      = RUN_MTE_MOVING;
    motorRunning  = true;
    programActive = true;
    displayOff();
    setBacklight(NEO_RUNNING);
  }
  return true;
}

// ─────────────────────────────────────────────
//  DISPLAY
// ─────────────────────────────────────────────
void drawMenu(const char** items, uint8_t count, const char* title) {
  const uint8_t lineH = 13, startY = 22;
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, title);
    u8g2.drawHLine(0, 12, 128);
    for (uint8_t i = 0; i < VISIBLE_ROWS; i++) {
      uint8_t idx = menuScroll + i;
      if (idx >= count) break;
      uint8_t y = startY + i * lineH;
      if (idx == (uint8_t)menuCursor) {
        u8g2.drawBox(0, y-10, 128, lineH); u8g2.setDrawColor(0);
      }
      u8g2.drawStr(2, y, items[idx]);
      u8g2.setDrawColor(1);
    }
    if (count > VISIBLE_ROWS) {
      uint8_t bH = max(4,(int)(52/count));
      u8g2.drawBox(126, menuScroll*52/count+12, 2, bH);
    }
  } while (u8g2.nextPage());
}

void drawDynamicMenu(const char* title, uint8_t count,
                     void(*lbl)(uint8_t,char*,uint8_t)) {
  const uint8_t lineH = 13, startY = 22;
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, title);
    u8g2.drawHLine(0, 12, 128);
    for (uint8_t i = 0; i < VISIBLE_ROWS; i++) {
      uint8_t idx = menuScroll + i;
      if (idx >= count) break;
      uint8_t y = startY + i * lineH;
      if (idx == (uint8_t)menuCursor) {
        u8g2.drawBox(0, y-10, 128, lineH); u8g2.setDrawColor(0);
      }
      char buf[22]; lbl(idx, buf, sizeof(buf));
      u8g2.drawStr(2, y, buf);
      u8g2.setDrawColor(1);
    }
  } while (u8g2.nextPage());
}

// Confirm screens show Yes/No as scrollable items
void drawConfirm(const char* l1, const char* l2 = nullptr) {
  const char* opts[] = { "Yes — confirm", "No  — cancel" };
  const uint8_t lineH = 13, startY = 36;
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 12, l1);
    if (l2) u8g2.drawStr(0, 24, l2);
    u8g2.drawHLine(0, 27, 128);
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t y = startY + i * lineH;
      if (i == (uint8_t)menuCursor) {
        u8g2.drawBox(0, y - 10, 128, lineH); u8g2.setDrawColor(0);
      }
      u8g2.drawStr(4, y, opts[i]);
      u8g2.setDrawColor(1);
    }
  } while (u8g2.nextPage());
}

void drawValueEditor() {
  char vb[12]; ltoa(ved.value, vb, 10);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, ved.label);
    u8g2.drawHLine(0, 12, 128);
    u8g2.setFont(u8g2_font_10x20_tf);
    u8g2.drawStr((128 - u8g2.getStrWidth(vb)) / 2, 45, vb);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 62, "Turn=adjust");
    u8g2.drawStr(82, 62, "ENC=OK");
  } while (u8g2.nextPage());
}

const char* runStateLabel() {
  switch(runState) {
    case RUN_MOVING_TO_START: return "To Start...";
    case RUN_MOVING_TO_STOP:  return "To Stop...";
    case RUN_DWELLING:        return "Dwelling...";
    case RUN_MOVING_TO_END:   return "To End...";
    case RUN_RETURNING:       return "Returning...";
    case RUN_FL_FORWARD:      return "Forward...";
    case RUN_FL_BACKWARD:     return "Returning...";
    case RUN_MTE_MOVING:      return "Moving...";
    default:                  return "Running";
  }
}

void drawRunScreen() {
  char prog[14];
  if      (activeSlot == 4) strncpy(prog, "Full Loop",   sizeof(prog));
  else if (activeSlot == 5) strncpy(prog, "To Endpoint", sizeof(prog));
  else if (activeSlot >= 0) snprintf(prog, sizeof(prog), "Slot %d", activeSlot+1);
  else                      strncpy(prog, "None",        sizeof(prog));

  float posMm = (float)stepper.currentPosition() / STEPS_PER_MM;
  char posBuf[10]; dtostrf(posMm, 5, 1, posBuf);
  char loopBuf[10];
  if (activeSlot >= 0 && activeSlot < NUM_SLOTS)
    snprintf(loopBuf, sizeof(loopBuf), "Lp:%d", currentLoop+1);
  else
    snprintf(loopBuf, sizeof(loopBuf), "Lp:%d", currentLoop+1);

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, runStateLabel());
    u8g2.drawHLine(0, 12, 128);
    u8g2.drawStr(0,  26, "Prog:"); u8g2.drawStr(36, 26, prog);
    u8g2.drawStr(0,  40, "Pos: "); u8g2.drawStr(36, 40, posBuf); u8g2.drawStr(90,40,"mm");
    u8g2.drawStr(0,  54, loopBuf);
    u8g2.drawStr(60, 54, "JOG=Pause");
  } while (u8g2.nextPage());
}

void drawPauseScreen() {
  float posMm = (float)stepper.currentPosition() / STEPS_PER_MM;
  char posBuf[10]; dtostrf(posMm, 5, 1, posBuf);
  const char* opts[] = { "Resume", "Cancel Program" };
  const uint8_t lineH = 13, startY = 40;
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_9x18B_tf);
    u8g2.drawStr(28, 18, "PAUSED");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 30, "Pos:"); u8g2.drawStr(28, 30, posBuf); u8g2.drawStr(76, 30, "mm");
    u8g2.drawHLine(0, 32, 128);
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t y = startY + i * lineH;
      if (i == (uint8_t)menuCursor) {
        u8g2.drawBox(0, y - 10, 128, lineH); u8g2.setDrawColor(0);
      }
      u8g2.drawStr(4, y, opts[i]);
      u8g2.setDrawColor(1);
    }
  } while (u8g2.nextPage());
}

void drawEstopScreen() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_9x18B_tf);
    u8g2.drawStr(14, 22, "! E-STOP !");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(4, 38, "Motors disabled");
    u8g2.drawStr(4, 52, "CONF = Main Menu");
  } while (u8g2.nextPage());
}

void drawHomeRequired() {
  // Yes/No confirm — cursor 0=Yes 1=No
  const char* opts[] = { "Yes — home now", "No  — cancel" };
  const uint8_t lineH = 13, startY = 38;
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Home required");
    u8g2.drawHLine(0, 12, 128);
    u8g2.drawStr(0, 26, "Motor not homed.");
    u8g2.drawStr(0, 36, "Home before running?");  // fits on line with small font
    u8g2.drawHLine(0, 38, 128);
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t y = startY + 2 + i * lineH;
      if (i == (uint8_t)menuCursor) {
        u8g2.drawBox(0, y - 10, 128, lineH);
        u8g2.setDrawColor(0);
      }
      u8g2.drawStr(4, y, opts[i]);
      u8g2.setDrawColor(1);
    }
  } while (u8g2.nextPage());
}

// ─────────────────────────────────────────────
//  DYNAMIC MENU LABEL CALLBACKS
// ─────────────────────────────────────────────
void labelSelectProg(uint8_t idx, char* buf, uint8_t len) {
  if      (idx == 0) strncpy(buf, "Full Loop",        len);
  else if (idx == 1) strncpy(buf, "Move to Endpoint", len);
  else if (idx == 6) strncpy(buf, "< Back",           len);
  else { char s[16]; slotSummary(idx-2,s,sizeof(s)); snprintf(buf,len,"S%d:%s",idx-1,s); }
}

void labelClearProg(uint8_t idx, char* buf, uint8_t len) {
  if      (idx == 4) strncpy(buf, "Clear All", len);
  else if (idx == 5) strncpy(buf, "< Back",    len);
  else { char s[16]; slotSummary(idx,s,sizeof(s)); snprintf(buf,len,"S%d:%s",idx+1,s); }
}

// ─────────────────────────────────────────────
//  MENU NAVIGATION
// ─────────────────────────────────────────────
void openMenu(MenuID id, int cur=0) {
  currentMenu=id; menuCursor=cur; menuScroll=0;
}

void scrollMenu(int d, uint8_t n) {
  menuCursor += d;
  if (menuCursor < 0)          menuCursor = 0;
  if (menuCursor >= (int)n)    menuCursor = n-1;
  if (menuCursor < menuScroll) menuScroll = menuCursor;
  if (menuCursor >= menuScroll+VISIBLE_ROWS) menuScroll = menuCursor-VISIBLE_ROWS+1;
}

void openValueEditor(const char* lbl, long cur,
                     long mn, long mx, long stp,
                     void* tgt, MenuID done) {
  ved = {lbl, cur, mn, mx, stp, done, tgt};
  openMenu(MNU_EDIT_VALUE);
}

// ─────────────────────────────────────────────
//  ACTIONS
// ─────────────────────────────────────────────
void doHomeAll() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(24, 32, "Homing...");
  } while (u8g2.nextPage());
  homeMotor();
}

// ─────────────────────────────────────────────
//  EEPROM LAYOUT
//  Byte 0        : magic number (0xAB = valid data)
//  Bytes 1–2     : fullLoop.speedMms  (uint16_t)
//  Bytes 3–4     : fullLoop.loopCount (uint16_t)
//  Bytes 5+      : ProgramSlot[0..3]
// ─────────────────────────────────────────────
#define EEPROM_MAGIC_ADDR   0
#define EEPROM_MAGIC_VALUE  0xAB
#define EEPROM_FL_SPEED     1
#define EEPROM_FL_LOOPS     3
#define EEPROM_SLOTS_START  5

int slotAddr(int idx) {
  return EEPROM_SLOTS_START + idx * (int)sizeof(ProgramSlot);
}

void eepromSaveSlot(int idx) {
  if (idx < 0 || idx >= NUM_SLOTS) return;
  EEPROM.put(slotAddr(idx), slots[idx]);
}

void eepromSaveFullLoop() {
  EEPROM.put(EEPROM_FL_SPEED, fullLoop.speedMms);
  EEPROM.put(EEPROM_FL_LOOPS, fullLoop.loopCount);
}

void eepromLoad() {
  uint8_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic != EEPROM_MAGIC_VALUE) {
    // First boot — write defaults
    memset(slots, 0, sizeof(slots));
    fullLoop.speedMms  = DEFAULT_SPEED_MMS;
    fullLoop.loopCount = DEFAULT_LOOP_COUNT;
    EEPROM.put(EEPROM_MAGIC_ADDR, (uint8_t)EEPROM_MAGIC_VALUE);
    eepromSaveFullLoop();
    for (int i = 0; i < NUM_SLOTS; i++) eepromSaveSlot(i);
    return;
  }
  EEPROM.get(EEPROM_FL_SPEED, fullLoop.speedMms);
  EEPROM.get(EEPROM_FL_LOOPS, fullLoop.loopCount);
  if (fullLoop.speedMms < 1 || fullLoop.speedMms > 100)
    fullLoop.speedMms = DEFAULT_SPEED_MMS;
  for (int i = 0; i < NUM_SLOTS; i++) {
    EEPROM.get(slotAddr(i), slots[i]);
    if (slots[i].stopCount > MAX_STOPS)
      memset(&slots[i], 0, sizeof(ProgramSlot));
  }
}

void doSaveProgram() {
  if (wipSlot < 0) return;
  if (!wipHasStart || !wipHasEnd) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0,16,"Cannot save:");
      if (!wipHasStart) u8g2.drawStr(0,32,"No start set");
      if (!wipHasEnd)   u8g2.drawStr(0,46,"No end set");
    } while (u8g2.nextPage());
    delay(1600); return;
  }
  wip.valid      = true;
  slots[wipSlot] = wip;
  eepromSaveSlot(wipSlot);
}

void doClearSlot(int idx) {
  if (idx < 0) {
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
      slots[i].valid = false;
      eepromSaveSlot(i);
    }
  } else {
    slots[idx].valid = false;
    eepromSaveSlot(idx);
  }
}

// ─────────────────────────────────────────────
//  MENU INPUT HANDLER
// ─────────────────────────────────────────────
void handleMenuInput(int delta, bool sel) {
  switch (currentMenu) {

    case MNU_ROOT:
      scrollMenu(delta, rootCount);
      if (sel) switch (menuCursor) {
        case 0: doHomeAll(); break;
        case 1:
          if (programActive) { decelStop(); stopMotorHard(); openMenu(MNU_ROOT); }
          else if (startProgram()) openMenu(MNU_RUN_SCREEN);
          break;
        case 2: openMenu(MNU_SELECT_PROGRAM);     break;
        case 3: openMenu(MNU_CLEAR_PROGRAM);      break;
        case 4: openMenu(MNU_FULL_LOOP_SETTINGS); break;
        case 5: openMenu(MNU_MOTOR_CONTROL);      break;
        case 6: openMenu(MNU_PROG_MOVEMENT);      break;
      }
      break;

    case MNU_SELECT_PROGRAM:
      // 0=Full Loop, 1=Move to Endpoint, 2-5=Slots, 6=< Back
      scrollMenu(delta, 7);
      if (sel) {
        if (menuCursor == 6) { openMenu(MNU_ROOT); break; }
        activeSlot = (menuCursor <= 1) ? menuCursor + 4 : menuCursor - 2;
        openMenu(MNU_ROOT);
      }
      break;

    case MNU_CLEAR_PROGRAM:
      // 0-3=Slots, 4=Clear All, 5=< Back
      scrollMenu(delta, 6);
      if (sel) {
        if (menuCursor == 5) { openMenu(MNU_ROOT); break; }
        clearSlotTarget = (menuCursor == 4) ? -1 : menuCursor;
        openMenu(menuCursor == 4 ? MNU_CONFIRM_CLEAR_ALL : MNU_CONFIRM_CLEAR);
      }
      break;

    case MNU_CONFIRM_CLEAR:
      // 0=Yes  1=No
      scrollMenu(delta, 2);
      if (sel) {
        if (menuCursor == 0) doClearSlot(clearSlotTarget);
        openMenu(MNU_CLEAR_PROGRAM);
      }
      break;

    case MNU_CONFIRM_CLEAR_ALL:
      scrollMenu(delta, 2);
      if (sel) {
        if (menuCursor == 0) doClearSlot(-1);
        openMenu(MNU_CLEAR_PROGRAM);
      }
      break;

    case MNU_FULL_LOOP_SETTINGS:
      // 0=Speed, 1=Loop Count, 2=Reset, 3=< Back
      scrollMenu(delta, fullLoopCount);
      if (sel) switch (menuCursor) {
        case 0: openValueEditor("FL Speed (mm/s)", fullLoop.speedMms, 1, 100, 1, &fullLoop.speedMms, MNU_FULL_LOOP_SETTINGS); break;
        case 1: openValueEditor("Loop Count(0=cont)", fullLoop.loopCount, 0, 9999, 1, &fullLoop.loopCount, MNU_FULL_LOOP_SETTINGS); break;
        case 2: fullLoop.speedMms = DEFAULT_SPEED_MMS; fullLoop.loopCount = DEFAULT_LOOP_COUNT; eepromSaveFullLoop(); break;
        case 3: openMenu(MNU_ROOT); break;
      }
      break;

    case MNU_MOTOR_CONTROL:
      // 0=Home, 1=Disable, 2=< Back
      scrollMenu(delta, motorCtrlCount);
      if (sel) switch (menuCursor) {
        case 0: doHomeAll(); break;
        case 1: openMenu(MNU_CONFIRM_DISABLE); break;
        case 2: openMenu(MNU_ROOT); break;
      }
      break;

    case MNU_CONFIRM_DISABLE:
      // 0=Yes  1=No
      scrollMenu(delta, 2);
      if (sel) {
        if (menuCursor == 0) { decelStop(); motorRunning = false; }
        openMenu(MNU_MOTOR_CONTROL);
      }
      break;

    case MNU_PROG_MOVEMENT:
      // 0=Slot, 1=Home, 2=Speed, 3=Start, 4=Stop Pt, 5=End, 6=Loop, 7=Save, 8=< Back
      scrollMenu(delta, progMovCount);
      if (sel) switch (menuCursor) {
        case 0: openMenu(MNU_PROG_SELECT_SLOT); break;
        case 1: doHomeAll(); break;
        case 2:
          openValueEditor("Prog Speed (mm/s)",
            wip.speedMms ? wip.speedMms : DEFAULT_SPEED_MMS,
            1, 100, 1, &wip.speedMms, MNU_PROG_MOVEMENT);
          break;
        case 3: wip.startPos = stepper.currentPosition(); wipHasStart = true; break;
        case 4:
          if (wip.stopCount < MAX_STOPS) {
            wip.stops[wip.stopCount].position = stepper.currentPosition();
            wip.stops[wip.stopCount].timerSec = 0;
            openValueEditor("Stop Timer (sec)", 0, 0, 3600, 1,
              &wip.stops[wip.stopCount].timerSec, MNU_ADD_STOP_TIMER);
          }
          break;
        case 5: wip.endPos = stepper.currentPosition(); wipHasEnd = true; break;
        case 6:
          openValueEditor("Loop Count(0=cont)", wip.loopCount, 0, 9999, 1,
            &wip.loopCount, MNU_PROG_MOVEMENT);
          break;
        case 7: doSaveProgram(); openMenu(MNU_ROOT); break;
        case 8: openMenu(MNU_ROOT); break;
      }
      break;

    case MNU_PROG_SELECT_SLOT:
      // 0-3=Slots, 4=< Back
      scrollMenu(delta, progSlotCount);
      if (sel) {
        if (menuCursor == 4) { openMenu(MNU_PROG_MOVEMENT); break; }
        wipSlot = menuCursor;
        if (slots[wipSlot].valid) { wip = slots[wipSlot]; wipHasStart = wipHasEnd = true; }
        else { memset(&wip, 0, sizeof(wip)); wipHasStart = wipHasEnd = false; }
        openMenu(MNU_PROG_MOVEMENT);
      }
      break;

    case MNU_ADD_STOP_TIMER:
      wip.stopCount++;
      openMenu(MNU_PROG_MOVEMENT);
      break;

    case MNU_EDIT_VALUE:
      if (delta) {
        ved.value += delta * ved.step;
        if (ved.value < ved.minVal) ved.value = ved.minVal;
        if (ved.value > ved.maxVal) ved.value = ved.maxVal;
      }
      if (sel) {
        if (ved.maxVal <= 65535 && ved.minVal >= 0)
          *((uint16_t*)ved.target) = (uint16_t)ved.value;
        else
          *((long*)ved.target) = ved.value;
        // Persist full loop changes immediately
        if (ved.onDone == MNU_FULL_LOOP_SETTINGS) eepromSaveFullLoop();
        openMenu(ved.onDone == MNU_ADD_STOP_TIMER ? MNU_ADD_STOP_TIMER : ved.onDone);
      }
      break;

    case MNU_RUN_SCREEN:
      // Nothing selectable — PLAY/PAUSE button handles pause
      break;

    case MNU_PAUSE_SCREEN:
      // 0=Resume  1=Cancel
      scrollMenu(delta, 2);
      if (sel) {
        if (menuCursor == 0) { setBacklight(NEO_RUNNING); resumeProgram(); }
        else                 { cancelProgram(); }
      }
      break;

    case MNU_ESTOP_SCREEN:
      // 0=Return to Main Menu
      if (sel) openMenu(MNU_ROOT);
      break;

    case MNU_HOME_REQUIRED:
      // 0=Yes — home then launch,  1=No — back to root
      scrollMenu(delta, 2);
      if (sel) {
        if (menuCursor == 0) {
          doHomeAll();
          if (isHomed) {
            launchProgram();
            openMenu(MNU_RUN_SCREEN);
          } else {
            openMenu(MNU_ROOT);   // homing aborted (e-stop during home)
          }
        } else {
          openMenu(MNU_ROOT);
        }
      }
      break;

    default: openMenu(MNU_ROOT); break;
  }
}

// ─────────────────────────────────────────────
//  ROOT MENU — custom draw with active program
//  status line at the bottom of the screen.
//  Replaces the last visible row with the
//  currently selected program so selection
//  is always confirmed visually.
// ─────────────────────────────────────────────
void buildActiveProgramLabel(char* buf, uint8_t len) {
  if      (activeSlot == 4) snprintf(buf, len, "Pgm: Full Loop");
  else if (activeSlot == 5) snprintf(buf, len, "Pgm: To Endpoint");
  else if (activeSlot >= 0 && activeSlot < NUM_SLOTS) {
    if (slots[activeSlot].valid)
      snprintf(buf, len, "Pgm: Slot %d", activeSlot + 1);
    else
      snprintf(buf, len, "Pgm: Slot %d (empty)", activeSlot + 1);
  } else {
    snprintf(buf, len, "Pgm: none selected");
  }
}

void drawRootMenu() {
  // Show 3 menu items (not 4) to leave room for the status line
  const uint8_t lineH  = 13;
  const uint8_t startY = 22;
  const uint8_t visibleItems = 3;

  // Build status label
  char statusBuf[22];
  buildActiveProgramLabel(statusBuf, sizeof(statusBuf));

  u8g2.firstPage();
  do {
    // Title
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Main Menu");
    u8g2.drawHLine(0, 12, 128);

    // Menu items (3 visible rows)
    for (uint8_t i = 0; i < visibleItems; i++) {
      uint8_t idx = menuScroll + i;
      if (idx >= rootCount) break;
      uint8_t y = startY + i * lineH;
      if (idx == (uint8_t)menuCursor) {
        u8g2.drawBox(0, y - 10, 128, lineH);
        u8g2.setDrawColor(0);
      }
      u8g2.drawStr(2, y, rootItems[idx]);
      u8g2.setDrawColor(1);
    }

    // Scroll indicator
    if (rootCount > visibleItems) {
      uint8_t bH = max(4, (int)(38 / rootCount));
      u8g2.drawBox(126, menuScroll * 38 / rootCount + 12, 2, bH);
    }

    // Status bar — separator then active program
    u8g2.drawHLine(0, 53, 128);
    u8g2.setFont(u8g2_font_5x7_tf);   // smaller font fits longer labels
    u8g2.drawStr(2, 63, statusBuf);
  } while (u8g2.nextPage());
}

// ─────────────────────────────────────────────
//  DRAW DISPATCH
// ─────────────────────────────────────────────
void drawCurrentMenu() {
  switch (currentMenu) {
    case MNU_ROOT:               drawRootMenu(); break;
    case MNU_SELECT_PROGRAM:     drawDynamicMenu("Select Program",7,labelSelectProg); break;
    case MNU_CLEAR_PROGRAM:      drawDynamicMenu("Clear Program",6,labelClearProg); break;
    case MNU_FULL_LOOP_SETTINGS: drawMenu(fullLoopItems,fullLoopCount,"Full Loop"); break;
    case MNU_MOTOR_CONTROL:      drawMenu(motorCtrlItems,motorCtrlCount,"Motor Control"); break;
    case MNU_PROG_MOVEMENT:      drawMenu(progMovItems,progMovCount,"Program Movement"); break;
    case MNU_PROG_SELECT_SLOT:   drawMenu(progSlotItems,progSlotCount,"Select Slot"); break;
    case MNU_CONFIRM_CLEAR: {
      char l1[20],l2[20];
      snprintf(l1,sizeof(l1),"Clear Slot %d?",clearSlotTarget+1);
      slotSummary(clearSlotTarget,l2,sizeof(l2));
      drawConfirm(l1,l2); break;
    }
    case MNU_CONFIRM_CLEAR_ALL:  drawConfirm("Clear ALL slots?","Cannot be undone"); break;
    case MNU_CONFIRM_DISABLE:    drawConfirm("Disable motors?","Carriage may move"); break;
    case MNU_EDIT_VALUE:         drawValueEditor(); break;
    case MNU_ADD_STOP_TIMER:     break;
    case MNU_RUN_SCREEN:         drawRunScreen(); break;
    case MNU_PAUSE_SCREEN:       drawPauseScreen(); break;
    case MNU_ESTOP_SCREEN:       drawEstopScreen(); break;
    case MNU_HOME_REQUIRED:      drawHomeRequired(); break;
  }
}

// ─────────────────────────────────────────────
//  SPLASH SCREEN
//  Drawn with U8g2 primitives — no bitmap needed
//
//  Layout (128 × 64):
//
//   "Camera Slider"  — large font, centred, y=18
//   "v1.0"           — small font, centred,  y=30
//   ─────────────────────────────────────────   y=38  (rail top edge)
//   ═════════════════════════════════════════   y=40  (rail body, 4px tall)
//   ─────────────────────────────────────────   y=44  (rail bottom edge)
//   [■■■]  carriage block (12×10) slides L→R   y=34
//
// ─────────────────────────────────────────────
#define FIRMWARE_VER   "v1.2-12864"
#define RAIL_Y         40    // top of rail bar
#define RAIL_H         4     // rail height in pixels
#define CARRIAGE_W     14    // carriage block width
#define CARRIAGE_H     10    // carriage block height
#define CARRIAGE_Y     (RAIL_Y - CARRIAGE_H + 2)  // sits on the rail
#define RAIL_LEFT      4
#define RAIL_RIGHT     (128 - 4)
#define SPLASH_MS      3000  // total display time

// Draw one frame of the splash at carriage x position
void drawSplashFrame(int carriageX) {
  u8g2.firstPage();
  do {
    // ── Title ──
    u8g2.setFont(u8g2_font_9x18B_tf);
    uint8_t tw = u8g2.getStrWidth("Camera Slider");
    u8g2.drawStr((128 - tw) / 2, 18, "Camera Slider");

    // ── Version ──
    u8g2.setFont(u8g2_font_6x10_tf);
    uint8_t vw = u8g2.getStrWidth(FIRMWARE_VER);
    u8g2.drawStr((128 - vw) / 2, 30, FIRMWARE_VER);

    // ── Rail ──
    // End caps (circles suggest round rail ends)
    u8g2.drawCircle(RAIL_LEFT,  RAIL_Y + RAIL_H/2, RAIL_H/2 + 1, U8G2_DRAW_ALL);
    u8g2.drawCircle(RAIL_RIGHT, RAIL_Y + RAIL_H/2, RAIL_H/2 + 1, U8G2_DRAW_ALL);
    // Rail body
    u8g2.drawBox(RAIL_LEFT, RAIL_Y, RAIL_RIGHT - RAIL_LEFT, RAIL_H);
    // Highlight line on top of rail
    u8g2.setDrawColor(0);
    u8g2.drawHLine(RAIL_LEFT, RAIL_Y, RAIL_RIGHT - RAIL_LEFT);
    u8g2.setDrawColor(1);

    // ── Carriage block ──
    // Outer box
    u8g2.drawRBox(carriageX, CARRIAGE_Y, CARRIAGE_W, CARRIAGE_H, 2);
    // Inner highlight (gives a 3-D look)
    u8g2.setDrawColor(0);
    u8g2.drawHLine(carriageX + 2, CARRIAGE_Y + 2, CARRIAGE_W - 4);
    u8g2.setDrawColor(1);

    // ── "Ready" text fades in on last quarter of animation ──
    // (shown statically — no true fade on OLED, just appears)
  } while (u8g2.nextPage());
}

void drawSplashScreen() {
  unsigned long start  = millis();
  unsigned long dur    = SPLASH_MS;

  // Carriage travels from left to right across the rail
  int travelPx = (RAIL_RIGHT - RAIL_LEFT) - CARRIAGE_W;

  while (millis() - start < dur) {
    unsigned long elapsed = millis() - start;
    // Ease in-out: use sine-like feel via a simple smooth step
    float t = (float)elapsed / (float)dur;            // 0.0 → 1.0
    float ease = t * t * (3.0f - 2.0f * t);          // smoothstep
    int cx = RAIL_LEFT + (int)(ease * travelPx);
    drawSplashFrame(cx);
  }

  // Hold final frame briefly so it doesn't snap away
  drawSplashFrame(RAIL_LEFT + travelPx);
  delay(400);
}

// ─────────────────────────────────────────────
//  Called each loop when a new code is received.
//  Mirrors physical button behaviour exactly.
//  Blocked during active run (same as jog buttons)
//  — except Start/Pause which is always active.
// ─────────────────────────────────────────────

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Motor
  pinMode(EN_PIN,    OUTPUT); enableMotor(false);

  // Encoder & buttons
  pinMode(ENC_CLK,   INPUT_PULLUP);
  pinMode(ENC_DT,    INPUT_PULLUP);
  pinMode(ENC_SW,    INPUT_PULLUP);
  pinMode(BTN_BACK,  INPUT_PULLUP);
  pinMode(BTN_CONF,  INPUT_PULLUP);
  pinMode(BTN_LEFT,      INPUT_PULLUP);
  pinMode(BTN_RIGHT,     INPUT_PULLUP);
  pinMode(BTN_PLAYPAUSE, INPUT_PULLUP);

  // Endstops — both NC switches wired in parallel to single pin
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encoderISR, CHANGE);


  // NeoPixel backlight
  neo.begin();
  neo.setBrightness(80);
  setBacklight(NEO_IDLE);

  // Stepper
  stepper.setMaxSpeed(JOG_FAST_SPEED);
  stepper.setAcceleration(JOG_FAST_SPEED * 2);
  eepromLoad();   // load saved slots and full loop settings

  // Display
  delay(100);
  u8g2.begin();
  u8g2.setContrast(200);
  drawSplashScreen();

  openMenu(MNU_ROOT);
}

// ─────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────
void loop() {
  // ── Encoder delta ──────────────────────────
  int delta = 0;
  noInterrupts(); delta = encDelta; encDelta = 0; interrupts();

  // ── Buttons ────────────────────────────────
  updateAllButtons();

  // ── Encoder long press = E-Stop ────────────
  if (btnEnc.held && !encLongFired) {
    handleEstop();
    drawCurrentMenu();
    return;
  }
  // Reset long-press flag when encoder released
  if (!btnEnc.held && digitalRead(ENC_SW) == HIGH) encLongFired = false;

  }

  // ── PLAY/PAUSE button ──────────────────────
  //    In Program Movement: short press = add stop point
  //    Elsewhere:
  //      Short press = start / pause / resume
  //      Long press  = cancel program
  if (currentMenu == MNU_PROG_MOVEMENT) {
    if (btnPlayPause.pressed) {
      // Add stop point at current carriage position
      if (wip.stopCount < MAX_STOPS) {
        wip.stops[wip.stopCount].position = stepper.currentPosition();
        wip.stops[wip.stopCount].timerSec = 0;
        openValueEditor("Stop Timer (sec)", 0, 0, 3600, 1,
          &wip.stops[wip.stopCount].timerSec, MNU_ADD_STOP_TIMER);
      }
    }
  } else if (btnPlayPause.held && programActive && runState == RUN_PAUSED) {
    cancelProgram();
  } else if (btnPlayPause.pressed) {
    if (programActive && runState != RUN_PAUSED) {
      pauseProgram();
    } else if (runState == RUN_PAUSED) {
      setBacklight(NEO_RUNNING);
      resumeProgram();
    } else {
      if (startProgram()) openMenu(MNU_RUN_SCREEN);
    }
  }

  // ── Jog buttons ────────────────────────────
  // Jog is always available except during active run
  handleJog();

  // ── Encoder short press = select ───────────
  bool sel = btnEnc.pressed;
  if (delta || sel) handleMenuInput(delta, sel);

  // ── Program execution tick ─────────────────
  if (programActive && runState != RUN_PAUSED) {
    tickProgram();
    return;   // display off during run — skip draw
  }

  // ── Draw ───────────────────────────────────
  drawCurrentMenu();
}
