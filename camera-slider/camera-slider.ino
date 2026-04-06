/*
 * ============================================================
 *  Camera Slider Controller  —  v1.2
 *  Hardware : Arduino Uno
 *             SH1106 1.3" OLED  (I2C 0x3C, SDA=A4, SCL=A5)
 *             EC11 rotary encoder + 2 buttons
 *             TMC2209 + NEMA 17
 *             2 endstops
 *             Wired handheld: LEFT, RIGHT, PLAY/PAUSE buttons
 *             (Display + buttons in separate handheld enclosure)
  *
 *  PIN MAP
 *  -------
 *  OLED SDA/SCL  → A4 / A5         OLED VCC  → 3.3 V
 *  EC11 CLK      → D2  (interrupt) EC11 DT   → D3
 *  EC11 SW       → D4  (select)
 *  BTN  BACK     → D5              BTN CONF  → D6
 *  BTN  LEFT     → A0              BTN RIGHT → A1
 *  BTN  ESTOP    → A2
  *  STEP / DIR / EN → D9 / D10 / D11
 *  Endstop LEFT  → D12  (NC, INPUT_PULLUP)
 *  Endstop RIGHT → D13  (NC, INPUT_PULLUP)
 *
  *  Libraries: U8g2, AccelStepper  (Library Manager)
 *
 *  --------
 *  1. Upload IR_Learn.ino to discover your remote's button codes.
 *  2. Open Serial Monitor at 115200 baud and press each button.
 * ============================================================
 */

#include <U8g2lib.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <EEPROM.h>

// ─────────────────────────────────────────────

// ─────────────────────────────────────────────

// ─────────────────────────────────────────────

//  Replace 0x00000000 with your remote's codes.
// ─────────────────────────────────────────────

// ─────────────────────────────────────────────
//  DISPLAY
// ─────────────────────────────────────────────
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ─────────────────────────────────────────────
//  PINS
// ─────────────────────────────────────────────
#define ENC_CLK    2
#define ENC_DT     3
#define ENC_SW     4
#define BTN_BACK   5
#define BTN_CONF   6
#define STEP_PIN   9
#define DIR_PIN    10
#define EN_PIN     11
#define END_LEFT   12
#define END_RIGHT  13
#define BTN_LEFT   A0
#define BTN_RIGHT  A1
#define BTN_ESTOP  A2

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

Button btnEnc   = { ENC_SW    };
Button btnBack  = { BTN_BACK  };
Button btnConf  = { BTN_CONF  };
Button btnLeft  = { BTN_LEFT  };
Button btnRight = { BTN_RIGHT };
Button btnEstop = { BTN_ESTOP };

void updateButton(Button &b) {
  bool cur = digitalRead(b.pin);
  b.pressed = false;
  if (cur == LOW && b.last == HIGH) { b.downAt = millis(); b.held = false; }
  if (cur == LOW && !b.held && millis() - b.downAt >= BTN_HOLD_MS) b.held = true;
  if (cur == HIGH && b.last == LOW) { if (!b.held) b.pressed = true; b.held = false; }
  b.last = cur;
}

void updateAllButtons() {
  updateButton(btnEnc);  updateButton(btnBack);  updateButton(btnConf);
  updateButton(btnLeft); updateButton(btnRight);  updateButton(btnEstop);
}

// ─────────────────────────────────────────────
//  MENU IDs
// ─────────────────────────────────────────────
enum MenuID {
  MNU_ROOT, MNU_SELECT_PROGRAM, MNU_CLEAR_PROGRAM,
  MNU_FULL_LOOP_SETTINGS, MNU_MOTOR_CONTROL, MNU_PROG_MOVEMENT,
  MNU_PROG_SELECT_SLOT, MNU_CONFIRM_CLEAR, MNU_CONFIRM_CLEAR_ALL,
  MNU_CONFIRM_DISABLE, MNU_EDIT_VALUE, MNU_ADD_STOP_TIMER,
  MNU_RUN_SCREEN, MNU_PAUSE_SCREEN, MNU_ESTOP_SCREEN
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
void enableMotor(bool en)  { digitalWrite(EN_PIN, en ? LOW : HIGH); }
bool endstopLeft()         { return digitalRead(END_LEFT)  == LOW; }
bool endstopRight()        { return digitalRead(END_RIGHT) == LOW; }

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
  displayOn();   // program finished — wake display, return to menu
  openMenu(MNU_ROOT);
}

void decelStop() {
  stepper.stop();
  while (stepper.isRunning()) stepper.run();
  stepper.setSpeed(0);
}

void homeMotor() {
  isHomed = false;
  enableMotor(true);
  setRunSpeed(HOME_SPEED);
  stepper.setSpeed(-HOME_SPEED);
  while (!endstopLeft()) {
    stepper.runSpeed();
    // Allow e-stop even during homing
    if (digitalRead(BTN_ESTOP) == LOW) { decelStop(); enableMotor(false); return; }
  }
  stepper.setCurrentPosition(0);
  stepper.stop();
  isHomed = true;
}

// ─────────────────────────────────────────────
//  JOG  (global — locked out during active run)
// ─────────────────────────────────────────────
void handleJog() {
  if (programActive && runState != RUN_PAUSED) return;  // locked during run

  bool lHeld = (digitalRead(BTN_LEFT)  == LOW);
  bool rHeld = (digitalRead(BTN_RIGHT) == LOW);

  if (lHeld && !endstopLeft()) {
    enableMotor(true);
    stepper.setMaxSpeed(JOG_FAST_SPEED);
    stepper.setSpeed(-JOG_FAST_SPEED);
    stepper.runSpeed();
    motorRunning = true;
  } else if (rHeld && !endstopRight()) {
    enableMotor(true);
    stepper.setMaxSpeed(JOG_FAST_SPEED);
    stepper.setSpeed(JOG_FAST_SPEED);
    stepper.runSpeed();
    motorRunning = true;
  } else if (btnLeft.pressed && !endstopLeft()) {
    enableMotor(true);
    setRunSpeed(JOG_SLOW_SPEED);
    stepper.move(-JOG_SLOW_STEPS);
    while (stepper.distanceToGo() != 0) stepper.run();
    if (!programActive) enableMotor(false);
    motorRunning = false;
  } else if (btnRight.pressed && !endstopRight()) {
    enableMotor(true);
    setRunSpeed(JOG_SLOW_SPEED);
    stepper.move(JOG_SLOW_STEPS);
    while (stepper.distanceToGo() != 0) stepper.run();
    if (!programActive) enableMotor(false);
    motorRunning = false;
  } else {
    // No jog active — if we were fast-jogging outside a program, stop motor
    if (motorRunning && !programActive) {
      stepper.stop();
      enableMotor(false);
      motorRunning = false;
    }
  }
}

// ─────────────────────────────────────────────
//  E-STOP
// ─────────────────────────────────────────────
void handleEstop() {
  decelStop();
  enableMotor(false);
  motorRunning  = false;
  programActive = false;
  runState      = RUN_IDLE;
  isHomed       = false;   // position unknown after e-stop
  displayOn();             // wake display to show e-stop screen
  currentMenu   = MNU_ESTOP_SCREEN;
  menuCursor    = 0;
  menuScroll    = 0;
}

// ─────────────────────────────────────────────
//  PAUSE / RESUME
// ─────────────────────────────────────────────
void pauseProgram() {
  decelStop();
  // Keep motors energized to hold position
  enableMotor(true);
  motorRunning = false;
  runState     = RUN_PAUSED;
  displayOn();             // wake display to show pause screen
  currentMenu  = MNU_PAUSE_SCREEN;
}

void resumeProgram() {
  motorRunning = true;
  currentMenu  = MNU_RUN_SCREEN;

  if (activeSlot == 4) {
    float spd = fullLoop.speedMms * STEPS_PER_MM;
    setRunSpeed(spd);
    if (runState == RUN_FL_BACKWARD)
      stepper.moveTo(0);
    else {
      runState = RUN_FL_FORWARD;
      stepper.moveTo(MAX_TRAVEL_STEPS);
    }

  } else if (activeSlot == 5) {
    setRunSpeed(fullLoop.speedMms * STEPS_PER_MM);
    stepper.moveTo(MAX_TRAVEL_STEPS);
    runState = RUN_MTE_MOVING;

  } else if (activeSlot >= 0 && activeSlot < NUM_SLOTS) {
    ProgramSlot &p = slots[activeSlot];
    float spd = p.speedMms * STEPS_PER_MM;
    setRunSpeed(spd);
    if (runState == RUN_DWELLING) return;  // dwell timer resumes naturally
    if (currentStop < p.stopCount) {
      stepper.moveTo(p.stops[currentStop].position);
      runState = RUN_MOVING_TO_STOP;
    } else {
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

  // ── Endstop safety ───────────────────────────
  if (motorRunning) {
    if (stepper.speed() < 0 && endstopLeft())  { decelStop(); stopMotorHard(); }
    if (stepper.speed() > 0 && endstopRight()) { decelStop(); stopMotorHard(); }
  }
}

// ─────────────────────────────────────────────
//  PROGRAM LAUNCH
// ─────────────────────────────────────────────
bool startProgram() {
  if (!isHomed) {
    // Show warning
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(4, 20, "Not homed!");
      u8g2.drawStr(4, 36, "Home motors first");
      u8g2.drawStr(4, 52, "CONF to continue");
    } while (u8g2.nextPage());
    delay(1800);
    return false;
  }

  if (activeSlot >= 0 && activeSlot < NUM_SLOTS) {
    if (!slots[activeSlot].valid) {
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(4, 32, "No program in slot!");
      } while (u8g2.nextPage());
      delay(1500);
      return false;
    }
    ProgramSlot &p = slots[activeSlot];
    float spd = p.speedMms * STEPS_PER_MM;
    enableMotor(true);
    setRunSpeed(spd);
    stepper.moveTo(p.startPos);
    currentLoop   = 0;
    currentStop   = 0;
    runState      = RUN_MOVING_TO_START;
    motorRunning  = true;
    programActive = true;
    displayOff();  // screen off for the duration of the run

  } else if (activeSlot == 4) {
    // Full Loop
    enableMotor(true);
    float spd = fullLoop.speedMms * STEPS_PER_MM;
    setRunSpeed(spd);
    stepper.moveTo(MAX_TRAVEL_STEPS);
    currentLoop   = 0;
    runState      = RUN_FL_FORWARD;
    motorRunning  = true;
    programActive = true;
    displayOff();

  } else if (activeSlot == 5) {
    // Move to Endpoint — uses full loop speed, moves to right endstop
    enableMotor(true);
    float spd = fullLoop.speedMms * STEPS_PER_MM;
    setRunSpeed(spd);
    stepper.moveTo(MAX_TRAVEL_STEPS);
    runState      = RUN_MTE_MOVING;
    motorRunning  = true;
    programActive = true;
    displayOff();

  } else {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(4, 32, "No program selected");
    } while (u8g2.nextPage());
    delay(1500);
    return false;
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

void drawConfirm(const char* l1, const char* l2 = nullptr) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 12, l1);
    if (l2) u8g2.drawStr(0, 26, l2);
    u8g2.drawStr(0, 50, "BACK = Cancel");
    u8g2.drawStr(0, 62, "CONF = Confirm");
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
    u8g2.drawStr((128 - u8g2.getStrWidth(vb))/2, 45, vb);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 62, "BACK");
    u8g2.drawStr(90, 62, "CONF=OK");
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
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_9x18B_tf);
    u8g2.drawStr(28, 22, "PAUSED");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 36, "Pos:"); u8g2.drawStr(30, 36, posBuf); u8g2.drawStr(80,36,"mm");
    u8g2.drawStr(0, 50, "CONF/JOG = Resume");
    u8g2.drawStr(0, 62, "BACK = Cancel");
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
//  Bytes 5+      : ProgramSlot[0..3]  (sizeof each = varies, use put/get)
//
//  EEPROM.put() writes only bytes that changed,
//  protecting the 100k write-cycle limit.
// ─────────────────────────────────────────────
#define EEPROM_MAGIC_ADDR   0
#define EEPROM_MAGIC_VALUE  0xAB
#define EEPROM_FL_SPEED     1    // uint16_t  2 bytes
#define EEPROM_FL_LOOPS     3    // uint16_t  2 bytes
#define EEPROM_SLOTS_START  5    // ProgramSlot × 4

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
    // First boot — initialise defaults and write magic
    memset(slots, 0, sizeof(slots));
    fullLoop.speedMms  = DEFAULT_SPEED_MMS;
    fullLoop.loopCount = DEFAULT_LOOP_COUNT;
    // Write defaults so future boots load cleanly
    EEPROM.put(EEPROM_MAGIC_ADDR, (uint8_t)EEPROM_MAGIC_VALUE);
    eepromSaveFullLoop();
    for (int i = 0; i < NUM_SLOTS; i++) eepromSaveSlot(i);
    return;
  }

  // Load full loop settings
  EEPROM.get(EEPROM_FL_SPEED, fullLoop.speedMms);
  EEPROM.get(EEPROM_FL_LOOPS, fullLoop.loopCount);

  // Sanity-check loaded values
  if (fullLoop.speedMms < 1 || fullLoop.speedMms > 100)
    fullLoop.speedMms = DEFAULT_SPEED_MMS;

  // Load program slots
  for (int i = 0; i < NUM_SLOTS; i++) {
    EEPROM.get(slotAddr(i), slots[i]);
    // If slot data looks corrupt, mark invalid
    if (slots[i].stopCount > MAX_STOPS) {
      memset(&slots[i], 0, sizeof(ProgramSlot));
    }
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
  eepromSaveSlot(wipSlot);   // persist to EEPROM
}

void doClearSlot(int idx) {
  if (idx < 0) {
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
      slots[i].valid = false;
      eepromSaveSlot(i);     // persist each cleared slot
    }
  } else {
    slots[idx].valid = false;
    eepromSaveSlot(idx);
  }
}

// ─────────────────────────────────────────────
//  MENU INPUT HANDLER
// ─────────────────────────────────────────────
void handleMenuInput(int delta, bool sel, bool back) {
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
      scrollMenu(delta, 7);
      if (back || (sel && menuCursor==6)) { openMenu(MNU_ROOT); break; }
      if (sel) {
        activeSlot = (menuCursor <= 1) ? menuCursor+4 : menuCursor-2;
        openMenu(MNU_ROOT);
      }
      break;

    case MNU_CLEAR_PROGRAM:
      scrollMenu(delta, 6);
      if (back || (sel && menuCursor==5)) { openMenu(MNU_ROOT); break; }
      if (sel) {
        clearSlotTarget = (menuCursor==4) ? -1 : menuCursor;
        openMenu(menuCursor==4 ? MNU_CONFIRM_CLEAR_ALL : MNU_CONFIRM_CLEAR);
      }
      break;

    case MNU_CONFIRM_CLEAR:
      if (back) { openMenu(MNU_CLEAR_PROGRAM); break; }
      if (sel)  { doClearSlot(clearSlotTarget); openMenu(MNU_CLEAR_PROGRAM); }
      break;

    case MNU_CONFIRM_CLEAR_ALL:
      if (back) { openMenu(MNU_CLEAR_PROGRAM); break; }
      if (sel)  { doClearSlot(-1); openMenu(MNU_CLEAR_PROGRAM); }
      break;

    case MNU_FULL_LOOP_SETTINGS:
      scrollMenu(delta, fullLoopCount);
      if (back || (sel && menuCursor==3)) { openMenu(MNU_ROOT); break; }
      if (sel) switch (menuCursor) {
        case 0: openValueEditor("FL Speed (mm/s)", fullLoop.speedMms, 1, 100, 1, &fullLoop.speedMms, MNU_FULL_LOOP_SETTINGS); break;
        case 1: openValueEditor("Loop Count(0=cont)", fullLoop.loopCount, 0, 9999, 1, &fullLoop.loopCount, MNU_FULL_LOOP_SETTINGS); break;
        case 2: fullLoop.speedMms=DEFAULT_SPEED_MMS; fullLoop.loopCount=DEFAULT_LOOP_COUNT; eepromSaveFullLoop(); break;
      }
      break;

    case MNU_MOTOR_CONTROL:
      scrollMenu(delta, motorCtrlCount);
      if (back || (sel && menuCursor==2)) { openMenu(MNU_ROOT); break; }
      if (sel) switch (menuCursor) {
        case 0: doHomeAll(); break;
        case 1: openMenu(MNU_CONFIRM_DISABLE); break;
      }
      break;

    case MNU_CONFIRM_DISABLE:
      if (back) { openMenu(MNU_MOTOR_CONTROL); break; }
      if (sel)  { decelStop(); enableMotor(false); motorRunning=false; openMenu(MNU_MOTOR_CONTROL); }
      break;

    case MNU_PROG_MOVEMENT:
      scrollMenu(delta, progMovCount);
      if (back || (sel && menuCursor==8)) { openMenu(MNU_ROOT); break; }
      if (sel) switch (menuCursor) {
        case 0: openMenu(MNU_PROG_SELECT_SLOT); break;
        case 1: doHomeAll(); break;
        case 2: // Set Speed
          openValueEditor("Prog Speed (mm/s)", wip.speedMms ? wip.speedMms : DEFAULT_SPEED_MMS,
            1, 100, 1, &wip.speedMms, MNU_PROG_MOVEMENT);
          break;
        case 3: wip.startPos=stepper.currentPosition(); wipHasStart=true; break;
        case 4:
          if (wip.stopCount < MAX_STOPS) {
            wip.stops[wip.stopCount].position = stepper.currentPosition();
            wip.stops[wip.stopCount].timerSec = 0;
            openValueEditor("Stop Timer (sec)", 0, 0, 3600, 1,
              &wip.stops[wip.stopCount].timerSec, MNU_ADD_STOP_TIMER);
          }
          break;
        case 5: wip.endPos=stepper.currentPosition(); wipHasEnd=true; break;
        case 6: openValueEditor("Loop Count(0=cont)", wip.loopCount, 0, 9999, 1, &wip.loopCount, MNU_PROG_MOVEMENT); break;
        case 7: doSaveProgram(); openMenu(MNU_ROOT); break;
      }
      break;

    case MNU_PROG_SELECT_SLOT:
      scrollMenu(delta, progSlotCount);
      if (back || (sel && menuCursor==4)) { openMenu(MNU_PROG_MOVEMENT); break; }
      if (sel && menuCursor<4) {
        wipSlot = menuCursor;
        if (slots[wipSlot].valid) { wip=slots[wipSlot]; wipHasStart=wipHasEnd=true; }
        else { memset(&wip,0,sizeof(wip)); wipHasStart=wipHasEnd=false; }
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
      if (back) { openMenu(ved.onDone); break; }
      if (sel) {
        if (ved.maxVal<=65535 && ved.minVal>=0) *((uint16_t*)ved.target)=(uint16_t)ved.value;
        else *((long*)ved.target)=ved.value;
        // If we just changed a Full Loop setting, persist it
        if (ved.onDone == MNU_FULL_LOOP_SETTINGS) eepromSaveFullLoop();
        openMenu(ved.onDone==MNU_ADD_STOP_TIMER ? MNU_ADD_STOP_TIMER : ved.onDone);
      }
      break;

    case MNU_RUN_SCREEN:
      // Jog buttons pause (handled in handleJog via programActive flag)
      if (back) { pauseProgram(); break; }  // back from run = pause
      break;

    case MNU_PAUSE_SCREEN:
      if (back) { cancelProgram(); openMenu(MNU_RUN_SCREEN); break; }
      if (sel)  { resumeProgram(); break; }
      // Jog buttons also resume (checked in loop)
      break;

    case MNU_ESTOP_SCREEN:
      if (sel) openMenu(MNU_ROOT);
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
#define FIRMWARE_VER   "v1.0"
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
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(EN_PIN,    OUTPUT); enableMotor(false);
  pinMode(ENC_CLK,   INPUT_PULLUP);
  pinMode(ENC_DT,    INPUT_PULLUP);
  pinMode(ENC_SW,    INPUT_PULLUP);
  pinMode(BTN_BACK,  INPUT_PULLUP);
  pinMode(BTN_CONF,  INPUT_PULLUP);
  pinMode(BTN_LEFT,  INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_ESTOP, INPUT_PULLUP);
  pinMode(END_LEFT,  INPUT_PULLUP);
  pinMode(END_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encoderISR, CHANGE);


  stepper.setMaxSpeed(JOG_FAST_SPEED);
  stepper.setAcceleration(JOG_FAST_SPEED * 2);
  eepromLoad();   // load saved slots and full loop settings

  u8g2.begin();
  drawSplashScreen();

  openMenu(MNU_ROOT);
}

// ─────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────
void loop() {
  // ── Encoder ──
  int delta = 0;
  noInterrupts(); delta=encDelta; encDelta=0; interrupts();

  // ── Buttons ──
  updateAllButtons();

    }
  }

  // ── E-STOP — highest priority ──
  if (btnEstop.pressed || btnEstop.held) {
    handleEstop();   // displayOn() called inside
    drawCurrentMenu();
    return;
  }

  // ── Pause trigger: jog button pressed during active run ──
  if (programActive && runState != RUN_PAUSED) {
    if (btnLeft.pressed || btnRight.pressed) {
      pauseProgram();  // displayOn() called inside
    }
  }

  // ── Resume from pause: jog button or CONF ──
  if (currentMenu == MNU_PAUSE_SCREEN) {
    if (btnLeft.pressed || btnRight.pressed) resumeProgram();
  }

  // ── Jog (only when not in active run) ──
  handleJog();

  // ── Menu input ──
  bool sel  = btnEnc.pressed || btnConf.pressed;
  bool back = btnBack.pressed;
  if (delta || sel || back) handleMenuInput(delta, sel, back);

  // ── Program execution tick ──
  if (programActive && runState != RUN_PAUSED) {
    tickProgram();
    return;   // display is off — skip draw entirely while running
  }

  // ── Draw (only when not actively running) ──
  drawCurrentMenu();
}
