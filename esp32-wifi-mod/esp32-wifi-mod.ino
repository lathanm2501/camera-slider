/*
 * ============================================================
 *  Camera Slider Controller  —  ESP32 WiFi Mod  v1.2
 *  Hardware : ESP32 DevKit v1 (38-pin)
 *             BIGTREETECH Mini12864 V2.0  (SPI, UC1701)
 *             TMC2209 + NEMA 17
 *             2 endstops  (separate pins)
 *             LEFT / RIGHT / PLAY-PAUSE buttons
 *             WS2812 NeoPixel backlight (on Mini12864)
 *
 *  WiFi     : Station mode with AP fallback + captive portal
 *             Web UI  →  http://<ip>/
 *             REST API →  http://<ip>/api/...
 *
 *  Architecture:
 *    Core 0  — WiFi, web server, DNS
 *    Core 1  — Motor, display, buttons  (loop())
 *
 *  PIN MAP  (ESP32 DevKit 38-pin)
 *  ──────────────────────────────────────────────────────────
 *  ⚠  GPIO 34,35,36,39 are INPUT-ONLY — no internal pullup.
 *     Use external 10k resistors to 3.3V for these pins.
 *
 *  Mini12864 EXP2:
 *    ENC_CLK  → GPIO 34  (ext pullup)
 *    ENC_DT   → GPIO 35  (ext pullup)
 *    MOSI     → GPIO 23
 *    SCK      → GPIO 18
 *    LCD_CS   → GPIO 5
 *
 *  Mini12864 EXP1:
 *    ENC_SW   → GPIO 32
 *    LCD_RST  → GPIO 33
 *    LCD_A0   → GPIO 25
 *    NEOPIXEL → GPIO 26
 *
 *  External buttons (INPUT_PULLUP, active LOW):
 *    LEFT     → GPIO 27
 *    RIGHT    → GPIO 14
 *    PLAY/PAU → GPIO 12
 *
 *  Endstops (NC — ext pullup to 3.3V):
 *    LEFT     → GPIO 36
 *    RIGHT    → GPIO 39
 *
 *  TMC2209:
 *    STEP     → GPIO 16
 *    DIR      → GPIO 17
 *    EN       → GND (permanently enabled)
 *
 *  Libraries (install via Library Manager):
 *    U8g2, AccelStepper, Adafruit NeoPixel,
 *    ArduinoJson (v6), ESPAsyncWebServer, AsyncTCP
 * ============================================================
 */

// ─────────────────────────────────────────────
//  LIBRARIES
// ─────────────────────────────────────────────
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>
#include <AccelStepper.h>
#include <Preferences.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SPI.h>

// ─────────────────────────────────────────────
//  VERSION
// ─────────────────────────────────────────────
#define FIRMWARE_VER  "v1.2-ESP32"

// ─────────────────────────────────────────────
//  PINS
// ─────────────────────────────────────────────
#define ENC_CLK    34
#define ENC_DT     35
#define ENC_SW     32
#define LCD_RST    33
#define LCD_A0     25
#define NEO_PIN    26
#define LCD_MOSI   23
#define LCD_SCK    18
#define LCD_CS      5
#define BTN_LEFT   27
#define BTN_RIGHT  14
#define BTN_PLAY   12
#define END_LEFT   36
#define END_RIGHT  39
#define STEP_PIN   16
#define DIR_PIN    17

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
#define MAX_TRAVEL_STEPS   (STEPS_PER_MM * 200)
#define BTN_HOLD_MS        400

// ─────────────────────────────────────────────
//  WIFI CONFIG
// ─────────────────────────────────────────────
#define AP_SSID        "CameraSlider"
#define AP_PASSWORD    "slider123"
#define AP_IP          IPAddress(192, 168, 4, 1)
#define WIFI_TIMEOUT   10000
#define DNS_PORT       53

// ─────────────────────────────────────────────
//  DATA STRUCTURES
// ─────────────────────────────────────────────
#define MAX_STOPS  10
#define NUM_SLOTS   4

struct StopPoint { long position; uint16_t timerSec; };

struct ProgramSlot {
  bool      valid;
  uint16_t  speedMms, loopCount;
  long      startPos, endPos;
  uint8_t   stopCount;
  StopPoint stops[MAX_STOPS];
};

struct FullLoopSettings { uint16_t speedMms, loopCount; };

// ─────────────────────────────────────────────
//  OBJECTS
// ─────────────────────────────────────────────
U8G2_UC1701_MINI12864_1_4W_SW_SPI u8g2(
  U8G2_R0, LCD_SCK, LCD_MOSI, LCD_CS, LCD_A0, LCD_RST);
Adafruit_NeoPixel neo(2, NEO_PIN, NEO_GRB + NEO_KHZ800);
AccelStepper      stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Preferences       prefs;
AsyncWebServer    webServer(80);
DNSServer         dnsServer;

// ─────────────────────────────────────────────
//  BACKLIGHT
// ─────────────────────────────────────────────
#define NEO_IDLE    neo.Color(  0,180,  0)
#define NEO_RUNNING neo.Color(  0,  0,180)
#define NEO_PAUSED  neo.Color(200,120,  0)
#define NEO_ESTOP   neo.Color(255,  0,  0)
#define NEO_HOMING  neo.Color(200,200,200)
#define NEO_WIFI    neo.Color(  0,120,200)
void setBacklight(uint32_t c){neo.fill(c);neo.show();}

// ─────────────────────────────────────────────
//  RUNTIME STATE
// ─────────────────────────────────────────────
ProgramSlot      slots[NUM_SLOTS];
FullLoopSettings fullLoop = {DEFAULT_SPEED_MMS, DEFAULT_LOOP_COUNT};
ProgramSlot      wip;
int              wipSlot     = -1;
bool             wipHasStart = false, wipHasEnd = false;

int  activeSlot    = -1;
bool isHomed       = false;
bool motorRunning  = false;
bool programActive = false;

enum RunState {
  RUN_IDLE, RUN_MOVING_TO_START, RUN_MOVING_TO_STOP,
  RUN_DWELLING, RUN_MOVING_TO_END, RUN_RETURNING, RUN_PAUSED, RUN_DONE,
  RUN_FL_FORWARD, RUN_FL_BACKWARD, RUN_FL_DONE, RUN_MTE_MOVING
};
RunState     runState    = RUN_IDLE;
uint16_t     currentLoop = 0;
uint8_t      currentStop = 0;
unsigned long dwellStart = 0;
long          dwellMs    = 0;

enum WifiState { WIFI_CONNECTING, WIFI_STATION, WIFI_AP, WIFI_PORTAL };
WifiState wifiState   = WIFI_CONNECTING;
String    wifiIP      = "";
bool      portalActive= false;

// ── Thread-safe command queue (web → motor) ──
struct WebCmd {
  enum Type { NONE,HOME,START,STOP,PAUSE,RESUME,
              JOG_L,JOG_R,JOG_STOP,SEL_SLOT,
              FL_SPEED,FL_LOOPS,ESTOP } type=NONE;
  long p1=0, p2=0;
};
volatile WebCmd pendingCmd;
portMUX_TYPE    cmdMux = portMUX_INITIALIZER_UNLOCKED;

void postCmd(WebCmd::Type t, long p1=0, long p2=0){
  portENTER_CRITICAL(&cmdMux);
  pendingCmd={t,p1,p2};
  portEXIT_CRITICAL(&cmdMux);
}
WebCmd consumeCmd(){
  WebCmd c;
  portENTER_CRITICAL(&cmdMux);
  c=pendingCmd; pendingCmd.type=WebCmd::NONE;
  portEXIT_CRITICAL(&cmdMux);
  return c;
}

// ─────────────────────────────────────────────
//  ENCODER ISR
// ─────────────────────────────────────────────
volatile int encDelta=0;
void IRAM_ATTR encoderISR(){
  bool clk=digitalRead(ENC_CLK), dt=digitalRead(ENC_DT);
  if(!clk) encDelta += (dt!=clk)?+1:-1;
}

// ─────────────────────────────────────────────
//  BUTTONS
// ─────────────────────────────────────────────
struct Button{
  uint8_t pin; bool last=HIGH,pressed=false,held=false;
  unsigned long downAt=0;
};
Button btnEnc={ENC_SW},btnLeft={BTN_LEFT},btnRight={BTN_RIGHT},btnPlay={BTN_PLAY};
bool encLongFired=false;
int  pauseCursor=0;

void updateBtn(Button &b){
  bool c=digitalRead(b.pin); b.pressed=false;
  if(c==LOW&&b.last==HIGH){b.downAt=millis();b.held=false;}
  if(c==LOW&&!b.held&&millis()-b.downAt>=BTN_HOLD_MS) b.held=true;
  if(c==HIGH&&b.last==LOW){if(!b.held) b.pressed=true; b.held=false;}
  b.last=c;
}
void updateAllBtns(){updateBtn(btnEnc);updateBtn(btnLeft);updateBtn(btnRight);updateBtn(btnPlay);}

// ─────────────────────────────────────────────
//  MENU
// ─────────────────────────────────────────────
enum MenuID {
  MNU_ROOT,MNU_SEL_PROG,MNU_CLR_PROG,MNU_FULL_LOOP,MNU_MOTOR_CTRL,
  MNU_PROG_MOV,MNU_PROG_SLOT,MNU_CONF_CLEAR,MNU_CONF_CLEARALL,
  MNU_CONF_DISABLE,MNU_EDIT_VAL,MNU_ADD_TIMER,MNU_RUN,MNU_PAUSE,
  MNU_ESTOP,MNU_HOME_REQ,MNU_WIFI,MNU_WIFI_RESET
};
MenuID currentMenu=MNU_ROOT;
int menuCursor=0,menuScroll=0;
#define VIS 4

struct VEd{const char*lbl;long val,mn,mx,stp;MenuID done;void*tgt;};
VEd ved;
int clearTarget=-1;

const char*rootItems[]={"Home All Motors","Start / Stop","Select Program","Clear Program","Full Loop Settings","Motor Control","Program Movement","WiFi Status"};
const uint8_t rootCount=8;
const char*flItems[]={"Set Speed","Set Loop Count","Reset to Default","< Back"};
const uint8_t flCount=4;
const char*mCtrlItems[]={"Home All Motors","Disable Motors","< Back"};
const uint8_t mCtrlCount=3;
const char*pmItems[]={"Select Slot","Home","Set Speed","Set Start","Add Stop Point","Set End Point","Set Loop Count","Save Program","< Back"};
const uint8_t pmCount=9;
const char*psItems[]={"Slot 1","Slot 2","Slot 3","Slot 4","< Back"};
const uint8_t psCount=5;
const char*wifiItems[]={"Reset WiFi Creds","< Back"};
const uint8_t wifiCount=2;

// ─────────────────────────────────────────────
//  PERSISTENCE
// ─────────────────────────────────────────────
void saveSlot(int i){
  char k[8]; snprintf(k,sizeof(k),"slot%d",i);
  prefs.begin("cs",false); prefs.putBytes(k,&slots[i],sizeof(ProgramSlot)); prefs.end();
}
void loadAll(){
  prefs.begin("cs",true);
  for(int i=0;i<NUM_SLOTS;i++){
    char k[8]; snprintf(k,sizeof(k),"slot%d",i);
    if(prefs.isKey(k)) prefs.getBytes(k,&slots[i],sizeof(ProgramSlot));
    else memset(&slots[i],0,sizeof(ProgramSlot));
  }
  fullLoop.speedMms =prefs.getUShort("fl_spd",DEFAULT_SPEED_MMS);
  fullLoop.loopCount=prefs.getUShort("fl_lp", DEFAULT_LOOP_COUNT);
  prefs.end();
}
void saveFL(){
  prefs.begin("cs",false);
  prefs.putUShort("fl_spd",fullLoop.speedMms);
  prefs.putUShort("fl_lp", fullLoop.loopCount);
  prefs.end();
}
bool loadWifi(String&s,String&p){
  prefs.begin("wifi",true); s=prefs.getString("ssid",""); p=prefs.getString("pass",""); prefs.end();
  return s.length()>0;
}
void saveWifi(const String&s,const String&p){
  prefs.begin("wifi",false); prefs.putString("ssid",s); prefs.putString("pass",p); prefs.end();
}
void clearWifi(){ prefs.begin("wifi",false); prefs.clear(); prefs.end(); }

// ─────────────────────────────────────────────
//  MOTOR HELPERS
// ─────────────────────────────────────────────
void enableMotor(bool){}
bool endstopLeft() {return digitalRead(END_LEFT)==LOW;}
bool endstopRight(){return digitalRead(END_RIGHT)==LOW;}
void setSpd(float s){stepper.setMaxSpeed(s);stepper.setAcceleration(s*2);}
void displayOn() {u8g2.setPowerSave(0);}
void displayOff(){u8g2.setPowerSave(1);}

void openMenu(MenuID id,int cur=0){currentMenu=id;menuCursor=cur;menuScroll=0;}

void stopHard(){
  stepper.stop(); while(stepper.isRunning()) stepper.run(); stepper.setSpeed(0);
  motorRunning=false; programActive=false; runState=RUN_IDLE;
  displayOn(); setBacklight(NEO_IDLE); openMenu(MNU_ROOT);
}
void decel(){stepper.stop();while(stepper.isRunning()) stepper.run();stepper.setSpeed(0);}

void homeMotor(){
  isHomed=false; setBacklight(NEO_HOMING); setSpd(HOME_SPEED);
  stepper.setSpeed(-HOME_SPEED);
  unsigned long t=millis();
  while(!endstopLeft()){
    stepper.runSpeed();
    if(btnEnc.held&&!encLongFired) break;
    if(millis()-t>30000) break;
  }
  stepper.setCurrentPosition(0); stepper.stop();
  if(endstopLeft()) isHomed=true;
  setBacklight(NEO_IDLE);
}

void jogLeft(bool fast){
  if(endstopLeft()) return;
  if(fast){stepper.setMaxSpeed(JOG_FAST_SPEED);stepper.setSpeed(-JOG_FAST_SPEED);stepper.runSpeed();motorRunning=true;}
  else{setSpd(JOG_SLOW_SPEED);stepper.move(-JOG_SLOW_STEPS);while(stepper.distanceToGo())stepper.run();motorRunning=false;}
}
void jogRight(bool fast){
  if(endstopRight()) return;
  if(fast){stepper.setMaxSpeed(JOG_FAST_SPEED);stepper.setSpeed(JOG_FAST_SPEED);stepper.runSpeed();motorRunning=true;}
  else{setSpd(JOG_SLOW_SPEED);stepper.move(JOG_SLOW_STEPS);while(stepper.distanceToGo())stepper.run();motorRunning=false;}
}
void handleJog(){
  if(programActive&&runState!=RUN_PAUSED) return;
  bool lH=(digitalRead(BTN_LEFT)==LOW), rH=(digitalRead(BTN_RIGHT)==LOW);
  if     (lH&&!endstopLeft())  jogLeft(true);
  else if(rH&&!endstopRight()) jogRight(true);
  else if(btnLeft.pressed&&!endstopLeft())  jogLeft(false);
  else if(btnRight.pressed&&!endstopRight()) jogRight(false);
  else if(motorRunning&&!programActive){stepper.stop();motorRunning=false;}
}

// ─────────────────────────────────────────────
//  E-STOP / PAUSE / RESUME / CANCEL
// ─────────────────────────────────────────────
void handleEstop(){
  decel(); motorRunning=false; programActive=false; runState=RUN_IDLE;
  isHomed=false; encLongFired=true; displayOn(); setBacklight(NEO_ESTOP);
  currentMenu=MNU_ESTOP; menuCursor=0; menuScroll=0;
}
void pauseProg(){
  decel(); motorRunning=false; runState=RUN_PAUSED;
  displayOn(); setBacklight(NEO_PAUSED);
  currentMenu=MNU_PAUSE; menuCursor=0; menuScroll=0; pauseCursor=0;
}
void resumeProg(){
  motorRunning=true; currentMenu=MNU_RUN; setBacklight(NEO_RUNNING);

  if(activeSlot==4){
    float spd=fullLoop.speedMms*STEPS_PER_MM; setSpd(spd);
    if(runState==RUN_FL_BACKWARD) stepper.moveTo(0);
    else{ runState=RUN_FL_FORWARD; stepper.moveTo(MAX_TRAVEL_STEPS); }

  } else if(activeSlot==5){
    setSpd(fullLoop.speedMms*STEPS_PER_MM);
    stepper.moveTo(MAX_TRAVEL_STEPS); runState=RUN_MTE_MOVING;

  } else if(activeSlot>=0&&activeSlot<NUM_SLOTS){
    ProgramSlot&p=slots[activeSlot]; float spd=p.speedMms*STEPS_PER_MM;
    setSpd(spd);
    if(runState==RUN_DWELLING) return;  // dwell timer resumes naturally
    if(currentStop<p.stopCount){
      stepper.moveTo(p.stops[currentStop].position); runState=RUN_MOVING_TO_STOP;
    } else {
      if(runState==RUN_RETURNING){ setSpd(HOME_SPEED); stepper.moveTo(p.startPos); }
      else{ stepper.moveTo(p.endPos); runState=RUN_MOVING_TO_END; }
    }
  }
}
void cancelProg(){
  long rp=(activeSlot>=0&&activeSlot<NUM_SLOTS)?slots[activeSlot].startPos:0;
  setSpd(HOME_SPEED); stepper.moveTo(rp);
  runState=RUN_RETURNING; motorRunning=true; currentMenu=MNU_RUN;
}

// ─────────────────────────────────────────────
//  SLOT SUMMARY
// ─────────────────────────────────────────────
void slotSum(int i,char*buf,uint8_t len){
  if(!slots[i].valid){snprintf(buf,len,"Empty");return;}
  uint16_t t=0; for(uint8_t j=0;j<slots[i].stopCount;j++) t+=slots[i].stops[j].timerSec;
  snprintf(buf,len,"%dsp %ds %s",slots[i].stopCount,t,slots[i].loopCount==0?"lp":"nl");
}

// ─────────────────────────────────────────────
//  PROGRAM EXECUTION STATE MACHINE
// ─────────────────────────────────────────────
void tickProg(){
  if(!programActive) return;
  if(activeSlot>=0&&activeSlot<NUM_SLOTS){
    ProgramSlot&p=slots[activeSlot]; float spd=p.speedMms*STEPS_PER_MM;
    switch(runState){
      case RUN_MOVING_TO_START:
        if(!stepper.isRunning()){
          currentStop=0;currentLoop=0;
          if(p.stopCount>0){setSpd(spd);stepper.moveTo(p.stops[0].position);runState=RUN_MOVING_TO_STOP;}
          else{setSpd(spd);stepper.moveTo(p.endPos);runState=RUN_MOVING_TO_END;}
        } break;
      case RUN_MOVING_TO_STOP:
        if(!stepper.isRunning()){dwellStart=millis();dwellMs=(long)p.stops[currentStop].timerSec*1000UL;runState=RUN_DWELLING;}
        break;
      case RUN_DWELLING:
        if(millis()-dwellStart>=(unsigned long)dwellMs){
          currentStop++;
          if(currentStop<p.stopCount){setSpd(spd);stepper.moveTo(p.stops[currentStop].position);runState=RUN_MOVING_TO_STOP;}
          else{setSpd(spd);stepper.moveTo(p.endPos);runState=RUN_MOVING_TO_END;}
        } break;
      case RUN_MOVING_TO_END:
        if(!stepper.isRunning()){
          currentLoop++;
          bool done=(p.loopCount!=0)&&(currentLoop>=p.loopCount);
          if(!done){currentStop=0;setSpd(spd);stepper.moveTo(p.startPos);runState=RUN_MOVING_TO_START;}
          else{setSpd(HOME_SPEED);stepper.moveTo(p.startPos);runState=RUN_RETURNING;}
        } break;
      case RUN_RETURNING: if(!stepper.isRunning()) runState=RUN_DONE; break;
      case RUN_DONE: stopHard(); return;
      default: break;
    }
    if(motorRunning) stepper.run();
  } else if(activeSlot==4){
    float spd=fullLoop.speedMms*STEPS_PER_MM;
    switch(runState){
      case RUN_FL_FORWARD:
        if(!stepper.isRunning()){setSpd(spd);stepper.moveTo(0);runState=RUN_FL_BACKWARD;} break;
      case RUN_FL_BACKWARD:
        if(!stepper.isRunning()){
          currentLoop++;
          bool done=(fullLoop.loopCount!=0)&&(currentLoop>=fullLoop.loopCount);
          if(!done){setSpd(spd);stepper.moveTo(MAX_TRAVEL_STEPS);runState=RUN_FL_FORWARD;}
          else runState=RUN_FL_DONE;
        } break;
      case RUN_FL_DONE: stopHard(); return;
      default: break;
    }
    if(motorRunning) stepper.run();
  } else if(activeSlot==5){
    if(runState==RUN_MTE_MOVING&&!stepper.isRunning()){stopHard();return;}
    if(motorRunning) stepper.run();
  }
  // Endstop reversal
  if(motorRunning){
    bool hL=(stepper.speed()<=0&&endstopLeft()), hR=(stepper.speed()>=0&&endstopRight());
    if(hL||hR){
      decel();
      bool lp=false,has=false;
      if(activeSlot>=0&&activeSlot<NUM_SLOTS){lp=(slots[activeSlot].loopCount==0);has=lp||(currentLoop+1<slots[activeSlot].loopCount);}
      else if(activeSlot==4){lp=(fullLoop.loopCount==0);has=lp||(currentLoop+1<fullLoop.loopCount);}
      if(has&&activeSlot!=5){
        float spd=(activeSlot==4)?fullLoop.speedMms*STEPS_PER_MM:slots[activeSlot].speedMms*STEPS_PER_MM;
        setSpd(spd);
        if(hL){stepper.moveTo(MAX_TRAVEL_STEPS);runState=(activeSlot==4)?RUN_FL_FORWARD:RUN_MOVING_TO_END;}
        else{long rp=(activeSlot>=0&&activeSlot<NUM_SLOTS)?slots[activeSlot].startPos:0;stepper.moveTo(rp);currentLoop++;runState=(activeSlot==4)?RUN_FL_BACKWARD:RUN_MOVING_TO_START;}
      } else stopHard();
    }
  }
}

// ─────────────────────────────────────────────
//  LAUNCH / START
// ─────────────────────────────────────────────
bool launchProg(){
  if(activeSlot>=0&&activeSlot<NUM_SLOTS){
    ProgramSlot&p=slots[activeSlot];
    setSpd(p.speedMms*STEPS_PER_MM); stepper.moveTo(p.startPos);
    currentLoop=0;currentStop=0;runState=RUN_MOVING_TO_START;
  } else if(activeSlot==4){
    setSpd(fullLoop.speedMms*STEPS_PER_MM); stepper.moveTo(MAX_TRAVEL_STEPS);
    currentLoop=0;runState=RUN_FL_FORWARD;
  } else if(activeSlot==5){
    setSpd(fullLoop.speedMms*STEPS_PER_MM); stepper.moveTo(MAX_TRAVEL_STEPS);
    runState=RUN_MTE_MOVING;
  } else return false;
  motorRunning=true;programActive=true;displayOff();setBacklight(NEO_RUNNING);openMenu(MNU_RUN);return true;
}
bool startProg(){
  if(activeSlot<0) return false;
  if(activeSlot<NUM_SLOTS&&!slots[activeSlot].valid) return false;
  if(!isHomed){openMenu(MNU_HOME_REQ);return false;}
  return launchProg();
}

// ─────────────────────────────────────────────
//  DISPLAY
// ─────────────────────────────────────────────
void scrollMenu(int d,uint8_t n){
  menuCursor+=d;
  if(menuCursor<0) menuCursor=0;
  if(menuCursor>=(int)n) menuCursor=n-1;
  if(menuCursor<menuScroll) menuScroll=menuCursor;
  if(menuCursor>=menuScroll+VIS) menuScroll=menuCursor-VIS+1;
}
void openVEd(const char*l,long v,long mn,long mx,long s,void*t,MenuID d){ved={l,v,mn,mx,s,d,t};openMenu(MNU_EDIT_VAL);}

void drawMenu(const char**items,uint8_t n,const char*title){
  u8g2.firstPage();do{
    u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(0,10,title);u8g2.drawHLine(0,12,128);
    for(uint8_t i=0;i<VIS;i++){
      uint8_t idx=menuScroll+i; if(idx>=n) break;
      uint8_t y=22+i*13;
      if(idx==(uint8_t)menuCursor){u8g2.drawBox(0,y-10,128,13);u8g2.setDrawColor(0);}
      u8g2.drawStr(2,y,items[idx]);u8g2.setDrawColor(1);
    }
    if(n>VIS){uint8_t bH=max(4,(int)(52/n));u8g2.drawBox(126,menuScroll*52/n+12,2,bH);}
  }while(u8g2.nextPage());
}
void drawDynMenu(const char*title,uint8_t n,void(*lbl)(uint8_t,char*,uint8_t)){
  u8g2.firstPage();do{
    u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(0,10,title);u8g2.drawHLine(0,12,128);
    for(uint8_t i=0;i<VIS;i++){
      uint8_t idx=menuScroll+i; if(idx>=n) break;
      uint8_t y=22+i*13;
      if(idx==(uint8_t)menuCursor){u8g2.drawBox(0,y-10,128,13);u8g2.setDrawColor(0);}
      char buf[22];lbl(idx,buf,sizeof(buf));u8g2.drawStr(2,y,buf);u8g2.setDrawColor(1);
    }
  }while(u8g2.nextPage());
}
void drawConf(const char*l1,const char*l2=nullptr){
  const char*opts[]={"Yes — confirm","No  — cancel"};
  u8g2.firstPage();do{
    u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(0,10,l1);u8g2.drawHLine(0,12,128);
    if(l2) u8g2.drawStr(0,24,l2);u8g2.drawHLine(0,27,128);
    for(uint8_t i=0;i<2;i++){uint8_t y=40+i*13;
      if(i==(uint8_t)menuCursor){u8g2.drawBox(0,y-10,128,13);u8g2.setDrawColor(0);}
      u8g2.drawStr(4,y,opts[i]);u8g2.setDrawColor(1);}
  }while(u8g2.nextPage());
}
void drawVEd(){
  char vb[12];ltoa(ved.val,vb,10);
  u8g2.firstPage();do{
    u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(0,10,ved.lbl);u8g2.drawHLine(0,12,128);
    u8g2.setFont(u8g2_font_10x20_tf);u8g2.drawStr((128-u8g2.getStrWidth(vb))/2,45,vb);
    u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(0,62,"Turn=adjust  ENC=OK");
  }while(u8g2.nextPage());
}
void drawRun(){
  char prog[14],pb[10];
  if(activeSlot==4)      strncpy(prog,"Full Loop",sizeof(prog));
  else if(activeSlot==5) strncpy(prog,"To Endpt", sizeof(prog));
  else if(activeSlot>=0) snprintf(prog,sizeof(prog),"Slot %d",activeSlot+1);
  else                   strncpy(prog,"None",     sizeof(prog));
  dtostrf((float)stepper.currentPosition()/STEPS_PER_MM,5,1,pb);
  u8g2.firstPage();do{
    u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(0,10,"** RUNNING **");u8g2.drawHLine(0,12,128);
    u8g2.drawStr(0,26,"Prog:");u8g2.drawStr(36,26,prog);
    u8g2.drawStr(0,40,"Pos: ");u8g2.drawStr(36,40,pb);u8g2.drawStr(90,40,"mm");
    u8g2.drawStr(0,54,wifiIP.c_str());
  }while(u8g2.nextPage());
}
void drawPause(){
  char pb[10];dtostrf((float)stepper.currentPosition()/STEPS_PER_MM,5,1,pb);
  const char*opts[]={"Resume","Cancel Program"};
  u8g2.firstPage();do{
    u8g2.setFont(u8g2_font_9x18B_tf);u8g2.drawStr(28,18,"PAUSED");
    u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(0,30,"Pos:");u8g2.drawStr(28,30,pb);u8g2.drawStr(76,30,"mm");
    u8g2.drawHLine(0,32,128);
    for(uint8_t i=0;i<2;i++){uint8_t y=44+i*13;
      if(i==(uint8_t)pauseCursor){u8g2.drawBox(0,y-10,128,13);u8g2.setDrawColor(0);}
      u8g2.drawStr(4,y,opts[i]);u8g2.setDrawColor(1);}
  }while(u8g2.nextPage());
}
void drawEstop(){
  u8g2.firstPage();do{
    u8g2.setFont(u8g2_font_9x18B_tf);u8g2.drawStr(14,22,"! E-STOP !");
    u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(4,38,"Motors disabled");u8g2.drawStr(4,52,"ENC=Main Menu");
  }while(u8g2.nextPage());
}
void drawHomeReq(){
  const char*opts[]={"Yes — home now","No  — cancel"};
  u8g2.firstPage();do{
    u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(0,10,"Home required");u8g2.drawHLine(0,12,128);
    u8g2.drawStr(0,24,"Not homed.");u8g2.drawStr(0,34,"Home before running?");u8g2.drawHLine(0,37,128);
    for(uint8_t i=0;i<2;i++){uint8_t y=50+i*13;
      if(i==(uint8_t)menuCursor){u8g2.drawBox(0,y-10,128,13);u8g2.setDrawColor(0);}
      u8g2.drawStr(4,y,opts[i]);u8g2.setDrawColor(1);}
  }while(u8g2.nextPage());
}
void drawWifi(){
  u8g2.firstPage();do{
    u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(0,10,"WiFi Status");u8g2.drawHLine(0,12,128);
    const char*s=(wifiState==WIFI_STATION)?"Connected":(wifiState==WIFI_AP)?"Access Point":"Portal";
    u8g2.drawStr(0,26,s);u8g2.drawStr(0,40,wifiIP.c_str());
    for(uint8_t i=0;i<wifiCount;i++){uint8_t y=50+i*13;
      if(i==(uint8_t)menuCursor){u8g2.drawBox(0,y-10,128,13);u8g2.setDrawColor(0);}
      u8g2.drawStr(2,y,wifiItems[i]);u8g2.setDrawColor(1);}
  }while(u8g2.nextPage());
}
void drawRoot(){
  char sb[22];
  snprintf(sb,sizeof(sb),"Pgm:%s %s",
    activeSlot==4?"FL":activeSlot==5?"EP":activeSlot>=0?(char[4]{"S"+(char)('1'+activeSlot)}):"No",
    isHomed?"H":"!");
  u8g2.firstPage();do{
    u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(0,10,"Main Menu");u8g2.drawHLine(0,12,128);
    for(uint8_t i=0;i<3;i++){
      uint8_t idx=menuScroll+i;if(idx>=rootCount) break;
      uint8_t y=22+i*13;
      if(idx==(uint8_t)menuCursor){u8g2.drawBox(0,y-10,128,13);u8g2.setDrawColor(0);}
      u8g2.drawStr(2,y,rootItems[idx]);u8g2.setDrawColor(1);
    }
    u8g2.drawHLine(0,53,128);u8g2.setFont(u8g2_font_5x7_tf);u8g2.drawStr(2,63,sb);
  }while(u8g2.nextPage());
}

void labelSelProg(uint8_t i,char*b,uint8_t l){
  if(i==0) strncpy(b,"Full Loop",l);
  else if(i==1) strncpy(b,"Move to Endpt",l);
  else if(i==6) strncpy(b,"< Back",l);
  else{char s[16];slotSum(i-2,s,sizeof(s));snprintf(b,l,"S%d:%s",i-1,s);}
}
void labelClrProg(uint8_t i,char*b,uint8_t l){
  if(i==4) strncpy(b,"Clear All",l);
  else if(i==5) strncpy(b,"< Back",l);
  else{char s[16];slotSum(i,s,sizeof(s));snprintf(b,l,"S%d:%s",i+1,s);}
}

void drawAll(){
  switch(currentMenu){
    case MNU_ROOT:        drawRoot(); break;
    case MNU_SEL_PROG:    drawDynMenu("Select Program",7,labelSelProg); break;
    case MNU_CLR_PROG:    drawDynMenu("Clear Program",6,labelClrProg); break;
    case MNU_FULL_LOOP:   drawMenu(flItems,flCount,"Full Loop"); break;
    case MNU_MOTOR_CTRL:  drawMenu(mCtrlItems,mCtrlCount,"Motor Control"); break;
    case MNU_PROG_MOV:    drawMenu(pmItems,pmCount,"Program Movement"); break;
    case MNU_PROG_SLOT:   drawMenu(psItems,psCount,"Select Slot"); break;
    case MNU_CONF_CLEAR:{char l1[20],l2[20];snprintf(l1,20,"Clear Slot %d?",clearTarget+1);slotSum(clearTarget,l2,20);drawConf(l1,l2);break;}
    case MNU_CONF_CLEARALL: drawConf("Clear ALL slots?","Cannot be undone"); break;
    case MNU_CONF_DISABLE:  drawConf("Disable motors?","Carriage may move"); break;
    case MNU_EDIT_VAL:    drawVEd(); break;
    case MNU_ADD_TIMER:   break;
    case MNU_RUN:         drawRun(); break;
    case MNU_PAUSE:       drawPause(); break;
    case MNU_ESTOP:       drawEstop(); break;
    case MNU_HOME_REQ:    drawHomeReq(); break;
    case MNU_WIFI:        drawWifi(); break;
    case MNU_WIFI_RESET:  drawConf("Reset WiFi?","Will restart"); break;
  }
}

// ─────────────────────────────────────────────
//  SPLASH
// ─────────────────────────────────────────────
void drawSplashFrame(int cx){
  u8g2.firstPage();do{
    u8g2.setFont(u8g2_font_9x18B_tf);
    uint8_t tw=u8g2.getStrWidth("Camera Slider");u8g2.drawStr((128-tw)/2,18,"Camera Slider");
    u8g2.setFont(u8g2_font_6x10_tf);
    uint8_t vw=u8g2.getStrWidth(FIRMWARE_VER);u8g2.drawStr((128-vw)/2,30,FIRMWARE_VER);
    u8g2.drawCircle(4,42,3,U8G2_DRAW_ALL);u8g2.drawCircle(124,42,3,U8G2_DRAW_ALL);
    u8g2.drawBox(4,40,120,4);u8g2.setDrawColor(0);u8g2.drawHLine(4,40,120);u8g2.setDrawColor(1);
    u8g2.drawRBox(cx,32,14,10,2);u8g2.setDrawColor(0);u8g2.drawHLine(cx+2,34,10);u8g2.setDrawColor(1);
  }while(u8g2.nextPage());
}
void drawSplash(){
  unsigned long t=millis(); int tr=110-14;
  while(millis()-t<3000){float p=(float)(millis()-t)/3000;float e=p*p*(3-2*p);drawSplashFrame(4+(int)(e*tr));}
  drawSplashFrame(4+tr);delay(300);
}

// ─────────────────────────────────────────────
//  ACTIONS
// ─────────────────────────────────────────────
void doHome(){
  u8g2.firstPage();do{u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(24,32,"Homing...");}while(u8g2.nextPage());
  homeMotor();
}
void doSave(){
  if(wipSlot<0||!wipHasStart||!wipHasEnd){
    u8g2.firstPage();do{u8g2.setFont(u8g2_font_6x10_tf);u8g2.drawStr(0,16,"Cannot save:");
      if(!wipHasStart)u8g2.drawStr(0,32,"No start set");
      if(!wipHasEnd)  u8g2.drawStr(0,46,"No end set");}while(u8g2.nextPage());
    delay(1600);return;
  }
  wip.valid=true;slots[wipSlot]=wip;saveSlot(wipSlot);
}
void doClear(int i){
  if(i<0){for(int j=0;j<NUM_SLOTS;j++){slots[j].valid=false;saveSlot(j);}}
  else{slots[i].valid=false;saveSlot(i);}
}

// ─────────────────────────────────────────────
//  MENU INPUT
// ─────────────────────────────────────────────
void handleMenu(int d,bool sel){
  switch(currentMenu){
    case MNU_ROOT:
      scrollMenu(d,rootCount);
      if(sel) switch(menuCursor){
        case 0:doHome();break; case 1:programActive?stopHard():startProg();break;
        case 2:openMenu(MNU_SEL_PROG);break; case 3:openMenu(MNU_CLR_PROG);break;
        case 4:openMenu(MNU_FULL_LOOP);break; case 5:openMenu(MNU_MOTOR_CTRL);break;
        case 6:openMenu(MNU_PROG_MOV);break; case 7:openMenu(MNU_WIFI);break;
      } break;
    case MNU_SEL_PROG:
      scrollMenu(d,7);
      if(sel){if(menuCursor==6){openMenu(MNU_ROOT);break;}
        activeSlot=(menuCursor<=1)?menuCursor+4:menuCursor-2;openMenu(MNU_ROOT);}break;
    case MNU_CLR_PROG:
      scrollMenu(d,6);
      if(sel){if(menuCursor==5){openMenu(MNU_ROOT);break;}
        clearTarget=(menuCursor==4)?-1:menuCursor;
        openMenu(menuCursor==4?MNU_CONF_CLEARALL:MNU_CONF_CLEAR);}break;
    case MNU_CONF_CLEAR:
      scrollMenu(d,2);if(sel){if(menuCursor==0)doClear(clearTarget);openMenu(MNU_CLR_PROG);}break;
    case MNU_CONF_CLEARALL:
      scrollMenu(d,2);if(sel){if(menuCursor==0)doClear(-1);openMenu(MNU_CLR_PROG);}break;
    case MNU_FULL_LOOP:
      scrollMenu(d,flCount);
      if(sel) switch(menuCursor){
        case 0:openVEd("FL Speed(mm/s)",fullLoop.speedMms,1,100,1,&fullLoop.speedMms,MNU_FULL_LOOP);break;
        case 1:openVEd("Loop Count(0=cont)",fullLoop.loopCount,0,9999,1,&fullLoop.loopCount,MNU_FULL_LOOP);break;
        case 2:fullLoop.speedMms=DEFAULT_SPEED_MMS;fullLoop.loopCount=DEFAULT_LOOP_COUNT;saveFL();break;
        case 3:openMenu(MNU_ROOT);break;
      } break;
    case MNU_MOTOR_CTRL:
      scrollMenu(d,mCtrlCount);
      if(sel) switch(menuCursor){
        case 0:doHome();break; case 1:openMenu(MNU_CONF_DISABLE);break; case 2:openMenu(MNU_ROOT);break;
      } break;
    case MNU_CONF_DISABLE:
      scrollMenu(d,2);if(sel){if(menuCursor==0){decel();motorRunning=false;}openMenu(MNU_MOTOR_CTRL);}break;
    case MNU_PROG_MOV:
      scrollMenu(d,pmCount);
      if(sel) switch(menuCursor){
        case 0:openMenu(MNU_PROG_SLOT);break; case 1:doHome();break;
        case 2:openVEd("Prog Speed(mm/s)",wip.speedMms?wip.speedMms:DEFAULT_SPEED_MMS,1,100,1,&wip.speedMms,MNU_PROG_MOV);break;
        case 3:wip.startPos=stepper.currentPosition();wipHasStart=true;break;
        case 4:if(wip.stopCount<MAX_STOPS){wip.stops[wip.stopCount].position=stepper.currentPosition();wip.stops[wip.stopCount].timerSec=0;openVEd("Stop Timer(sec)",0,0,3600,1,&wip.stops[wip.stopCount].timerSec,MNU_ADD_TIMER);}break;
        case 5:wip.endPos=stepper.currentPosition();wipHasEnd=true;break;
        case 6:openVEd("Loop Count(0=cont)",wip.loopCount,0,9999,1,&wip.loopCount,MNU_PROG_MOV);break;
        case 7:doSave();openMenu(MNU_ROOT);break; case 8:openMenu(MNU_ROOT);break;
      } break;
    case MNU_PROG_SLOT:
      scrollMenu(d,psCount);
      if(sel){if(menuCursor==4){openMenu(MNU_PROG_MOV);break;}
        wipSlot=menuCursor;
        if(slots[wipSlot].valid){wip=slots[wipSlot];wipHasStart=wipHasEnd=true;}
        else{memset(&wip,0,sizeof(wip));wipHasStart=wipHasEnd=false;}
        openMenu(MNU_PROG_MOV);}break;
    case MNU_ADD_TIMER: wip.stopCount++;openMenu(MNU_PROG_MOV);break;
    case MNU_EDIT_VAL:
      if(d){ved.val+=d*ved.stp;if(ved.val<ved.mn)ved.val=ved.mn;if(ved.val>ved.mx)ved.val=ved.mx;}
      if(sel){
        if(ved.mx<=65535&&ved.mn>=0)*((uint16_t*)ved.tgt)=(uint16_t)ved.val;
        else *((long*)ved.tgt)=ved.val;
        if(ved.done==MNU_FULL_LOOP) saveFL();
        openMenu(ved.done==MNU_ADD_TIMER?MNU_ADD_TIMER:ved.done);
      } break;
    case MNU_RUN: break;
    case MNU_PAUSE:
      if(d) pauseCursor=(pauseCursor==0)?1:0;
      if(sel){if(pauseCursor==0)resumeProg();else cancelProg();}break;
    case MNU_ESTOP:
      if(sel){setBacklight(NEO_IDLE);openMenu(MNU_ROOT);}break;
    case MNU_HOME_REQ:
      scrollMenu(d,2);
      if(sel){if(menuCursor==0){doHome();if(isHomed)launchProg();else openMenu(MNU_ROOT);}else openMenu(MNU_ROOT);}break;
    case MNU_WIFI:
      scrollMenu(d,wifiCount);
      if(sel) switch(menuCursor){case 0:openMenu(MNU_WIFI_RESET);break;case 1:openMenu(MNU_ROOT);break;}break;
    case MNU_WIFI_RESET:
      scrollMenu(d,2);
      if(sel){if(menuCursor==0){clearWifi();delay(300);ESP.restart();}else openMenu(MNU_WIFI);}break;
    default: openMenu(MNU_ROOT);break;
  }
}

// ─────────────────────────────────────────────
//  WEB COMMAND HANDLER
// ─────────────────────────────────────────────
void handleWebCmd(WebCmd cmd){
  switch(cmd.type){
    case WebCmd::HOME:     doHome(); break;
    case WebCmd::START:    if(!programActive) startProg(); break;
    case WebCmd::STOP:     if(programActive){decel();stopHard();} break;
    case WebCmd::PAUSE:    if(programActive&&runState!=RUN_PAUSED) pauseProg(); break;
    case WebCmd::RESUME:   if(runState==RUN_PAUSED) resumeProg(); break;
    case WebCmd::JOG_L:    jogLeft(cmd.p1>0); break;
    case WebCmd::JOG_R:    jogRight(cmd.p1>0); break;
    case WebCmd::JOG_STOP: if(motorRunning&&!programActive){stepper.stop();motorRunning=false;} break;
    case WebCmd::SEL_SLOT: activeSlot=(int)cmd.p1; break;
    case WebCmd::FL_SPEED: fullLoop.speedMms=(uint16_t)cmd.p1; saveFL(); break;
    case WebCmd::FL_LOOPS: fullLoop.loopCount=(uint16_t)cmd.p1; saveFL(); break;
    case WebCmd::ESTOP:    handleEstop(); break;
    default: break;
  }
}

// ─────────────────────────────────────────────
//  REST API STATUS
// ─────────────────────────────────────────────
String statusJson(){
  StaticJsonDocument<512> doc;
  doc["pos_mm"]        =(float)stepper.currentPosition()/STEPS_PER_MM;
  doc["homed"]         =isHomed;
  doc["running"]       =motorRunning;
  doc["paused"]        =(runState==RUN_PAUSED);
  doc["program_active"]=programActive;
  doc["active_slot"]   =activeSlot;
  doc["loop"]          =currentLoop;
  const char*st="idle";
  switch(runState){
    case RUN_MOVING_TO_START:st="to_start";break; case RUN_MOVING_TO_STOP:st="to_stop";break;
    case RUN_DWELLING:st="dwelling";break;         case RUN_MOVING_TO_END:st="to_end";break;
    case RUN_RETURNING:st="returning";break;       case RUN_PAUSED:st="paused";break;
    case RUN_FL_FORWARD:st="fl_fwd";break;         case RUN_FL_BACKWARD:st="fl_rev";break;
    default:break;
  }
  doc["state"]    =st;
  doc["wifi_ip"]  =wifiIP;
  doc["wifi_mode"]=(wifiState==WIFI_STATION)?"station":"ap";
  String o; serializeJson(doc,o); return o;
}

// ─────────────────────────────────────────────
//  HTML (stored in flash)
// ─────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Camera Slider</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0f172a;color:#c8e6c9;font-family:Arial,sans-serif;padding:16px}
h1{color:#90caf9;font-size:20px;margin-bottom:16px}
h2{color:#90caf9;font-size:13px;margin:14px 0 6px}
.card{background:#1e293b;border-radius:8px;padding:14px;margin-bottom:10px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.stat{background:#0f172a;border-radius:6px;padding:8px}
.sl{color:#4a7c59;font-size:10px}.sv{color:#c8e6c9;font-size:15px;font-weight:bold}
.row{display:flex;flex-wrap:wrap;gap:6px;margin-top:6px}
button{border:none;border-radius:6px;padding:9px 12px;cursor:pointer;font-size:12px;font-weight:bold;color:#fff;flex:1;min-width:70px}
.g{background:#1b5e20}.b{background:#0d47a1}.a{background:#e65100}.r{background:#b71c1c}.gr{background:#374151}
select,input[type=text],input[type=password],input[type=number]{background:#0f172a;color:#c8e6c9;border:1px solid #374151;border-radius:6px;padding:8px;width:100%;margin-top:4px;font-size:13px}
#pb{height:6px;background:#1e293b;border-radius:3px;margin-top:6px;overflow:hidden}
#pf{height:100%;background:#0d47a1;border-radius:3px;transition:width .3s;width:0}
</style></head><body>
<h1>&#127909; Camera Slider</h1>
<div class="card">
  <div class="grid">
    <div class="stat"><div class="sl">Position</div><div class="sv" id="pos">--</div></div>
    <div class="stat"><div class="sl">State</div><div class="sv" id="st">--</div></div>
    <div class="stat"><div class="sl">Program</div><div class="sv" id="pg">--</div></div>
    <div class="stat"><div class="sl">Loop</div><div class="sv" id="lp">--</div></div>
  </div>
  <div id="pb"><div id="pf"></div></div>
</div>
<div class="card"><h2>Run Control</h2><div class="row">
  <button class="g" onclick="c('home')">&#8962; Home</button>
  <button class="b" id="bs" onclick="c('start')">&#9654; Start</button>
  <button class="a" id="bp" onclick="c('pause')">&#9646;&#9646; Pause</button>
  <button class="r" onclick="c('estop')">&#9940; E-Stop</button>
</div></div>
<div class="card"><h2>Program</h2>
  <select id="ps" onchange="sp(this.value)">
    <option value="4">Full Loop</option><option value="5">Move to Endpoint</option>
    <option value="0">Slot 1</option><option value="1">Slot 2</option>
    <option value="2">Slot 3</option><option value="3">Slot 4</option>
  </select>
</div>
<div class="card"><h2>Jog</h2><div class="row">
  <button class="gr" onmousedown="js('left',0)" ontouchstart="js('left',0)" onmouseup="jx()" ontouchend="jx()">&#9664; Step L</button>
  <button class="gr" onmousedown="js('left',1)" ontouchstart="js('left',1)" onmouseup="jx()" ontouchend="jx()">&#9668;&#9668; Hold L</button>
  <button class="gr" onmousedown="js('right',1)" ontouchstart="js('right',1)" onmouseup="jx()" ontouchend="jx()">&#9658;&#9658; Hold R</button>
  <button class="gr" onmousedown="js('right',0)" ontouchstart="js('right',0)" onmouseup="jx()" ontouchend="jx()">Step R &#9654;</button>
</div></div>
<div class="card"><h2>Full Loop Settings</h2>
  Speed (mm/s):<input type="number" id="fls" min="1" max="100" value="10">
  Loop Count (0=&#8734;):<input type="number" id="fll" min="0" max="9999" value="0">
  <div class="row" style="margin-top:8px"><button class="gr" onclick="sfl()">Save</button></div>
</div>
<div class="card"><h2>WiFi Settings</h2>
  SSID:<input type="text" id="ws" placeholder="Network name">
  Password:<input type="password" id="wp" placeholder="Password">
  <div class="row" style="margin-top:8px"><button class="gr" onclick="swf()">Save &amp; Restart</button></div>
</div>
<script>
const pn={'-1':'None','0':'Slot 1','1':'Slot 2','2':'Slot 3','3':'Slot 4','4':'Full Loop','5':'To Endpt'};
const sm={idle:'Idle',to_start:'To Start',to_stop:'To Stop',dwelling:'Dwelling',to_end:'To End',returning:'Return',paused:'Paused',fl_fwd:'Forward',fl_rev:'Reverse'};
let jt=null;
function c(a){fetch('/api/'+a,{method:'POST'}).then(poll);}
function sp(s){fetch('/api/select',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({slot:+s})});}
function js(d,f){fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({dir:d,fast:f})});if(f)jt=setInterval(()=>fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({dir:d,fast:1})}),200);}
function jx(){clearInterval(jt);fetch('/api/jog/stop',{method:'POST'});}
function sfl(){fetch('/api/fullloop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({speed:+document.getElementById('fls').value,loops:+document.getElementById('fll').value})});}
function swf(){fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid:document.getElementById('ws').value,pass:document.getElementById('wp').value})});}
function poll(){fetch('/api/status').then(r=>r.json()).then(d=>{
  document.getElementById('pos').textContent=d.pos_mm.toFixed(1)+' mm';
  document.getElementById('st').textContent=sm[d.state]||d.state;
  document.getElementById('pg').textContent=pn[String(d.active_slot)]||'None';
  document.getElementById('lp').textContent=d.loop;
  document.getElementById('pf').style.width=(d.pos_mm/200*100).toFixed(1)+'%';
  document.getElementById('ps').value=String(d.active_slot);
  document.getElementById('bs').innerHTML=d.running?'&#9646;&#9646; Stop':'&#9654; Start';
  document.getElementById('bp').innerHTML=d.paused?'&#9654; Resume':'&#9646;&#9646; Pause';
}).catch(()=>{});}
setInterval(poll,500);poll();
</script></body></html>)HTML";

const char PORTAL_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Camera Slider — WiFi Setup</title>
<style>body{background:#0f172a;color:#c8e6c9;font-family:Arial;padding:24px;max-width:400px;margin:0 auto}
h1{color:#90caf9;margin-bottom:20px}input{background:#1e293b;color:#c8e6c9;border:1px solid #374151;border-radius:6px;padding:10px;width:100%;margin:6px 0 14px;font-size:14px}
button{background:#0d47a1;color:#fff;border:none;border-radius:6px;padding:12px;width:100%;font-size:15px;cursor:pointer}
label{color:#4a7c59;font-size:12px}#m{margin-top:14px;color:#66bb6a}</style></head>
<body><h1>&#127909; Camera Slider WiFi Setup</h1>
<p style="color:#888;margin-bottom:18px">Enter your WiFi details to connect to your network.</p>
<label>Network Name (SSID)</label><input type="text" id="s" placeholder="WiFi network name">
<label>Password</label><input type="password" id="p" placeholder="WiFi password">
<button onclick="save()">Connect &amp; Save</button><p id="m"></p>
<script>function save(){fetch('/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid:document.getElementById('s').value,pass:document.getElementById('p').value})}).then(()=>{document.getElementById('m').textContent='Saved! Device restarting...';});}</script>
</body></html>)HTML";

// ─────────────────────────────────────────────
//  WEB SERVER
// ─────────────────────────────────────────────
void setupWeb(){
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin","*");

  webServer.on("/setup",HTTP_GET,[](AsyncWebServerRequest*r){r->send(200,"text/html",PORTAL_HTML);});
  webServer.on("/save",HTTP_POST,[](AsyncWebServerRequest*r){},nullptr,
    [](AsyncWebServerRequest*r,uint8_t*d,size_t l,size_t,size_t){
      StaticJsonDocument<256>doc; if(!deserializeJson(doc,d,l)){
        String s=doc["ssid"].as<String>(),p=doc["pass"].as<String>();
        if(s.length()>0){saveWifi(s,p);r->send(200,"application/json","{\"ok\":true}");delay(400);ESP.restart();return;}
      }r->send(400,"application/json","{\"error\":\"invalid\"}");
  });

  webServer.on("/",HTTP_GET,[](AsyncWebServerRequest*r){r->send(200,"text/html",INDEX_HTML);});
  webServer.onNotFound([](AsyncWebServerRequest*r){r->redirect("http://192.168.4.1/setup");});

  webServer.on("/api/status",HTTP_GET,[](AsyncWebServerRequest*r){r->send(200,"application/json",statusJson());});

  auto pc=[](WebCmd::Type t){return[t](AsyncWebServerRequest*r){postCmd(t);r->send(200,"application/json","{\"ok\":true}");};};
  webServer.on("/api/home",   HTTP_POST,pc(WebCmd::HOME));
  webServer.on("/api/start",  HTTP_POST,pc(WebCmd::START));
  webServer.on("/api/stop",   HTTP_POST,pc(WebCmd::STOP));
  webServer.on("/api/pause",  HTTP_POST,pc(WebCmd::PAUSE));
  webServer.on("/api/resume", HTTP_POST,pc(WebCmd::RESUME));
  webServer.on("/api/estop",  HTTP_POST,pc(WebCmd::ESTOP));
  webServer.on("/api/jog/stop",HTTP_POST,pc(WebCmd::JOG_STOP));

  webServer.on("/api/jog",HTTP_POST,[](AsyncWebServerRequest*r){},nullptr,
    [](AsyncWebServerRequest*r,uint8_t*d,size_t l,size_t,size_t){
      StaticJsonDocument<128>doc;if(!deserializeJson(doc,d,l)){
        String dir=doc["dir"]|"left";int fast=doc["fast"]|0;
        postCmd(dir=="right"?WebCmd::JOG_R:WebCmd::JOG_L,fast);}
      r->send(200,"application/json","{\"ok\":true}");
  });
  webServer.on("/api/select",HTTP_POST,[](AsyncWebServerRequest*r){},nullptr,
    [](AsyncWebServerRequest*r,uint8_t*d,size_t l,size_t,size_t){
      StaticJsonDocument<64>doc;if(!deserializeJson(doc,d,l)) postCmd(WebCmd::SEL_SLOT,doc["slot"]|-1);
      r->send(200,"application/json","{\"ok\":true}");
  });
  webServer.on("/api/fullloop",HTTP_POST,[](AsyncWebServerRequest*r){},nullptr,
    [](AsyncWebServerRequest*r,uint8_t*d,size_t l,size_t,size_t){
      StaticJsonDocument<128>doc;if(!deserializeJson(doc,d,l)){
        postCmd(WebCmd::FL_SPEED,doc["speed"]|DEFAULT_SPEED_MMS);
        postCmd(WebCmd::FL_LOOPS,doc["loops"]|DEFAULT_LOOP_COUNT);}
      r->send(200,"application/json","{\"ok\":true}");
  });
  webServer.on("/api/wifi",HTTP_POST,[](AsyncWebServerRequest*r){},nullptr,
    [](AsyncWebServerRequest*r,uint8_t*d,size_t l,size_t,size_t){
      StaticJsonDocument<256>doc;if(!deserializeJson(doc,d,l)){
        saveWifi(doc["ssid"]|"",doc["pass"]|"");
        r->send(200,"application/json","{\"ok\":true}");delay(400);ESP.restart();return;}
      r->send(400,"application/json","{\"error\":\"invalid\"}");
  });

  // GET /api/programs — returns all slot summaries + full loop settings
  webServer.on("/api/programs",HTTP_GET,[](AsyncWebServerRequest*r){
    StaticJsonDocument<1024> doc;
    JsonObject fl = doc.createNestedObject("full_loop");
    fl["speed_mms"]  = fullLoop.speedMms;
    fl["loop_count"] = fullLoop.loopCount;
    JsonArray arr = doc.createNestedArray("slots");
    for(int i=0;i<NUM_SLOTS;i++){
      JsonObject s=arr.createNestedObject();
      s["slot"]       = i;
      s["valid"]      = slots[i].valid;
      s["speed_mms"]  = slots[i].speedMms;
      s["loop_count"] = slots[i].loopCount;
      s["stop_count"] = slots[i].stopCount;
      uint16_t totalSec=0;
      for(uint8_t j=0;j<slots[i].stopCount;j++) totalSec+=slots[i].stops[j].timerSec;
      s["total_dwell_sec"] = totalSec;
      s["start_mm"] = (float)slots[i].startPos / STEPS_PER_MM;
      s["end_mm"]   = (float)slots[i].endPos   / STEPS_PER_MM;
    }
    String out; serializeJson(doc,out);
    r->send(200,"application/json",out);
  });

  webServer.begin();
}

// ─────────────────────────────────────────────
//  WIFI TASK  (Core 0)
// ─────────────────────────────────────────────
void wifiTask(void*){
  String ssid,pass; bool got=loadWifi(ssid,pass);
  if(got){
    WiFi.mode(WIFI_STA); WiFi.begin(ssid.c_str(),pass.c_str());
    unsigned long t=millis();
    while(WiFi.status()!=WL_CONNECTED&&millis()-t<WIFI_TIMEOUT) vTaskDelay(200/portTICK_PERIOD_MS);
  }
  if(WiFi.status()==WL_CONNECTED){
    wifiState=WIFI_STATION; wifiIP=WiFi.localIP().toString(); setupWeb();
  } else {
    WiFi.mode(WIFI_AP); WiFi.softAPConfig(AP_IP,AP_IP,IPAddress(255,255,255,0));
    WiFi.softAP(AP_SSID,AP_PASSWORD);
    wifiIP=AP_IP.toString(); wifiState=got?WIFI_AP:WIFI_PORTAL;
    portalActive=true; dnsServer.start(DNS_PORT,"*",AP_IP); setupWeb();
  }
  while(true){if(portalActive) dnsServer.processNextRequest();vTaskDelay(10/portTICK_PERIOD_MS);}
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup(){
  Serial.begin(115200);
  // Input-only pins (no internal pullup) — need external 10k to 3.3V
  pinMode(ENC_CLK, INPUT); pinMode(ENC_DT,   INPUT);
  pinMode(END_LEFT,INPUT); pinMode(END_RIGHT, INPUT);
  // Normal inputs
  pinMode(ENC_SW,   INPUT_PULLUP); pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT,INPUT_PULLUP); pinMode(BTN_PLAY, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_CLK),encoderISR,CHANGE);

  stepper.setMaxSpeed(JOG_FAST_SPEED);
  stepper.setAcceleration(JOG_FAST_SPEED*2);

  neo.begin(); neo.setBrightness(80); setBacklight(NEO_WIFI);

  loadAll();

  delay(100); u8g2.begin(); u8g2.setContrast(200); drawSplash();

  xTaskCreatePinnedToCore(wifiTask,"WiFi",8192,nullptr,1,nullptr,0);
  delay(500); setBacklight(NEO_IDLE);
  openMenu(MNU_ROOT);
}

// ─────────────────────────────────────────────
//  LOOP  (Core 1)
// ─────────────────────────────────────────────
void loop(){
  int delta=0; noInterrupts();delta=encDelta;encDelta=0;interrupts();
  updateAllBtns();

  // Encoder long press = E-Stop
  if(btnEnc.held&&!encLongFired){handleEstop();drawAll();return;}
  if(!btnEnc.held&&digitalRead(ENC_SW)==HIGH) encLongFired=false;

  // Web commands
  WebCmd wc=consumeCmd(); if(wc.type!=WebCmd::NONE) handleWebCmd(wc);

  // PLAY button
  if(currentMenu==MNU_PROG_MOV){
    if(btnPlay.pressed&&wip.stopCount<MAX_STOPS){
      wip.stops[wip.stopCount].position=stepper.currentPosition();
      wip.stops[wip.stopCount].timerSec=0;
      openVEd("Stop Timer(sec)",0,0,3600,1,&wip.stops[wip.stopCount].timerSec,MNU_ADD_TIMER);
    }
  } else if(btnPlay.held&&programActive&&runState==RUN_PAUSED){
    cancelProg();
  } else if(btnPlay.pressed){
    if(programActive&&runState!=RUN_PAUSED) pauseProg();
    else if(runState==RUN_PAUSED)           resumeProg();
    else                                     startProg();
  }

  handleJog();

  bool sel=btnEnc.pressed;
  if(delta||sel) handleMenu(delta,sel);

  if(programActive&&runState!=RUN_PAUSED){tickProg();return;}
  drawAll();
}
