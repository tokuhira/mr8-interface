
//
//  @file RC_Interface_for_MR-8.ino
//  @brief RC Interface for MR-8 (MUX/S.BUS/4WS)
//  @author tokuhira
//  @date 2021/09/29

//  QT Py     Device         　
//   0 (A0)   <=  MR8(A/E ch) via 74HC32
//   1 (A1)   <=  MR8(B/F ch) via 74HC32
//   2 (A2)   <=  MR8(C/G ch) via 74HC32
//   3 (A3)   <=  MR8(D/H ch) via 74HC32
//   4 (SDA)  =>  Controll Lost (Link down flag)
//   5 (SCL)  =>  Controll External to 74HC4053 (Pull-down)
//   6 (TX)   =>  S.BUS to host (Serial1 at QT Py)
//   7 (RX)   <=  Command from host (Serial1 at QT Py)
//   8 (SCK)  =>  MR8 D ch Steering PWM Reversed for 4WS
//   9 (MISO) =>  External Steering PWM Reversed for 4WS
//  10 (MOSI) <=  External Steering PWM from host

#include <Servo.h>
#include <Streaming.h>
#include <FUTABA_SBUS.h> // https://github.com/mikeshub/FUTABA_SBUS

Servo ExStrRev;
Servo MrStrRev;

FUTABA_SBUS sBus;

const int KO_PWM_CENTER = 1496;
const int KO_PWM_MIN = 890;
const int KO_PWM_MAX = 2100;

const int JR_PWM_CENTER = 1500;
const int JR_PWM_MIN = 900;
const int JR_PWM_MAX = 2100;

const int FUTABA_PWM_CENTER = 1520;
const int FUTABA_PWM_MIN = 1020;
const int FUTABA_PWM_MAX = 2020;

const unsigned long LINK_LIMIT0 = 15000 * 30; // waiting 0.45 sec as 30 cycles at 15 msec period (66.6 Hz)
const unsigned long LINK_LIMIT1 = 15000 * 20; // waiting 0.30 sec as 20 cycles at 15 msec period (66.6 Hz)
const unsigned long SHIFT_LIMIT = 2100 + 900; // threshold between A-D and E-H
const unsigned long PRESS_LIMIT = 1745; // neutral = 1496, pressed = 1995
const unsigned long WIDTH_MINIMUM = 900 - 100; // minimum width: 890 usec. with margin
const unsigned long WIDTH_MAXIMUM = 2100 + 100; // maximum width: 2100 usec. with margin

// const int DEBOUNCE_WAIT_COUNT = 3; // pressing E and/or F are valid when they hosd the cycles

const int MR8_AE_PIN = 0;
const int MR8_BF_PIN = 1;
const int MR8_CG_PIN = 2;
const int MR8_DH_PIN = 3;

const int CON_LST_PIN = 4;
const int CON_EXT_PIN = 5;

const int EX_STR_PIN = 10;
const int EX_STR_REV_PIN = 9;
const int MR_STR_REV_PIN = 8;

volatile unsigned long startPulse; // dual use for ABCD and EFGH
volatile unsigned long startPulseEx; // for external steering

volatile int *getPulseAE; // switching getPulse[0] or [4]
volatile int *getPulseBF; // switching getPulse[1] or [5]
volatile int *getPulseCG; // switching getPulse[2] or [6]
volatile int *getPulseDH; // switching getPulse[3] or [7]

volatile int getPulse[9]; // A/B/C/D/E/F/G/H/ExStr
volatile int outPulse[2]; // ReverseD/ReverseExStr

volatile unsigned long debounceCount[2];

volatile int mc8Link;
volatile int extMode;

inline bool isValidWidth(unsigned long pulse)
{
  return WIDTH_MINIMUM <= pulse && pulse <= WIDTH_MAXIMUM;
}

void chAEChangeInterupt()
{
  unsigned long nowPulse = micros();
  unsigned long elapsed = nowPulse - startPulse;
  if (digitalRead(MR8_AE_PIN) == HIGH)
  {
    startPulse = nowPulse; // rising event is shared by ch A/B/C/D and E/F/G/H
    if (elapsed >= SHIFT_LIMIT)
    {
      mc8Link = elapsed <= LINK_LIMIT0; // MR-8 2.4 GHz wireless link has been lost
      extMode = elapsed <= LINK_LIMIT1 ? extMode : false; // the link is going to lost
      getPulseAE = getPulse;
      getPulseBF = getPulse + 1;
      getPulseCG = getPulse + 2;
      getPulseDH = getPulse + 3;
    }
    else
    {
      getPulseAE = getPulse + 4;
      getPulseBF = getPulse + 5;
      getPulseCG = getPulse + 6;
      getPulseDH = getPulse + 7;
    }
  }
  else if (isValidWidth(elapsed))
  {
    *getPulseAE = elapsed;
  }
}

void chBFFallingInterupt()
{
  unsigned long nowPulse = micros();
  unsigned long elapsed = nowPulse - startPulse;
  if (isValidWidth(elapsed))
  {
    *getPulseBF = elapsed;
  }
}

void chCGFallingInterupt()
{
  unsigned long nowPulse = micros();
  unsigned long elapsed = nowPulse - startPulse;
  if (isValidWidth(elapsed))
  {
    *getPulseCG = elapsed;
  }
}

void chDHFallingInterupt()
{
  unsigned long nowPulse = micros();
  unsigned long elapsed = nowPulse - startPulse;
  if (isValidWidth(elapsed))
  {
    *getPulseDH = elapsed;
  }
}

void chExStrChangeInterupt()
{
  unsigned long nowPulse = micros();
  if (digitalRead(EX_STR_PIN) == HIGH)
  {
    startPulseEx = nowPulse;
  }
  else
  {
    unsigned long elapsed = nowPulse - startPulseEx;
    if (isValidWidth(elapsed))
    {
      getPulse[8] = elapsed;
    }
  }
}

int mapSteeringReverseKO(int pulse)
{
  if (pulse < KO_PWM_MIN || pulse > KO_PWM_MAX)
  {
    return KO_PWM_CENTER;
  }
  return map(pulse, KO_PWM_MIN, KO_PWM_MAX, KO_PWM_MAX, KO_PWM_MIN);
}

int mapSteeringReverseJR(int pulse)
{
  if (pulse < JR_PWM_MIN || pulse > JR_PWM_MAX)
  {
    return JR_PWM_CENTER;
  }
  return map(pulse, JR_PWM_MIN, JR_PWM_MAX, JR_PWM_MAX, JR_PWM_MIN);
}

int mapSteeringReverseKO2JR(int pulse)
{
  if (pulse < KO_PWM_MIN || pulse > KO_PWM_MAX)
  {
    return JR_PWM_CENTER;
  }
  return map(pulse, KO_PWM_MIN, KO_PWM_MAX, JR_PWM_MAX, JR_PWM_MIN);
}

int mapSteeringReverseFUTABA(int pulse)
{
  if (pulse < FUTABA_PWM_MIN || pulse > FUTABA_PWM_MAX)
  {
    return FUTABA_PWM_CENTER;
  }
  return map(pulse, FUTABA_PWM_MIN, FUTABA_PWM_MAX, FUTABA_PWM_MAX, FUTABA_PWM_MIN);
}

int mapSteeringReverseKO2FUTABA(int pulse)
{
  if (pulse < KO_PWM_MIN || pulse > KO_PWM_MAX)
  {
    return FUTABA_PWM_CENTER;
  }
  return map(pulse, KO_PWM_MIN, KO_PWM_MAX, FUTABA_PWM_MAX, FUTABA_PWM_MIN);
}

void setup() {
  Serial.begin(115200);

  // for PWM read
  pinMode(MR8_AE_PIN, INPUT);
  pinMode(MR8_BF_PIN, INPUT);
  pinMode(MR8_CG_PIN, INPUT);
  pinMode(MR8_DH_PIN, INPUT);
  pinMode(EX_STR_PIN, INPUT);
  noInterrupts();
  attachInterrupt(digitalPinToInterrupt(MR8_AE_PIN), chAEChangeInterupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MR8_BF_PIN), chBFFallingInterupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(MR8_CG_PIN), chCGFallingInterupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(MR8_DH_PIN), chDHFallingInterupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(EX_STR_PIN), chExStrChangeInterupt, CHANGE);
  interrupts();

  // for PWM write
  MrStrRev.attach(MR_STR_REV_PIN);
  ExStrRev.attach(EX_STR_REV_PIN);

  // for multiplexer switch
  pinMode(CON_LST_PIN, OUTPUT);
  pinMode(CON_EXT_PIN, OUTPUT);

  // status control
  debounceCount[0] = 0;
  debounceCount[1] = 0;
  mc8Link = false;
  extMode = false;

  // for S.BUS output using Serial1
  sBus.begin();
}

void loop() {
  // switching multiplexer
  if (getPulse[5] >= PRESS_LIMIT)
  {
    extMode = false;
  }
  else if (getPulse[4] >= PRESS_LIMIT)
  {
    extMode = true;
  }
  digitalWrite(CON_LST_PIN, mc8Link ? LOW : HIGH); // 2.4 GHz link down as high
  digitalWrite(CON_EXT_PIN, extMode ? HIGH : LOW); // external mode active high

  // PWM of steering reversed
  outPulse[0] = mapSteeringReverseKO2FUTABA(getPulse[3]); // MC-8 -> MR-8 -> TSU-06
  MrStrRev.writeMicroseconds(outPulse[0]);
  outPulse[1] = mapSteeringReverseFUTABA(getPulse[8]); // JR Beat2 T/R -> TSU-06
  ExStrRev.writeMicroseconds(outPulse[1]);

  // S.BUS output
  sBus.FeedLine();
  if (sBus.toChannels == 1)
  {
    sBus.UpdateServos();
    sBus.UpdateChannels();
    sBus.toChannels = 0;
    Serial << sBus.channels[0] << "," << sBus.channels[1] << "," << sBus.channels[2] << "\r\n";
  }

#ifdef DEBUG
  // debug print makes PWM poor
  Serial.print(getPulse[0]); // PWM read:  A ch
  Serial.print(",");
  Serial.print(getPulse[1]); // PWM read:  B ch
  Serial.print(",");
  Serial.print(getPulse[2]); // PWM read:  C ch
  Serial.print(",");
  Serial.print(getPulse[3]); // PWM read:  D ch
  Serial.print(",");
  Serial.print(getPulse[4]); // PWM read:  E ch
  Serial.print(",");
  Serial.print(getPulse[5]); // PWM read:  F ch
  Serial.print(",");
  Serial.print(getPulse[6]); // PWM read:  G ch
  Serial.print(",");
  Serial.print(getPulse[7]); // PWM read:  H ch
  Serial.print(",");
  Serial.print(getPulse[8]); // PWM read:  External Steering
  Serial.print(",");
  Serial.print(outPulse[0]); // PWM write: D ch REVERSED
  Serial.print(",");
  Serial.print(outPulse[1]); // PWM write: External Steering REVERSED
  Serial.print(",");
  Serial.print(mc8Link ? 1000 : 0); // State flag: RC links ok?
  Serial.print(",");
  Serial.print(extMode ? 1000 : 0); // State flag: is external?
  Serial.println(" ");
  // delay(5); // slow loop makes PWM poor
#endif

}
