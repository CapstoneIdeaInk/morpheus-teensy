/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  Gradient-based PID control of line-drawing robot Morpheus v0.1    S04 Idea Ink , SUTD Capstone. Idea Ink Illustrations Pte Ltd. 2021
 *      programmers : Caleb Foo (original) ; Marcel Prasetyo (latest editor)
 *  Teensy 4.1
 *  motors used : 
 *      
 *  Tuneable quantities :
 *  
 *  PIDG : Kp , Ki
 *  maxPerpendicularVelocity -> how much to correct back . related to PIDG Kp
 *  defaultCurvilinearSpeed , and when and where to change the curvilinear speed.
 * 
 * 
 * BEGINWITHBASICCALIBRATION
 * MANUALSPEEDCALIBRATION  -> wait for command from WiFi instead of automatically at startup.
 * 
 * 
 * important flags / states :
 *  doingCmd , drawing, Servo postion, desiredHori/VertMM.
 * 
 *  
 */

//#define ENCODER_OPTIMIZE_INTERRUPTS     // increase performance, but may conflict if any other library uses interrupt, such as wifi, SPI.

#include <WiFiNINA.h>
#include <Encoder.h>
#include <PID_v2.h>
#include <Servo.h>
#include <SPI.h>
#include <math.h>

#define MorpheusVersion "Morpheus - Gradient V0.2.4 - Rev. 12 August 2021"

// toggles and flags
#define pumpIntervalTime 180  // 3 minutes // how often to pump while drawing ? only accrues while actually in drawing == true state
#define automaticallyScheduleRecalibrationVerticalHome false  // do you want to automatically schedule vertical home recalibration because of drifting ? this is done every large G0.
#define automaticVertHomeRecalibrationG0Threshold 20  // 20 mm of a G0 jump, then we do vertical calibration. otherwise we do it by timing also.
#define autoRecalibrateHomeInterval 60000   // 60 seconds, 1 minute
#define defaultStartWithPumping true              // pump every time robot starts up ?

#define motorWallSpeed 255
#define motorHoriStart 25
#define motorHoriEnd 45
#define motorHoriDrawingStart 25
#define motorHoriDrawingEnd 255
#define motorVertUpStart 150
#define motorVertUpEnd 255
#define motorVertDownStart 150
#define motorVertDownEnd 255
#define motorVertUpDrawingStart 60
#define motorVertUpDrawingEnd 255
#define motorVertDownDrawingStart 60
#define motorVertDownDrawingEnd 255
#define horiMotorFrequency 20   // pwm frequency
#define vertMotorFrequency 20
#define wallMotorFrequency 20

// some values
#define defaultCheckEndThreshold 2.8
#define defaultMaxPerpendicularVelocity 180 // mm/ s
#define defaultP2pActivationDistanceMM 7
#define RecalibrationFullThrottlePWM 255 // vertical motor

//manual
//#define motorWallSpeed 255
//#define motorHoriStart 25
//#define motorHoriEnd 70
//#define motorHoriDrawingStart 25
//#define motorHoriDrawingEnd 70
//#define motorVertUpStart 190
//#define motorVertUpEnd 255  
//#define motorVertDownStart 190
//#define motorVertDownEnd 255
//#define motorVertUpDrawingStart 190
//#define motorVertUpDrawingEnd 255
//#define motorVertDownDrawingStart 190
//#define motorVertDownDrawingEnd 255
//#define maxPerpendicularVelocity 255 // mm/ s

// SPEED CALIBRATION SEQUENCE MACROS
#define MANUALSPEEDCALIBRATION  0
#define BEGINWITHBASICCALIBRATION 1

// SPEED CONTROL MACROS
#define   MOTORSTOPALL 0
#define   MOTORSTOPHORI 1
#define   MOTORSTOPVERT 2
#define   IGNOREMOTOR -999

// Vertical calibration macros
#define DONTRETURNTOPREVIOUSPOS false
#define RETURNTOPREVIOUSPOS true

// On the fly gradient correction of x-y relative scale for correct gradient-making.
// want to carry over angular correction trhoughout all segments of drawing ? below variable is answer.
#define defaultCarryoverAngularCorrection false // change this setting to tggole 
#define defaultCorrectionSpeed 0.03               // more short term, should be scaled to the line segment. by 50% of the line segment, 1.1/0.9 scaling is reached. dynamically calculated.
#define defaultCarryOverCorrectionSpeed 0.0025  // more stable and long term. in 60 seconds, 1.15/0.85 scaling is achieved.
                                                // we cap this at 1.35 overall difference , i.e. 1.15/0.85 

#define manualP2pActivationDistanceMM 7.5
#define gIntegralThreshold 2
#define EpsilonOutOfBoundsThreshold 8.5
#define unsafeRegime 15
#define CorrectionAngleDetectionThreshold 1.2
// correctionangle flags
#define angleCW 0
#define angleCCW 1
#define angleNEUTRAL 2

#define HORI_MM_TO_PTS 26.668 //4096 One Full Revolution 48.89 π = 153.59 (full revolution) 26.668 per mm
#define VERT_MM_TO_PTS 13.333//800 One Full Revolution 60mm(full revolution) 13.333 points per mm
#define BUFFER_SIZE 1024
#define INPUT_SIZE 30
//PINS
#define SPIWIFI       SPI  // The SPI port
#define SPIWIFI_SS    10   // Chip select pin
#define ESP32_RESETN  5   // Reset pin
#define SPIWIFI_ACK   7   // a.k.a BUSY or READY pin
#define ESP32_GPIO0   -1
#define motorHori_PWM     9 //Updated
#define motorWall_1PWM    4 //Updated
#define motorWall_2PWM    3 //Updated
#define motorHori_1PinA    24 //Updated
#define motorHori_1PinB    25 //Updated
#define motorHori_2PinA    27 //Updated 4 August 2021swap with motorHori2B
#define motorHori_2PinB    26 //Updated 4 August 2021 swap with motorHori2A
#define motorWall_1PinA   22 //Updated
#define motorWall_1PinB   21 //Updated
#define motorWall_2PinA   20 //Updated
#define motorWall_2PinB   19 //Updated
#define encoderHori_A     41  //Updated
#define encoderHori_B     40 //Updated
#define encoderVert_A     14 //Updated
#define encoderVert_B     15 //Updated
#define motorVert_PWM     8 //Updated
#define motorVert_PinA    17 //Updated
#define motorVert_PinB    16 //Updated
#define ServoPin          29 //Updated
#define limitSwitch_A     33 //Updated
#define limitSwitch_B     23 //Updated
#define limitSwitch_VertCalibrate 34 //Updated

#define ServoPosPump      55
#define ServoPosExtend    175
#define ServoPosNeutral   100

// code to reset PID's ...
#define RESETALL 0
#define RESETPIDH 1
#define RESETPIDV 2
#define RESETPIDG 3
#define pwmFactorGLimit 200 //so maximum range is 100 . max range 200 ... ? sounds fair.
//if Kp is ~10 ... then 1 m is 10. 20 mm is 200.
#define defaultCurvilinearSpeed 40 // 100 mm/s
#define clampingFraction 0.4    // percentage between 0 and minimum PWM speed on either axis where it clamps

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "Morpheus";        // your network SSID (name)
char pass[] = "IdeaInk1";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(8080);
char buffer[BUFFER_SIZE];

//const char *TestCommands[] = {
//  "gcode~G90",
//  "gcode~G21",
//  "gcode~G0~X0~Y0",
//  "gcode~G1~X100~Y100",
//  "gcode~G1~X100~Y200",
//  "gcode~G0~X200~Y200",
//  "gcode~G0~X0~Y0"
//};

Encoder myHoriEnc(encoderHori_A, encoderHori_B);
Encoder myVertEnc(encoderVert_A, encoderVert_B);
WiFiClient client;
long oldHoriPosition = -999;
long oldVertPosition = -999;
long reachedHoriValue = 0;
long reachedVertValue = 0;
long relativeHoriHome = 0;
long relativeVertHome = 0;
Servo myservo;
int pos = ServoPosNeutral;
//System
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
bool isHome = true;
bool againstWall = false;
bool lastAgainstWall = false;
bool readyToServe = false;
int last_ch_id = 0;
bool drawing = false;
bool bumped = false;
bool doingCmd = false;
int numberOfCommands = 0;
int commandNum = 0;
int reachedCount = 0;
double previousModifier = 0;
double previousTempSpeed = 0;
double desiredHoriMM = 0;
double desiredVertMM = 0;
double currentHoriMM = 0;
double currentVertMM = 0;
double initialHoriMM = 0;
double initialVertMM = 0;
int lA = 1;
int lB = 1;
//Motors
double Kp_H = 1, Ki_H = 1.9 , Kd_H = 0;
double Kp_V = 0.8, Ki_V = 8.9  , Kd_V = 0;
double Kp_G = 1, Ki_G = 0.6  , Kd_G = 0;  // tune Kp between 0 to 3 dont go beyond that. do not use I do not use D unless authorized
double currentHoriPos = 0, /* where we are */
       currentVertPos = 0,
       pwmFactorH = 0,
       pwmFactorV = 0,
//       bounds = 25,     // unused
       desiredHoriPos = 0,//1000mm 26076
       desiredVertPos = 0,//15cm
       pDistance_squared = 0,
       pDistance = 0,
       p2pActivationDistanceMM = defaultP2pActivationDistanceMM,
       pwmFactorG = 0,
       PWMperMMHori = 200 / 150, // initial values, can be changed based on testing. will be dynamically updated by calibration sequences.
       PWMperMMVert = 200 / 150,
       extraPWM_X = 0,
       extraPWM_Y = 0;
//YtoXScale = PWMperMMVert / PWMperMMHori ; // how much more is Y to X in terms of PWM speed. this will be adjusted on the fly with floatingYtoXScale
//floatingYtoXScale = YtoXScale;
double perpendicularVelocity_X = 0,
       perpendicularVelocity_Y = 0,
       curvilinearVelocity_X = 0,   // supposed velocities, theoretically / desired velocities.
       curvilinearVelocity_Y = 0,
       curvilinearSpeed = defaultCurvilinearSpeed ,
       maxPerpendicularVelocity = defaultMaxPerpendicularVelocity ;
bool transitionedToPID = false;
double correctionCW = 0;
double correctionCCW = 0;
double gIntegralError = 0;
double yCorrection = 1;
double xCorrection = 1;
double correctionSpeed = defaultCorrectionSpeed;
double checkEndThreshold = 0;
bool startedCount = false;
unsigned long elapseduS = 0;
unsigned long lastMicros = 1;
bool carryoverAngularCorrection = defaultCarryoverAngularCorrection;
bool needPump = false;
unsigned long sinceLastPump = 0;
unsigned long drawTimer = 0;
unsigned long sinceLastAutoRecalibrateHome = 0;
unsigned long autoHomeTimer = 0;

long absoluteZeroY = 0;
long homeRelativeToAbsZero = 0;

double pwm_min_V_adjusted = motorVertUpDrawingStart;
double pwm_max_V_adjusted = motorVertUpDrawingEnd;
double pwm_min_H_adjusted = motorHoriDrawingStart;
double pwm_max_H_adjusted = motorHoriDrawingEnd;

PID myPIDH(&currentHoriPos, &pwmFactorH, &desiredHoriPos, Kp_H, Ki_H, Kd_H, DIRECT);
PID myPIDV(&currentVertPos, &pwmFactorV, &desiredVertPos, Kp_V, Ki_V, Kd_V, DIRECT);
PID myPIDG(&pDistance, &pwmFactorG, 0, Kp_G, Ki_G, Kd_G, DIRECT);   // yes reverse. say setpoint is 0. error = setpoint - distance. this will always be a negative number. so we choose reverse since kp value will be reversed in the PID library for reverse.
double MM_TO_PTS(double mm, bool horiVert) {
  if (horiVert) {
    return (double)round(mm * HORI_MM_TO_PTS);
  } else {
    return (double)round(mm * VERT_MM_TO_PTS);
  }
}

double PTS_TO_MM(double points, bool horiVert) {
  if (horiVert) {
    return (double)round(points / HORI_MM_TO_PTS);
  } else {
    return (double)round(points / VERT_MM_TO_PTS);
  }
}


bool read_till_eol() {
  static int i = 0;
  if (client.available()) {
    buffer[i++] = client.read();
    if (i == BUFFER_SIZE)  i = 0;
    if (i > 1 && buffer[i - 2] == 13 && buffer[i - 1] == 10) {
      buffer[i] = 0;
      i = 0;
      Serial.print(buffer);
      return true;
    }
  }
  return false;
}

void processCMD(char* cmds) {
  if (strncmp(cmds, "gcode", 5) == 0) {
    //Serial.println(cmds);
    processGCode(cmds);
  } else if (strncmp(cmds, "robot", 5) == 0) {
    //processRobotCode(client,cmds);
  }
}

void moveServoPos(int pos) {
  int currentPos = myservo.read();
  if (currentPos != pos) {
    if (pos > currentPos) {
      for (int i = 0; i < abs(pos - currentPos); i++) {
        myservo.write(currentPos + i);
        delay(5);
      }
    } else if (pos < currentPos) {
      for (int i = 0; i < abs(pos - currentPos); i++) {
        myservo.write(currentPos - i);
        delay(5);
      }
    }
  }
}

void processGCode(char* movement) {
  Serial.println(movement);
  char* cmd = strtok(movement, "~"); //sample channel: ID0:gcode~G1~X100~Y100
  //Serial.println(cmd);
  int g = -999;
  double x, y = -9999;
  while (cmd != 0) {
    switch (*cmd) {
      case 'G': sscanf(cmd + 1, "%i", &g);
        //Serial.print(g);
        break;
      case 'X': sscanf(cmd + 1, "%lf", &x);
        //Serial.print(x);
        break;
      case 'Y': sscanf(cmd + 1, "%lf", &y);
        //Serial.print(y);
        break;
      default: break;
    }
    cmd = strtok(0, "~");
  }
  //if(!doingCmd){
  if (g >= 0) {
    switch (g) {
      case 0:
        if (x != -9999) {
          desiredHoriPos = MM_TO_PTS(x, true) + relativeHoriHome; //1000mm 26076
          desiredHoriMM = x;
        } else {
          desiredHoriPos = relativeHoriHome;
          desiredHoriMM = 0;
        }
        if (y != -9999) {
          desiredVertPos = MM_TO_PTS(y, false) + relativeVertHome; //15cm
          desiredVertMM = y;
        } else {
          desiredVertPos = relativeVertHome;
          desiredVertMM = 0;
        }
        drawing = false;
        moveServoPos(ServoPosNeutral);
        break;
      case 1:
        if (x != -9999) {
          desiredHoriPos = MM_TO_PTS(x, true) + relativeHoriHome; //1000mm 26076
          desiredHoriMM = x;
        } else {
          desiredHoriPos = currentHoriPos;
          desiredHoriMM = currentHoriMM;
        }
        if (y != -9999) {
          desiredVertPos = MM_TO_PTS(y, false) + relativeVertHome; //15cm
          desiredVertMM = y;
        } else {
          desiredVertPos = currentVertPos;
          desiredVertMM = currentVertMM;
        }
        drawing = true;
        moveServoPos(ServoPosExtend);
        break;
      default:
        //ignore
        break;
    }
    Serial.println(desiredHoriPos);
    Serial.println(desiredVertPos);

    if (client.connected()) client.println("gcode~ack");
    doingCmd = true;

    initialHoriMM = currentHoriMM;
    initialVertMM = currentVertMM;

    // interrupt with pumpingmarker ...
    if ((sinceLastPump/1000) > pumpIntervalTime) {
      if (client.connected()) client.println("robot~pumping");
      PumpingMarker();
      if (client.connected()) client.println("robot~donePumping");
     }

    // check for vertical home recalibration
    if (g == 0 && automaticallyScheduleRecalibrationVerticalHome && sqrt(pow(desiredHoriMM,2) + pow(desiredVertMM,2)) >= automaticVertHomeRecalibrationG0Threshold && 
          sinceLastAutoRecalibrateHome > autoRecalibrateHomeInterval  ) { // maybe default 20
      if (client.connected()) client.println("robot~recalibratingHome");
      reCalibrateVertical(RETURNTOPREVIOUSPOS); // recalibrate home and then return to previous position, even . 
      sinceLastAutoRecalibrateHome = 0;
      if (client.connected()) client.println("robot~doneRecalibratingHome");
    }

    // add to the timer
    if (drawing) {
      drawTimer = millis();             // this will be added to sinceLastPump after checkend.
      autoHomeTimer = millis();         // autorecalibration of vertical home
    }
  }
}

bool checkWall() {
  lA = digitalRead(limitSwitch_A);
  lB = digitalRead(limitSwitch_B);
  if (lA == 1 || lB == 1) {
    if (lA == 1  && lB == 0) {
      analogWrite(motorWall_1PWM, motorWallSpeed);
      analogWrite(motorWall_2PWM, 150);
      digitalWrite(motorWall_1PinA, HIGH);
      digitalWrite(motorWall_2PinA, HIGH);
      digitalWrite(motorWall_1PinB, LOW);
      digitalWrite(motorWall_2PinB, LOW);

    } else if (lA == 0  && lB == 1) {
      analogWrite(motorWall_1PWM, 150);
      analogWrite(motorWall_2PWM, motorWallSpeed);
      digitalWrite(motorWall_1PinA, HIGH);
      digitalWrite(motorWall_2PinA, HIGH);
      digitalWrite(motorWall_1PinB, LOW);
      digitalWrite(motorWall_2PinB, LOW);
    } else {
      analogWrite(motorWall_1PWM, motorWallSpeed);
      analogWrite(motorWall_2PWM, motorWallSpeed);
      digitalWrite(motorWall_1PinA, HIGH);
      digitalWrite(motorWall_2PinA, HIGH);
      digitalWrite(motorWall_1PinB, LOW);
      digitalWrite(motorWall_2PinB, LOW);
    }
    return false;
  } else {
    analogWrite(motorWall_1PWM, 150);
    analogWrite(motorWall_2PWM, 150);
    digitalWrite(motorWall_1PinA, HIGH);
    digitalWrite(motorWall_2PinA, HIGH);
    digitalWrite(motorWall_1PinB, LOW);
    digitalWrite(motorWall_2PinB, LOW);
    return true;
  }
}
void printStatus() {
  if (currentHoriPos != oldHoriPosition || currentVertPos != oldVertPosition) {
    oldHoriPosition = currentHoriPos;
    oldVertPosition = currentVertPos;
    Serial.print(currentHoriPos);
    Serial.print(":");
    Serial.print(pwmFactorH);
    Serial.print(">");
    Serial.print(desiredHoriPos);
    Serial.print(",");
    Serial.print(currentVertPos);
    Serial.print(":");
    Serial.print(pwmFactorV);
    Serial.print(">");
    Serial.print(desiredVertPos);
    Serial.print("->Servo:");
    Serial.println(pos);
  }
}


int pwmHoriOut(int out, bool move, bool dCmd, bool draw, double modifier) {
  int tempSpeed;
  int tempMaxSpeed;
  if (floor(out) == 0) {
    out = floor(out);
  }
  if (draw) {
    tempMaxSpeed = map(modifier, 0, 1, pwm_min_H_adjusted, pwm_max_H_adjusted);
    tempSpeed = map(abs(out), 0, 255, pwm_min_H_adjusted, tempMaxSpeed);
  } else {
    tempMaxSpeed = map(modifier, 0, 1, motorHoriStart, motorHoriEnd);
    tempSpeed = map(abs(out), 0, 255, motorHoriStart, tempMaxSpeed);
  }

  if (out > 0 && move && dCmd) {    // positive x direction
    analogWrite(motorHori_PWM, tempSpeed);
    digitalWrite(motorHori_1PinA, LOW);
    digitalWrite(motorHori_1PinB, HIGH);
    digitalWrite(motorHori_2PinA, LOW);
    digitalWrite(motorHori_2PinB, HIGH);
  } else if (out < 0 && move && dCmd) {    // negative x direction
    analogWrite(motorHori_PWM, tempSpeed);
    digitalWrite(motorHori_1PinA, HIGH);
    digitalWrite(motorHori_1PinB, LOW);
    digitalWrite(motorHori_2PinA, HIGH);
    digitalWrite(motorHori_2PinB, LOW);
  } else if (!move) {

    analogWrite(motorHori_PWM, 0);
    digitalWrite(motorHori_1PinA, LOW);
    digitalWrite(motorHori_1PinB, LOW);
    digitalWrite(motorHori_2PinA, LOW);
    digitalWrite(motorHori_2PinB, LOW);
  } else {

    analogWrite(motorHori_PWM, 0);
    digitalWrite(motorHori_1PinA, LOW);
    digitalWrite(motorHori_1PinB, LOW);
    digitalWrite(motorHori_2PinA, LOW);
    digitalWrite(motorHori_2PinB, LOW);
  }
  if (previousModifier != modifier || previousTempSpeed != tempSpeed) {
    //      Serial.print(tempSpeed);
    //      Serial.print(":");
    //      Serial.println(modifier);
    previousModifier = modifier;
    previousTempSpeed = tempSpeed;
  }
  return tempSpeed;
}

int pwmVertOut(int out, bool moves, bool dCmd, bool draw, double modifier) {
  //Serial.println(out);
  int tempSpeed;
  int tempMaxSpeed;
  if (floor(out) == 0) {
    out = floor(out);
  }
  if (draw) {
    if (out > 0) {
      tempMaxSpeed = map(modifier, 0, 1, pwm_min_V_adjusted, pwm_max_V_adjusted);
      tempSpeed = map(abs(out), 0, 255, pwm_min_V_adjusted, tempMaxSpeed);
    } else if (out < 0) {
      tempMaxSpeed = map(modifier, 0, 1, pwm_min_V_adjusted, pwm_max_V_adjusted);
      tempSpeed = map(abs(out), 0, 255, pwm_min_V_adjusted, tempMaxSpeed);
    } else {
      tempSpeed = 0;
    }
  } else {
    if (out > 0) {
      tempMaxSpeed = map(modifier, 0, 1, motorVertUpStart, motorVertUpEnd);
      tempSpeed = map(abs(out), 0, 255, motorVertUpStart, tempMaxSpeed);
    } else if (out < 0) {
      tempMaxSpeed = map(modifier, 0, 1, motorVertDownStart, motorVertDownEnd);
      tempSpeed = map(abs(out), 0, 255, motorVertDownStart, tempMaxSpeed);
    } else {
      tempSpeed = 0;
    }
  }
  if (out > 0 && moves && dCmd) {
    analogWrite(motorVert_PWM, tempSpeed);
    digitalWrite(motorVert_PinA, LOW);
    digitalWrite(motorVert_PinB, HIGH);
  } else if (out < 0 && moves && dCmd) {
    analogWrite(motorVert_PWM, tempSpeed);
    digitalWrite(motorVert_PinA, HIGH);
    digitalWrite(motorVert_PinB, LOW);
  } else {
    analogWrite(motorVert_PWM, 0);
    digitalWrite(motorVert_PinA, LOW);
    digitalWrite(motorVert_PinB, LOW);
  }
  return tempSpeed;
}

bool checkEndMove(long dHoriPos, long dVertPos, long HoriPos, long VertPos) {
  if (sqrt ( pow(dHoriPos - HoriPos,2 ) + pow (dVertPos - VertPos, 2)) <= checkEndThreshold) {
//  if (abs(dHoriPos - HoriPos) <= checkEndThreshold && abs(dVertPos - VertPos) <= checkEndThreshold) {   // use the above instead for circular detection shape.
    if (doingCmd) {
      doingCmd = false;                                     // TODO : another kind of command might make doingCmd = false expression appear elsewhere too
      if (client.connected()) client.println("gcode~ok");

      if (drawing) {                                        // drawing should only ever be set true inside processgcode()
        drawing = false;
        sinceLastPump += millis() - drawTimer ;             // drawTimer is only added to sincelast pump after checkendmove and drawing. is only started after G1 processed.
        sinceLastAutoRecalibrateHome += millis() - autoHomeTimer;
      }

      Serial.println("resettingPID");
      ResetPID(RESETALL);
      ResetControlVariables();
      Serial.println("new to go position");
      //reachedCount ++;
    }
    return false;   // no longer doing.
  }
  return true;      // still doing. used for caleb p2p PID
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}


void calibrateVertHome() {
  unsigned long vertTime = 0;
  myservo.write(ServoPosNeutral);
  int i = digitalRead(limitSwitch_VertCalibrate);
  while (i == 1) {
    pwmVertOut(-155, true, true, false, 1);
    
    absoluteZeroY = myVertEnc.read();
    i = digitalRead(limitSwitch_VertCalibrate);
  }
  analogWrite(motorVert_PWM, 0);
  digitalWrite(motorVert_PinA, LOW);
  digitalWrite(motorVert_PinB, LOW);

  // 
  vertTime = millis();
  while (millis() - vertTime < 600) {
    pwmVertOut(150, true, true, false, 1);
  }
  analogWrite(motorVert_PWM, 0);
  digitalWrite(motorVert_PinA, LOW);
  digitalWrite(motorVert_PinB, LOW);
  relativeVertHome = myVertEnc.read();
  homeRelativeToAbsZero = relativeVertHome - absoluteZeroY;

  // to reset , give when reached absolute zero record the delta.
  // myVertEnc.read(); 
  // then set relativeVertHome = (new) absoluteZeroY + homeRelativeToAbsZero.
  // using whatever drifted values we recover the real world sense.
  
  desiredVertPos = relativeVertHome;
  initialHoriMM = 0;
  initialVertMM = 0;
  Serial.println("relativeVertHome:");
  Serial.println(relativeVertHome);
}

bool pumping = false;
void PumpingMarker() {   // special parameter for many more times pumping cycle esp. at beginning of drawing, etc.
   pumping = true;
   StopAxisMotors(MOTORSTOPALL);
   StopWallMotors();
   
  Serial.println("Pumping...");
  for (int i = 0; i < 2; i++) {
    moveServoPos(ServoPosPump);
    delay(3000);   // this is perhaps should be 2 s.
    myservo.write(ServoPosNeutral);
    delay(3000);  // this is too long, perhaps just 0.5 s.
  }
  pumping = false;
  sinceLastPump = 0;                      // restart pump timer
  Serial.println("Done Pumping");

  if (drawing) moveServoPos(ServoPosExtend);
}

double MMtoPWMHori(double vel) {
  // this is based on say about a 100 mm/s of speed test of the motor speed  .... rather than very slow movement.
  // if we find out the speed and PWM to actually not be linearly related we have to find a way to describe it.
    // the extra is the constant in the linear equation
  return extraPWM_X + PWMperMMHori * vel;
}

double MMtoPWMVert(double vel) {
  return extraPWM_Y + PWMperMMVert * vel;
}


void basicVelocityPWMCalibration() {
  unsigned long deltaTH;
  unsigned long deltaTV;
  double curVertMM = 0,
         curHoriMM = 0;
//  double oldVertMM = 0,
//         oldHoriMM = 0;
  int power_H, power_V;
  bool deltaTH_Done = false,
       deltaTV_Done = false;
  while(!checkWall());
  unsigned long timeInit = millis();
  while(curHoriMM < 600 && curVertMM < 600){
    int power_H = motorHoriDrawingEnd;
        power_V = motorVertUpDrawingEnd;
    currentHoriPos = myHoriEnc.read();
    currentVertPos = myVertEnc.read();
    curHoriMM = PTS_TO_MM(currentHoriPos - relativeHoriHome, true);
    curVertMM = PTS_TO_MM(currentVertPos - relativeVertHome, false);
//    if((curHoriMM != oldHoriMM) || (curVertMM != oldVertMM)){
//      Serial.print(curHoriMM);
//      Serial.print(":");
//      Serial.println(curVertMM);
//      oldHoriMM = curHoriMM;
//      oldVertMM = curVertMM;
//    }
    if(curHoriMM > 500){
      if(!deltaTH_Done){
        deltaTH = millis() - timeInit;
      }
      power_H = 0;
      deltaTH_Done = true;
    }
    if(curVertMM > 500){
      if(!deltaTV_Done){
        deltaTV = millis() - timeInit;
      }
      power_V = 0;
      deltaTV_Done = true;
    }
    checkWall();
    analogWrite(motorHori_PWM, power_H);
    analogWrite(motorVert_PWM, power_V);
    analogWrite(motorWall_1PWM, 150);
    analogWrite(motorWall_2PWM, 150);
    digitalWrite(motorWall_1PinA, HIGH);
    digitalWrite(motorWall_2PinA, HIGH);
    digitalWrite(motorWall_1PinB, LOW);
    digitalWrite(motorWall_2PinB, LOW);
    digitalWrite(motorHori_1PinA, LOW);
    digitalWrite(motorHori_1PinB, HIGH);
    digitalWrite(motorHori_2PinA, LOW);
    digitalWrite(motorHori_2PinB, HIGH);
    digitalWrite(motorVert_PinA, LOW);
    digitalWrite(motorVert_PinB, HIGH);
    if(curHoriMM > 500 && curVertMM > 500){
      break;
    }
  }
  Serial.print("deltaXTime:");
  Serial.println(deltaTH);
  Serial.print("deltaYTime:");
  Serial.println(deltaTV);
  StopAxisMotors(MOTORSTOPALL);
  double speed_max_X = 500/deltaTH*1000;
  double speed_max_Y = 500/deltaTV*1000;
  delay(1000);
  
  double initSlowMM_H = curHoriMM;
  double initSlowMM_V = curVertMM;
  deltaTH_Done = false;
  deltaTV_Done = false;
  while(!checkWall());
  timeInit = millis();
  while(curHoriMM < initSlowMM_H+250 && curVertMM < initSlowMM_V+250){
    power_H = motorHoriDrawingStart;
    power_V = motorVertUpDrawingStart;
    currentHoriPos = myHoriEnc.read();
    currentVertPos = myVertEnc.read();
    curHoriMM = PTS_TO_MM(currentHoriPos - relativeHoriHome, true);
    curVertMM = PTS_TO_MM(currentVertPos - relativeVertHome, false);
    if(curHoriMM > initSlowMM_H+150){
      if(!deltaTH_Done){
        deltaTH = millis() - timeInit;
      }
      power_H = 0;
      deltaTH_Done = true;
    }
    if(curVertMM > initSlowMM_V+150){
      if(!deltaTV_Done){
        deltaTV = millis() - timeInit;
      }
      power_V = 0;
      deltaTV_Done = true;
    }
    checkWall();
    analogWrite(motorHori_PWM, power_H);
    analogWrite(motorVert_PWM, power_V);
    digitalWrite(motorHori_1PinA, LOW);
    digitalWrite(motorHori_1PinB, HIGH);
    digitalWrite(motorHori_2PinA, LOW);
    digitalWrite(motorHori_2PinB, HIGH);
    digitalWrite(motorVert_PinA, LOW);
    digitalWrite(motorVert_PinB, HIGH);
    if(curHoriMM > initSlowMM_H+150 && curVertMM > initSlowMM_V+150){
      break;
    }
  }
  Serial.print("deltaXTime:");
  Serial.println(deltaTH);
  Serial.print("deltaYTime:");
  Serial.println(deltaTV);
  StopAxisMotors(MOTORSTOPALL);
  double speed_min_X = 150/deltaTH*1000;
  double speed_min_Y = 150/deltaTV*1000; 
  Serial.print("speed_min_X:");
  Serial.println(speed_min_X);
  Serial.print("speed_min_Y:");
  Serial.println(speed_min_Y);
  Serial.print("speed_max_X:");
  Serial.println(speed_max_X);
  Serial.print("speed_max_Y:");
  Serial.println(speed_max_Y);
  double mY = (speed_max_Y - speed_min_Y) / (motorVertUpDrawingEnd - motorVertUpDrawingStart) ;
  double cY = speed_max_Y - mY * motorVertUpDrawingEnd ;
  extraPWM_Y = (0 - cY) / mY ; //
  PWMperMMVert = 1 / mY ;
  Serial.print("PWM_per_MM_V:");
  Serial.println(PWMperMMVert);
  double mX = (speed_max_X - speed_min_X) / (motorHoriDrawingEnd - motorHoriDrawingStart) ;
  double cX = speed_max_X - mX * motorHoriDrawingEnd ;//1.0311
  extraPWM_X = (0 - cX) / mX ;
  PWMperMMHori = 1 / mX ;
  Serial.print("PWM_per_MM_H:");
  Serial.println(PWMperMMHori);

  if (speed_max_Y > speed_max_X) {
    pwm_max_V_adjusted = (speed_max_X - cY) / mY ;
    
  } else if (speed_max_X > speed_max_Y) {
    pwm_max_H_adjusted = (speed_max_Y - cX) / mX ;
  }

  if (speed_min_Y < speed_min_X) {
    pwm_min_V_adjusted = (speed_min_X - cY) / mY ;
  } else if (speed_min_X < speed_min_Y) {
    pwm_min_H_adjusted = (speed_min_Y - cX) / mX ;
  }  
  Serial.print("PWM_Adjusted_H:");
  Serial.print(pwm_min_H_adjusted);
  Serial.print(":");
  Serial.println(pwm_max_H_adjusted);
  Serial.print("PWM_Adjusted_V:");
  Serial.print(pwm_min_V_adjusted);
  Serial.print(":");
  Serial.println(pwm_max_V_adjusted);
  
}

void setSpeedMMtoPWM(double velX, double velY) {
  double PWM_X = 0;
  double PWM_Y = 0;
  bool up = false;
  double scaleFactor = 1;
  // converts MM speed to PWM speed..
  PWM_X = MMtoPWMHori(velX);
  PWM_Y = MMtoPWMVert(velY);
  up = PWM_Y > 0;

  // --- CEILING ---
  // in case the PWM values exceed the allowable for drawing, to scale them down.
  // if resultant PWM speed is above the threshold set, we scale it down based on the one that hit ceiling first.

   Serial.print("  X: ");
  Serial.print(PWM_X);
  Serial.print("  Y: ");
  Serial.print(PWM_Y);
  Serial.print("  ");

  //handle case when X exceeds.
  if (abs(PWM_X) >= motorHoriDrawingEnd) {
    scaleFactor = motorHoriDrawingEnd / PWM_X;
    PWM_X = PWM_X * scaleFactor;    // preserve the correct x-y scale difference
    PWM_Y = PWM_Y * scaleFactor;
  }
  // if Y still exceeds nevertheless
  if ((up && abs(PWM_Y) >= motorVertUpDrawingEnd) || (!up && abs(PWM_Y) >= motorVertDownDrawingEnd)) {
    scaleFactor = up ? motorVertUpDrawingEnd / PWM_Y : motorVertDownDrawingEnd / PWM_Y ;  // conditional assignment (ternary operator ? )
    PWM_X = PWM_X * scaleFactor;
    PWM_Y = PWM_Y * scaleFactor;
  }

  // --- FLOOR ---
  // if resultant PWM speed is below the threshold set, we clamp it to the nearest PWM values - lower limit or zero, depending on clampingFraction set ('propensity')
  // clamping & clamping rules

  int signX = (PWM_X > 0) - (PWM_X < 0);
  int signY = (PWM_Y > 0) - (PWM_Y < 0);

  if (abs(PWM_X) < clampingFraction * motorHoriDrawingStart) {  // if inferior tendency
    PWM_X = 0;
  } else if (abs(PWM_X) < motorHoriDrawingStart ) {  // if superior tendency
    PWM_X = motorHoriDrawingStart * signX;
  }
  if (up) {
    if (abs(PWM_Y) < clampingFraction * motorVertUpDrawingStart) {       // if up
      PWM_Y = 0;
    } else if (abs(PWM_Y) < motorVertUpDrawingStart) {                   // if up
      PWM_Y = motorVertUpDrawingStart * signY;
    }
  } else if (abs(PWM_Y) < clampingFraction * motorVertDownDrawingStart) {   // if down
    PWM_Y = 0;
  } else if (abs(PWM_Y) < motorVertDownDrawingStart) {        // if down
    PWM_Y = motorVertDownDrawingStart * signY;
  }
  Serial.print("  X: ");
  Serial.print(PWM_X);
  Serial.print("  Y: ");
  Serial.println(PWM_Y);
  
  moveMotors(PWM_X, PWM_Y);

}

//        directly control the pwm of axis motors, using a signed value, and move them. // this code is simplified motor power decision tree based on current physical setup, using ternary operator and logic
void moveMotors(double pwmX, double pwmY) {  // feed -999 to ignore either motor.                                       
    bool positive = pwmX > 0;                                            // X
    bool zero = pwmX == 0;
    if (pwmX > IGNOREMOTOR) {                      // feed -999 to ignore either motor. IGNOREMOTOR = -999
      analogWrite(motorHori_PWM, abs(pwmX));
      digitalWrite(motorHori_1PinA, positive || zero ? LOW : HIGH );    // note for ternary operator : set ALL LOW only if zero.
      digitalWrite(motorHori_1PinB, positive         ? HIGH : LOW);    // originally : positive && !zero ? -> is it's positive, it's definitely not zero !
      digitalWrite(motorHori_2PinA, positive || zero ? LOW : HIGH );
      digitalWrite(motorHori_2PinB, positive         ? HIGH : LOW );
      Serial.print(" moveX: ");
      Serial.print(pwmX);
    }

    if (pwmY > IGNOREMOTOR) {                     // feed -999 to ignore either motor.   
    positive = pwmY > 0;                                            // Y
    zero = pwmY == 0;
    analogWrite(motorVert_PWM, abs(pwmY));                    
    digitalWrite(motorVert_PinA, positive || zero ? LOW : HIGH );
    digitalWrite(motorVert_PinB, positive         ? HIGH : LOW);
    Serial.print(" moveY: ");
    Serial.print(pwmY);
    }
    if (pwmX > IGNOREMOTOR || pwmY > IGNOREMOTOR) Serial.println(" after moved motors");
}

void StopAxisMotors(int motorStopCode) {
    if (motorStopCode == MOTORSTOPALL || motorStopCode == MOTORSTOPHORI) {
      moveMotors(0, IGNOREMOTOR);
    }
    if (motorStopCode == MOTORSTOPALL || motorStopCode == MOTORSTOPVERT) {
      moveMotors(IGNOREMOTOR, 0);
    }
}

void StopWallMotors() {
  analogWrite(motorWall_1PWM, 0);
  analogWrite(motorWall_2PWM, 0);
  digitalWrite(motorWall_1PinA, LOW);
  digitalWrite(motorWall_2PinA, LOW);
  digitalWrite(motorWall_1PinB, LOW);
  digitalWrite(motorWall_2PinB, LOW);
}

bool ResetPID(int id) {
    long tempVal = 0;
    long tempValOut = 0;
  //initialize whatever PID regime is being used.
  // id 0 = RESET ALL - id 1 : RESET PIDH  -  id 2 : RESET PIDV - id 3 : RESET PIDG
  //PID myPIDH(&currentHoriPos, &pwmFactorH, &desiredHoriPos, Kp_H, Ki_H, Kd_H, DIRECT);
  //PID myPIDV(&currentVertPos, &pwmFactorV, &desiredVertPos, Kp_V, Ki_V, Kd_V, DIRECT);
  //PID myPIDG(&pDistance, &pwmFactorG, 0, Kp_G, Ki_G, Kd_G, REVERSE);
  bool resetOK = false;

  // PID H
  if (id == 1 || id == 0) {
    tempVal = currentHoriPos;
    tempValOut = pwmFactorH;
    // set zero for PID reset
    currentHoriPos = 0;
    pwmFactorH = 0;
    myPIDH.SetMode(MANUAL);
    myPIDH.SetMode(AUTOMATIC);
    // restore
    currentHoriPos = tempVal;
    pwmFactorH = tempValOut;
    resetOK = true;
  }

  // PID V
  if (id == 2 || id == 0) {
    tempVal = currentVertPos;
    tempValOut = pwmFactorV;
    // set zero for PID reset
    currentVertPos = 0;
    pwmFactorV = 0;
    myPIDV.SetMode(MANUAL);
    myPIDV.SetMode(AUTOMATIC);
    // restore
    currentVertPos = tempVal;
    pwmFactorV = tempValOut;
    resetOK = true;
  }

  // PID G
  if (id == 3 || id == 0) {
    tempVal = pDistance;
    tempValOut = pwmFactorG;
    // set zero for PID reset
    pDistance = 0.001;
    pwmFactorG = 0.001;
    myPIDG.SetMode(MANUAL);
    myPIDG.SetMode(AUTOMATIC);
    // restore
    pDistance = tempVal;
    pwmFactorG = tempValOut;
    resetOK = true;
  }
  return resetOK;
}

void InitializeCalibration(int mode) {
  switch (mode) {
    case MANUALSPEEDCALIBRATION :
      pwm_min_V_adjusted = 150;
      Serial.print(" PWM-min-V: ");
      Serial.print(pwm_min_V_adjusted);
      pwm_max_V_adjusted = 255;
      Serial.print(" PWM-max-V: ");
      Serial.print(pwm_max_V_adjusted);
      pwm_min_H_adjusted = 25;
      Serial.print(" PWM-min-H: ");
      Serial.print(pwm_min_H_adjusted);
      pwm_max_H_adjusted = 45;
      Serial.print(" PWM-max-H: ");
      Serial.print(pwm_max_H_adjusted);
      PWMperMMVert = 3.7;
      Serial.print(" PWMperMMVert: ");
      Serial.print(PWMperMMVert);
      PWMperMMHori = 0.97;
      Serial.print(" PWMperMMHori: ");
      Serial.print(PWMperMMHori);
      extraPWM_Y = 52.62;
      Serial.print(" ExtraPWM_Y: ");
      Serial.print(extraPWM_Y);
      extraPWM_X = 7.34;
      Serial.print(" ExtraPWM_X: ");
      Serial.print(extraPWM_X);
      maxPerpendicularVelocity = 200;
      Serial.print(" max perp velocity: ");
      Serial.print(maxPerpendicularVelocity);
      p2pActivationDistanceMM = manualP2pActivationDistanceMM;
      Serial.print(" P2p Activation Distance: ");
      Serial.print(p2pActivationDistanceMM);
      curvilinearSpeed = 45;
      Serial.print(" curvilinear speed ");
      Serial.print(curvilinearSpeed);
      Serial.println("");
      break;
    case BEGINWITHBASICCALIBRATION :
      Serial.println("----Calibrating Speed----");
      basicVelocityPWMCalibration();
      break;
    default:
      break;
  }
  if (maxPerpendicularVelocity == 0) maxPerpendicularVelocity = defaultMaxPerpendicularVelocity;
  if (curvilinearSpeed == 0) curvilinearSpeed = defaultCurvilinearSpeed;
  if (carryoverAngularCorrection) correctionSpeed = defaultCarryOverCorrectionSpeed ;
  else correctionSpeed = defaultCorrectionSpeed;
  if (defaultStartWithPumping) needPump = true;
  checkEndThreshold = defaultCheckEndThreshold;
}


void setup() {
  myservo.attach(ServoPin);
  analogWriteFrequency(motorHori_PWM, horiMotorFrequency);
  analogWriteFrequency(motorWall_1PWM, wallMotorFrequency);
  analogWriteFrequency(motorWall_2PWM, wallMotorFrequency);
  analogWriteFrequency(motorVert_PWM, vertMotorFrequency);
  pinMode(limitSwitch_A, INPUT_PULLUP);
  pinMode(limitSwitch_B, INPUT_PULLUP);
  pinMode(limitSwitch_VertCalibrate, INPUT_PULLUP);
  pinMode(motorHori_PWM, OUTPUT);
  pinMode(motorWall_1PWM, OUTPUT);
  pinMode(motorWall_2PWM, OUTPUT);
  pinMode(motorHori_1PinA, OUTPUT);
  pinMode(motorHori_1PinB, OUTPUT);
  pinMode(motorHori_2PinA, OUTPUT);
  pinMode(motorHori_2PinB, OUTPUT);
  pinMode(motorWall_1PinA, OUTPUT);
  pinMode(motorWall_1PinB, OUTPUT);
  pinMode(motorWall_2PinA, OUTPUT);
  pinMode(motorWall_2PinB, OUTPUT);
  pinMode(motorVert_PWM, OUTPUT);
  pinMode(motorVert_PinA, OUTPUT);
  pinMode(motorVert_PinB, OUTPUT);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  //  while (!Serial) {
  //    ; // wait for serial port to connect. Needed for native USB port only
  //  }
  delay(1000);
  Serial.println("----Morpheus Starting----");
  Serial.println(MorpheusVersion);
  Serial.println("----Calibrating Home----");
  calibrateVertHome();
  InitializeCalibration(MANUALSPEEDCALIBRATION);
  if (needPump) {
    delay(500);
  PumpingMarker();
  }
  Serial.println("Access Point Web Server");
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address of will be 192.168.4.1
  // WiFi.config(IPAddress(10, 0, 0, 1));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  bool fail = true;
  while (fail) {
    // keep trying Create open network. Change this line if you want to create an WEP network:
    status = WiFi.beginAP(ssid, pass);
    if (status != WL_AP_LISTENING) {
      Serial.println("Creating access point failed");
    } else {
      break;
    }

  }
  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
  myPIDH.SetMode(AUTOMATIC);
  myPIDH.SetSampleTime(1);
  myPIDH.SetOutputLimits(-255, 255);  // this actually has from negative to positive ... so this is at least ok.
  myPIDV.SetMode(AUTOMATIC);
  myPIDV.SetSampleTime(1);
  myPIDV.SetOutputLimits(-255, 255);//173
  myPIDG.SetMode(AUTOMATIC);
  myPIDG.SetSampleTime(1);
  myPIDG.SetOutputLimits(0 - pwmFactorGLimit , pwmFactorGLimit);
  digitalWrite(motorHori_1PinA, LOW);
  digitalWrite(motorHori_1PinB, LOW);
  digitalWrite(motorHori_2PinA, LOW);
  digitalWrite(motorHori_2PinB, LOW);
  digitalWrite(motorVert_PinA, LOW);
  digitalWrite(motorVert_PinB, LOW);
  digitalWrite(motorWall_1PinA, LOW);
  digitalWrite(motorWall_2PinA, LOW);
  digitalWrite(motorWall_1PinB, LOW);
  digitalWrite(motorWall_2PinB, LOW);
  //  numberOfCommands = sizeof(TestCommands); //GCommands
}


void ResetControlVariables() {
        transitionedToPID = false;
        correctionCCW = 0;
        correctionCW = 0;
        gIntegralError = 0; // reset PID for G
        startedCount = false;
        if (!carryoverAngularCorrection) {
          yCorrection = 1;
          xCorrection = 1;
        }
}

void updateCurrentPosition() {
  currentHoriPos = myHoriEnc.read();
  currentVertPos = myVertEnc.read();
  currentHoriMM = PTS_TO_MM(currentHoriPos - relativeHoriHome, true);
  currentVertMM = PTS_TO_MM(currentVertPos - relativeVertHome, false);
}
  

void bumpSequence() {
  doingCmd = false;
  drawing = false;
  reCalibrateVertical(DONTRETURNTOPREVIOUSPOS);   //  do not return to previous position -> that is basically still on the limit switch.

  unsigned long tempTime = millis(); // millis() uses interrupts ... 
  while ((millis() - tempTime) < 800) { // 0.8 second     . moves up for a while to get away from vertical limit bump.
    moveMotors(0, 170);
  }
  StopAxisMotors(MOTORSTOPALL);

  updateCurrentPosition();
  desiredHoriMM = currentHoriMM;
  desiredVertMM = currentVertMM;
  
  if (client.connected()) client.println("robot~afterBumpingGround"); 
}

double reCalibrateVertical( bool returnToPreviousPosition) {
//  doingCmd = false; // could even be doing cmd when automatic scheduling ! be careful 

  // recalibrates relative vert home, and returns the previously held vertical position, in recalibrated MM distance, assuming no distortion since then until end of function.
  moveServoPos(ServoPosNeutral);
  StopAxisMotors(MOTORSTOPALL);
  
  long tempY = currentVertPos;  // encoder
  double mmDown = currentVertMM; 
  Serial.println("--- Recalibrating vertical home . . .  ---");
  while (true) {
    currentVertPos = myVertEnc.read();
    currentVertMM = PTS_TO_MM(currentVertPos - relativeVertHome, false);
    if (digitalRead(limitSwitch_VertCalibrate) == 0) break;
    else if (currentVertMM > 0.2*mmDown) {
      analogWrite(motorVert_PWM, RecalibrationFullThrottlePWM);
      Serial.println(" moving down full throttle ");
    } else
    {
      analogWrite(motorVert_PWM, motorVertDownDrawingStart + 5 * sqrt(RecalibrationFullThrottlePWM)); // slower speed technically.
      Serial.println(" moving down partial throttle ");
    }
      digitalWrite(motorVert_PinA, HIGH);
      digitalWrite(motorVert_PinB, LOW);
  }
  StopAxisMotors(MOTORSTOPVERT);
  absoluteZeroY = myVertEnc.read();
  relativeVertHome = absoluteZeroY + homeRelativeToAbsZero; //recalibrate home.
  
  double previousVertMM = PTS_TO_MM(tempY - relativeVertHome, false);
  
  if(returnToPreviousPosition) {
    while (true) {
      currentVertPos = myVertEnc.read();
      currentVertMM = PTS_TO_MM(currentVertPos - relativeVertHome, false);
      if ((previousVertMM - currentVertMM) <= 3 ) break;
      else if (currentVertMM < 0.9 * previousVertMM) {
      analogWrite(motorVert_PWM, RecalibrationFullThrottlePWM);
      Serial.println(" moving up full throttle ");
    } else
    {
      analogWrite(motorVert_PWM, motorVertDownDrawingStart + 4 * sqrt(RecalibrationFullThrottlePWM)); // slower speed technically.
      Serial.println(" moving up partial throttle ");
    }
      digitalWrite(motorVert_PinA, LOW);
      digitalWrite(motorVert_PinB, HIGH);
    }
    StopAxisMotors(MOTORSTOPVERT);
    if (drawing) moveServoPos(ServoPosExtend);
  }

//  desiredVertMM = currentVertMM;        // this can't be here, otherwise when dring automatic calibration, and desiredVert and hort already there, wiped out.
//  desiredHoriMM = currentHoriMM;

  Serial.println(" -- Done recalibrating vertical home --");
  ResetPID(RESETALL);
  
  return previousVertMM;
}

void loop() {
  client = server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
    while (client.connected() || doingCmd) {    // don't stop doing if the client is disconnected
      if (read_till_eol()) {
        processCMD(buffer);
      }

      
      // Calculations
      currentHoriPos = myHoriEnc.read();
      currentVertPos = myVertEnc.read();
      currentHoriMM = PTS_TO_MM(currentHoriPos - relativeHoriHome, true);
      currentVertMM = PTS_TO_MM(currentVertPos - relativeVertHome, false);
      double relY = abs(desiredVertMM - currentVertMM);
      double relX = abs(desiredHoriMM - currentHoriMM);
      double signedRelY = desiredVertMM - currentVertMM;
      double signedRelX = desiredHoriMM - currentHoriMM;
      double fullDevX = desiredHoriMM - initialHoriMM;
      double fullDevY = desiredVertMM - initialVertMM;
      double diff = 1,
             gradient = 0,
             inverse_gradient = 0,
             c1,
             c2,
             intersect_X = 0,
             intersect_Y = 0,
             inter_current_relX = 0,
             inter_current_relY = 0;
      if (fullDevX != 0 && fullDevY != 0) {
        gradient = fullDevY / fullDevX;
        inverse_gradient = -1 / gradient;
        c1 = desiredVertMM - desiredHoriMM * gradient;
        c2 = currentVertMM - currentHoriMM * inverse_gradient;
        intersect_X = (c2 - c1) / (gradient - inverse_gradient);
        intersect_Y = gradient * intersect_X + c1;
        inter_current_relX = intersect_X - currentHoriMM;
        inter_current_relY = intersect_Y - currentVertMM;
        pDistance_squared = pow(inter_current_relX, 2) + pow(inter_current_relY, 2);
        pDistance = sqrt(pDistance_squared);
      }
      if (floor(relX) != 0 && floor(relY) != 0) {
        diff = relY / relX;
      }

      // Check end of move.
      bool d = checkEndMove(desiredHoriMM, desiredVertMM, currentHoriMM, currentVertMM);
      if (transitionedToPID == true && !d) {  // in other words, done task.
        ResetControlVariables();
      }

      // PUMPING check block if pumping; may not be needed because pumping code is blocking
      if (pumping) return;
      
      // VERTICAL bump check
      if (digitalRead(limitSwitch_VertCalibrate) != 1) {  // bumped
        // stop to protect motors and motor gears.
        StopAxisMotors(MOTORSTOPALL);
        if (client.connected()) client.println("robot~bumpedGround"); // then begin recalibration, busy. finish, then not busy. if in the midst of drawing, ask mario to resend previous command after finish recalibrating.
        bumpSequence();
        
//        bumped = true;      // not really used.
          /// PANIC . recalibrate vert, and then go up ... pretend previous position is wherever you are right now, make desired to be currentposition. and then doingcmd becomes false, dont let it rampage.
//        doingCmd = false;                                             // if bump vertical limit, stop doingCmd,
// if (client.connected()) client.println("robot~bumpedGround"); 
        
         // this bump is effectively illegal as it is below the relativehome.
         // once this happens, set bump. only clear bump once we get a new command on what to do or what to clear.
        return; // what to do ? must implement in the future. stop, retract, reset all things, and wait for instructions
      }

      //  out of bounds checking
      double eps = EpsilonOutOfBoundsThreshold;   // epsilon ε , a threshold beyond the boundaries for tolerance.
      bool outOfBounds =  (  (currentHoriMM > initialHoriMM + eps && currentHoriMM > desiredHoriMM + eps) ||  // if out out bounds use caleb PID
                                            (currentHoriMM < initialHoriMM - eps && currentHoriMM < desiredHoriMM - eps) ||
                                            (currentVertMM > initialVertMM + eps && currentVertMM > desiredVertMM + eps) ||
                                            (currentVertMM < initialVertMM - eps && currentVertMM < desiredVertMM - eps) );
      bool isP2PPID = ((fullDevX == 0 || fullDevY == 0) || sqrt(pow(relX, 2) + pow(relY, 2)) <= p2pActivationDistanceMM || outOfBounds || transitionedToPID ) ;
      if(doingCmd){
        againstWall = checkWall();
        if (isP2PPID) {                                 /// begin point to point 2-axis-independent PID blind walk.
          transitionedToPID = true;
          myPIDH.Compute();
          myPIDV.Compute();
          double mgradient = 0;
          if (fullDevX != 0) mgradient = fullDevY / fullDevX;
          else mgradient = 1000;
          // handle very steep gradients
          if (mgradient > 300 && sqrt(pow(relX, 2) + pow(relY, 2)) > p2pActivationDistanceMM ) { // there's a an unsolved problem where the 
            pwmHoriOut(0, againstWall, d, drawing, 1);
            pwmVertOut(pwmFactorV, againstWall, d, false, 1);
          } else if (mgradient < 1/300 && sqrt(pow(relX, 2) + pow(relY, 2)) > p2pActivationDistanceMM) {
            pwmHoriOut(pwmFactorH, againstWall, d, false, 1);
            pwmVertOut(0, againstWall, d, drawing, 1);
          }
          else if (diff > 1) {
            //Serial.println(1/diff);
            pwmHoriOut(pwmFactorH, againstWall, d, drawing, 1 / diff);
            pwmVertOut(pwmFactorV, againstWall, d, drawing, 1);
          } else if (diff < 1) {
            //Serial.println(diff);
            pwmHoriOut(pwmFactorH, againstWall, d, drawing, 1);
            pwmVertOut(pwmFactorV, againstWall, d, drawing, diff);
          } else {
            //Serial.println(1);
            pwmHoriOut(pwmFactorH, againstWall, d, drawing, 1);
            pwmVertOut(pwmFactorV, againstWall, d, drawing, 1);
          }
          Serial.print("P2P ->");
          Serial.print(" Current X: ");
          Serial.print(currentHoriPos);
          Serial.print("   ");
          Serial.print(currentHoriMM);
          Serial.print(" Current Y: ");
          Serial.print(currentVertPos);
          Serial.print("   ");
          Serial.print(currentVertMM);
          Serial.print(" :relX");
          Serial.print(relX);
          Serial.print(" :relY");
          Serial.print(relY);
          Serial.print(" :SrelX");
          Serial.print(signedRelX);
          Serial.print(" :SrelY");
          Serial.print(signedRelY);
          Serial.print(" : pwmfactorH");
          Serial.print(pwmFactorH);
          Serial.print(" : pwmfactorV");
          Serial.println(pwmFactorV);
        
      }
          
      else {                // gradient code
        if (againstWall && !transitionedToPID) {  // dont bother doing all the calculations if not on the ground.
//            myPIDG.Compute(); // not working and breaks the teensy or while loop somehow. perhaps the range just needs to be from negative to positive ?

          // Timecounting.
          unsigned long timeNow = micros();
          if (!startedCount) {
            startedCount = true;
            lastMicros = timeNow;
          }
          elapseduS = timeNow - lastMicros;
          if (elapseduS < 0) elapseduS = 1;    // clock has gone over ... every 70 minutes this overflows. just ignore it and don't bother, it's a small time difference.
          lastMicros = timeNow;
          
            // PROPORTIONAL & INTEGRAL EQUATION
            if (pDistance < unsafeRegime) pwmFactorG = Kp_G * pow(pDistance, 1.5) + Ki_G * gIntegralError;
            else if (pDistance >= unsafeRegime) pwmFactorG = Kp_G * pow(pDistance, 2) +  Ki_G * gIntegralError;
            
            // INTEGRAL SUMMATION
            if (pDistance > gIntegralThreshold) gIntegralError += pDistance * elapseduS / 1000000; // integral = sum of error * dt (infinitesimal error) , in seconds
            else gIntegralError = 0;              // SAFE REGIME. reset integral error if close by pDistance - please don't confuse it by wanting to clear 
              //the integral in impossible ways, since integral error is usually cleared by going to the negative error,  
              //        there's no negative error here

            // CLAMP
            pwmFactorG = constrain(pwmFactorG, 0, pwmFactorGLimit);
          double pDistanceAway = pDistance;
          
          // PERPENDICULAR velocity component perpendicular to the gradient direction.
          if (pDistanceAway == 0) {
            perpendicularVelocity_X = 0;
            perpendicularVelocity_Y = 0;
          } else {
            perpendicularVelocity_X = inter_current_relX / pDistanceAway * pwmFactorG / pwmFactorGLimit * maxPerpendicularVelocity ;
            perpendicularVelocity_Y = inter_current_relY / pDistanceAway * pwmFactorG / pwmFactorGLimit * maxPerpendicularVelocity ;
          }
          
          double fullDevDistance = sqrt(fullDevX * fullDevX + fullDevY * fullDevY);
          
          // CURVILINEAR baseline velocity to draw the correct line.
          if (pDistance <= 0.5 * unsafeRegime) {
            curvilinearVelocity_X = fullDevX / fullDevDistance * curvilinearSpeed ;
            curvilinearVelocity_Y = fullDevY / fullDevDistance * curvilinearSpeed ;
          } else {
            double brake = pDistance - (0.5 * unsafeRegime) / 0.7 * unsafeRegime;
            curvilinearVelocity_X = xCorrection * constrain(1 - brake, 0, 1) * fullDevX / fullDevDistance * curvilinearSpeed ;
            curvilinearVelocity_Y = yCorrection * constrain(1 - brake, 0, 1) * fullDevY / fullDevDistance * curvilinearSpeed ;
          }

          // Correction angle and curvilinear correction on the fly .. keep a cumulative account of correction .. and correction of the curvilinear velocity.
          int angle = 0;
          
          if ( (inter_current_relX < 0 - CorrectionAngleDetectionThreshold && currentVertMM - initialVertMM > 3) || 
               (inter_current_relX > CorrectionAngleDetectionThreshold && currentVertMM - initialVertMM < -3) )  {
                // counterclockwise
                angle = angleCCW;
                correctionCCW += elapseduS / 1000000 * constrain(pDistance, 0, 5);    // constrain to exclude egregious anomalies
                 
               }
          else if ( (inter_current_relX < 0 - CorrectionAngleDetectionThreshold && currentVertMM - initialVertMM < -3) || 
               (inter_current_relX > CorrectionAngleDetectionThreshold && currentVertMM - initialVertMM > 3) )  {
                // clockwise
                angle = angleCW;
                correctionCW += elapseduS / 1000000 * constrain(pDistance, 0, 5);
               }
          else angle = angleNEUTRAL;

          //correction speed calculation
          if (!carryoverAngularCorrection) correctionSpeed =  0.2 / ( fullDevDistance / curvilinearSpeed); // 0.1 /( 50% * fulldev / curvispeed). 0.1/0.5 = 0.2 . more short term, should be scaled to the line segment. by 50% of the line segment, 1.1/0.9 scaling is reached. dynamically calculated.
          double delta = correctionSpeed * elapseduS / 1000000 ;  //how much correction change should be applied.
          
          if (abs(correctionCCW - correctionCW ) <= 0.75 * fullDevDistance / curvilinearSpeed ) {   // we do not exclude the case when the motors are crippled -> that is, unable by nature to achieve the required gradient. that is doomed and must be handled another way e.g. 100 high slope.
            // 3 mm deviation average. 3 mm *  (fullDevDistance / curvilinearSpeed) * 0.25 
            // if you've accumulated 3mm of average offset 
            // for at least 25% of the whole full deviation journey expected to travel by curvilinear speed.
            // we correct it.
          } else if (correctionCCW > correctionCW) {
              if (gradient > 0 && fullDevX != 0) {
                xCorrection -= delta;
                yCorrection += delta;
              } else if (gradient < 0) {
                xCorrection += delta;
                yCorrection -= delta;
              }
              // if correct drawing direction is in the top-right or bottom-left quadrant, and CCW correction, means x is too long y too short.
              // if correct drawing direction is in the top-left or bottom-right quadrant, and CCW correction, means x is too short y too long.
          }
          else {
              if (gradient > 0 && fullDevX != 0) {
                xCorrection += delta;
                yCorrection -= delta;
              } else if (gradient < 0) {
                xCorrection -= delta;
                yCorrection += delta;
              }
            // if correct drawing direction is in the top-right or bottom-left quadrant, and CW correction, means x is too short y too long.
            // if correct drawing direction is in the top-left or bottom-right quadrant, and CW correction, means x is too long y too short .
          }

          // PRINTINGS
          Serial.print("Grad->");
          Serial.print(" pDistance: ");
          Serial.print(pDistance);
          Serial.print(" Correction Angle θ : ") ;
          if ( angle == angleCCW ) Serial.print("Counter");
          else if ( angle == angleCW ) Serial.print("Clockwise");
          else Serial.print("Neutral");
          Serial.print(" xCorrection: ");
          Serial.print(xCorrection);
          Serial.print(" yCorrection: ");
          Serial.print(yCorrection);
          Serial.print(" Current X: ");
          Serial.print(currentHoriPos);
          Serial.print("   ");
          Serial.print(currentHoriMM);
          Serial.print(" Current Y: ");
          Serial.print(currentVertPos);
          Serial.print("   ");
          Serial.print(currentVertMM);
          Serial.print(" fullDevX: :");
          Serial.print(fullDevX);
          Serial.print(" fullDevY: ");
          Serial.print(fullDevY);
          Serial.print("\n");
          Serial.print(" perpVelX: ");
          Serial.print(perpendicularVelocity_X);
          Serial.print(" perpVelY: ");
          Serial.print(perpendicularVelocity_Y);
          Serial.print(" curvilinearVelX: ");
          Serial.print(curvilinearVelocity_X);
          Serial.print(" curvilinearVelY: ");
          Serial.print(curvilinearVelocity_Y);
          // combined velocity of curvilinear and perpendicular direction (superposition)
          setSpeedMMtoPWM(curvilinearVelocity_X + perpendicularVelocity_X, curvilinearVelocity_Y + perpendicularVelocity_Y );
        }
      }
     } else{      // if doingCmd, else.
      StopAxisMotors(MOTORSTOPALL);
      StopWallMotors(); 
    }

    // TODO mario should tell us that drawing has finished so that we dont keep pumping ... and mario should tell us the beginning of a new drawing so that we pump, extra hard !
    // only increment pumptimer every time we reach the end of a line segment, while drawing
  }
 }
  Serial.println("idle");       // should the robot continue finishing its movements first before idling if there is no client ?
  //moveServoPos(ServoPosExtend);
  pwmHoriOut(0, true, false, true, 1);
  pwmVertOut(0, true, false, true, 1);
  
  delay(1000);
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();
    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
      readyToServe = true;
    } else {
      if (!doingCmd) {
        Serial.println("Waiting for next command; Please reconnect");
        readyToServe = false;
        delay(2000);
      }
    }
  }
}
