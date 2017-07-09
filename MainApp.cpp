//#include <EEPROM.h>
#include <Arduino.h>
#include <Bounce2.h>
#include <QTRSensors.h>
#include <avr/pgmspace.h>
#include <math.h>

// #define LED_RED 6
// #define LED_YELLOW 5
// #define LED_GREEN 10

#define MOTOR_1_PIN_A 5
#define MOTOR_1_PIN_B 6

#define MOTOR_2_PIN_A 3
#define MOTOR_2_PIN_B 11

#define SWITCH_INPUT 2
#define SWITCH_GND 4

#define WHEEL_SPEED 140
#define RAMP_UP 240
#define RAMP_DOWN -80

#define GONDALA_3 3
#define ROTATION_FULL 12

#define QTR_PIN_GND PIN_A0   // PC0
#define QTR_PIN_VCC PIN_A2   // PC1
#define QTR_PIN_DATA PIN_A1  // PC2

#define QTR_MAX_VALUE_WHITE 999
#define QTR_MIN_VALUE_BLACK 1000

uint8_t wheelSpeed = 0;
int servoPosition = 0;
bool rampUp = false;
bool wheelGoing = false;
bool sequence = false;
bool debug = false;

Bounce bounce;
QTRSensorsRC qtr;

int counter = 0;
bool qtrTest = false;
bool qtrCalibrate = false;
bool qtrCalibrated = false;
uint16_t startCalibrateMS;
unsigned int qtr_values;
bool isQtrBlack = true;

uint8_t sequence_index = 0;
bool sequence_reverse = false;
static uint8_t sequence_counts[] = {3, 3, 3, 3, 37, 3, 6, 3, 3, 6, 37};
#define SEQUENCE_COUNT_LENGTH 11

void driveWheel(uint8_t speed, bool reverse = false) {
  if (!reverse) {
    analogWrite(MOTOR_1_PIN_A, speed);
    digitalWrite(MOTOR_1_PIN_B, LOW);
  } else {
    analogWrite(MOTOR_1_PIN_B, speed);
    digitalWrite(MOTOR_1_PIN_A, LOW);
  }
  Serial.print(F("Wheel "));
  Serial.println(reverse ? F("REVERSE ") : F("FORWARDS "));
  Serial.println(speed);
}

void driveServo(int position) {
  if (position >= 0) {
    analogWrite(MOTOR_2_PIN_A, position);
    digitalWrite(MOTOR_2_PIN_B, LOW);
  } else {
    analogWrite(MOTOR_2_PIN_B, -position);
    digitalWrite(MOTOR_2_PIN_A, LOW);
  }
  Serial.print(F("Servo "));
  Serial.println(position);
}

void putRampUp() {
  driveServo(RAMP_UP);
  Serial.println(F("Ramp Up"));
  rampUp = true;
}

void putRampDown() {
  driveServo(RAMP_DOWN);
  Serial.println(F("Ramp Down"));
  rampUp = false;
}

void startWheel() {
  driveWheel(WHEEL_SPEED, sequence_reverse);
  Serial.println(F("Wheel Going "));
  wheelGoing = true;
}

void stopWheel() {
  driveWheel(0);
  Serial.println(F("Wheel Stopped"));
  wheelGoing = false;
}

void setup() {
  Serial.begin(57600);
  delay(1000);
  Serial.println(F("HELLO "));

  pinMode(SWITCH_GND, OUTPUT);
  digitalWrite(SWITCH_GND, LOW);
  pinMode(SWITCH_INPUT, INPUT_PULLUP);

  pinMode(QTR_PIN_GND, OUTPUT);
  digitalWrite(QTR_PIN_GND, LOW);
  pinMode(QTR_PIN_VCC, OUTPUT);
  digitalWrite(QTR_PIN_VCC, HIGH);
  unsigned char pins[1] = {QTR_PIN_DATA};
  qtr.init(pins, (unsigned char)1);

  pinMode(MOTOR_1_PIN_A, OUTPUT);
  pinMode(MOTOR_1_PIN_B, OUTPUT);
  digitalWrite(MOTOR_1_PIN_A, LOW);
  digitalWrite(MOTOR_1_PIN_B, LOW);

  pinMode(MOTOR_2_PIN_A, OUTPUT);
  pinMode(MOTOR_2_PIN_B, OUTPUT);
  digitalWrite(MOTOR_2_PIN_A, LOW);
  digitalWrite(MOTOR_2_PIN_B, LOW);

  bounce.attach(SWITCH_INPUT);
  bounce.interval(5);

  putRampDown();
  stopWheel();
}

bool startedFromSwitch = false;

void loop() {
  bounce.update();
  if (bounce.fell()) {
    if (!sequence) {
      // sequence = true;
      putRampDown();
      if (!qtrCalibrated) {
        sequence_reverse = false;
        sequence_index = 0;
        Serial.println(F("Start Sequence Calibrate"));
        qtrCalibrate = true;
        startCalibrateMS = millis();
        startedFromSwitch = true;
      }
      startWheel();
    } else {
      Serial.println(F("Stop Sequence"));
      qtrCalibrate = false;
      sequence = false;
      stopWheel();
      putRampDown();
    }
  }

  if (sequence) {
    qtr.readCalibrated(&qtr_values);
    if (debug) {
      Serial.print("QTR Value ");
      Serial.println(qtr_values);
    }
    if (isQtrBlack && qtr_values < QTR_MAX_VALUE_WHITE) {  // switched to white
      Serial.println(F("White"));
      isQtrBlack = false;
      counter++;
      if (counter > sequence_counts[sequence_index]) {  // GONDALA_3){
        counter = 0;

        stopWheel();
        delay(1000);
        putRampUp();
        delay(10000);
        putRampDown();
        delay(1000);
        if (sequence) {
          startWheel();
        }
        sequence_index++;
        if (sequence_index >= SEQUENCE_COUNT_LENGTH) {
          sequence_index = 0;
          sequence_reverse = !sequence_reverse;
        }
      }
      Serial.print(F("Counter "));
      Serial.print(counter);
      Serial.print(F(" Of "));
      Serial.println(sequence_counts[sequence_index]);
    } else if (!isQtrBlack &&
               qtr_values >= QTR_MIN_VALUE_BLACK) {  // switched to black
      isQtrBlack = true;
      Serial.println(F("Black"));
    }
    // delay(10);
  }

  //   counter++;

  //   if(sequence && counter == GONDALA_3){
  //     counter =0;
  //     stopWheel();
  //     delay(1000);
  //     putRampUp();
  //     delay(10000);
  //     putRampDown();
  //     delay(1000);
  //     if(!sequence){
  //       startWheel();
  //     }
  //   }
  //   Serial.print(F("Counter"));
  //   Serial.println(counter);
  // }

  if (qtrTest) {
    qtr.read(&qtr_values);
    Serial.print(F("QTR Value "));
    Serial.print(qtr_values);
    Serial.print(F(" C "));
    qtr.readCalibrated(&qtr_values);
    Serial.println(qtr_values);
    delay(10);
  }

  if (qtrCalibrate) {
    qtr.calibrate();
    // delay(5);
    if (millis() - startCalibrateMS > 30000) {
      qtrCalibrate = false;
      Serial.print(F("Calibrate Finished "));
      Serial.print((int)qtr.calibratedMinimumOn);
      Serial.print(" ");
      Serial.println((int)qtr.calibratedMaximumOn);
      Serial.println(F("Starting Sequence"));
      // qtrT//iming = true;
      qtrCalibrated = true;
      // isQtrBlack = false;
      if (startedFromSwitch) {
        sequence = true;
      } else {
        stopWheel();
      }
      counter = 0;
    }
  }
}

void serialEvent() {
  if (Serial.available() > 0) {
    uint8_t byte1 = Serial.read();
    if (byte1 == '0') {
      while (!Serial.available()) {
        delay(1);
      }
      uint8_t byte2 = Serial.read();
      if (byte2 == ' ') {
        // Serial.println(F("Reboot"));
        // pinMode(0, INPUT);
        cli();
        MCUSR = 0;
        goto * 0x7800;  // bootloader.
      }
    }

    if (!sequence) {
      if (byte1 == '+') {
        if (wheelSpeed < 250) {
          wheelSpeed += 10;
          driveWheel(wheelSpeed);
        }
      }
      if (byte1 == '-') {
        if (wheelSpeed - 10 > 0) {
          wheelSpeed -= 10;
          driveWheel(wheelSpeed);
        }
      }

      if (byte1 == '>') {
        if (servoPosition < 255) {
          servoPosition += 5;
          driveServo(servoPosition);
        }
      }
      if (byte1 == '<') {
        if (servoPosition - 5 > -255) {
          servoPosition -= 5;
          driveServo(servoPosition);
        }
      }
      if (byte1 == 'w') {
        if (wheelGoing) {
          stopWheel();
        } else {
          startWheel();
        }
      }
      if (byte1 == 'r') {
        if (rampUp) {
          putRampDown();
        } else {
          putRampUp();
        }
      }
      if (byte1 == 'q') {
        qtrTest = !qtrTest;
      }

      if (byte1 == 'c') {
        Serial.println(F("Calibrating again"));
        stopWheel();
        putRampDown();
        qtrCalibrated = false;
        qtrCalibrate = true;
        startCalibrateMS = millis();
        startWheel();
      }
    }
    if (byte1 == 's') {
      if (sequence) {
        Serial.println(F("Sequence Stop"));
        stopWheel();
        sequence = false;
        counter = 0;
      } else {
        if (!qtrCalibrated) {
          Serial.println(F("Not Calibrated"));
        } else {
          Serial.println(F("Sequence Start"));
          startedFromSwitch = false;
          sequence = true;
          startWheel();
          counter = 0;
          sequence_reverse = false;
          sequence_index = 0;
        }
      }
    }

    if (byte1 == 'd') {
      debug = !debug;
      if (debug) {
        Serial.println(F("Debug On"));
      } else {
        Serial.println(F("Debug Off"));
      }
    }

    if (byte1 == 'v') {
      sequence_reverse = !sequence_reverse;
    }
  }
}
