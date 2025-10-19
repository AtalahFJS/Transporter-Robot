#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <ESP32Servo.h>

XboxSeriesXControllerESP32_asukiaaa::Core xboxController("40:8e:2c:4b:e7:0f");
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo gripper;

#define pwm_a 32
#define pin_a1 33
#define pin_a2 25

#define pwm_b 14
#define pin_b1 26
#define pin_b2 27

#define pwm_c 17
#define pin_c1 16
#define pin_c2 4

#define pwm_d 15
#define pin_d1 0
#define pin_d2 2

#define pwm_e 19
#define pin_e1 18
#define pin_e2 5

#define servoPin 23

#define SDA 21
#define SCL 22

int basePWM = 155;
int maxPWM = 255;

bool status = false;
bool statusgripper = false;
bool lastBtnA = false;

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("ROOBICS");
  lcd.setCursor(0, 1);
  lcd.print("Topo bot");

  pinMode(pin_a1, OUTPUT);
  pinMode(pin_a2, OUTPUT);
  pinMode(pin_b1, OUTPUT);
  pinMode(pin_b2, OUTPUT);
  pinMode(pin_c1, OUTPUT);
  pinMode(pin_c2, OUTPUT);
  pinMode(pin_d1, OUTPUT);
  pinMode(pin_d2, OUTPUT);

  pinMode(pwm_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(pwm_c, OUTPUT);
  pinMode(pwm_d, OUTPUT);
  pinMode(pwm_e, OUTPUT);

  pinMode(pin_e1, OUTPUT);
  pinMode(pin_e2, OUTPUT);
  pinMode(pwm_e, OUTPUT);

  Serial.begin(115200);
  xboxController.begin();
  ledcAttach(servoPin, 50, 16);


}

void loop() {
  xboxController.onLoop();

  if (xboxController.isConnected()) {
    int16_t joyLX = xboxController.xboxNotif.joyLHori - 32768;
    int16_t joyLY = xboxController.xboxNotif.joyLVert - 32768;
    int16_t joyRX = xboxController.xboxNotif.joyRHori - 32768;
    int16_t trigR = xboxController.xboxNotif.trigRT;
    bool btnA = xboxController.xboxNotif.btnA;
    bool btnUp = xboxController.xboxNotif.btnDirUp;
    bool btnDown = xboxController.xboxNotif.btnDirDown;

    float normLX = (float)joyLX / 32768.0f;
    float normLY = -(float)joyLY / 32768.0f;
    float normRX = (float)joyRX / 32768.0f;

    if (abs(normLX) < 0.05) normLX = 0;
    if (abs(normLY) < 0.05) normLY = 0;
    if (abs(normRX) < 0.07) normRX = 0;
    int pwmValue = basePWM + map(trigR, 0, 1023, 0, 100);
    pwmValue = constrain(pwmValue, 0, maxPWM);
    if (normRX != 0) {
      mecanumRotate(normRX, pwmValue);
    } else {
      mecanumDrive(normLX, normLY, 0, pwmValue);
    }
    lifterControl(btnUp, btnDown);

    if (btnA && !lastBtnA) {
      status = true;
      stgripper();
    } else {
      status = false;
    }
    lastBtnA = btnA;

    actiongripper();
    }
}

void setMotor(int pwmPin, int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    //analogWrite(pwmPin, 0);
  }
}

void mecanumDrive(float x, float y, float rotation, int pwmBase) {
  float theta = atan2(y, x);
  float power = sqrt(x * x + y * y);
  power = constrain(power, 0.0, 1.0);

  float sinTheta = sin(theta - PI / 4);
  float cosTheta = cos(theta - PI / 4);
  float maxVal = max(abs(sinTheta), abs(cosTheta));

  float vFL = power * cosTheta / maxVal + rotation;
  float vFR = power * sinTheta / maxVal - rotation;
  float vRL = power * sinTheta / maxVal + rotation;
  float vRR = power * cosTheta / maxVal - rotation;

  float maxWheel = max(max(abs(vFL), abs(vFR)), max(abs(vRL), abs(vRR)));
  if (maxWheel > 1.0) {
    vFL /= maxWheel;
    vFR /= maxWheel;
    vRL /= maxWheel;
    vRR /= maxWheel;
  }
  setMotor(pwm_a, pin_a1, pin_a2, vFL * pwmBase);
  setMotor(pwm_b, pin_b1, pin_b2, vFR * pwmBase);
  setMotor(pwm_c, pin_c1, pin_c2, vRL * pwmBase);
  setMotor(pwm_d, pin_d1, pin_d2, vRR * pwmBase);
}

// just an experiment
void mecanumRotate(float rx, int pwmBase) {

  if (rx > 0 && !rx <0) {
    setMotor(pwm_a, pin_a1, pin_a2, rx * pwmBase);
    setMotor(pwm_b, pin_b1, pin_b2, rx * pwmBase);
    // setMotor(pwm_c, pin_c1, pin_c2, 0);
    // setMotor(pwm_d, pin_d1, pin_d2, 0);
  } 
  else if (rx < 0 && rx > 0) {
    // setMotor(pwm_a, pin_a1, pin_a2, 0);
    // setMotor(pwm_b, pin_b1, pin_b2, 0);
    setMotor(pwm_c, pin_c1, pin_c2, -rx * pwmBase);
    setMotor(pwm_d, pin_d1, pin_d2, -rx * pwmBase);
  } 
  // else {
  //   setMotor(pwm_a, pin_a1, pin_a2, 0);
  //   setMotor(pwm_b, pin_b1, pin_b2, 0);
  //   setMotor(pwm_c, pin_c1, pin_c2, 0);
  //   setMotor(pwm_d, pin_d1, pin_d2, 0);
  // }
}

// Lifter
void lifterControl(bool up, bool down) {
  if (up && !down) {
    digitalWrite(pin_e1, HIGH);
    digitalWrite(pin_e2, LOW);
    analogWrite(pwm_e, 255);
  } else if (down && !up) {
    digitalWrite(pin_e1, LOW);
    digitalWrite(pin_e2, HIGH);
    analogWrite(pwm_e, 255);
  } else {
    digitalWrite(pin_e1, LOW);
    digitalWrite(pin_e2, LOW);
  //   analogWrite(pwm_e, 0);
  }
}

void stgripper() {
  if (status == true) {
    if (statusgripper == false) {
      statusgripper = true;
    } else {
      statusgripper = false;
    }
  }
}

void actiongripper() {
  int dutyMin = 1639;
  int dutyMax = 4407;
  if (statusgripper == true) {
    ledcWrite(servoPin, dutyMin);
    delay(100);
  } else {
    ledcWrite(servoPin, dutyMax);
    delay(100);
  }
}
