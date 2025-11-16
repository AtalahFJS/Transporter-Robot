#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

XboxSeriesXControllerESP32_asukiaaa::Core xboxController("40:8e:2c:4b:e7:0f");
LiquidCrystal_I2C lcd(0x27, 16, 2);

//motor A depan kanan
#define pwm_a 32
#define pin_a1 33
#define pin_a2 25

//motor B depan kiri
#define pwm_b 14
#define pin_b1 26
#define pin_b2 27

//motor C belakang kiri
#define pwm_c 17
#define pin_c1 16
#define pin_c2 4

//motor D belakang kanan
#define pwm_d 15
#define pin_d1 0
#define pin_d2 2

//motor lifter
#define pwm_e 19
#define pin_e1 18
#define pin_e2 5

//servo gripper
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
    int btnRight = xboxController.xboxNotif.btnDirRight;
    int btnLeft = xboxController.xboxNotif.btnDirLeft;
    
    Serial.println(btnLeft);
    Serial.println(btnRight);


    float normLX = (float)joyLX / 32768.0f;
    float normLY = -(float)joyLY / 32768.0f;
    float normRX = (float)joyRX / 32768.0f;

    if (abs(normLX) < 0.05) normLX = 0;
    if (abs(normLY) < 0.05) normLY = 0;
    if (abs(normRX) < 0.07) normRX = 0;
    int trigVal = map(trigR, 0, 1023, 0, 100);
    int trigVal2 = map(trigR, 0, 1023, 0, 255);
    int pwmValue = basePWM + trigVal;
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
    gerakbutton(btnLeft,btnRight,trigVal2);
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
  // Hitung kecepatan relatif tiap roda
  float vFL = ( y + x + rotation );
  float vFR = ( y - x - rotation );
  float vRL = ( y - x + rotation );
  float vRR = ( y + x - rotation );

  // Normalisasi agar nilai tidak melebihi 1.0
  float maxWheel = max(max(abs(vFL), abs(vFR)), max(abs(vRL), abs(vRR)));
  if (maxWheel > 1.0) {
    vFL /= maxWheel;
    vFR /= maxWheel;
    vRL /= maxWheel;
    vRR /= maxWheel;
  }

  // Konversi ke PWM dan kirim ke motor
  setMotor(pwm_a, pin_a1, pin_a2, vFL * pwmBase);
  setMotor(pwm_b, pin_b1, pin_b2, vFR * pwmBase);
  setMotor(pwm_c, pin_c1, pin_c2, vRL * pwmBase);
  setMotor(pwm_d, pin_d1, pin_d2, vRR * pwmBase);
}
// just an experiment
void mecanumRotate(float rx, int pwmBase) {

  if (rx > 0) {
    setMotor(pwm_a, pin_a1, pin_a2, rx * pwmBase);
    setMotor(pwm_b, pin_b1, pin_b2, rx * pwmBase);
    // setMotor(pwm_c, pin_c1, pin_c2, 0);
    // setMotor(pwm_d, pin_d1, pin_d2, 0);
  } 
  else if (rx < 0) {
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

void gerakbutton(int buttonLeft, int buttonRight, int boost){
  if(buttonLeft == 1){
    //motor A
    digitalWrite(pin_a1, LOW);
    digitalWrite(pin_a2, HIGH);
    analogWrite(pwm_a, boost);
    //moktor B
    digitalWrite(pin_b1, HIGH);
    digitalWrite(pin_b2, LOW);
    analogWrite(pwm_b, boost);
    //motor C
    digitalWrite(pin_c1, HIGH);
    digitalWrite(pin_c2, LOW);
    analogWrite(pwm_c, boost);
    //motor D
    digitalWrite(pin_d1, LOW);
    digitalWrite(pin_d2, HIGH);
    analogWrite(pwm_d, boost);
  } else if(buttonRight == 1){
    //motor A
    digitalWrite(pin_a1, HIGH);
    digitalWrite(pin_a2, LOW);
    analogWrite(pwm_a, boost);
    //moktor B
    digitalWrite(pin_b1, LOW);
    digitalWrite(pin_b2, HIGH);
    analogWrite(pwm_b, boost);
    //motor C
    digitalWrite(pin_c1, LOW);
    digitalWrite(pin_c2, HIGH);
    analogWrite(pwm_c, boost);
    //motor D
    digitalWrite(pin_d1, HIGH);
    digitalWrite(pin_d2, LOW);
    analogWrite(pwm_d, boost);
  } else {
    digitalWrite(pin_a1, LOW);
    digitalWrite(pin_a2, LOW);
    //analogWrite(pwm_a, 200);
    //moktor B
    digitalWrite(pin_b1, LOW);
    digitalWrite(pin_b2, LOW);
    //analogWrite(pwm_b, 200);
    //motor C
    digitalWrite(pin_c1, LOW);
    digitalWrite(pin_c2, LOW);
    //analogWrite(pwm_c, 200);
    //motor D
    digitalWrite(pin_d1, LOW);
    
    digitalWrite(pin_d2, LOW);
    //analogWrite(pwm_d, 200);
  }
}
