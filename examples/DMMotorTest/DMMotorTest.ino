#include <RP2040PIO_CAN.h>
#include <DMMotorController.h>

// 1: MITモード
// 2: 位置制御モード
// 3: 速度制御モード
const int CONTROL_MODE = 3;
const int DELAY_AFTER_STARTUP_MS = 2000;

// --- CAN設定 ---
const uint8_t CAN_TX_PIN = 0;
const uint8_t CAN_RX_PIN = 1;

// --- モーター設定 ---
const byte MOTOR_ID = 0x01;
DMMotorController motor(MOTOR_ID, CAN);

// --- 制御パラメータ ---
const float VELOCITY_LIMIT_RPM = 50.0f;
const float MIT_KP = 2.0f;
const float MIT_KD = 0.8f;

// --- 状態管理用の変数 ---
int motionStep = 0;
unsigned long stepTimer = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("--- DMMotorController Library Demo ---");

  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  if (CAN.begin(CanBitRate::BR_1000k)) {
    Serial.println("CAN bus initialized.");
  } else {
    Serial.println("CAN bus initialization failed!");
    while (1);
  }

  // --- 起動処理 ---
  Serial.print("Waiting for motor to be enabled...");
  while (!motor.isEnabled()) {
    motor.enableMotor(); 
    motor.update();      
    Serial.print(".");
    delay(100);
  }

  Serial.println("\nMotor enabled. Playing startup sound.");
  motor.playStartupBeep(2000, 100);
  
  Serial.print("Waiting for ");
  Serial.print(DELAY_AFTER_STARTUP_MS);
  Serial.println(" ms before starting operation...");
  delay(DELAY_AFTER_STARTUP_MS); 

  motor.setMode((DMMotorController::ControlMode)CONTROL_MODE);
  Serial.print("Initial mode set to: ");
  Serial.println(CONTROL_MODE);
  Serial.println("Operation started.");

  stepTimer = millis();
  motionStep = 1;
}

void loop() {
  motor.update();

  if (motor.isEnabled()) {
    switch (CONTROL_MODE) {
      case 1: 
        if (motionStep == 1) {
          motor.setMIT(0.0f, 0.0f, MIT_KP, MIT_KD, 0.0f);
          if (millis() - stepTimer > 2000) {
            motionStep = 2;
            stepTimer = millis();
          }
        } else if (motionStep == 2) {
          motor.setMIT(1.57f, 0.0f, MIT_KP, MIT_KD, 0.0f);
          if (millis() - stepTimer > 2000) {
            motionStep = 3;
            stepTimer = millis();
          }
        } else if (motionStep == 3) {
          motor.setMIT(-1.57f, 0.0f, MIT_KP, MIT_KD, 0.0f); 
        }
        break;

      case 2: 
        if (motionStep == 1) {
          motor.goToPosition(0.0f, VELOCITY_LIMIT_RPM);  // 0度
          if (millis() - stepTimer > 2000) {
            motionStep = 2; 
            stepTimer = millis();
          }
        } else if (motionStep == 2) {
          motor.goToPosition(1.57f, VELOCITY_LIMIT_RPM);  // 90度
          if (millis() - stepTimer > 2000) {
            motionStep = 3; 
            stepTimer = millis();
          }
        } else if (motionStep == 3) {
          motor.goToPosition(1.57f, VELOCITY_LIMIT_RPM);  // 90度で停止
        }
        break;

      case 3: 
        motor.setVelocityRPM(60.0f);
        break;
    }

    MotorFeedback feedback;
    if (motor.readFeedback(feedback)) {
      Serial.print("Step: ");
      Serial.print(motionStep);
      Serial.print(", Pos: ");
      Serial.print(feedback.position_deg, 2);
      Serial.print(" deg, Vel: ");
      Serial.print(feedback.velocity_rpm, 2);
      Serial.print(" RPM, Torque: ");
      Serial.print(feedback.torque_nm, 3);
      Serial.println(" Nm");
    }
  } else {
    motor.enableMotor();
  }

  delay(1);
}
