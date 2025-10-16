#include "DMMotorController.h"

const float DMMotorController::P_MAX = 12.5f;
const float DMMotorController::V_MAX = 30.0f;
const float DMMotorController::T_MAX = 10.0f;
const float DMMotorController::KP_MAX = 500.0f;
const float DMMotorController::KD_MAX = 5.0f;

DMMotorController::DMMotorController(byte slaveId, RP2040PIO_CAN &can)
  : _slaveId(slaveId), _can(&can), _currentMode(MIT) {} // 初期モードをMITに設定

void DMMotorController::begin() {
  enableMotor();
}

void DMMotorController::setMode(ControlMode mode) {
  _currentMode = mode;
  switchControlMode(mode);
}

DMMotorController::ControlMode DMMotorController::getMode() {
    return _currentMode;
}

void DMMotorController::goToPosition(float target_rad, float speed_rpm) {
  float velocity_limit_rad_s = speed_rpm * (PI / 30.0f);
  sendPositionCommand(target_rad, velocity_limit_rad_s);
}

void DMMotorController::setVelocityRPM(float target_rpm) {
  float velocity_rad_s = target_rpm * (PI / 30.0f);
  sendVelocityCommand(velocity_rad_s);
}

void DMMotorController::setMIT(float p_des, float v_des, float kp, float kd, float t_ff) {
  sendMITCommand(p_des, v_des, kp, kd, t_ff);
}

bool DMMotorController::readFeedback(MotorFeedback &feedback) {
  if (_can->available() > 0) {
    CanMsg rxMsg = _can->read();
    if (rxMsg.id == 0x000) {
      uint16_t pos_raw = (uint16_t)(rxMsg.data[1] << 8 | rxMsg.data[2]);
      uint16_t vel_raw = (uint16_t)((rxMsg.data[3] << 4) | (rxMsg.data[4] >> 4));
      uint16_t tor_raw = (uint16_t)(((rxMsg.data[4] & 0x0F) << 8) | rxMsg.data[5]);
      
      float position_rad = uint_to_float(pos_raw, -P_MAX, P_MAX, 16);
      
      feedback.position_deg = position_rad * 180.0f / PI;
      feedback.velocity_rpm = uint_to_float(vel_raw, -V_MAX, V_MAX, 12) * 30.0f / PI;
      feedback.torque_nm = uint_to_float(tor_raw, -T_MAX, T_MAX, 12);
      feedback.temp_mos = (int8_t)rxMsg.data[6];
      feedback.temp_rotor = (int8_t)rxMsg.data[7];
      return true;
    }
  }
  return false;
}

void DMMotorController::playStartupBeep(int frequency, int duration_ms) {
  long period_us = 1000000 / frequency;
  long half_period_us = period_us / 2;
  long iterations = (long)duration_ms * 1000 / period_us;

  uint8_t original_mode = _currentMode;
  switchControlMode(3); 
  delay(10);

  for (long i = 0; i < iterations; i++) {
    sendVelocityCommand(0.5f);
    delayMicroseconds(half_period_us);
    sendVelocityCommand(-0.5f);
    delayMicroseconds(half_period_us);
  }
  sendVelocityCommand(0.0f);

  delay(10);

  switchControlMode(original_mode);
}

void DMMotorController::enableMotor() {
  CanMsg txMsg = {};
  txMsg.id = _slaveId;
  txMsg.data_length = 8;
  for (int i = 0; i < 7; i++) { txMsg.data[i] = 0xFF; }
  txMsg.data[7] = 0xFC;
  _can->write(txMsg);
  delay(100);
}

void DMMotorController::switchControlMode(uint8_t mode_code) {
  CanMsg txMsg = {};
  txMsg.id = 0x7FF;
  txMsg.data_length = 8;
  txMsg.data[0] = _slaveId;
  txMsg.data[1] = 0x00;
  txMsg.data[2] = 0x55;
  txMsg.data[3] = 0x0A;
  txMsg.data[4] = mode_code;
  for (int i = 5; i < 8; i++) { txMsg.data[i] = 0x00; }
  _can->write(txMsg);
  delay(10);
}

void DMMotorController::sendVelocityCommand(float velocity_rad_s) {
  CanMsg txMsg = {};
  union { float f; byte b[4]; } converter;
  converter.f = velocity_rad_s;
  
  txMsg.id = 0x200 + _slaveId;
  txMsg.data_length = 8;
  for (int i = 0; i < 4; i++) { txMsg.data[i] = converter.b[i]; }
  for (int i = 4; i < 8; i++) { txMsg.data[i] = 0; }
  _can->write(txMsg);
}

void DMMotorController::sendPositionCommand(float position_rad, float velocity_limit_rad_s) {
  CanMsg txMsg = {};
  union { float f; byte b[4]; } pos_conv, vel_conv;
  pos_conv.f = position_rad;
  vel_conv.f = velocity_limit_rad_s;

  txMsg.id = 0x100 + _slaveId;
  txMsg.data_length = 8;
  for (int i = 0; i < 4; i++) { txMsg.data[i] = pos_conv.b[i]; }
  for (int i = 0; i < 4; i++) { txMsg.data[i+4] = vel_conv.b[i]; }
  _can->write(txMsg);
}

void DMMotorController::sendMITCommand(float p_des, float v_des, float kp, float kd, float t_ff) {
  CanMsg txMsg = {};
  txMsg.id = _slaveId;
  txMsg.data_length = 8;
  
  uint16_t p_uint = float_to_uint(p_des, -P_MAX, P_MAX, 16);
  uint16_t v_uint = float_to_uint(v_des, -V_MAX, V_MAX, 12);
  uint16_t kp_uint = float_to_uint(kp, 0, KP_MAX, 12);
  uint16_t kd_uint = float_to_uint(kd, 0, KD_MAX, 12);
  uint16_t t_uint = float_to_uint(t_ff, -T_MAX, T_MAX, 12);

  txMsg.data[0] = p_uint >> 8;
  txMsg.data[1] = p_uint & 0xFF;
  txMsg.data[2] = v_uint >> 4;
  txMsg.data[3] = ((v_uint & 0x0F) << 4) | (kp_uint >> 8);
  txMsg.data[4] = kp_uint & 0xFF;
  txMsg.data[5] = kd_uint >> 4;
  txMsg.data[6] = ((kd_uint & 0x0F) << 4) | (t_uint >> 8);
  txMsg.data[7] = t_uint & 0xFF;

  _can->write(txMsg);
}

float DMMotorController::uint_to_float(uint16_t x, float min_val, float max_val, int bits) {
  float span = max_val - min_val;
  float normalized = (float)x / (float)((1 << bits) - 1);
  return normalized * span + min_val;
}

uint16_t DMMotorController::float_to_uint(float x, float min_val, float max_val, int bits) {
  float span = max_val - min_val;
  float clamped_x = (x < min_val) ? min_val : ((x > max_val) ? max_val : x);
  float normalized = (clamped_x - min_val) / span;
  return (uint16_t)(normalized * ((1 << bits) - 1));
}