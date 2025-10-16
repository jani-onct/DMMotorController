#ifndef DM_MOTOR_CONTROLLER_H
#define DM_MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <RP2040PIO_CAN.h>

struct MotorFeedback {
  float position_deg;
  float velocity_rpm;
  float torque_nm;
  int8_t temp_mos;
  int8_t temp_rotor;
  uint8_t status;
};

class DMMotorController {
public:
  DMMotorController(byte slaveId, RP2040PIO_CAN &can);

  void begin();
  void enableMotor();

  enum ControlMode {
    MIT = 1,      
    POSITION = 2, 
    VELOCITY = 3  
  };
  void setMode(ControlMode mode);
  ControlMode getMode();

  void goToPosition(float target_rad, float speed_rpm);
  void setVelocityRPM(float target_rpm);
  void setMIT(float p_des, float v_des, float kp, float kd, float t_ff);

  void playStartupBeep(int frequency, int duration_ms);

  bool readFeedback(MotorFeedback &feedback);

private:

  // void enableMotor();
  void switchControlMode(uint8_t mode_code);
  void sendVelocityCommand(float velocity_rad_s);
  void sendPositionCommand(float position_rad, float velocity_limit_rad_s);
  void sendMITCommand(float p_des, float v_des, float kp, float kd, float t_ff);

  float uint_to_float(uint16_t x, float min_val, float max_val, int bits);
  uint16_t float_to_uint(float x, float min_val, float max_val, int bits);

  RP2040PIO_CAN* _can;
  byte _slaveId;
  ControlMode _currentMode;

  static const float P_MAX;
  static const float V_MAX;
  static const float T_MAX;
  static const float KP_MAX;
  static const float KD_MAX;
};

#endif