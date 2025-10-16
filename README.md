## **API Reference**

### **DMMotorController(byte slaveId, RP2040PIO\_CAN \&can)**

Constructor. Creates a motor controller object.

### **void begin()**

Enables the motor. Call this in your setup() function.

### **void setMode(ControlMode mode)**

Sets the control mode.

* DMMotorController::MIT  
* DMMotorController::POSITION  
* DMMotorController::VELOCITY

### **void goToPosition(float target\_rad, float speed\_rpm)**

Commands the motor to move to a target position (in radians) with a maximum speed (in RPM). (Position Mode)

### **void setVelocityRPM(float target\_rpm)**

Commands the motor to spin at a target velocity (in RPM). (Velocity Mode)

### **void setMIT(float p\_des, float v\_des, float kp, float kd, float t\_ff)**

Sends a command in MIT mode.

* p\_des: Target position (rad)  
* v\_des: Target velocity (rad/s)  
* kp: Position gain  
* kd: Velocity gain  
* t\_ff: Torque feed-forward (Nm)

### **bool readFeedback(MotorFeedback \&feedback)**

Reads feedback data from the motor. Returns true if a new message was received. The data is stored in the feedback struct.

### **void playStartupBeep(int frequency, int duration\_ms)**

Plays a beep sound using the motor.

## **License**

This library is released under the [MIT License](https://opensource.org/licenses/MIT).