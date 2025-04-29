//  ███████████   █████████   ████████
// ░░███░░░░░███ ███░░░░░███ ███░░░░███
//  ░███    ░███░███    ░░░ ░░░    ░███
//  ░██████████ ░░█████████    ███████
//  ░███░░░░░░   ░░░░░░░░███  ███░░░░
//  ░███         ███    ░███ ███      █
//  █████       ░░█████████ ░██████████
// ░░░░░         ░░░░░░░░░  ░░░░░░░░░░

// ██████  ███████ ███████ ██ ███    ██ ███████     ██████  ███████ ██████ 
// ██   ██ ██      ██      ██ ████   ██ ██          ██   ██ ██           ██ 
// ██   ██ █████   █████   ██ ██ ██  ██ █████       ██████  ███████  █████  
// ██   ██ ██      ██      ██ ██  ██ ██ ██          ██           ██ ██     
// ██████  ███████ ██      ██ ██   ████ ███████     ██      ███████ ███████ 
//-------------------------------------------------Define PS2 Related Stuff-----------------------------------------------------------//
#include <PS2X_lib.h>
#define PS2_CLK 13  // PS2 Controller Clock pin
#define PS2_DAT 10  // PS2 Controller Data pin
#define PS2_CMD 11  // PS2 Controller Command pin
#define PS2_SEL 12  // PS2 Controller Select pin

PS2X ps2x;         // PS2X library instance for PS2 controller
int error = 0;     // Variable to store PS2 controller initialization error
byte vibrate = 0;  // Vibration control variable
byte type = 0;     // Vibration ps2 controller type
//----------------------------------------------------------------------------------------------------------------------------------//
// ██████  ███████ ██████      ███████ ███████ ████████     ██    ██ ██████ 
// ██   ██ ██           ██     ██      ██         ██        ██    ██ ██   ██ 
// ██████  ███████  █████      ███████ █████      ██        ██    ██ ██████  
// ██           ██ ██               ██ ██         ██        ██    ██ ██     
// ██      ███████ ███████     ███████ ███████    ██         ██████  ██ 
//-------------------------------------------------Set Up PS2 Receiver-----------------------------------------------------------//
void ps2_setup() {
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);  //GamePad(clock(CLK), command(CMD), attention(CS), data(DAT), Pressures?, Rumble?)

  if (error == 0) {
    Serial.println("Found Controller, configured successful");
  }

  else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println("Unknown Controller type");
      break;
    case 1:
      Serial.println("DualShock Controller Found");
      break;
    case 2:
      Serial.println("GuitarHero Controller Found");
      break;
  }
}
//----------------------------------------------------------------------------------------------------------------------------//









//  ██████   ██████                                                                      █████   ███   █████ █████                        ████
// ░░██████ ██████                                                                      ░░███   ░███  ░░███ ░░███                        ░░███
//  ░███░█████░███   ██████   ██████   ██████   ████████   █████ ████ █████████████      ░███   ░███   ░███  ░███████    ██████   ██████  ░███   █████
//  ░███░░███ ░███  ███░░███ ███░░███ ░░░░░███ ░░███░░███ ░░███ ░███ ░░███░░███░░███     ░███   ░███   ░███  ░███░░███  ███░░███ ███░░███ ░███  ███░░
//  ░███ ░░░  ░███ ░███████ ░███ ░░░   ███████  ░███ ░███  ░███ ░███  ░███ ░███ ░███     ░░███  █████  ███   ░███ ░███ ░███████ ░███████  ░███ ░░█████
//  ░███      ░███ ░███░░░  ░███  ███ ███░░███  ░███ ░███  ░███ ░███  ░███ ░███ ░███      ░░░█████░█████░    ░███ ░███ ░███░░░  ░███░░░   ░███  ░░░░███
//  █████     █████░░██████ ░░██████ ░░████████ ████ █████ ░░████████ █████░███ █████       ░░███ ░░███      ████ █████░░██████ ░░██████  █████ ██████
// ░░░░░     ░░░░░  ░░░░░░   ░░░░░░   ░░░░░░░░ ░░░░ ░░░░░   ░░░░░░░░ ░░░░░ ░░░ ░░░░░         ░░░   ░░░      ░░░░ ░░░░░  ░░░░░░   ░░░░░░  ░░░░░ ░░░░░░

// ██████  ███████ ███████ ██ ███    ██ ███████     ███    ███  ██████  ████████  ██████  ██████      ██████  ██ ███    ██ ███████ 
// ██   ██ ██      ██      ██ ████   ██ ██          ████  ████ ██    ██    ██    ██    ██ ██   ██     ██   ██ ██ ████   ██ ██      
// ██   ██ █████   █████   ██ ██ ██  ██ █████       ██ ████ ██ ██    ██    ██    ██    ██ ██████      ██████  ██ ██ ██  ██ ███████ 
// ██   ██ ██      ██      ██ ██  ██ ██ ██          ██  ██  ██ ██    ██    ██    ██    ██ ██   ██     ██      ██ ██  ██ ██      ██ 
// ██████  ███████ ██      ██ ██   ████ ███████     ██      ██  ██████     ██     ██████  ██   ██     ██      ██ ██   ████ ███████ 
//-------------------------------------------------Define Motor Pins-----------------------------------------------------------//
// Motor FL controller (Front Left)
int motor_FL_speed_pin = 2;
int motor_FL_brk_pin = 22;
int motor_FL_dir_pin = 23;
int motor_FL_read_speed_pin = 18;  // Digital Pins With Interrupts	2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)
volatile int motor_FL_numRevolutions = 0;
int motor_FL_rpm = 0;


// Motor FR controller(Front right)
int motor_FR_speed_pin = 3;
int motor_FR_brk_pin = 24;
int motor_FR_dir_pin = 25;
int motor_FR_read_speed_pin = 19;  // Digital Pins With Interrupts	2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)
volatile int motor_FR_numRevolutions = 0;
int motor_FR_rpm = 0;


// Motor RL controller (Rear Left)
int motor_RL_speed_pin = 4;
int motor_RL_brk_pin = 26;
int motor_RL_dir_pin = 27;
int motor_RL_read_speed_pin = 20;  // Digital Pins With Interrupts	2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)
volatile int motor_RL_numRevolutions = 0;
int motor_RL_rpm = 0;


// Motor RR controller (Rear Right)
int motor_RR_speed_pin = 5;
int motor_RR_brk_pin = 28;
int motor_RR_dir_pin = 29;
int motor_RR_read_speed_pin = 21;  // Digital Pins With Interrupts	2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)
volatile int motor_RR_numRevolutions = 0;
int motor_RR_rpm = 0;

// ms counter
unsigned long lastMillis = 0;

// Variable for speed adjustment
float motor_speed_rate = 0.2;
float motor_speed_rate_adj = 0.1;
float motor_speed_rate_max = 0.7;
float motor_speed_rate_min = 0.1;
//----------------------------------------------------------------------------------------------------------------------------------//
// ███    ███  ██████  ████████  ██████  ██████      ██████  ██ ███    ██ ███████     ███████ ███████ ████████ ██    ██ ██████ 
// ████  ████ ██    ██    ██    ██    ██ ██   ██     ██   ██ ██ ████   ██ ██          ██      ██         ██    ██    ██ ██   ██ 
// ██ ████ ██ ██    ██    ██    ██    ██ ██████      ██████  ██ ██ ██  ██ ███████     ███████ █████      ██    ██    ██ ██████  
// ██  ██  ██ ██    ██    ██    ██    ██ ██   ██     ██      ██ ██  ██ ██      ██          ██ ██         ██    ██    ██ ██     
// ██      ██  ██████     ██     ██████  ██   ██     ██      ██ ██   ████ ███████     ███████ ███████    ██     ██████  ██ 
//---------------------------------------------------------------Motor Pins Setup-------------------------------------------------------------//
void motor_pins_setup() {
  // Set Pin Mode
  pinMode(motor_FL_speed_pin, OUTPUT);
  pinMode(motor_FL_dir_pin, OUTPUT);
  pinMode(motor_FL_brk_pin, OUTPUT);
  pinMode(motor_FL_read_speed_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_FL_read_speed_pin), motor_FL_numRevolutions, RISING);  // Attach an interrupt to the sensor pin

  pinMode(motor_FR_speed_pin, OUTPUT);
  pinMode(motor_FR_dir_pin, OUTPUT);
  pinMode(motor_FR_brk_pin, OUTPUT);
  pinMode(motor_FR_read_speed_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_FR_read_speed_pin), motor_FR_Revolutions_counter, RISING);  // Attach an interrupt to the sensor pin

  pinMode(motor_RL_speed_pin, OUTPUT);
  pinMode(motor_RL_dir_pin, OUTPUT);
  pinMode(motor_RL_brk_pin, OUTPUT);
  pinMode(motor_RL_read_speed_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_RL_read_speed_pin), motor_RL_Revolutions_counter, RISING);  // Attach an interrupt to the sensor pin

  pinMode(motor_RR_speed_pin, OUTPUT);
  pinMode(motor_RR_dir_pin, OUTPUT);
  pinMode(motor_RR_brk_pin, OUTPUT);
  pinMode(motor_RR_read_speed_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_RR_read_speed_pin), motor_RR_Revolutions_counter, RISING);  // Attach an interrupt to the sensor pin
}
//----------------------------------------------------------------------------------------------------------------------------//


// ██████  ██████  ██ ██    ██ ███████         ███    ███  ██████  ██████  ███████      ██ ██ 
// ██   ██ ██   ██ ██ ██    ██ ██              ████  ████ ██    ██ ██   ██ ██          ██   ██ 
// ██   ██ ██████  ██ ██    ██ █████           ██ ████ ██ ██    ██ ██   ██ █████       ██   ██ 
// ██   ██ ██   ██ ██  ██  ██  ██              ██  ██  ██ ██    ██ ██   ██ ██          ██   ██ 
// ██████  ██   ██ ██   ████   ███████ ███████ ██      ██  ██████  ██████  ███████      ██ ██  
//-----------------------------------------------------------------drive_mode()-----------------------------------------------------------------//
void drive_mode() {
  // Serial.println("motorAction");
  int speed_val = 20;
  int lr_dif = 10;

  // Map Joystick Values
  int L_Joy_Y = map(ps2x.Analog(PSS_LY), 255, 0, 255, -255);
  int L_Joy_X = map(ps2x.Analog(PSS_LX), 0, 255, 255, -255);
  int R_Joy_X = map(ps2x.Analog(PSS_RX), 0, 255, -255, 255);

  // // Set centre deadzone
  // int deadzone = 11;
  // L_Joy_X = L_Joy_Y <= deadzone && L_Joy_Y >= -deadzone ? 0 : L_Joy_Y;
  // L_Joy_X = L_Joy_X <= deadzone && L_Joy_X >= -deadzone ? 0 : L_Joy_X;
  // R_Joy_X = R_Joy_X <= deadzone && R_Joy_X >= -deadzone ? 0 : R_Joy_X;

  // Overwrite Joystick values if PSB_PAD_UP/PSB_PAD_DOWN/PSB_PAD_LEFT/PSB_PAD_RIGHT  is pressed. Make the wheel goes absolutely Front/Back/Left/Right
  if (ps2x.Button(PSB_PAD_UP)) {
    // Serial.println("UP");
    L_Joy_Y = 255;
    L_Joy_X = 0;
    R_Joy_X = 0;
  } else if (ps2x.Button(PSB_PAD_DOWN)) {
    // Serial.println("DOWN");
    L_Joy_Y = -255;
    L_Joy_X = 0;
    R_Joy_X = 0;
  } else if (ps2x.Button(PSB_PAD_LEFT)) {
    // Serial.println("LEFT");
    L_Joy_Y = 0;
    L_Joy_X = -255;
    R_Joy_X = 0;
  } else if (ps2x.Button(PSB_PAD_RIGHT)) {
    // Serial.println("RIGHT");
    L_Joy_Y = 0;
    L_Joy_X = 255;
    R_Joy_X = 0;
  }


  calculate_Motor_Power_For_Mecanum_Wheel(L_Joy_Y, L_Joy_X, R_Joy_X);
}

void calculate_Motor_Power_For_Mecanum_Wheel(int forward, int sideway, int rotate) {
  // constrain motor power within -255 - 255
  int FL = constrain((forward + sideway + rotate), -255, 255);
  int FR = constrain((forward - sideway - rotate), -255, 255);
  int RL = constrain((forward - sideway + rotate), -255, 255);
  int RR = constrain((forward + sideway - rotate), -255, 255);

  // Serial.println(FL);
  // Serial.println(FR);
  // Serial.println(RL);
  // Serial.println(RR);


  // Change speed level (speed rate)
  if (ps2x.Button(PSB_L1) && motor_speed_rate > motor_speed_rate_min) {
    motor_speed_rate -= motor_speed_rate_adj;
  }
  if (ps2x.Button(PSB_R1) && motor_speed_rate < motor_speed_rate_max) {
    motor_speed_rate += motor_speed_rate_adj;
  }

  // Adjust Motor Power Output to speed level (speed rate)
  FL *= motor_speed_rate;
  FR *= motor_speed_rate;
  RL *= motor_speed_rate;
  RR *= motor_speed_rate;



  RPM_read();
  int FL_pid_adjusted = pid_cal(1, FL, motor_FL_rpm);
  int FR_pid_adjusted = pid_cal(2, FR, motor_FR_rpm);
  int RL_pid_adjusted = pid_cal(3, RL, motor_RL_rpm);
  int RR_pid_adjusted = pid_cal(4, RR*6, motor_RR_rpm);

  Serial.print("\n");
  Serial.print(RR);
  Serial.print(" : ");
  Serial.print(RR_pid_adjusted);


  // Break the motor when power within a range
  int break_range = 10;
  digitalWrite(motor_FL_brk_pin, (FL <= break_range && FL >= -break_range) ? HIGH : LOW);  // Motor FL break
  digitalWrite(motor_FR_brk_pin, (FR <= break_range && FR >= -break_range) ? HIGH : LOW);  // Motor FR break
  digitalWrite(motor_RL_brk_pin, (RL <= break_range && RL >= -break_range) ? HIGH : LOW);  // Motor RL break
  digitalWrite(motor_RR_brk_pin, (RR <= break_range && RR >= -break_range) ? HIGH : LOW);  // Motor RR break
  // Write motor power
  analogWrite(motor_FL_speed_pin, abs(FL));  // Motor FL speed, positive value
  analogWrite(motor_FR_speed_pin, abs(FR));  // Motor FR speed, positive value
  analogWrite(motor_RL_speed_pin, abs(RL));  // Motor RL speed, positive value
  analogWrite(motor_RR_speed_pin, abs(RR));  // Motor RR speed, positive value

  // Change Motor Direction if Power value is negative
  digitalWrite(motor_FL_dir_pin, FL < 0 ? HIGH : LOW);  // Motor FL direction
  digitalWrite(motor_FR_dir_pin, FR < 0 ? LOW : HIGH);  // Motor FR direction
  digitalWrite(motor_RL_dir_pin, RL < 0 ? HIGH : LOW);  // Motor RL direction
  digitalWrite(motor_RR_dir_pin, RR < 0 ? LOW : HIGH);  // Motor RR direction
}
//----------------------------------------------------------------------------------------------------------------------------------//



// ██████  ██████  ███    ███         ██████  ███████  █████  ██████       ██ ██ 
// ██   ██ ██   ██ ████  ████         ██   ██ ██      ██   ██ ██   ██     ██   ██ 
// ██████  ██████  ██ ████ ██         ██████  █████   ███████ ██   ██     ██   ██ 
// ██   ██ ██      ██  ██  ██         ██   ██ ██      ██   ██ ██   ██     ██   ██ 
// ██   ██ ██      ██      ██ ███████ ██   ██ ███████ ██   ██ ██████       ██ ██  
//-------------------------------------------------------------------RPM_read()---------------------------------------------------------------//
void RPM_read() {
  if (millis() - lastMillis >= 1000) {  // Calculate RPM every second
    // for motor_FL
    detachInterrupt(digitalPinToInterrupt(motor_FL_read_speed_pin));  // Detach interrupt before calculating RPM
    motor_FL_rpm = motor_FL_numRevolutions * 60;                      // Calculate RPM
    motor_FL_rpm /= 12;
    Serial.print("\n\nmotor_FR_rpm = ");
    Serial.println(motor_FL_rpm);                                                                      // Print RPM
    motor_FL_numRevolutions = 0;                                                                       // Reset the revolution count
    lastMillis = millis();                                                                             // Reset the last counted time
    attachInterrupt(digitalPinToInterrupt(motor_FL_read_speed_pin), motor_FL_numRevolutions, RISING);  // Re-attach interrupt after calculating RPM

    // for motor_RR
    detachInterrupt(digitalPinToInterrupt(motor_FR_read_speed_pin));  // Detach interrupt before calculating RPM
    motor_FR_rpm = motor_FR_numRevolutions * 60;                      // Calculate RPM
    motor_FR_rpm /= 12;
    Serial.print("motor_FR_rpm = ");
    Serial.println(motor_FR_rpm);                                                                           // Print RPM
    motor_FR_numRevolutions = 0;                                                                            // Reset the revolution count
    lastMillis = millis();                                                                                  // Reset the last counted time
    attachInterrupt(digitalPinToInterrupt(motor_FR_read_speed_pin), motor_FR_Revolutions_counter, RISING);  // Re-attach interrupt after calculating RPM

    // for motor_RR
    detachInterrupt(digitalPinToInterrupt(motor_RL_read_speed_pin));  // Detach interrupt before calculating RPM
    motor_RL_rpm = motor_RL_numRevolutions * 60;                      // Calculate RPM
    motor_RL_rpm /= 12;
    Serial.print("motor_RL_rpm = ");
    Serial.println(motor_RL_rpm);                                                                           // Print RPM
    motor_RL_numRevolutions = 0;                                                                            // Reset the revolution count
    lastMillis = millis();                                                                                  // Reset the last counted time
    attachInterrupt(digitalPinToInterrupt(motor_RL_read_speed_pin), motor_RL_Revolutions_counter, RISING);  // Re-attach interrupt after calculating RPM

    // for motor_RR
    detachInterrupt(digitalPinToInterrupt(motor_RR_read_speed_pin));  // Detach interrupt before calculating RPM
    motor_RR_rpm = motor_RR_numRevolutions * 60;                      // Calculate RPM
    motor_RR_rpm /= 12;
    Serial.print("motor_RR_rpm = ");
    Serial.println(motor_RR_rpm);                                                                           // Print RPM
    motor_RR_numRevolutions = 0;                                                                            // Reset the revolution count
    lastMillis = millis();                                                                                  // Reset the last counted time
    attachInterrupt(digitalPinToInterrupt(motor_RR_read_speed_pin), motor_RR_Revolutions_counter, RISING);  // Re-attach interrupt after calculating RPM
  }
}


void motor_FL_Revolutions_counter() {
  motor_FL_numRevolutions++;  // This function is called by the interrupt and increments the motor_RR_numRevolutions counter
}
void motor_FR_Revolutions_counter() {
  motor_FR_numRevolutions++;  // This function is called by the interrupt and increments the motor_RR_numRevolutions counter
}
void motor_RL_Revolutions_counter() {
  motor_RL_numRevolutions++;  // This function is called by the interrupt and increments the motor_RR_numRevolutions counter
}
void motor_RR_Revolutions_counter() {
  motor_RR_numRevolutions++;  // This function is called by the interrupt and increments the motor_RR_numRevolutions counter
}
//----------------------------------------------------------------------------------------------------------------------------------//


// ██████  ███████ ███████ ██ ███    ██ ███████     ██████  ██ ██████ 
// ██   ██ ██      ██      ██ ████   ██ ██          ██   ██ ██ ██   ██ 
// ██   ██ █████   █████   ██ ██ ██  ██ █████       ██████  ██ ██   ██ 
// ██   ██ ██      ██      ██ ██  ██ ██ ██          ██      ██ ██   ██ 
// ██████  ███████ ██      ██ ██   ████ ███████     ██      ██ ██████  
//-------------------------------------------------Define PID Related Stuff-----------------------------------------------------------//
#include <PID_v1.h>
double FL_Setpoint, FL_PID_Input, FL_PID_Output;                               // Variables for front left motor PID control
double FR_Setpoint, FR_PID_Input, FR_PID_Output;                               // Variables for front right motor PID control
double RL_Setpoint, RL_PID_Input, RL_PID_Output;                               // Variables for back left motor PID control
double RR_Setpoint, RR_PID_Input, RR_PID_Output;                               // Variables for back right motor PID control
double Kp = 100, Ki = 0, Kd = 4;                                               // PID tuning parameters
PID FL_pid(&FL_PID_Input, &FL_PID_Output, &FL_Setpoint, Kp, Ki, Kd, DIRECT);   // PID controller for front left motor
PID FR_pid(&FR_PID_Input, &FR_PID_Output, &FR_Setpoint, Kp, Ki, Kd, REVERSE);  // PID controller for front right motor
PID RL_pid(&RL_PID_Input, &RL_PID_Output, &RL_Setpoint, Kp, Ki, Kd, DIRECT);   // PID controller for back left motor
PID RR_pid(&RR_PID_Input, &RR_PID_Output, &RR_Setpoint, Kp, Ki, Kd, REVERSE);  // PID controller for back right motor
//-----------------------------------------------------------------------------------------------------------------------------------//
// ██████  ██ ██████      ███████ ███████ ████████     ██    ██ ██████ 
// ██   ██ ██ ██   ██     ██      ██         ██        ██    ██ ██   ██ 
// ██████  ██ ██   ██     ███████ █████      ██        ██    ██ ██████  
// ██      ██ ██   ██          ██ ██         ██        ██    ██ ██     
// ██      ██ ██████      ███████ ███████    ██         ██████  ██ 
//-------------------------------------------------Set Up PIDs Mode and OutputLimits-----------------------------------------------------------//
void pid_setup() {
  int pid_limit = 1500;
  FL_pid.SetMode(AUTOMATIC);                      // Set PID to automatic mode for front left motor
  FL_pid.SetOutputLimits(-pid_limit, pid_limit);  // Set output limits for front left motor
  FR_pid.SetMode(AUTOMATIC);                      // Set PID to automatic mode for front right motor
  FR_pid.SetOutputLimits(-pid_limit, pid_limit);  // Set output limits for front right motor
  RL_pid.SetMode(AUTOMATIC);                      // Set PID to automatic mode for back left motor
  RL_pid.SetOutputLimits(-pid_limit, pid_limit);  // Set output limits for back left motor
  RR_pid.SetMode(AUTOMATIC);                      // Set PID to automatic mode for back right motor
  RR_pid.SetOutputLimits(-pid_limit, pid_limit);  // Set output limits for back right motor
}
//--------------------------------------------------------------------------------------------------------------------------------------------//
// ██████  ██ ██████           ██████  █████  ██       ██     ██ 
// ██   ██ ██ ██   ██         ██      ██   ██ ██      ██       ██ 
// ██████  ██ ██   ██         ██      ███████ ██      ██       ██ 
// ██      ██ ██   ██         ██      ██   ██ ██      ██       ██ 
// ██      ██ ██████  ███████  ██████ ██   ██ ███████  ██     ██  
//-------------------------------------------------Calculate the Needed Power a Specific Motor Need with PID Algorithum-----------------------------------------------------------//
double pid_cal(int motor_id, double par_Setpoint, double par_PID_INPUT) {
  if (motor_id == 1) {
    FL_Setpoint = par_Setpoint;
    FL_PID_Input = par_PID_INPUT;
    FL_pid.Compute();
    return FL_PID_Output;
  } else if (motor_id == 2) {
    FR_Setpoint = par_Setpoint;
    FR_PID_Input = par_PID_INPUT;
    FR_pid.Compute();
    return FR_PID_Output;
  } else if (motor_id == 3) {
    RL_Setpoint = par_Setpoint;
    RL_PID_Input = par_PID_INPUT;
    RL_pid.Compute();
    return RL_PID_Output;
  } else if (motor_id == 4) {
    RR_Setpoint = par_Setpoint;
    RR_PID_Input = par_PID_INPUT;
    RR_pid.Compute();
    return RR_PID_Output;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//











//                    █████                           ███ ███      ████                                ███ ███
//                   ░░███                           ███ ░░███    ░░███                               ███ ░░███
//   █████   ██████  ███████   █████ ████ ████████  ███   ░░███    ░███   ██████   ██████  ████████  ███   ░░███
//  ███░░   ███░░███░░░███░   ░░███ ░███ ░░███░░███░███    ░███    ░███  ███░░███ ███░░███░░███░░███░███    ░███
// ░░█████ ░███████   ░███     ░███ ░███  ░███ ░███░███    ░███    ░███ ░███ ░███░███ ░███ ░███ ░███░███    ░███
//  ░░░░███░███░░░    ░███ ███ ░███ ░███  ░███ ░███░░███   ███     ░███ ░███ ░███░███ ░███ ░███ ░███░░███   ███
//  ██████ ░░██████   ░░█████  ░░████████ ░███████  ░░███ ██░      █████░░██████ ░░██████  ░███████  ░░███ ██░
// ░░░░░░   ░░░░░░     ░░░░░    ░░░░░░░░  ░███░░░    ░░░ ░░░      ░░░░░  ░░░░░░   ░░░░░░   ░███░░░    ░░░ ░░░
//                                        ░███                                             ░███
//                                        █████                                            █████
//                                       ░░░░░                                            ░░░░░
//----------------------------------------------------------------------------------------------------------------------------//
void setup() {
  Serial.begin(57600);
  ps2_setup();         // Setup PS2 controller
  motor_pins_setup();  //Setup Motor pinMode
}

void loop() {
  ps2x.read_gamepad(false, 1);  //read controller
  drive_mode();
  RPM_read();
  delay(100);
}
//------------------------------------------------------------------------------- 
