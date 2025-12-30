/*
 * ROS 2 Serial Bridge Firmware (Dumb Driver - Motor Only)
 * Capabilities: 
 * - Differential Drive Control (PID + Encoders)
 * - Protocol: CSV over Serial @ 115200 baud
 * * OUT (To Pi): "L_TICKS,R_TICKS"
 * * IN (From Pi): "linear_x,angular_z" (e.g., "0.5,-0.1")
 */

// No external libraries needed for basics!

/* ================= CONFIGURATION ================= */

#define CONTROL_RATE_MS  50    // 20Hz Update Rate
#define PWM_MAX          255
#define WHEEL_BASE       0.245 
#define TICKS_PER_METER  981.5

// *** MOTOR DIRECTION CONFIG ***
// Adjust these to make the robot drive straight and forward
#define INVERT_L false
#define INVERT_R false

// Pins
#define ENC_L_A 3
#define ENC_L_B 12
#define ENC_R_A 2
#define ENC_R_B 11
#define ENA 9
#define IN1 5
#define IN2 6
#define ENB 10
#define IN3 7
#define IN4 8

/* ================= PID & STATE ================= */

struct PID {
  float kp; float ki; float kd;
  float integral; float prev_err; float max_integral;
};

// Tunings
PID pidL = {100.0, 0.1, 0.0, 0.0, 0.0, 50.0}; 
PID pidR = {100.0, 0.1, 0.0, 0.0, 0.0, 50.0};

volatile int32_t enc_ticks_L = 0;
volatile int32_t enc_ticks_R = 0;
float target_vL = 0, target_vR = 0;

/* ================= INTERRUPTS ================= */

void isrLeft() { 
  int val = digitalRead(ENC_L_B);
  if (INVERT_L) enc_ticks_L += val ? 1 : -1;
  else          enc_ticks_L += val ? -1 : 1;
}

void isrRight() { 
  int val = digitalRead(ENC_R_B);
  if (INVERT_R) enc_ticks_R += val ? -1 : 1;
  else          enc_ticks_R += val ? 1 : -1;
}

/* ================= PID LOGIC ================= */

int pidUpdate(PID &pid, float target, float measured, float dt) {
  float err = target - measured;
  pid.integral += err * dt;
  pid.integral = constrain(pid.integral, -pid.max_integral, pid.max_integral);
  float deriv = (err - pid.prev_err) / dt;
  pid.prev_err = err;
  return constrain((int)(pid.kp * err + pid.ki * pid.integral + pid.kd * deriv), -PWM_MAX, PWM_MAX);
}

void setMotor(int pwm, int en, int inA, int inB) {
  if (pwm > 0) {
    digitalWrite(inA, HIGH); digitalWrite(inB, LOW); analogWrite(en, pwm);
  } else if (pwm < 0) {
    digitalWrite(inA, LOW); digitalWrite(inB, HIGH); analogWrite(en, -pwm);
  } else {
    digitalWrite(inA, LOW); digitalWrite(inB, LOW); analogWrite(en, 0);
  }
}

/* ================= SERIAL PARSER (INPUT) ================= */

void checkSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int commaIndex = input.indexOf(',');
    
    if (commaIndex != -1) {
      String v_str = input.substring(0, commaIndex);
      String w_str = input.substring(commaIndex + 1);
      
      float v = v_str.toFloat();
      float w = w_str.toFloat();
      
      // Calculate individual wheel targets
      target_vL = v - (w * WHEEL_BASE / 2.0);
      target_vR = v + (w * WHEEL_BASE / 2.0);
    }
  }
}

/* ================= SETUP ================= */

void setup() {
  // 1. Init Serial (Fast baud rate for data stream)
  Serial.begin(115200);

  // 2. Init Pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, RISING);
}

/* ================= MAIN LOOP ================= */

void loop() {
  static unsigned long lastTime = 0;
  unsigned long now = millis();

  // 1. Always check for new commands
  checkSerial();

  // 2. Run Control Loop at 20Hz (50ms)
  if (now - lastTime >= CONTROL_RATE_MS) {
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // --- ENCODER READ ---
    long currTicksL, currTicksR;
    noInterrupts();
    currTicksL = enc_ticks_L;
    currTicksR = enc_ticks_R;
    interrupts();

    // --- SEND DATA TO PI (CSV) ---
    // Protocol: L_TICKS, R_TICKS
    // Force 16-bit wrap-around so Python handles it correctly
    Serial.print((int16_t)currTicksL); Serial.print(",");
    Serial.println((int16_t)currTicksR);

    // --- PID MOTOR CONTROL ---
    
    // Calculate Velocity
    static long prevTicksL = 0, prevTicksR = 0;
    float velL = (currTicksL - prevTicksL) / TICKS_PER_METER / dt;
    float velR = (currTicksR - prevTicksR) / TICKS_PER_METER / dt;
    prevTicksL = currTicksL; prevTicksR = currTicksR;

    int pwmL = 0, pwmR = 0;

    // Silence Motors at Zero Target
    if (abs(target_vL) < 0.001 && abs(target_vR) < 0.001) {
      pidL.integral = 0; pidR.integral = 0;
      pidL.prev_err = 0; pidR.prev_err = 0;
    } else {
      pwmL = pidUpdate(pidL, target_vL, velL, dt);
      pwmR = pidUpdate(pidR, target_vR, velR, dt);
      
      // Motor Inversion Logic
      if (INVERT_L) pwmL = -pwmL;
      if (INVERT_R) pwmR = -pwmR;
    }

    setMotor(pwmL, ENA, IN1, IN2);
    setMotor(pwmR, ENB, IN3, IN4);
  }
}