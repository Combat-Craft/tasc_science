#include <ESP32Servo.h>

// ===== SERVO (ELBOW) =====
Servo elbowServo;
const int ELBOW_SERVO_PIN = 13;        // Pin for servo signal
const int ELBOW_SERVO_MIN = 0;
const int ELBOW_SERVO_MAX = 180;
const int ELBOW_SERVO_CENTER = 90;
int elbow_servo_target = 90;
int elbow_servo_current = 90;

// ===== SHOULDER MOTOR (BTS7960) =====
const int SHOULDER_RPWM = 19;
const int SHOULDER_LPWM = 21;

// PWM settings
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;

// Ramp settings
const int PWM_START = 0;
const int PWM_TARGET = 255;
const int PWM_RAMP_STEP = 4;
const unsigned long PWM_RAMP_INTERVAL_MS = 20;

// ===== PROTOCOL - MATCHES ROS2 =====
const uint8_t PACKET_HEADER = 0xAA;

const uint8_t CMD_STOP = 0;
const uint8_t CMD_EXTEND = 1;
const uint8_t CMD_RETRACT = 2;

// Timeout safety
const unsigned long CMD_TIMEOUT_MS = 300;
unsigned long last_cmd_ms = 0;

// ===== STATE MACHINE =====
// Packet: [0xAA] [Shoulder_CMD] [Servo_Angle] [Checksum]
enum ParseState
{
  WAIT_HEADER,
  WAIT_SHOULDER,
  WAIT_ELBOW_SERVO,
  WAIT_CHECKSUM
};

ParseState parse_state = WAIT_HEADER;
uint8_t rx_shoulder = CMD_STOP;
uint8_t rx_elbow_servo = 90;

// Shoulder motor state
uint8_t shoulder_target_cmd = CMD_STOP;
uint8_t shoulder_active_cmd = CMD_STOP;
int shoulder_pwm = 0;
unsigned long last_shoulder_ramp_ms = 0;

// ===== MOTOR FUNCTIONS =====
void stopMotor(int rpwm_pin, int lpwm_pin)
{
  ledcWrite(rpwm_pin, 0);
  ledcWrite(lpwm_pin, 0);
}

void driveMotor(int cmd, int pwm_val, int rpwm_pin, int lpwm_pin)
{
  if (cmd == CMD_EXTEND)
  {
    ledcWrite(rpwm_pin, pwm_val);
    ledcWrite(lpwm_pin, 0);
  }
  else if (cmd == CMD_RETRACT)
  {
    ledcWrite(rpwm_pin, 0);
    ledcWrite(lpwm_pin, pwm_val);
  }
  else
  {
    stopMotor(rpwm_pin, lpwm_pin);
  }
}

void stopShoulderMotor()
{
  shoulder_target_cmd = CMD_STOP;
  shoulder_active_cmd = CMD_STOP;
  shoulder_pwm = 0;
  stopMotor(SHOULDER_RPWM, SHOULDER_LPWM);
}

void updateShoulderRamp()
{
  unsigned long now = millis();

  if (shoulder_target_cmd == CMD_STOP)
  {
    shoulder_active_cmd = CMD_STOP;
    if (shoulder_pwm > 0 && (now - last_shoulder_ramp_ms) >= PWM_RAMP_INTERVAL_MS)
    {
      shoulder_pwm -= PWM_RAMP_STEP;
      if (shoulder_pwm < 0) {
        shoulder_pwm = 0;
      }
      last_shoulder_ramp_ms = now;
    }

    if (shoulder_pwm == 0)
    {
      stopMotor(SHOULDER_RPWM, SHOULDER_LPWM);
    }
    else
    {
      driveMotor(shoulder_active_cmd, shoulder_pwm, SHOULDER_RPWM, SHOULDER_LPWM);
    }
    return;
  }

  // If direction changes, force a stop before reversing
  if (shoulder_active_cmd != CMD_STOP && shoulder_active_cmd != shoulder_target_cmd)
  {
    if ((now - last_shoulder_ramp_ms) >= PWM_RAMP_INTERVAL_MS)
    {
      shoulder_pwm -= PWM_RAMP_STEP;
      if (shoulder_pwm < 0) {
        shoulder_pwm = 0;
      }
      last_shoulder_ramp_ms = now;
    }

    if (shoulder_pwm == 0)
    {
      shoulder_active_cmd = CMD_STOP;
      stopMotor(SHOULDER_RPWM, SHOULDER_LPWM);
    }
    else
    {
      driveMotor(shoulder_active_cmd, shoulder_pwm, SHOULDER_RPWM, SHOULDER_LPWM);
    }
    return;
  }

  // Start motion
  if (shoulder_active_cmd == CMD_STOP)
  {
    shoulder_active_cmd = shoulder_target_cmd;
    if (shoulder_pwm < PWM_START)
    {
      shoulder_pwm = PWM_START;
    }
    driveMotor(shoulder_active_cmd, shoulder_pwm, SHOULDER_RPWM, SHOULDER_LPWM);
    last_shoulder_ramp_ms = now;
    return;
  }

  // Continue ramping toward target
  if ((now - last_shoulder_ramp_ms) >= PWM_RAMP_INTERVAL_MS)
  {
    shoulder_pwm += PWM_RAMP_STEP;
    if (shoulder_pwm > PWM_TARGET) {
      shoulder_pwm = PWM_TARGET;
    }
    last_shoulder_ramp_ms = now;
  }

  driveMotor(shoulder_active_cmd, shoulder_pwm, SHOULDER_RPWM, SHOULDER_LPWM);
}

// ===== SERVO FUNCTIONS =====
void updateElbowServo()
{
  // Immediate update (no smoothing needed since ROS2 handles smoothing)
  if (elbow_servo_current != elbow_servo_target)
  {
    elbow_servo_current = elbow_servo_target;
    elbowServo.write(elbow_servo_current);
  }
}

// ===== PARSING =====
void handleIncomingByte(uint8_t b)
{
  switch (parse_state)
  {
    case WAIT_HEADER:
      if (b == PACKET_HEADER)
      {
        parse_state = WAIT_SHOULDER;
      }
      break;

    case WAIT_SHOULDER:
      rx_shoulder = b;
      parse_state = WAIT_ELBOW_SERVO;
      break;

    case WAIT_ELBOW_SERVO:
      rx_elbow_servo = b;
      parse_state = WAIT_CHECKSUM;
      break;

    case WAIT_CHECKSUM:
    {
      // Checksum = shoulder_cmd ^ servo_angle (matches ROS2)
      uint8_t expected_checksum = rx_shoulder ^ rx_elbow_servo;

      if (b == expected_checksum &&
          rx_shoulder <= CMD_RETRACT &&
          rx_elbow_servo <= 180)  // Servo angle 0-180
      {
        // Update shoulder motor
        shoulder_target_cmd = rx_shoulder;
        
        // Update elbow servo
        elbow_servo_target = constrain(rx_elbow_servo, ELBOW_SERVO_MIN, ELBOW_SERVO_MAX);
        
        last_cmd_ms = millis();
      }

      parse_state = WAIT_HEADER;
      break;
    }

    default:
      parse_state = WAIT_HEADER;
      break;
  }
}

// ===== SETUP =====
void setup()
{
  Serial.begin(115200);

  // Initialize elbow servo
  elbowServo.setPeriodHertz(50);
  elbowServo.attach(ELBOW_SERVO_PIN, 500, 2400);
  elbowServo.write(ELBOW_SERVO_CENTER);
  elbow_servo_current = ELBOW_SERVO_CENTER;
  elbow_servo_target = ELBOW_SERVO_CENTER;

  // Initialize shoulder motor PWM
  if (!ledcAttach(SHOULDER_RPWM, PWM_FREQ, PWM_RESOLUTION)) {
    while (true) {}
  }
  if (!ledcAttach(SHOULDER_LPWM, PWM_FREQ, PWM_RESOLUTION)) {
    while (true) {}
  }

  stopShoulderMotor();
  last_cmd_ms = millis();
  
  Serial.println("READY");
}

// ===== LOOP =====
void loop()
{
  // Process incoming serial data
  while (Serial.available() > 0)
  {
    uint8_t b = static_cast<uint8_t>(Serial.read());
    handleIncomingByte(b);
  }

  // Timeout safety - stop shoulder motor
  if ((millis() - last_cmd_ms) > CMD_TIMEOUT_MS)
  {
    shoulder_target_cmd = CMD_STOP;
    // Don't reset servo on timeout - hold position
  }

  // Update shoulder motor
  updateShoulderRamp();

  // Update elbow servo
  updateElbowServo();

  delay(5);
}
