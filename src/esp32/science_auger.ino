#include <ESP32Servo.h>

// ===== SERVO (BEAKER) =====
Servo beakerServo;
const int BEAKER_SERVO_PIN = 23;        // Pin for servo signal
const int BEAKER_SERVO_MIN = 0;
const int BEAKER_SERVO_MAX = 180;
const int BEAKER_SERVO_START = 0;
int beaker_servo_target = 90;
int beaker_servo_current = 90;

// ===== LIFT MOTOR (BTS7960) =====
const int LIFT_RPWM = 18;
const int LIFT_LPWM = 19;

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
// Packet: [0xAA] [Lift_CMD] [Servo_Angle] [Checksum]
enum ParseState
{
  WAIT_HEADER,
  WAIT_LIFT,
  WAIT_BEAKER_SERVO,
  WAIT_CHECKSUM
};

ParseState parse_state = WAIT_HEADER;
uint8_t rx_lift = CMD_STOP;
uint8_t rx_beaker_servo = 0;

// Lift motor state
uint8_t lift_target_cmd = CMD_STOP;
uint8_t lift_active_cmd = CMD_STOP;
int lift_pwm = 0;
unsigned long last_lift_ramp_ms = 0;

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

void stopLiftMotor()
{
  lift_target_cmd = CMD_STOP;
  lift_active_cmd = CMD_STOP;
  lift_pwm = 0;
  stopMotor(LIFT_RPWM, LIFT_LPWM);
}

void updateLiftRamp()
{
  unsigned long now = millis();

  if (lift_target_cmd == CMD_STOP)
  {
    lift_active_cmd = CMD_STOP;
    if (lift_pwm > 0 && (now - last_lift_ramp_ms) >= PWM_RAMP_INTERVAL_MS)
    {
      lift_pwm -= PWM_RAMP_STEP;
      if (lift_pwm < 0) {
        lift_pwm = 0;
      }
      last_lift_ramp_ms = now;
    }

    if (lift_pwm == 0)
    {
      stopMotor(LIFT_RPWM, LIFT_LPWM);
    }
    else
    {
      driveMotor(lift_active_cmd, lift_pwm, LIFT_RPWM, LIFT_LPWM);
    }
    return;
  }

  // If direction changes, force a stop before reversing
  if (lift_active_cmd != CMD_STOP && lift_active_cmd != lift_target_cmd)
  {
    if ((now - last_lift_ramp_ms) >= PWM_RAMP_INTERVAL_MS)
    {
      lift_pwm -= PWM_RAMP_STEP;
      if (lift_pwm < 0) {
        lift_pwm = 0;
      }
      last_lift_ramp_ms = now;
    }

    if (lift_pwm == 0)
    {
      lift_active_cmd = CMD_STOP;
      stopMotor(LIFT_RPWM, LIFT_LPWM);
    }
    else
    {
      driveMotor(lift_active_cmd, lift_pwm, LIFT_RPWM, LIFT_LPWM);
    }
    return;
  }

  // Start motion
  if (lift_active_cmd == CMD_STOP)
  {
    lift_active_cmd = lift_target_cmd;
    if (lift_pwm < PWM_START)
    {
      lift_pwm = PWM_START;
    }
    driveMotor(lift_active_cmd, lift_pwm, LIFT_RPWM, LIFT_LPWM);
    last_lift_ramp_ms = now;
    return;
  }

  // Continue ramping toward target
  if ((now - last_lift_ramp_ms) >= PWM_RAMP_INTERVAL_MS)
  {
    lift_pwm += PWM_RAMP_STEP;
    if (lift_pwm > PWM_TARGET) {
      lift_pwm = PWM_TARGET;
    }
    last_lift_ramp_ms = now;
  }

  driveMotor(lift_active_cmd, lift_pwm, LIFT_RPWM, LIFT_LPWM);
}

// ===== SERVO FUNCTIONS =====
void updateBeakerServo()
{
  /// Move immediately to the discrete position commanded by ROS 2.
  if (beaker_servo_current != beaker_servo_target)
  {
    beaker_servo_current = beaker_servo_target;
    beakerServo.write(beaker_servo_current);
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
        parse_state = WAIT_LIFT;
      }
      break;

    case WAIT_LIFT:
      rx_lift = b;
      parse_state = WAIT_BEAKER_SERVO;
      break;

    case WAIT_BEAKER_SERVO:
      rx_beaker_servo = b;
      parse_state = WAIT_CHECKSUM;
      break;

    case WAIT_CHECKSUM:
    {
      // Checksum = lift_cmd ^ servo_angle (matches ROS2)
      uint8_t expected_checksum = rx_lift ^ rx_beaker_servo;

      if (b == expected_checksum &&
          rx_lift <= CMD_RETRACT &&
          rx_beaker_servo <= 180)  // Servo angle 0-180
      {
        // Update lift motor
        lift_target_cmd = rx_lift;
        
        // Update beaker servo
        beaker_servo_target = constrain(rx_beaker_servo, BEAKER_SERVO_MIN, BEAKER_SERVO_MAX);
        
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

  // Initialize beaker servo
  beakerServo.setPeriodHertz(50);
  beakerServo.attach(BEAKER_SERVO_PIN, 500, 2400);
  beakerServo.write(BEAKER_SERVO_START);
  beaker_servo_current = BEAKER_SERVO_START;
  beaker_servo_target = BEAKER_SERVO_START;

  // Initialize lift motor PWM
  if (!ledcAttach(LIFT_RPWM, PWM_FREQ, PWM_RESOLUTION)) {
    while (true) {}
  }
  if (!ledcAttach(LIFT_LPWM, PWM_FREQ, PWM_RESOLUTION)) {
    while (true) {}
  }

  stopLiftMotor();
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

  // Timeout safety - stop lift motor
  if ((millis() - last_cmd_ms) > CMD_TIMEOUT_MS)
  {
    lift_target_cmd = CMD_STOP;
    // Don't reset servo on timeout - hold position
  }

  // Update lift motor
  updateLiftRamp();

  // Update beaker servo
  updateBeakerServo();

  delay(5);
}
