#include <ESP32Servo.h>

// =============================================================================
// SCIENCE ESP32 FIRMWARE
//
// Incoming binary packet:
//   [0xAA] [lift command] [tray angle] [checksum]
//
// Lift commands:
//   0 = stop
//   1 = extend
//   2 = retract
//
// Tray angle:
//   0 to 180 degrees
//
// Checksum:
//   lift command XOR tray angle
// =============================================================================

// =============================================================================
// BEAKER SERVO
// =============================================================================

Servo beakerServo;

const int BEAKER_SERVO_PIN = 22;
const int BEAKER_SERVO_MIN = 0;
const int BEAKER_SERVO_MAX = 180;
const int BEAKER_SERVO_START = 0;

// Same pulse range as the standalone test that worked.
const int BEAKER_SERVO_MIN_PULSE_US = 500;
const int BEAKER_SERVO_MAX_PULSE_US = 2400;

// =============================================================================
// LIFT MOTOR, BTS7960
// =============================================================================

const int LIFT_RPWM_PIN = 18;
const int LIFT_LPWM_PIN = 19;

const int LIFT_RPWM_CHANNEL = 6;
const int LIFT_LPWM_CHANNEL = 7;

const int PWM_FREQUENCY = 1000;
const int PWM_RESOLUTION = 8;

const int PWM_TARGET = 255;
const int PWM_RAMP_STEP = 4;
const unsigned long PWM_RAMP_INTERVAL_MS = 20;

// =============================================================================
// SERIAL PROTOCOL
// =============================================================================

const uint8_t PACKET_HEADER = 0xAA;

const uint8_t CMD_STOP = 0;
const uint8_t CMD_EXTEND = 1;
const uint8_t CMD_RETRACT = 2;

const unsigned long COMMAND_TIMEOUT_MS = 300;
unsigned long last_valid_command_ms = 0;

enum ParseState
{
  WAIT_HEADER,
  WAIT_LIFT,
  WAIT_TRAY,
  WAIT_CHECKSUM
};

ParseState parse_state = WAIT_HEADER;

uint8_t received_lift_command = CMD_STOP;
uint8_t received_tray_angle = BEAKER_SERVO_START;

// =============================================================================
// LIFT STATE
// =============================================================================

uint8_t lift_target_command = CMD_STOP;
uint8_t lift_active_command = CMD_STOP;

int lift_pwm = 0;
unsigned long last_lift_ramp_ms = 0;

// =============================================================================
// LIFT MOTOR FUNCTIONS
// =============================================================================

void writeLiftPwm(int rpwm, int lpwm)
{
  rpwm = constrain(rpwm, 0, PWM_TARGET);
  lpwm = constrain(lpwm, 0, PWM_TARGET);

  ledcWriteChannel(LIFT_RPWM_CHANNEL, rpwm);
  ledcWriteChannel(LIFT_LPWM_CHANNEL, lpwm);
}

void stopLiftOutput()
{
  writeLiftPwm(0, 0);
}

void driveLift(uint8_t command, int pwm_value)
{
  if (command == CMD_EXTEND)
  {
    writeLiftPwm(pwm_value, 0);
  }
  else if (command == CMD_RETRACT)
  {
    writeLiftPwm(0, pwm_value);
  }
  else
  {
    stopLiftOutput();
  }
}

void stopLiftImmediately()
{
  lift_target_command = CMD_STOP;
  lift_active_command = CMD_STOP;
  lift_pwm = 0;

  stopLiftOutput();
}

void updateLiftRamp()
{
  const unsigned long now = millis();

  // Stop request: ramp the active direction down to zero.
  if (lift_target_command == CMD_STOP)
  {
    if (
      lift_pwm > 0 &&
      now - last_lift_ramp_ms >= PWM_RAMP_INTERVAL_MS)
    {
      lift_pwm -= PWM_RAMP_STEP;

      if (lift_pwm < 0)
      {
        lift_pwm = 0;
      }

      last_lift_ramp_ms = now;
    }

    if (lift_pwm == 0)
    {
      lift_active_command = CMD_STOP;
      stopLiftOutput();
    }
    else
    {
      driveLift(lift_active_command, lift_pwm);
    }

    return;
  }

  // Direction change: ramp down before reversing.
  if (
    lift_active_command != CMD_STOP &&
    lift_active_command != lift_target_command)
  {
    if (now - last_lift_ramp_ms >= PWM_RAMP_INTERVAL_MS)
    {
      lift_pwm -= PWM_RAMP_STEP;

      if (lift_pwm < 0)
      {
        lift_pwm = 0;
      }

      last_lift_ramp_ms = now;
    }

    if (lift_pwm == 0)
    {
      lift_active_command = CMD_STOP;
      stopLiftOutput();
    }
    else
    {
      driveLift(lift_active_command, lift_pwm);
    }

    return;
  }

  // Start moving in the requested direction.
  if (lift_active_command == CMD_STOP)
  {
    lift_active_command = lift_target_command;
    lift_pwm = 0;
    last_lift_ramp_ms = now;

    driveLift(lift_active_command, lift_pwm);
    return;
  }

  // Ramp toward full PWM.
  if (now - last_lift_ramp_ms >= PWM_RAMP_INTERVAL_MS)
  {
    lift_pwm += PWM_RAMP_STEP;

    if (lift_pwm > PWM_TARGET)
    {
      lift_pwm = PWM_TARGET;
    }

    last_lift_ramp_ms = now;
  }

  driveLift(lift_active_command, lift_pwm);
}

// =============================================================================
// VALID PACKET HANDLING
// =============================================================================

void applyValidPacket(
  uint8_t lift_command,
  uint8_t tray_angle)
{
  lift_target_command = lift_command;

  const int safe_angle = constrain(
    static_cast<int>(tray_angle),
    BEAKER_SERVO_MIN,
    BEAKER_SERVO_MAX);

  // This intentionally matches the working standalone servo test:
  // receive an angle, constrain it, and immediately call Servo.write().
  beakerServo.write(safe_angle);

  last_valid_command_ms = millis();
}

// =============================================================================
// BINARY PACKET PARSER
// =============================================================================

void handleIncomingByte(uint8_t incoming_byte)
{
  switch (parse_state)
  {
    case WAIT_HEADER:
      if (incoming_byte == PACKET_HEADER)
      {
        parse_state = WAIT_LIFT;
      }
      break;

    case WAIT_LIFT:
      received_lift_command = incoming_byte;
      parse_state = WAIT_TRAY;
      break;

    case WAIT_TRAY:
      received_tray_angle = incoming_byte;
      parse_state = WAIT_CHECKSUM;
      break;

    case WAIT_CHECKSUM:
    {
      const uint8_t expected_checksum =
        received_lift_command ^ received_tray_angle;

      const bool checksum_is_valid =
        incoming_byte == expected_checksum;

      const bool lift_is_valid =
        received_lift_command <= CMD_RETRACT;

      const bool tray_is_valid =
        received_tray_angle <= BEAKER_SERVO_MAX;

      if (
        checksum_is_valid &&
        lift_is_valid &&
        tray_is_valid)
      {
        applyValidPacket(
          received_lift_command,
          received_tray_angle);
      }

      parse_state = WAIT_HEADER;
      break;
    }

    default:
      parse_state = WAIT_HEADER;
      break;
  }
}

// =============================================================================
// SETUP
// =============================================================================

void setup()
{
  Serial.begin(115200);

  // Assign the BTS7960 outputs to explicit LEDC channels.
  // These are separate from the channel selected by ESP32Servo.
  const bool rpwm_attached = ledcAttachChannel(
    LIFT_RPWM_PIN,
    PWM_FREQUENCY,
    PWM_RESOLUTION,
    LIFT_RPWM_CHANNEL);

  const bool lpwm_attached = ledcAttachChannel(
    LIFT_LPWM_PIN,
    PWM_FREQUENCY,
    PWM_RESOLUTION,
    LIFT_LPWM_CHANNEL);

  if (!rpwm_attached || !lpwm_attached)
  {
    while (true)
    {
      delay(1000);
    }
  }

  stopLiftImmediately();

  // Match the standalone working servo setup exactly.
  beakerServo.setPeriodHertz(50);
  beakerServo.attach(
    BEAKER_SERVO_PIN,
    BEAKER_SERVO_MIN_PULSE_US,
    BEAKER_SERVO_MAX_PULSE_US);

  beakerServo.write(BEAKER_SERVO_START);

  last_valid_command_ms = millis();

  Serial.println("READY");
}

// =============================================================================
// LOOP
// =============================================================================

void loop()
{
  while (Serial.available() > 0)
  {
    const int serial_value = Serial.read();

    if (serial_value >= 0)
    {
      handleIncomingByte(
        static_cast<uint8_t>(serial_value));
    }
  }

  // Stop only the lift when ROS 2 communication is lost.
  // The servo holds its last commanded angle.
  if (
    millis() - last_valid_command_ms >
    COMMAND_TIMEOUT_MS)
  {
    lift_target_command = CMD_STOP;
  }

  updateLiftRamp();

  delay(5);
}
