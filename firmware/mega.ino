/*
 * =============================================================================
 *  REHAB ROBOT — ARDUINO MEGA FIRMWARE
 *  firmware/mega.ino
 *
 *  WHAT THIS DOES:
 *    - Receives commands from Raspberry Pi over Serial (115200 baud)
 *    - Moves 4 stepper motors to target angles using step/dir pulses
 *    - Handles HALT (E-stop) and ENABLE (resume) commands
 *    - Sends sensor data + limit switch states back to Pi at ~100Hz
 *    - Sends handshake acknowledgement on startup
 *
 *  PACKET PROTOCOL (must match serial_comm.py exactly):
 *
 *    Pi → Arduino:
 *      "A:{ax1},{ax2},{ax3},{ax4}\n"   → move to angles in degrees
 *      "HALT\n"                         → emergency stop immediately
 *      "ENABLE\n"                       → resume after HALT
 *      "PING\n"                         → liveness check
 *      "RPI_READY\n"                    → handshake token from Pi
 *
 *    Arduino → Pi:
 *      "MEGA_READY\n"                   → handshake acknowledgement
 *      "S:{fsr},{emg},{ls0},...,{ls7}\n"→ sensor packet (100Hz)
 *      "ACK:MOVING\n"                   → angle command accepted
 *      "ACK:HALTED\n"                   → HALT acknowledged
 *      "ACK:ENABLED\n"                  → ENABLE acknowledged
 *      "PONG\n"                         → response to PING
 *      "ERR:{message}\n"               → error notification
 *
 *  MOTOR WIRING (confirmed from your working test):
 *    Motor 1 (Hip Ab/Ad)   : STEP=9,  DIR=8
 *    Motor 2 (Hip Flex/Ext): STEP=7,  DIR=6
 *    Motor 3 (Knee Flex)   : STEP=5,  DIR=4
 *    Motor 4 (Axis 4)      : STEP=3,  DIR=2
 *
 *  SENSOR WIRING:
 *    FSR  : A0
 *    sEMG : A1
 *    Limit switches: defined in LS_PINS array below
 *
 *  SAFETY NOTES:
 *    - HALT stops step pulses immediately. Motors hold position (safe).
 *    - Motors do NOT go limp on HALT — drivers stay enabled.
 *    - ENABLE resumes command acceptance. No hardware reset needed.
 *    - Limit switches use INPUT_PULLUP — NC wiring recommended.
 * =============================================================================
 */

// =============================================================================
//  PIN DEFINITIONS
// =============================================================================

// --- Stepper motor step/dir pins ---
const int STEP_PINS[4] = {9, 7, 5, 3};
const int DIR_PINS[4]  = {8, 6, 4, 2};

// --- Sensor analog pins ---
const int FSR_PIN = A0;
const int EMG_PIN = A1;

// --- Limit switch digital pins (INPUT_PULLUP) ---
// Order must match config.yaml limit_switches indices (0–7)
// Update these pin numbers once physically wired
const int LS_PINS[8] = {30, 31, 32, 33, 34, 35, 36, 37};

// --- Limit switch active state ---
// 0 = triggered when LOW  (NC wiring — recommended, safer)
// 1 = triggered when HIGH (NO wiring)
const int LS_ACTIVE_STATE = 0;

// =============================================================================
//  MOTION CONSTANTS
// =============================================================================

// Steps per degree: (200 steps × 8 microstepping × 26.67 gear ratio) / 360
const float STEPS_PER_DEGREE = 118.52;

// Step pulse delay in microseconds — controls motor speed
// Lower = faster. Do not go below 150 without testing for stall.
// 400 = safe starting speed. Reduce gradually during testing.
const int STEP_DELAY_US = 400;

// =============================================================================
//  SYSTEM STATE
// =============================================================================

// Current motor positions in steps (software encoder)
long currentSteps[4] = {0, 0, 0, 0};

// Target motor positions in steps
long targetSteps[4]  = {0, 0, 0, 0};

// Emergency stop flag
// true  = motors frozen, ignoring all angle commands
// false = normal operation
bool isHalted = false;

// =============================================================================
//  SENSOR STREAMING
// =============================================================================

// Timer for sensor packet transmission at ~100Hz
unsigned long lastSensorTime = 0;
const unsigned long SENSOR_INTERVAL_MS = 10;   // 10ms = 100Hz

// =============================================================================
//  SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);

  // Initialize motor pins
  for (int i = 0; i < 4; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i],  OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);
    digitalWrite(DIR_PINS[i],  LOW);
  }

  // Initialize limit switch pins with internal pullup
  for (int i = 0; i < 8; i++) {
    pinMode(LS_PINS[i], INPUT_PULLUP);
  }

  // Brief pause then send handshake — Pi waits for this
  delay(500);
  Serial.println("MEGA_READY");
}

// =============================================================================
//  MAIN LOOP
// =============================================================================

void loop() {
  // Always check serial first — HALT must be processed immediately
  handleSerial();

  // Move motors toward targets (non-blocking one-step-at-a-time)
  moveMotors();

  // Stream sensor data to Pi at 100Hz
  streamSensors();
}

// =============================================================================
//  SERIAL COMMAND HANDLER
// =============================================================================

void handleSerial() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  // ── HALT — emergency stop ────────────────────────────────────────────────
  if (input == "HALT") {
    isHalted = true;
    // Stop all motor targets at current position — freeze in place
    for (int i = 0; i < 4; i++) {
      targetSteps[i] = currentSteps[i];
    }
    Serial.println("ACK:HALTED");
    return;
  }

  // ── ENABLE — resume after HALT ───────────────────────────────────────────
  if (input == "ENABLE") {
    isHalted = false;
    Serial.println("ACK:ENABLED");
    return;
  }

  // ── PING — liveness check ────────────────────────────────────────────────
  if (input == "PING") {
    Serial.println("PONG");
    return;
  }

  // ── RPI_READY — handshake from Pi ───────────────────────────────────────
  // Pi sends this on startup to confirm connection.
  // We already sent MEGA_READY in setup() but send again in case Pi missed it.
  if (input == "RPI_READY") {
    Serial.println("MEGA_READY");
    return;
  }

  // ── A: — angle command ───────────────────────────────────────────────────
  // Format: A:{ax1},{ax2},{ax3},{ax4}
  // Example: A:45.00,0.00,90.00,0.00
  if (input.startsWith("A:")) {
    if (isHalted) {
      Serial.println("ERR:HALTED_IGNORING_ANGLE_CMD");
      return;
    }
    parseAngleCommand(input.substring(2));   // Strip "A:" prefix
    return;
  }

  // ── Unknown command ──────────────────────────────────────────────────────
  Serial.print("ERR:UNKNOWN_CMD:");
  Serial.println(input);
}

// =============================================================================
//  ANGLE COMMAND PARSER
// =============================================================================

void parseAngleCommand(String payload) {
  // payload = "45.00,0.00,90.00,0.00"
  // Split by comma and convert each to steps

  float angles[4] = {0.0, 0.0, 0.0, 0.0};
  int   axisIdx   = 0;

  while (axisIdx < 4 && payload.length() > 0) {
    int commaIdx = payload.indexOf(',');
    String token;

    if (commaIdx != -1) {
      token   = payload.substring(0, commaIdx);
      payload = payload.substring(commaIdx + 1);
    } else {
      token   = payload;
      payload = "";
    }

    token.trim();
    angles[axisIdx] = token.toFloat();
    axisIdx++;
  }

  if (axisIdx < 4) {
    Serial.println("ERR:INCOMPLETE_ANGLE_PACKET");
    return;
  }

  // Convert degrees to steps and set as new targets
  for (int i = 0; i < 4; i++) {
    targetSteps[i] = (long)(angles[i] * STEPS_PER_DEGREE);
  }

  Serial.println("ACK:MOVING");
}

// =============================================================================
//  MOTOR MOVEMENT — one step per loop iteration per axis
//
//  Non-blocking design: each call to moveMotors() advances each motor
//  by exactly ONE step if it hasn't reached its target yet.
//  This keeps the loop fast so serial commands are never delayed.
//
//  All 4 motors step simultaneously when all need to move,
//  giving coordinated multi-axis motion.
// =============================================================================

void moveMotors() {
  if (isHalted) return;

  bool anyNeedsStep = false;

  // Check which axes need to move and set directions
  for (int i = 0; i < 4; i++) {
    if (currentSteps[i] != targetSteps[i]) {
      anyNeedsStep = true;
      bool goForward = (targetSteps[i] > currentSteps[i]);
      digitalWrite(DIR_PINS[i], goForward ? HIGH : LOW);
    }
  }

  if (!anyNeedsStep) return;

  // Pulse HIGH on all axes that need to move
  for (int i = 0; i < 4; i++) {
    if (currentSteps[i] != targetSteps[i]) {
      digitalWrite(STEP_PINS[i], HIGH);
    }
  }

  delayMicroseconds(STEP_DELAY_US);

  // Pulse LOW and update counters
  for (int i = 0; i < 4; i++) {
    if (currentSteps[i] != targetSteps[i]) {
      digitalWrite(STEP_PINS[i], LOW);

      // Advance position counter by 1 step in correct direction
      if (currentSteps[i] < targetSteps[i]) currentSteps[i]++;
      else                                   currentSteps[i]--;
    }
  }

  delayMicroseconds(STEP_DELAY_US);
}

// =============================================================================
//  SENSOR STREAMING — sends packet to Pi at ~100Hz
//
//  Packet format (must match serial_comm.py _handle_sensor_packet):
//    S:{fsr},{emg},{ls0},{ls1},{ls2},{ls3},{ls4},{ls5},{ls6},{ls7}
//
//  FSR and EMG are raw 10-bit ADC values (0–1023).
//  Limit switch values are 0 (open) or 1 (triggered).
// =============================================================================

void streamSensors() {
  unsigned long now = millis();
  if (now - lastSensorTime < SENSOR_INTERVAL_MS) return;
  lastSensorTime = now;

  // Read sensors
  int fsrVal = analogRead(FSR_PIN);
  int emgVal = analogRead(EMG_PIN);

  // Read all limit switches
  int lsStates[8];
  for (int i = 0; i < 8; i++) {
    int pinState = digitalRead(LS_PINS[i]);
    // For INPUT_PULLUP + NC wiring: pin reads LOW when triggered
    lsStates[i] = (pinState == LS_ACTIVE_STATE) ? 1 : 0;
  }

  // Build and send packet
  Serial.print("S:");
  Serial.print(fsrVal);
  Serial.print(",");
  Serial.print(emgVal);
  for (int i = 0; i < 8; i++) {
    Serial.print(",");
    Serial.print(lsStates[i]);
  }
  Serial.println();
}
