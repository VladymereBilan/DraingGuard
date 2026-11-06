// DrainGuard firmware skeleton v0.2 (Wokwi simulation)
// New in v0.2: the rake. Motor direction pins, a simulated rake position, and
// jam detection from a current reading. A potentiometer stands in for the current sensor.
//   green LED  = motor forward (rake sweeps down the screen)
//   yellow LED = motor reverse (rake returns up)
//   red LED    = fault

// ---------- Pins (all safe ESP32 pins; 34 is an ADC1 pin, which keeps working with Wi-Fi) ----------
const int UP_TRIG     = 18;
const int UP_ECHO     = 19;
const int DOWN_TRIG   = 21;
const int DOWN_ECHO   = 22;
const int LED_PIN     = 23;   // fault light
const int MOTOR_IN1   = 25;   // HIGH = sweep down (motor driver IN1 later)
const int MOTOR_IN2   = 26;   // HIGH = return up  (motor driver IN2 later)
const int CURRENT_PIN = 34;   // current sensor later, potentiometer for now
const int HOME_SW     = 32;   // limit switches, used on the real rig
const int END_SW      = 33;

// ---------- Water settings ----------
const float SENSOR_HEIGHT_CM = 40.0;
const float MIN_VALID_CM     = 2.0;    // HC-SR04 limit. A JSN-SR04T cannot read closer than ~25
const float MAX_VALID_CM     = 400.0;
const float TRIGGER_CM       = 5.0;    // gap above normal that means "blocked"
const float CLEARED_CM       = 2.0;    // gap this close to normal means "cleared"
const unsigned long CONFIRM_MS = 5000; // demo value. Real rig: 30 s or more
const unsigned long SETTLE_MS  = 3000; // wait for flow to settle before checking
const int MAX_ATTEMPTS = 3;            // full clearing cycles before giving up

// ---------- Rake and jam settings ----------
const bool SIMULATE_RAKE = true;       // true: rake position is simulated. false: use the real limit switches
const unsigned long TRAVEL_MS = 2500;  // simulated time for the rake to cross the whole screen
const int JAM_LEVEL = 2800;            // current reading (0-4095) that counts as a jam
const unsigned long GRACE_MS       = 400;  // ignore current right after start (start-up surge)
const unsigned long JAM_CONFIRM_MS = 300;  // current must stay high this long to count as a jam
const unsigned long BACKOFF_MS     = 800;  // reverse this long after a jam
const unsigned long PAUSE_MS       = 500;  // stand still this long, then try again
const int MAX_JAM_RETRIES = 2;             // jams allowed in one cycle before giving up
const unsigned long SWEEP_TIMEOUT_MS = 6000;
const unsigned long HOME_TIMEOUT_MS  = 6000;

enum State { MONITORING, CONFIRMING, SWEEP_DOWN, SWEEP_UP, JAM_BACKOFF, SETTLING, FAULT };
State state = MONITORING;
unsigned long stateStart = 0;
int attempts = 0;

float baseline = 0;
float upLevel = 0, downLevel = 0;

float rakePos = 0;                 // 0 = home, 1 = end of travel (simulation only)
int motorDir = 0;                  // 1 = down, -1 = up, 0 = stopped
unsigned long lastRakeUpdate = 0;
unsigned long motorStartedAt = 0;
unsigned long jamSince = 0;
int jamRetries = 0;
int jamDir = 0;
unsigned long lastPrint = 0;

const char* stateName(State s) {
  switch (s) {
    case MONITORING:  return "MONITORING";
    case CONFIRMING:  return "CONFIRMING";
    case SWEEP_DOWN:  return "SWEEP_DOWN";
    case SWEEP_UP:    return "SWEEP_UP";
    case JAM_BACKOFF: return "JAM_BACKOFF";
    case SETTLING:    return "SETTLING";
    default:          return "FAULT";
  }
}

void setState(State s) {
  state = s;
  stateStart = millis();
  Serial.print(">> STATE: ");
  Serial.println(stateName(s));
}

// ================= Rake and motor =================

// Moves the simulated rake by however long the motor has been running
void updateRake() {
  unsigned long now = millis();
  float dt = (float)(now - lastRakeUpdate);
  lastRakeUpdate = now;
  if (motorDir != 0) {
    rakePos += motorDir * dt / TRAVEL_MS;
    if (rakePos < 0) rakePos = 0;
    if (rakePos > 1) rakePos = 1;
  }
}

bool rakeAtHome() { return SIMULATE_RAKE ? (rakePos <= 0.0) : (digitalRead(HOME_SW) == LOW); }
bool rakeAtEnd()  { return SIMULATE_RAKE ? (rakePos >= 1.0) : (digitalRead(END_SW) == LOW); }

// Always switch the "off" pin first so both motor pins are never HIGH together
void motorStop() {
  updateRake();
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  motorDir = 0;
}
void motorForward() {
  updateRake();
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN1, HIGH);
  motorDir = 1;
}
void motorReverse() {
  updateRake();
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  motorDir = -1;
}

bool homeRake() {
  unsigned long t0 = millis();
  while (!rakeAtHome()) {
    if (millis() - t0 > HOME_TIMEOUT_MS) { motorStop(); return false; }
    if (motorDir != -1) motorReverse();
    delay(10);
    updateRake();
  }
  motorStop();
  return true;
}

int readCurrent() {
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(CURRENT_PIN);
    delayMicroseconds(200);
  }
  return sum / 8;
}

void startSweepDown() {
  motorForward();
  motorStartedAt = millis();
  jamSince = 0;
  setState(SWEEP_DOWN);
}
void startSweepUp() {
  motorReverse();
  motorStartedAt = millis();
  jamSince = 0;
  setState(SWEEP_UP);
}
void beginCycle() {
  jamRetries = 0;
  startSweepDown();
}
void stopAndFault(const char* why) {
  motorStop();
  Serial.println(why);
  setState(FAULT);
}

void handleJam() {
  int dir = (state == SWEEP_DOWN) ? 1 : -1;
  motorStop();
  jamRetries++;
  Serial.print("JAM detected. Jam count this cycle: ");
  Serial.println(jamRetries);
  if (jamRetries > MAX_JAM_RETRIES) {
    stopAndFault("Repeated jams. Needs manual service.");
    return;
  }
  jamDir = dir;
  setState(JAM_BACKOFF);
}

// While the rake moves we stop reading the water. The water sensors take ~0.6 s per look,
// far too slow for stopping a motor. Here we only watch the current and the limit switches.
void motionStep() {
  updateRake();
  int amps = readCurrent();
  unsigned long now = millis();

  if (state == JAM_BACKOFF) {
    unsigned long t = now - stateStart;
    if (t < BACKOFF_MS) {
      if (jamDir == 1 && motorDir != -1) motorReverse();
      if (jamDir == -1 && motorDir != 1) motorForward();
    } else if (t < BACKOFF_MS + PAUSE_MS) {
      if (motorDir != 0) motorStop();
    } else {
      Serial.println("Retrying sweep.");
      if (jamDir == 1) startSweepDown(); else startSweepUp();
    }
  } else {
    if (now - motorStartedAt > GRACE_MS && amps > JAM_LEVEL) {
      if (jamSince == 0) jamSince = now;
      if (now - jamSince >= JAM_CONFIRM_MS) { handleJam(); return; }
    } else {
      jamSince = 0;
    }

    if (state == SWEEP_DOWN) {
      if (rakeAtEnd()) { motorStop(); startSweepUp(); }
      else if (now - stateStart > SWEEP_TIMEOUT_MS) stopAndFault("Rake never reached the end. Needs manual service.");
    } else {   // SWEEP_UP
      if (rakeAtHome()) { motorStop(); setState(SETTLING); }
      else if (now - stateStart > SWEEP_TIMEOUT_MS) stopAndFault("Rake never returned home. Needs manual service.");
    }
  }

  if (now - lastPrint >= 250) {
    lastPrint = now;
    Serial.printf("[%-11s] rake=%3.0f%%  current=%4d/4095\n", stateName(state), rakePos * 100, amps);
  }
  delay(10);
}

// ================= Water sensing =================

float readDistanceOnce(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(15);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 30000);
  if (us == 0) return -1;
  return us * 0.0343 / 2.0;
}

float readDistanceMedian(int trig, int echo) {
  float s[5];
  int n = 0;
  for (int i = 0; i < 5; i++) {
    float d = readDistanceOnce(trig, echo);
    if (d >= MIN_VALID_CM && d <= MAX_VALID_CM) s[n++] = d;
    delay(60);
  }
  if (n < 3) return -1;
  for (int i = 1; i < n; i++) {
    float k = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
    s[j + 1] = k;
  }
  return s[n / 2];
}

bool readGap(float &gap) {
  float upD   = readDistanceMedian(UP_TRIG, UP_ECHO);
  float downD = readDistanceMedian(DOWN_TRIG, DOWN_ECHO);
  if (upD < 0 || downD < 0) return false;
  upLevel   = SENSOR_HEIGHT_CM - upD;
  downLevel = SENSOR_HEIGHT_CM - downD;
  gap = upLevel - downLevel;
  return true;
}

void captureBaseline() {
  float sum = 0;
  int n = 0;
  for (int i = 0; i < 8; i++) {
    float g;
    if (readGap(g)) { sum += g; n++; }
  }
  baseline = n ? sum / n : 0;
}

void waterStep() {
  float gap;
  if (!readGap(gap)) {
    Serial.println("Sensor read failed, skipping this cycle");
    delay(200);
    return;
  }
  float excess = gap - baseline;

  switch (state) {
    case MONITORING:
      if (excess > TRIGGER_CM) setState(CONFIRMING);
      break;

    case CONFIRMING:
      if (excess <= TRIGGER_CM) {
        Serial.println("Gap dropped back. False alarm, ignoring.");
        setState(MONITORING);
      } else if (millis() - stateStart >= CONFIRM_MS) {
        attempts = 1;
        Serial.println("Blockage confirmed. Starting clearing cycle 1.");
        beginCycle();
      }
      break;

    case SETTLING:
      if (millis() - stateStart >= SETTLE_MS) {
        if (excess <= CLEARED_CM) {
          Serial.println("VERIFIED: flow restored.");
          attempts = 0;
          setState(MONITORING);
        } else if (attempts >= MAX_ATTEMPTS) {
          stopAndFault("Still blocked after max attempts. Needs manual service.");
        } else {
          attempts++;
          Serial.print("Still blocked. Starting clearing cycle ");
          Serial.println(attempts);
          beginCycle();
        }
      }
      break;

    case FAULT:
      digitalWrite(LED_PIN, (millis() / 150) % 2);
      if (excess <= CLEARED_CM) {
        Serial.println("Gap closed after manual service. Returning rake home.");
        if (homeRake()) {
          attempts = 0;
          digitalWrite(LED_PIN, LOW);
          setState(MONITORING);
        } else {
          Serial.println("Rake could not return home. Still needs service.");
        }
      }
      break;

    default:
      break;
  }

  Serial.printf("[%-11s] up=%5.1f cm  down=%5.1f cm  gap=%5.1f  above-normal=%5.1f\n",
                stateName(state), upLevel, downLevel, gap, excess);
}

// ================= Arduino entry points =================

void setup() {
  Serial.begin(115200);
  pinMode(UP_TRIG, OUTPUT);
  pinMode(UP_ECHO, INPUT);
  pinMode(DOWN_TRIG, OUTPUT);
  pinMode(DOWN_ECHO, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(HOME_SW, INPUT_PULLUP);
  pinMode(END_SW, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);
  motorStop();

  Serial.println("DrainGuard v0.2 starting. Keep both sliders equal and the current knob low.");
  if (!homeRake()) Serial.println("Warning: rake did not reach home at start-up.");
  delay(500);
  captureBaseline();
  Serial.print("Baseline captured. Normal gap (cm) x10 = ");
  Serial.println((int)(baseline * 10));
  setState(MONITORING);
}

void loop() {
  if (state == SWEEP_DOWN || state == SWEEP_UP || state == JAM_BACKOFF) motionStep();
  else waterStep();
}
