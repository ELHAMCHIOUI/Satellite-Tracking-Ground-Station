// G-5500 Antenna Rotator — Transistor Control (Arduino Mega)
//
// Transistor wiring (NPN e.g. BC547 / 2N2222):
//   Arduino pin HIGH → transistor ON → control pin shorted to G-5500 Pin8 (GND)
//   Arduino pin LOW  → transistor OFF → no connection
//
//   D4 HIGH → AZ left  (CCW) — G-5500 Pin4 to GND
//   D5 HIGH → AZ right (CW)  — G-5500 Pin2 to GND
//   D6 HIGH → EL down        — G-5500 Pin5 to GND
//   D7 HIGH → EL up          — G-5500 Pin3 to GND
//   A0      ← G-5500 Pin6 (AZ feedback)
//   A1      ← G-5500 Pin1 (EL feedback)  ⚠ use voltage divider — max 5.5V
//
// ── Calibration (your real measurements) ─────────────────────────────────────
//   AZ:  0°=0.16V  90°=1.14V  180°=2.20V  270°=3.35V  360°=4.40V
//   EL:  0°=0.00V  22.5°=0.75V  45°=1.50V  67°=2.20V  90°=2.90V
//        112°=3.70V  135°=4.40V  155.5°=5.20V  180°=5.50V
//
// ── Serial commands (9600 baud) ──────────────────────────────────────────────
//   L            → AZ left
//   R            → AZ right
//   U            → EL up
//   D            → EL down
//   S            → Stop all
//   POS          → Print position
//   SCAN         → Auto AZ sweep left↔right
//   STOPSCAN     → Stop scan
//   GOTO:180:45  → Go to AZ=180° EL=45°

// ── Pins ──────────────────────────────────────────────────────────────────────
const int PIN_LEFT  = 4;
const int PIN_RIGHT = 5;
const int PIN_DOWN  = 6;
const int PIN_UP    = 7;
const int PIN_AZ_FB = A0;
const int PIN_EL_FB = A1;
const int PIN_LED   = 13;

// ── Calibration tables ────────────────────────────────────────────────────────
const int   AZ_POINTS = 5;
const float AZ_VOLTS[] = { 0.16, 1.14, 2.20, 3.35, 4.40 };
const float AZ_DEGS[]  = {    0,   90,  180,  270,  360  };

// EL calibration — direct connection no voltage divider
// ADC clips at 5.0V = 1023 counts → max safe reading = 4.40V = 135°
// Points 155.5° (5.20V) and 180° (5.50V) exceed ADC range — excluded
// To unlock full 0-180°: add 10kΩ+8.2kΩ divider on A1 and restore full table
const int   EL_POINTS = 7;
const float EL_VOLTS[] = { 0.00, 0.75, 1.50, 2.20, 2.90, 3.70, 4.40 };
const float EL_DEGS[]  = {    0, 22.5,   45,   67,   90,  112,  135  };

// ── Scan timing ───────────────────────────────────────────────────────────────
const unsigned long SCAN_MOVE_MS  = 4000;
const unsigned long SCAN_PAUSE_MS = 2000;

// ── GOTO tolerance ────────────────────────────────────────────────────────────
const float AZ_TOL = 3.0;
const float EL_TOL = 2.0;

// ── State ─────────────────────────────────────────────────────────────────────
bool scanning = false;
bool gotoMode = false;
int  scanStep = 0;
unsigned long scanTimer  = 0;
unsigned long posTimer   = 0;
bool gotoValid = false;      // true only after a valid GOTO command received
float gotoAZ = -1, gotoEL = -1;   // -1 = not set, prevents accidental move to 0
String rxBuf = "";

// ── Motion control — HIGH = transistor ON = pin shorted to GND ───────────────
void allStop() {
  digitalWrite(PIN_LEFT,  LOW);
  digitalWrite(PIN_RIGHT, LOW);
  digitalWrite(PIN_DOWN,  LOW);
  digitalWrite(PIN_UP,    LOW);
  digitalWrite(PIN_LED,   LOW);
}

void moveLeft()  { allStop(); digitalWrite(PIN_LEFT,  HIGH); digitalWrite(PIN_LED, HIGH); }
void moveRight() { allStop(); digitalWrite(PIN_RIGHT, HIGH); digitalWrite(PIN_LED, HIGH); }
void moveUp()    { allStop(); digitalWrite(PIN_UP,    HIGH); digitalWrite(PIN_LED, HIGH); }
void moveDown()  { allStop(); digitalWrite(PIN_DOWN,  HIGH); digitalWrite(PIN_LED, HIGH); }

// ── Position feedback ─────────────────────────────────────────────────────────
float adcToVolts(int raw) {
  return raw * (5.0 / 1023.0);
}

float interpolate(float v, const float* volts, const float* degs, int n) {
  if (v <= volts[0])   return degs[0];
  if (v >= volts[n-1]) return degs[n-1];
  for (int i = 0; i < n - 1; i++) {
    if (v >= volts[i] && v <= volts[i+1]) {
      float t = (v - volts[i]) / (volts[i+1] - volts[i]);
      return degs[i] + t * (degs[i+1] - degs[i]);
    }
  }
  return degs[0];
}

float readAZ() {
  return interpolate(adcToVolts(analogRead(PIN_AZ_FB)), AZ_VOLTS, AZ_DEGS, AZ_POINTS);
}

float readEL() {
  int raw = analogRead(PIN_EL_FB);
  if (raw >= 1020) return 135.0;  // ADC clipping — above 135deg not readable without voltage divider
  return interpolate(adcToVolts(raw), EL_VOLTS, EL_DEGS, EL_POINTS);
}

void printPos() {
  Serial.print("POS  AZ: ");
  Serial.print(readAZ(), 1);
  Serial.print(" deg    EL: ");
  Serial.print(readEL(), 1);
  Serial.println(" deg");
}

// ── GOTO ──────────────────────────────────────────────────────────────────────
void runGoto() {
  if (!gotoValid) { allStop(); gotoMode = false; return; }  // safety guard
  float az    = readAZ();
  float el    = readEL();
  float azErr = gotoAZ - az;
  float elErr = gotoEL - el;

  // AZ
  if      (azErr >  AZ_TOL) { digitalWrite(PIN_RIGHT, HIGH); digitalWrite(PIN_LEFT, LOW); }
  else if (azErr < -AZ_TOL) { digitalWrite(PIN_LEFT,  HIGH); digitalWrite(PIN_RIGHT, LOW); }
  else                       { digitalWrite(PIN_LEFT,  LOW);  digitalWrite(PIN_RIGHT, LOW); }

  // EL
  if      (elErr >  EL_TOL) { digitalWrite(PIN_UP,   HIGH); digitalWrite(PIN_DOWN, LOW); }
  else if (elErr < -EL_TOL) { digitalWrite(PIN_DOWN, HIGH); digitalWrite(PIN_UP,   LOW); }
  else                       { digitalWrite(PIN_UP,   LOW);  digitalWrite(PIN_DOWN, LOW); }

  if (abs(azErr) <= AZ_TOL && abs(elErr) <= EL_TOL) {
    allStop();
    gotoMode = false;
    Serial.print("GOTO reached — AZ: ");
    Serial.print(az, 1);
    Serial.print("  EL: ");
    Serial.println(el, 1);
  }
}

// ── Scan ──────────────────────────────────────────────────────────────────────
void runScan() {
  unsigned long now = millis();
  switch (scanStep) {
    case 0:
      if (now - scanTimer >= SCAN_MOVE_MS)  { allStop();    scanTimer = now; scanStep = 1; Serial.println("SCAN: pause"); }
      break;
    case 1:
      if (now - scanTimer >= SCAN_PAUSE_MS) { moveLeft();   scanTimer = now; scanStep = 2; Serial.println("SCAN: left"); }
      break;
    case 2:
      if (now - scanTimer >= SCAN_MOVE_MS)  { allStop();    scanTimer = now; scanStep = 3; Serial.println("SCAN: pause"); }
      break;
    case 3:
      if (now - scanTimer >= SCAN_PAUSE_MS) { moveRight();  scanTimer = now; scanStep = 0; Serial.println("SCAN: right"); }
      break;
  }
}

// ── Command parser ────────────────────────────────────────────────────────────
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if      (cmd == "L")        { scanning = false; gotoMode = false; gotoValid = false; moveLeft();  Serial.println("CMD: left");  }
  else if (cmd == "R")        { scanning = false; gotoMode = false; gotoValid = false; moveRight(); Serial.println("CMD: right"); }
  else if (cmd == "U")        { scanning = false; gotoMode = false; gotoValid = false; moveUp();    Serial.println("CMD: up");    }
  else if (cmd == "D")        { scanning = false; gotoMode = false; gotoValid = false; moveDown();  Serial.println("CMD: down");  }
  else if (cmd == "S")        { scanning = false; gotoMode = false; gotoValid = false; allStop();   Serial.println("CMD: stop");  }
  else if (cmd == "POS")      { printPos(); }
  else if (cmd == "SCAN")     { gotoMode = false; scanning = true; scanStep = 0; scanTimer = millis(); moveRight(); Serial.println("CMD: scan started"); }
  else if (cmd == "STOPSCAN") { scanning = false; allStop(); Serial.println("CMD: scan stopped"); }

  else if (cmd.startsWith("GOTO:")) {
    int f = cmd.indexOf(':');
    int s = cmd.indexOf(':', f + 1);
    if (f != -1 && s != -1) {
      gotoAZ   = cmd.substring(f + 1, s).toFloat();
      gotoEL   = cmd.substring(s + 1).toFloat();
      if (gotoEL > 135.0) {
        gotoEL = 135.0;
        Serial.println("WARN: EL capped at 135deg — add voltage divider on A1 for full 180deg");
      }
      scanning  = false;
      gotoMode  = true;
      gotoValid = true;
      Serial.print("CMD: GOTO AZ="); Serial.print(gotoAZ, 1);
      Serial.print(" EL=");          Serial.println(gotoEL, 1);
    } else {
      Serial.println("ERR: use format GOTO:az:el   ex: GOTO:180:45");
    }

  } else if (cmd.length() > 0) {
    Serial.println("ERR: unknown — commands: L R U D S POS SCAN STOPSCAN GOTO:az:el");
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(PIN_LEFT,  OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);
  pinMode(PIN_DOWN,  OUTPUT);
  pinMode(PIN_UP,    OUTPUT);
  pinMode(PIN_LED,   OUTPUT);
  allStop();

  Serial.begin(9600);
  Serial.println("=====================================");
  Serial.println("  G-5500 — Transistor Control READY");
  Serial.println("  L / R / U / D   move");
  Serial.println("  S               stop");
  Serial.println("  POS             position");
  Serial.println("  SCAN/STOPSCAN   auto sweep");
  Serial.println("  GOTO:180:45     go to position");
  Serial.println("=====================================");
  posTimer = millis();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxBuf.length() > 0) { handleCommand(rxBuf); rxBuf = ""; }
    } else {
      rxBuf += c;
      if (rxBuf.length() > 30) rxBuf = "";
    }
  }

  if (gotoMode)       runGoto();
  else if (scanning)  runScan();

  if (millis() - posTimer >= 500) {
    posTimer = millis();
    printPos();
  }
}
