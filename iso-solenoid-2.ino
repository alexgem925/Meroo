// This has the manual control for the solenoid valve, where desired flow and pressure are set by user commands over Serial Monitor,
// and the solenoid valve current is adjusted accordingly. This is similar to connected-control/solenoidTest.ino but adapted for isolated control.
// everything is hardcoded for all values. 
// NOT CONNECTED TO PRESSURE OR FLOW SENSORS.
// TO TEST

//###########################################################################################################
//######################## Begin of Code: Serial Communication with Evaluation Board #########################
//########################                    DO NOT CHANGE BELOW                    #########################
//###########################################################################################################
const uint8_t VALUE_COUNT = 13;
float currentValues[VALUE_COUNT];
const uint16_t LINE_BUF_SIZE = 128;
char lineBuffer[LINE_BUF_SIZE];
uint16_t linePos = 0;
float tempValues[VALUE_COUNT];
void parseLine(char *line);
void serial1Update() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '\n') {
      if (linePos < LINE_BUF_SIZE) lineBuffer[linePos] = '\0';
      else lineBuffer[LINE_BUF_SIZE - 1] = '\0';
      parseLine(lineBuffer);
      linePos = 0;
    }
    else if (c == '\r') {
      // ignore CR
    }
    else {
      if (linePos < LINE_BUF_SIZE - 1) lineBuffer[linePos++] = c;
      else linePos = 0; // overflow -> reset
    }
  }
}
void parseLine(char *line) {
  uint8_t idx = 0;
  char *token = strtok(line, "\t");
  while (token != NULL && idx < VALUE_COUNT) {
    tempValues[idx] = atof(token);
    idx++;
    token = strtok(NULL, "\t");
  }
  if (idx == VALUE_COUNT) {
    noInterrupts();
    for (uint8_t i = 0; i < VALUE_COUNT; i++) currentValues[i] = tempValues[i];
    interrupts();
  }
}
float getValue(uint8_t index) {
  if (index >= VALUE_COUNT) return NAN;
  noInterrupts();
  float val = currentValues[index];
  interrupts();
  return val;
}
//###########################################################################################################
//######################### End of Code: Serial Communication with Evaluation Board ##########################
//###########################################################################################################

#include <stdarg.h>
#include <math.h>

// ============================ CONFIG YOU CARE ABOUT ============================
// Which valve output are you using? 0/1/2/3
static const int VALVE_CH = 0;

// IMPORTANT: set this to your safe max for the valve/driver test
static const float MAX_CURRENT_mA = 200.0f;   // adjust if needed

// How often to prompt for "actual flow"
static const uint32_t PROMPT_PERIOD_MS = 1000;

// How often to print telemetry regardless
static const uint32_t TELEMETRY_PERIOD_MS = 3000;

// Control tuning (simple beginner-friendly step control)
// - If error is big, change current more.
// - If error is small, change current slightly.
static const float SMALL_ERR_FLOW = 0.5f;     // sL/min (deadband)
static const float BIG_ERR_FLOW   = 5.0f;     // sL/min
static const float STEP_SMALL_mA  = 1.0f;     // mA per update (small tweak)
static const float STEP_MED_mA    = 5.0f;     // mA per update
static const float STEP_BIG_mA    = 15.0f;    // mA per update (big jump)
// ============================================================================

int dutyIndex(int ch)    { return ch * 3 + 0; }
int currentIndex(int ch) { return ch * 3 + 1; }
int sensorIndex(int ch)  { return ch * 3 + 2; }

// Helper to send formatted commands to Serial1
void sendCmd(const char* fmt, ...) {
  char buf[64];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial1.print(buf);  // fmt should include \n if needed
}

// ---------------------------- Valve control API ----------------------------
float g_cmdCurrent_mA = 0.0f;

void setValveCurrent_mA(float mA, bool printChange = true) {
  if (mA < 0) mA = 0;
  if (mA > MAX_CURRENT_mA) mA = MAX_CURRENT_mA;

  // Only send if it actually changed (avoid spamming Serial1)
  if (fabsf(mA - g_cmdCurrent_mA) < 0.01f) return;

  g_cmdCurrent_mA = mA;
  sendCmd("C%d:%.1f\n", VALVE_CH, g_cmdCurrent_mA);

  if (printChange) {
    Serial.print("[VALVE] Command current -> ");
    Serial.print(g_cmdCurrent_mA);
    Serial.println(" mA");
  }
}

// ------------------------- Manual “simulation” state -------------------------
float g_desiredPressure = NAN;   // whatever units YOU want (kPa/bar/etc) for testing
float g_desiredFlow     = NAN;   // sL/min (or whatever you want)
float g_actualFlow      = NAN;

bool  g_hasSetpoint      = false;
bool  g_waitingForActual = false;

// ------------------------- Serial input handling (USB) -------------------------
String g_line = "";

void printHelp() {
  Serial.println("======================================");
  Serial.println("Manual Blender Control (Current Mode)");
  Serial.println("Type commands then press ENTER.");
  Serial.println("");
  Serial.println("Set desired targets:");
  Serial.println("  sp <pressure> <flow>     e.g.  sp 50 12.5");
  Serial.println("  p  <pressure>            e.g.  p 60");
  Serial.println("  f  <flow>                e.g.  f 10");
  Serial.println("");
  Serial.println("Provide actual flow (when prompted or anytime):");
  Serial.println("  a  <flow>                e.g.  a 9.2");
  Serial.println("  <number>                 e.g.  9.2  (if prompted)");
  Serial.println("");
  Serial.println("Other:");
  Serial.println("  off                      -> 0 mA");
  Serial.println("  status                   -> print setpoints + current");
  Serial.println("  help                     -> show this");
  Serial.println("======================================");
}

void printStatus() {
  Serial.print("[STATUS] DesiredP="); Serial.print(g_desiredPressure);
  Serial.print("  DesiredF="); Serial.print(g_desiredFlow);
  Serial.print("  ActualF=");  Serial.print(g_actualFlow);
  Serial.print("  CmdI=");     Serial.print(g_cmdCurrent_mA);
  Serial.println(" mA");
}

float parseSecondNumber(const String& s: ) = delete; // (ignore; Arduino doesn't like overload tricks)

// Handle one full line
void handleUserLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("help") || line.equalsIgnoreCase("?")) {
    printHelp();
    return;
  }

  if (line.equalsIgnoreCase("status")) {
    printStatus();
    return;
  }

  if (line.equalsIgnoreCase("off")) {
    setValveCurrent_mA(0.0f);
    return;
  }

  // Split by spaces
  // We'll do simple parsing via sscanf for reliability.
  char buf[96];
  line.toCharArray(buf, sizeof(buf));

  // sp <pressure> <flow>
  float p, f;
  if (sscanf(buf, "sp %f %f", &p, &f) == 2) {
    g_desiredPressure = p;
    g_desiredFlow = f;
    g_hasSetpoint = true;
    Serial.print("[SET] Desired pressure="); Serial.print(g_desiredPressure);
    Serial.print("  Desired flow="); Serial.println(g_desiredFlow);
    g_waitingForActual = true; // start prompting
    return;
  }

  // p <pressure>
  if (sscanf(buf, "p %f", &p) == 1) {
    g_desiredPressure = p;
    Serial.print("[SET] Desired pressure="); Serial.println(g_desiredPressure);
    return;
  }

  // f <flow>
  if (sscanf(buf, "f %f", &f) == 1) {
    g_desiredFlow = f;
    g_hasSetpoint = true;
    Serial.print("[SET] Desired flow="); Serial.println(g_desiredFlow);
    g_waitingForActual = true; // start prompting
    return;
  }

  // a <actualFlow>
  if (sscanf(buf, "a %f", &f) == 1) {
    g_actualFlow = f;
    g_waitingForActual = false;  // we received it
    return;
  }

  // If we are waiting for actual flow, allow plain number:
  if (g_waitingForActual) {
    float x = line.toFloat();
    if (x != 0.0f || line == "0" || line == "0.0") {
      g_actualFlow = x;
      g_waitingForActual = false;
      return;
    }
  }

  Serial.println("[ERR] Unknown command. Type 'help'.");
}

void serialUsbUpdate() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleUserLine(g_line);
      g_line = "";
    } else if (c != '\r') {
      g_line += c;
      if (g_line.length() > 80) g_line.remove(0);
    }
  }
}

// ------------------------- Control logic (step-based) -------------------------
void updateValveFromFlowError() {
  if (!g_hasSetpoint) return;
  if (!isfinite(g_desiredFlow)) return;
  if (!isfinite(g_actualFlow)) return;

  float err = g_desiredFlow - g_actualFlow; // + means "need more flow" -> open more

  // deadband: close enough, do nothing
  if (fabsf(err) < SMALL_ERR_FLOW) {
    Serial.print("[CTRL] Flow OK (err=");
    Serial.print(err);
    Serial.println("). No change.");
    return;
  }

  float step = STEP_MED_mA;
  if (fabsf(err) >= BIG_ERR_FLOW) step = STEP_BIG_mA;
  else step = STEP_MED_mA;

  // smaller nudges when near target
  if (fabsf(err) < 2.0f) step = STEP_SMALL_mA;

  float newCmd = g_cmdCurrent_mA + (err > 0 ? step : -step);

  // saturation warning logic
  bool wouldSaturateHigh = (newCmd > MAX_CURRENT_mA);
  bool wouldSaturateLow  = (newCmd < 0.0f);

  setValveCurrent_mA(newCmd);

  Serial.print("[CTRL] DesiredF="); Serial.print(g_desiredFlow);
  Serial.print(" ActualF="); Serial.print(g_actualFlow);
  Serial.print(" Err="); Serial.print(err);
  Serial.print(" Step="); Serial.print(step);
  Serial.print(" -> CmdI="); Serial.print(g_cmdCurrent_mA);
  Serial.println(" mA");

  if (wouldSaturateHigh && err > 0) {
    Serial.println("[WARN] Valve is at MAX current (max open) but still below desired flow.");
  }
  if (wouldSaturateLow && err < 0) {
    Serial.println("[WARN] Valve is at 0 mA (closed) but flow is still above desired flow.");
  }
}

// ------------------------------- Setup ------------------------------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(460800);

  for (uint8_t i = 0; i < VALUE_COUNT; i++) currentValues[i] = 0.0f;

  // Staiger init
  sendCmd("F0:20000\n"); delay(10);
  sendCmd("F1:20000\n"); delay(10);
  sendCmd("F2:20000\n"); delay(10);
  sendCmd("F3:20000\n"); delay(10);

  // Proportional mode ON for current control
  sendCmd("V0:1\n"); delay(20);

  // Disable PID influence for this test (we are commanding current directly)
  sendCmd("p0:0\n"); delay(10);
  sendCmd("i0:0\n"); delay(10);
  sendCmd("d0:0\n"); delay(10);
  sendCmd("r0:0\n"); delay(10);

  // Start closed
  sendCmd("C0:0\n"); delay(10);
  g_cmdCurrent_mA = 0.0f;

  printHelp();
  Serial.print("Valve channel: "); Serial.println(VALVE_CH);
  Serial.print("Max current (mA): "); Serial.println(MAX_CURRENT_mA);
  Serial.println("Set desired target with: sp <pressure> <flow>");
}

// ------------------------------- Loop -------------------------------------
void loop() {
  // Must run continuously
  serial1Update();

  // Read user commands from Serial Monitor
  serialUsbUpdate();

  // Prompt every 1s for actual flow, after a setpoint exists
  static uint32_t tPrompt = 0;
  if (g_hasSetpoint && (millis() - tPrompt > PROMPT_PERIOD_MS)) {
    tPrompt = millis();
    if (g_waitingForActual) {
      Serial.print("[PROMPT] Enter ACTUAL flow now (a <flow> OR just number): ");
      Serial.println();
    } else {
      // We already got a new actual flow recently -> do control update now,
      // then ask again next tick
      updateValveFromFlowError();
      g_waitingForActual = true;
    }
  }

  // Telemetry every 3 seconds AND it will naturally also print on valve-change above
  static uint32_t tTele = 0;
  if (millis() - tTele > TELEMETRY_PERIOD_MS) {
    tTele = millis();
    int di = dutyIndex(VALVE_CH);
    int ci = currentIndex(VALVE_CH);
    int si = sensorIndex(VALVE_CH);

    Serial.print("[TEL] ");
    Serial.print("Duty[%]=");     Serial.print(getValue(di));
    Serial.print("\tI[mA]=");      Serial.print(getValue(ci));
    Serial.print("\tSE[mV]=");     Serial.print(getValue(si));
    Serial.print("\tSupply[mV]="); Serial.print(getValue(12));

    Serial.print("\tDesiredP=");   Serial.print(g_desiredPressure);
    Serial.print("\tDesiredF=");   Serial.print(g_desiredFlow);
    Serial.print("\tActualF=");    Serial.print(g_actualFlow);
    Serial.print("\tCmdI=");       Serial.print(g_cmdCurrent_mA);

    Serial.println();
  }
}
