/*
* STAIGER O2-CONTROLLED PSV WITH PRESSURE MONITORING
* ---------------------------------------------------------
* FEATURES:
* 1. O2% input converted to flow (150*(O2%-21%)/(0.95-0.21))
* 2. PSV breathing mode with dynamic flow control
* 3. PTE7300 pressure sensor monitoring
* 4. Combined telemetry output
* ---------------------------------------------------------
*/

#include <Arduino.h>
#include <stdarg.h>
#include <math.h>
#include <Wire.h>
#include <PTE7300_I2C.h>

// ============================ PID TUNING ============================
float Kp = 0.4;       // Proportional gain
float Ki = 0.8;       // Integral gain
float Kd = 0.0;       // Derivative gain

// ============================ PHYSICS MAP ============================
float FF_OFFSET = 106.0; 
float FF_SLOPE  = 0.34;  

// ============================ FLOW SENSOR CALIBRATION ============================
const float SE1_ZERO_V  = 2.452; 
const float SE1_RANGE_V = 1.666; 
const float FLOW_MAX_LMIN = 180.0; 

// ============================ PRESSURE SENSOR CONFIG ============================
PTE7300_I2C* mySensor;
int16_t DSP_S = -16000;  // Initialize to expected atmospheric value
const float FS_PSI = 5810.0f;   // match sensor full scale
float currentPressure_PSI = 0.0f;  // global storage for latest pressure reading
static const uint32_t PRESSURE_READ_PERIOD_MS = 100;  // Read pressure every 100ms
bool sensorConnected = false;

// Pressure conversion function
float dspToPsi(int16_t dsp) {
  // Direct conversion without tare - the sensor already outputs gauge pressure
  // Formula: PSI = (DSP_S + 16000) * (FS_PSI / 32000)
  float psi = ((float)dsp + 16000.0f) * (FS_PSI / 32000.0f);
  if (psi < 0) psi = 0;  // clamp tiny negatives
  return psi;
}

// ============================ CONTROL VARIABLES ============================
bool active = false;
bool breathingMode = false;
float targetFlow = 0;
float currentFlow = 0;
float integral = 0;
unsigned long lastLoop = 0;

// PSV Variables
float psvPeak = 0;
unsigned long breathStartTime = 0;
const int BREATH_PERIOD_MS = 3000; // 20 BPM
const int RISE_TIME_MS = 300;      // 0.3s Rise

// Cycle Tracking
int lastPhase = -1; // To detect new breath

// ============================ STAIGER SERIAL COMM ============================
const uint8_t VALUE_COUNT = 13;
float currentValues[VALUE_COUNT];
char lineBuffer[128];
uint16_t linePos = 0;

void parseLine(char *line) {
  uint8_t idx = 0;
  char *token = strtok(line, "\t");
  while (token != NULL && idx < VALUE_COUNT) {
    noInterrupts(); currentValues[idx] = atof(token); interrupts();
    idx++; token = strtok(NULL, "\t");
  }
}

void serial1Update() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '\n') {
      lineBuffer[min(linePos, (uint16_t)127)] = '\0';
      parseLine(lineBuffer); linePos = 0;
    } else if (c != '\r') {
      if (linePos < 127) lineBuffer[linePos++] = c;
    }
  }
}

float getValue(uint8_t index) {
  noInterrupts(); float val = currentValues[index]; interrupts();
  return val;
}

void sendCmd(const char* fmt, ...) {
  char buf[64];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial1.print(buf);
}

void setBoardCurrent(float mA) {
  if (mA < 0) mA = 0;
  if (mA > 300) mA = 300;
  sendCmd("C0:%.1f\n", mA);
}

float getFlow() {
  float mV = getValue(2); 
  if (isnan(mV) || mV < 100) return 0.0;
  float volts = mV / 1000.0;
  float flow = (volts - SE1_ZERO_V) * (FLOW_MAX_LMIN / SE1_RANGE_V);
  return (flow < 0) ? 0 : flow;
}

// ============================ OXYGEN CONVERSION ============================
// Formula: Flow = 150 * (O2% - 21%) / (0.95 - 0.21)
float o2PercentToFlow(float o2Percent) {
  // Convert percentage (e.g., 40) to decimal (0.40) if needed
  if (o2Percent > 1.0) {
    o2Percent = o2Percent / 100.0;
  }
  
  // Safety checks
  if (o2Percent < 0.21) {
    Serial.println("ERROR: O2% must be >= 21%");
    return 0;
  }
  if (o2Percent > 0.95) {
    Serial.println("ERROR: O2% must be <= 95%");
    return 0;
  }
  
  // Calculate flow using the formula
  float flow = 150.0 * (o2Percent - 0.21) / (0.95 - 0.21);
  
  Serial.print(">> O2: "); Serial.print(o2Percent * 100, 1); 
  Serial.print("% -> Flow: "); Serial.print(flow, 1); Serial.println(" L/min");
  
  return flow;
}

// ============================ PRESSURE SENSOR FUNCTIONS ============================
void updatePressureSensor() {
  static uint32_t tPressure = 0;
  
  // Read pressure sensor periodically
  if (millis() - tPressure > PRESSURE_READ_PERIOD_MS) {
    tPressure = millis();
    
    if (sensorConnected && mySensor->isConnected()) {
      int16_t tempDSP = mySensor->readDSP_S();
      // Only update if we got a valid reading (not zero)
      if (tempDSP != 0) {
        DSP_S = tempDSP;
        currentPressure_PSI = dspToPsi(DSP_S);
      }
    }
  }
}

// ============================ CONTROL LOGIC ============================

void setTarget(float flow) {
  breathingMode = false;
  if (flow <= 2.0) {
    active = false;
    targetFlow = 0;
    integral = 0;
    setBoardCurrent(0);
    Serial.println(">> STOPPED.");
    return;
  }
  Serial.print(">> TARGET: "); Serial.println(flow);

  if (!active) { setBoardCurrent(200.0); delay(40); }

  integral = 0;
  targetFlow = flow;
  active = true;
}

void startPSV(float maxFlow) {
  Serial.print(">> PSV MODE: Peak "); Serial.println(maxFlow);
  psvPeak = maxFlow;
  breathStartTime = millis();
  breathingMode = true;
  active = true;
  integral = 0;
  lastPhase = -1;
  setBoardCurrent(200.0); // Kick start
  delay(40);
}

void updateControl() {
  if (!active) return;

  if (millis() - lastLoop < 30) return; // 33Hz Loop
  float dt = (millis() - lastLoop) / 1000.0;
  lastLoop = millis();

  // --- PSV GENERATOR ---
  if (breathingMode) {
    unsigned long timeInCycle = (millis() - breathStartTime) % BREATH_PERIOD_MS;

    // DETECT NEW BREATH (Reset Integral)
    int currentPhase = timeInCycle / 100;
    if (currentPhase < lastPhase) {
       integral = 0; // CLEAN SLATE for the new breath!
    }
    lastPhase = currentPhase;

    if (timeInCycle < RISE_TIME_MS) {
      // PHASE 1: RISE
      targetFlow = ((float)timeInCycle / RISE_TIME_MS) * psvPeak;
    } else {
      // PHASE 2: DECAY
      float decayDuration = BREATH_PERIOD_MS - RISE_TIME_MS;
      float timeInDecay = timeInCycle - RISE_TIME_MS;
      targetFlow = psvPeak * (1.0 - (timeInDecay / decayDuration));
    }
    if (targetFlow < 2.0) targetFlow = 0;
  }

  // --- PID ---
  currentFlow = getFlow();
  float error = targetFlow - currentFlow;

  if (abs(error) < 0.5) error = 0.0;

  integral += (error * dt);
  if (integral > 40) integral = 40;
  if (integral < -40) integral = -40;

  float feedforward = FF_OFFSET + (targetFlow * FF_SLOPE);
  float output_mA = feedforward + (Kp * error) + (Ki * integral);

  // --- DYNAMIC FLOOR ---
  float dynamicFloor = 80.0 + (targetFlow * 0.5); 
  if (dynamicFloor > 112.0) dynamicFloor = 112.0;

  if (targetFlow > 1.0) {
    if (output_mA < dynamicFloor) output_mA = dynamicFloor;
  } else {
    output_mA = 0;
    integral = 0;
  }

  if (output_mA > 300) output_mA = 300;

  setBoardCurrent(output_mA);

  // Print telemetry with pressure
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print("T:"); Serial.print(targetFlow,1);
    Serial.print("\tF:"); Serial.print(currentFlow,1);
    Serial.print("\tP:"); Serial.print(currentPressure_PSI, 2);
    Serial.print(" psi");
    Serial.println();
  }
}

// ============================ MAIN ============================
void setup() {
  // USB Serial for PC communication
  Serial.begin(115200);
  delay(1500);
  
  // Serial1 for Staiger controller
  Serial1.begin(460800);
  
  // I2C for pressure sensor
  Wire.begin();
  Wire.setClock(100000);
  
  // Initialize pressure sensor
  mySensor = new PTE7300_I2C();
  mySensor->CRC(false); // use non-CRC address 0x6C
  
  delay(100);
  
  // Check if sensor is connected
  if (mySensor->isConnected()) {
    sensorConnected = true;
    Serial.println("Pressure sensor: CONNECTED");
  } else {
    sensorConnected = false;
    Serial.println("Pressure sensor: NOT DETECTED");
  }
  
  // Initialize Staiger controller
  for (uint8_t i = 0; i < VALUE_COUNT; i++) currentValues[i] = 0.0f;
  
  delay(1000);
  sendCmd("C0:0\n"); 
  sendCmd("V0:1\n"); 
  
  Serial.println("======================================");
  Serial.println("  O2-CONTROLLED PSV WITH PRESSURE");
  Serial.println("======================================");
  Serial.println("Commands:");
  Serial.println("  o2 [%]      - Set O2% (21-95), e.g., 'o2 40'");
  Serial.println("  psv [%]     - PSV mode with O2%, e.g., 'psv 60'");
  Serial.println("  set [flow]  - Direct flow control (L/min)");
  Serial.println("  pressure    - Show current pressure");
  Serial.println("  stop        - Stop valve");
  Serial.println("======================================");
}

String cmd = "";

void loop() {
  // Update Staiger board data
  serial1Update(); 
  
  // Update pressure sensor
  updatePressureSensor();
  
  // Handle user commands
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      cmd.trim();
      
      // O2 percentage command: "o2 40" sets constant flow based on 40% O2
      if (cmd.startsWith("o2 ")) {
        float o2Percent = cmd.substring(3).toFloat();
        float flow = o2PercentToFlow(o2Percent);
        if (flow > 0) setTarget(flow);
      }
      // PSV with O2 percentage: "psv 60" uses 60% O2 as peak
      else if (cmd.startsWith("psv ")) {
        float o2Percent = cmd.substring(4).toFloat();
        float flow = o2PercentToFlow(o2Percent);
        if (flow > 0) startPSV(flow);
      }
      // Direct flow control (bypass O2 calculation): "set 30"
      else if (cmd.startsWith("set ")) {
        float flow = cmd.substring(4).toFloat();
        Serial.print(">> Direct Flow: "); Serial.print(flow); Serial.println(" L/min");
        setTarget(flow);
      }
      // Pressure query
      else if (cmd == "pressure" || cmd == "p") {
        Serial.print("Current Pressure: ");
        Serial.print(currentPressure_PSI, 3);
        Serial.print(" PSI (DSP_S: ");
        Serial.print(DSP_S);
        Serial.println(")");
      }
      // Stop command
      else if (cmd == "stop" || cmd == "0") {
        setTarget(0);
      }
      // Help command
      else if (cmd == "help" || cmd == "?") {
        Serial.println("Commands:");
        Serial.println("  o2 [%]      - Set O2% (21-95)");
        Serial.println("  psv [%]     - PSV mode with O2%");
        Serial.println("  set [flow]  - Direct flow (L/min)");
        Serial.println("  pressure    - Show pressure");
        Serial.println("  stop        - Stop valve");
      }
      // Unknown command
      else if (cmd.length() > 0) {
        Serial.println("Unknown command. Type 'help' for commands.");
      }
      
      cmd = "";
    } else if (c != '\r') cmd += c;
  }
  
  // Update control loop
  updateControl();
}
