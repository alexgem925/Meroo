#include <Wire.h>
#include <PTE7300_I2C.h>

PTE7300_I2C* mySensor;
int16_t DSP_S;

const float FS_PSI = 5810.0f;   // match sensor full scale
// int16_t dsp_zero = 0;       // tare offset (raw counts) - not used for now

float dspToPsi(int16_t dsp) {
  // If you later enable tare:
  // dsp = dsp - dsp_zero;

  float psi = ((float)dsp + 16000.0f) * (FS_PSI / 32000.0f);

  // optional clamp for tiny negatives due to offset/noise
  if (psi < 0) psi = 0;
  return psi;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("Boot");

  Wire.begin();
  Wire.setClock(100000);

  mySensor = new PTE7300_I2C();
  mySensor->CRC(false); // use non-CRC address 0x6C

  // Optional tare (disabled for now)
  /*
  delay(50);
  if (mySensor->isConnected()) {
    dsp_zero = mySensor->readDSP_S();
    Serial.print("Tare DSP_S = ");
    Serial.println(dsp_zero);
  }
  */
}

void loop() {
  if (mySensor->isConnected()) {
    DSP_S = mySensor->readDSP_S();
    float psi = dspToPsi(DSP_S);

    Serial.print("DSP_S=");
    Serial.print(DSP_S);
    Serial.print("  PSI=");
    Serial.println(psi, 3);
  } else {
    Serial.println("Device not connected");
  }

  delay(100);
}