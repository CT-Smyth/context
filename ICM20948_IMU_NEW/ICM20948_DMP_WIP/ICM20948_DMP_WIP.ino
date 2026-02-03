#include <SPI.h>
#include "ICM_20948.h"

// --- Pins (XIAO nRF52840, adjust if needed) ---
static const uint8_t PIN_CS = D5; // IMU CS pin

ICM_20948_SPI myICM;

uint32_t frameCounter = 0;

// Convert float in [-1,1] to Q15 int16
int16_t floatToQ15(float x)
{
  if (x >  1.0f) x =  1.0f;
  if (x < -1.0f) x = -1.0f;

  long v = (long)lroundf(x * 32767.0f); // map -1..1 to -32768..32767-ish
  if (v >  32767) v =  32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("ICM-20948 SPI DMP Quat9 + Raw Accel + Raw Mag (integer frames)");

  // Start SPI
  SPI.begin();
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH); // idle high

  // Initialize IMU over SPI
  ICM_20948_Status_e status = myICM.begin(PIN_CS, SPI);
  Serial.print("ICM begin returned: ");
  Serial.println(myICM.statusString(status));

  if (status != ICM_20948_Stat_Ok) {
    Serial.println("IMU init failed, stopping.");
    while (1) {}
  }

  Serial.println("IMU connected. Initializing DMP...");

  bool success = true;

  // Initialize DMP
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable DMP orientation sensor (Quat9)
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  // Set DMP output data rate for Quat9
  // You've already observed that 5 gives ~100ms (~10 Hz)
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 5) == ICM_20948_Stat_Ok);

  // Enable FIFO and DMP, reset both
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP()  == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP()   == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO()  == ICM_20948_Stat_Ok);

  // Start the embedded magnetometer (AK09916) so mag reads work over SPI
  ICM_20948_Status_e magStatus = myICM.startupMagnetometer();
  Serial.print("startupMagnetometer returned: ");
  Serial.println(myICM.statusString(magStatus));
  success &= (magStatus == ICM_20948_Stat_Ok);

  if (!success) {
    Serial.println("DMP/mag config failed. Check ICM_20948_USE_DMP and wiring.");
    while (1) {}
  }

  Serial.println("DMP + Mag enabled. Polling FIFO for integer frames...");
  Serial.println("Format: frame q0 q1 q2 q3 ax ay az mx my mz");
}

void loop() {
  icm_20948_DMP_data_t dmpData;

  // Try to read one DMP frame from FIFO
  myICM.readDMPdataFromFIFO(&dmpData);

  // Only proceed if we actually got a valid frame
  if (myICM.status == ICM_20948_Stat_Ok ||
      myICM.status == ICM_20948_Stat_FIFOMoreDataAvail) {

    // Check if this frame contains Quat9 data
    if (dmpData.header & DMP_header_bitmap_Quat9) {

      frameCounter++;

      // --- 1) Quaternion from DMP (Q30 fixed point -> float) ---
      float fq1 = (float)dmpData.Quat9.Data.Q1 / 1073741824.0f; // 2^30
      float fq2 = (float)dmpData.Quat9.Data.Q2 / 1073741824.0f;
      float fq3 = (float)dmpData.Quat9.Data.Q3 / 1073741824.0f;

      // Rebuild q0 safely: q0^2 + q1^2 + q2^2 + q3^2 ≈ 1
      float mag2 = fq1*fq1 + fq2*fq2 + fq3*fq3;
      float t    = 1.0f - mag2;
      if (t < 0.0f) t = 0.0f;
      if (t > 1.0f) t = 1.0f;
      float fq0 = sqrtf(t);

      // --- 2) Convert quaternion to Q15 int16 ---
      int16_t q0 = floatToQ15(fq0);
      int16_t q1 = floatToQ15(fq1);
      int16_t q2 = floatToQ15(fq2);
      int16_t q3 = floatToQ15(fq3);

      // --- 3) Read fresh accel + mag (raw int16) ---
      myICM.getAGMT(); // updates myICM.agmt

      int16_t ax = myICM.agmt.acc.axes.x;
      int16_t ay = myICM.agmt.acc.axes.y;
      int16_t az = myICM.agmt.acc.axes.z;

      int16_t mx = myICM.agmt.mag.axes.x;
      int16_t my = myICM.agmt.mag.axes.y;
      int16_t mz = myICM.agmt.mag.axes.z;

      // --- 4) Print full integer frame ---
      // frameID q0 q1 q2 q3 ax ay az mx my mz
      Serial.print(frameCounter); Serial.print(" ");

      Serial.print(q0); Serial.print(" ");
      Serial.print(q1); Serial.print(" ");
      Serial.print(q2); Serial.print(" ");
      Serial.print(q3); Serial.print(" ");

      Serial.print(ax); Serial.print(" ");
      Serial.print(ay); Serial.print(" ");
      Serial.print(az); Serial.print(" ");

      Serial.print(mx); Serial.print(" ");
      Serial.print(my); Serial.print(" ");
      Serial.println(mz);
    }
  }

  // Small delay to avoid hammering SPI. DMP runs independently.
  delay(5);
}
