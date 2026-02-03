#include <SPI.h>
#include "ICM_20948.h"

// --- Pins (XIAO nRF52840, adjust if needed) ---
static const uint8_t PIN_CS = D5; // IMU CS pin

ICM_20948_SPI myICM;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("ICM-20948 SPI DMP Quat9 + Raw Mag (Polling)");

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

  // Initialize DMP (SparkFun’s recommended sequence)
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable DMP orientation sensor (Quat9)
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  // Set DMP output data rate for Quat9
  // Value = (DMP running rate / ODR) - 1
  // DMP default runs at 1125 Hz. For ~10 Hz: (1125/10) - 1 ≈ 111
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 5) == ICM_20948_Stat_Ok);

  // Enable FIFO and DMP, reset both
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP()  == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP()   == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO()  == ICM_20948_Stat_Ok);

  if (!success) {
    Serial.println("DMP config failed. Check ICM_20948_USE_DMP and wiring.");
    while (1) {}
  }

  Serial.println("DMP enabled. Polling FIFO for data...");
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

      // Quat9 gives Q1, Q2, Q3 and heading accuracy.
      // Q0 is reconstructed: q0^2 + q1^2 + q2^2 + q3^2 = 1
      double q1 = ((double)dmpData.Quat9.Data.Q1) / 1073741824.0; // 2^30
      double q2 = ((double)dmpData.Quat9.Data.Q2) / 1073741824.0;
      double q3 = ((double)dmpData.Quat9.Data.Q3) / 1073741824.0;
      double q0 = sqrt(1.0 - (q1*q1 + q2*q2 + q3*q3));

      // Raw magnetometer via the normal sensor path
      // (Note: DMP does fusion using its own mag pipeline, but we can still read raw here)
      float mx = myICM.magX();
      float my = myICM.magY();
      float mz = myICM.magZ();

      // Serial Plotter–friendly CSV:
      // q0,q1,q2,q3,mx,my,mz
      Serial.print(q0, 6); Serial.print(",");
      Serial.print(q1, 6); Serial.print(",");
      Serial.print(q2, 6); Serial.print(",");
      Serial.print(q3, 6); Serial.print(",");
      Serial.print(mx, 3); Serial.print(",");
      Serial.print(my, 3); Serial.print(",");
      Serial.println(mz, 3);
    }
  }

  // Don’t hammer the bus; ~5–10ms is fine
  delay(5);
}

