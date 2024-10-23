#include "mpu6500.h"
#include <Wire.h>
#include <BleMouse.h>  // BLE Mouse library

/* Mpu6500 object */
bfs::Mpu6500 imu;

/* Create a BLE Mouse object */
BleMouse bleMouse("AirMouse 1", "Hai Tran", 100);

/* Sensitivity factor to control cursor speed */
float sensitivity = 20.0;  // Lower value for slower cursor movement

/* Smoothing factor */
float smoothingFactor = 0.05;  // Lower value = smoother, slower response
float smoothCursorX = 0;
float smoothCursorY = 0;

/* Calibration data for gyroscope */
float gyroCalY = 0, gyroCalZ = 0;
int calibrationSamples = 100;  // Number of samples to take for calibration

/* Threshold to ignore small movements (noise) */
float threshold = 0.01;  // Adjust to match the noise level

/* Define GPIO pins for buttons */
const int leftButtonPin = 17;   // GPIO17 for left mouse button
const int rightButtonPin = 16;  // GPIO16 for right mouse button

/* Variables to hold button states */
int leftButtonState = HIGH;  // HIGH means not pressed (active-low buttons)
int rightButtonState = HIGH; // HIGH means not pressed (active-low buttons)

void calibrateGyro() {
  float sumY = 0, sumZ = 0;
  
  /* Collect multiple samples to calculate the baseline (average) */
  for (int i = 0; i < calibrationSamples; i++) {
    if (imu.Read()) {
      sumY += imu.gyro_y_radps();
      sumZ += imu.gyro_z_radps();
    }
    delay(20);  // Small delay between samples
  }

  /* Calculate the average (baseline) values */
  gyroCalY = sumY / calibrationSamples;
  gyroCalZ = sumZ / calibrationSamples;
}

void setup() {
  /* Start BLE Mouse */
  bleMouse.begin();

  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while (1) {}
  }
  /* Set the sample rate divider (for faster updates) */
  if (!imu.ConfigSrd(4)) {
    Serial.println("Error configuring SRD");
    while (1) {}
  }

  /* Calibrate the gyro to eliminate small drift */
  calibrateGyro();

  /* Initialize button pins */
  pinMode(leftButtonPin, INPUT_PULLUP);   // Configure GPIO19 as input with pull-up
  pinMode(rightButtonPin, INPUT_PULLUP);  // Configure GPIO17 as input with pull-up
}

void loop() {
  /* Check if BLE Mouse is connected */
  if (bleMouse.isConnected()) {
    /* Check if data read from IMU */
    if (imu.Read()) {
      /* Get gyroscope values in radians per second */
      float gyroY = imu.gyro_y_radps();
      float gyroZ = imu.gyro_z_radps();

      /* Subtract the calibrated baseline to remove drift */
      float adjustedGyroY = gyroY - gyroCalY;
      float adjustedGyroZ = gyroZ - gyroCalZ;

      /* Ignore small changes by applying the threshold */
      if (abs(adjustedGyroY) < threshold) adjustedGyroY = 0;
      if (abs(adjustedGyroZ) < threshold) adjustedGyroZ = 0;

      /* Swap Y and Z axis for movement mapping */
      int cursorX = adjustedGyroZ * sensitivity;  // Z-axis now controls horizontal movement
      int cursorY = adjustedGyroY * sensitivity;  // Y-axis now controls vertical movement

      /* Smooth the cursor movement */
      smoothCursorX = (smoothingFactor * cursorX) + ((1 - smoothingFactor) * smoothCursorX);
      smoothCursorY = (smoothingFactor * cursorY) + ((1 - smoothingFactor) * smoothCursorY);

      /* Move the cursor using BLE Mouse */
      bleMouse.move(smoothCursorX, -smoothCursorY);  // Negate Y for proper screen movement
    }

    /* Read the state of the buttons */
    leftButtonState = digitalRead(leftButtonPin);
    rightButtonState = digitalRead(rightButtonPin);

    /* Check if the left button is pressed */
    if (leftButtonState == LOW) {
      bleMouse.press(MOUSE_LEFT);   // Simulate left mouse button press
    } else {
      bleMouse.release(MOUSE_LEFT); // Release left mouse button when not pressed
    }

    /* Check if the right button is pressed */
    if (rightButtonState == LOW) {
      bleMouse.press(MOUSE_RIGHT);  // Simulate right mouse button press
    } else {
      bleMouse.release(MOUSE_RIGHT); // Release right mouse button when not pressed
    }
  }

  /* Add a small delay to avoid spamming cursor updates */
  delay(5);  // Adjust as needed for smoother movement
}
