#include "mpu6500.h"
#include <Wire.h>
#include <BleMouse.h>  // BLE Mouse library


/* Define GPIO pins for buttons */
const int leftButtonPin = 17;   // GPIO17 for left mouse button
const int rightButtonPin = 16;  // GPIO16 for right mouse button

/* Mpu6500 object */
bfs::Mpu6500 imu;

/* Create a BLE Mouse object */
BleMouse bleMouse("AirMouse 1", "<Your Name>", 100);

/* Sensitivity factor to control cursor speed */
float sensitivity = 20.0;  // Lower value for slower cursor movement

/* Smoothing factor */
float smoothingFactor = 0.1;  // Lower value = smoother, slower response
float smoothCursorX = 0;
float smoothCursorY = 0;

/* Calibration data for gyroscope */
float gyroCalY = 0, gyroCalZ = 0;
int calibrationSamples = 100;  // Default sample count for calibration

/* Threshold to ignore small movements (noise) */
float threshold = 0.01;  // Adjust to match the noise level

/* Variables to hold button states */
int leftButtonState = HIGH;   // HIGH means not pressed (active-low buttons)
int rightButtonState = HIGH;  // HIGH means not pressed (active-low buttons)

/* LED blinking parameters */
unsigned long previousMillis = 0;    // Store last time LED state changed
const long fastBlinkInterval = 100;  // Fast blink interval for calibration
const long slowBlinkInterval = 500;  // Slow blink interval when BLE not connected
bool ledState = LOW;                 // Current state of the LED

void blinkLedNonBlocking(long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    ledState = !ledState;  // Toggle LED state
    digitalWrite(LED_BUILTIN, ledState);
  }
}

void calibrateGyro(int sampleCount = 100) {
  float sumY = 0, sumZ = 0;

  /* Collect multiple samples to calculate the baseline (average) */
  for (int i = 0; i < sampleCount; i++) {
    if (imu.Read()) {
      sumY += imu.gyro_y_radps();
      sumZ += imu.gyro_z_radps();
    }
    delay(10);                               // Small delay between samples
    blinkLedNonBlocking(fastBlinkInterval);  // Fast blink during calibration
  }

  /* Calculate the average (baseline) values */
  gyroCalY = sumY / sampleCount;
  gyroCalZ = sumZ / sampleCount;
}

void setup() {
  /* Serial to display data */
  Serial.begin(115200);

  /* Configure LED */
  pinMode(LED_BUILTIN, OUTPUT);

  /* Initialize button pins */
  pinMode(leftButtonPin, INPUT_PULLUP);   // Configure GPIO19 as input with pull-up
  pinMode(rightButtonPin, INPUT_PULLUP);  // Configure GPIO17 as input with pull-up

  /* Start BLE Mouse */
  bleMouse.begin();

  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);

  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);

  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while (1) {}
  }

  /* Configure IMU Sample Rate Divider */
  if (!imu.ConfigSrd(4)) {
    Serial.println("Error configuring SRD");
    while (1) {}
  }

  /* Calibrate the gyro */
  calibrateGyro(calibrationSamples);

  /* Turn off LED after calibration */
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  /* Check if BLE Mouse is connected */
  if (!bleMouse.isConnected()) {
    /* Slow blink if BLE is not connected */
    blinkLedNonBlocking(slowBlinkInterval);
  } else {
    /* Turn off LED when bleMouse is connected */
    if (!ledState) {
      ledState = HIGH;
      digitalWrite(LED_BUILTIN, ledState);
    }

    /* Check if data read from IMU */
    if (imu.Read()) {
      /* Get gyroscope values in radians per second */
      float gyroY = imu.gyro_y_radps();
      float gyroZ = imu.gyro_z_radps();

      /* Remove drift by subtracting calibration values */
      float adjustedGyroY = gyroY - gyroCalY;
      float adjustedGyroZ = gyroZ - gyroCalZ;

      /* Apply threshold to filter out noise */
      if (abs(adjustedGyroY) < threshold) adjustedGyroY = 0;
      if (abs(adjustedGyroZ) < threshold) adjustedGyroZ = 0;

      /* Cursor movement mapped to Y and Z gyro axes */
      int cursorX = adjustedGyroZ * sensitivity;  // Horizontal movement
      int cursorY = adjustedGyroY * sensitivity;  // Vertical movement

      /* Smooth the cursor movement */
      smoothCursorX = (smoothingFactor * cursorX) + ((1 - smoothingFactor) * smoothCursorX);
      smoothCursorY = (smoothingFactor * cursorY) + ((1 - smoothingFactor) * smoothCursorY);

      Serial.print("Adjusted Z: ");
      Serial.print(adjustedGyroZ);
      Serial.print("\tAdjusted Y: ");
      Serial.print(adjustedGyroY);
      Serial.print("\n");

      /* Move the cursor using BLE Mouse */
      bleMouse.move(smoothCursorX, -smoothCursorY);  // Negate Y for screen orientation
    }

    /* Read the button states */
    leftButtonState = digitalRead(leftButtonPin);
    rightButtonState = digitalRead(rightButtonPin);

    /* Left button press handling */
    if (leftButtonState == LOW) {
      bleMouse.press(MOUSE_LEFT);
      delay(1);
    } else {
      bleMouse.release(MOUSE_LEFT);
      delay(1);
    }

    /* Right button press handling */
    if (rightButtonState == LOW) {
      bleMouse.press(MOUSE_RIGHT);
      delay(1);
    } else {
      bleMouse.release(MOUSE_RIGHT);
      delay(1);
    }
  }

  /* Small delay for smoother cursor updates */
  delay(2);
}
