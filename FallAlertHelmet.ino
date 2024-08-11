#include <Wire.h>
#include <Math.h>

const int MPU_ADDR = 0x68;
const int buzzerPin = 7;  // Buzzer pin number

int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z;

// Constants for sensor calibration (adjust these values for your specific sensor)
float accel_scale_factor = 16384.0;  // Accelerometer sensitivity scale factor for +/- 2g range
float gyro_scale_factor = 131.0;    // Gyroscope sensitivity scale factor for +/- 250 degrees/s range

// Variables for angle calculation and integration
float pitch, roll;                  // Pitch and roll angles in degrees
float pitch_prev, roll_prev;        // Previous pitch and roll angles
float gyro_x_rate, gyro_y_rate;     // Gyroscope rates in degrees per second
float dt = 0.01;                    // Integration time step in seconds

// Buzzer activation threshold
float buzzerThreshold = 75.0;       // Adjust this value as needed

void setup() {
  pinMode(A0, INPUT);
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600);

  Wire.begin();  // SDA pin: D2, SCL pin: D1
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() {
  readSensorData();
  calculatePitchRoll();

  if ((abs(roll) + abs(pitch)) > buzzerThreshold){
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  int analogValue = analogRead(A0);
  int piezo_signal = (analogValue / 100) * 5;

  Serial.print(piezo_signal);
  Serial.print("\t");

  if (piezo_signal > 10) {
    int i = 0;
    while (true) {
      digitalWrite(buzzerPin, HIGH);
      i = i + 1;
      if (i > 10000) {
        break;
      }
    }
  }

  Serial.print(pitch);
  Serial.print("/");
  Serial.println(roll);

  delay(10);
}

void readSensorData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  accelerometer_x = Wire.read() << 8 | Wire.read();
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}

void calculatePitchRoll() {
  float accel_x = accelerometer_x / accel_scale_factor;
  float accel_y = accelerometer_y / accel_scale_factor;
  float accel_z = accelerometer_z / accel_scale_factor;

  pitch = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / M_PI;
  roll = atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / M_PI;
}
