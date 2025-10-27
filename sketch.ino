/*
Nama: Ananda Naura Qurrota A’yun
NIM: 25/568126/SV/27369
Program Magang GMRT 2026 – Hari Ketiga
Topik: Following-Axis Servo Motor and Motion Detection
*/

#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// ===== Deklarasi Servo =====
Servo servo1, servo2, servo3, servo4, servo5;

// Pin Servo
#define SERVO1_PIN 13
#define SERVO2_PIN 12
#define SERVO3_PIN 14
#define SERVO4_PIN 27
#define SERVO5_PIN 26

// PIR Sensor
#define PIR_PIN 25

// ===== Variabel =====
int initialPos = 90;  // posisi awal (tegak lurus)
bool motionDetected = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Inisialisasi MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 tidak ditemukan!");
    while (1);
  }

  // Inisialisasi servo
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);

  // Set posisi awal semua servo
  servo1.write(initialPos);
  servo2.write(initialPos);
  servo3.write(initialPos);
  servo4.write(initialPos);
  servo5.write(initialPos);

  // Inisialisasi PIR
  pinMode(PIR_PIN, INPUT);

  Serial.println("Sistem Siap!");
}

void loop() {
  // ==== Baca sensor PIR ====
  motionDetected = digitalRead(PIR_PIN);

  // ==== Baca data MPU6050 ====
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Konversi sederhana ke derajat (asumsi normalisasi ±90°)
  float roll = a.acceleration.y * 5;   // perkiraan kasar
  float pitch = a.acceleration.x * 5;
  float yaw = g.gyro.z;

  // ==== LOGIKA ====
  if (motionDetected) {
    Serial.println("Gerakan terdeteksi oleh PIR!");
    moveAllServo(60);  // posisi bebas
    delay(1000);
    resetAllServo();
  } else {
    // Roll positif -> servo1 & servo2 melawan arah
    if (roll > 5) {
      servo1.write(initialPos - 20);
      servo2.write(initialPos - 20);
    } else if (roll < -5) {
      servo1.write(initialPos + 20);
      servo2.write(initialPos + 20);
    } else {
      servo1.write(initialPos);
      servo2.write(initialPos);
    }

    // Pitch positif -> servo3 & servo4 searah
    if (pitch > 5) {
      servo3.write(initialPos + 20);
      servo4.write(initialPos + 20);
    } else if (pitch < -5) {
      servo3.write(initialPos - 20);
      servo4.write(initialPos - 20);
    } else {
      servo3.write(initialPos);
      servo4.write(initialPos);
    }

    // Yaw positif/negatif -> servo5 mengikuti arah
    if (yaw > 0.5) {
      servo5.write(initialPos + 25);
    } else if (yaw < -0.5) {
      servo5.write(initialPos - 25);
    } else {
      servo5.write(initialPos);
    }
  }

  delay(200);
}

// ==== Fungsi bantu ====
void moveAllServo(int pos) {
  servo1.write(pos);
  servo2.write(pos);
  servo3.write(pos);
  servo4.write(pos);
  servo5.write(pos);
}

void resetAllServo() {
  servo1.write(initialPos);
  servo2.write(initialPos);
  servo3.write(initialPos);
  servo4.write(initialPos);
  servo5.write(initialPos);
}
