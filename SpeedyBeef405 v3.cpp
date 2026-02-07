#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Настройки последовательного порта для связи с Raspberry Pi
SoftwareSerial piSerial(10, 11); // RX, TX

// Настройки последовательного порта для связи с пультом через LoRa
#define LORA_TX 17
#define LORA_RX 16
SoftwareSerial loraSerial(LORA_RX, LORA_TX);

// Пины для управления режимом LoRa
#define M0_PIN 27
#define M1_PIN 14
#define AUX_PIN 13

// GPS данные БПЛА
double currentLat = 55.7522;  // пример
double currentLon = 37.6156;
double altitude = 120.0; // в метрах
double cameraTiltDeg = 30.0; // угол наклона камеры

// Переменные для хранения координат пожара
bool fireDetected = false;
int firePxX = 0;
int firePxY = 0;
double fireLat = 0.0;
double fireLon = 0.0;

// Константы камеры
const int CAM_WIDTH = 640;
const int CAM_HEIGHT = 480;
const double CAM_FOV_DEG = 66.0;

// Временные задержки для виртуального дуплекса
const int TX_DELAY_MS = 20;
const int RX_DELAY_MS = 50;

// Функция пересчёта пиксельных координат в GPS
void calculateFireGPS(int pxX, int pxY, double &lat, double &lon) {
  // Смещение от центра кадра в градусах
  double alpha = ((double)pxX - CAM_WIDTH / 2) * (CAM_FOV_DEG / CAM_WIDTH);
  double beta = ((double)pxY - CAM_HEIGHT / 2) * (CAM_FOV_DEG / CAM_HEIGHT);

  // Смещение в метрах на земле
  double dx = altitude * tan(radians(alpha));
  double dy = altitude * tan(radians(beta));

  // Перевод в GPS приближённо (1 градус ~ 111000 м)
  lat = currentLat + (dy / 111000.0);
  lon = currentLon + (dx / (111000.0 * cos(radians(currentLat))));
}

// Функция отправки пакета на пульт
void sendPacketToGround() {
  // Формируем пакет
  String packet = "FIRE:";
  packet += (fireDetected ? "1" : "0");
  packet += ";LAT:";
  packet += fireLat;
  packet += ";LON:";
  packet += fireLon;
  packet += ";\n";

  // Виртуальный дуплекс: перевод LoRa в режим передачи
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, LOW);
  delay(5);

  loraSerial.print(packet);
  delay(TX_DELAY_MS);

  // Перевод LoRa в режим приёма
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, HIGH);
  delay(RX_DELAY_MS);
}

void setup() {
  Serial.begin(115200);
  piSerial.begin(9600);
  loraSerial.begin(9600);

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(AUX_PIN, INPUT);

  // Изначально LoRa в режиме приёма
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, HIGH);

  Serial.println("Flight controller started");
}

void loop() {
  // --- Чтение данных от Raspberry Pi ---
  if (piSerial.available()) {
    String incoming = piSerial.readStringUntil('\n');

    if (incoming.indexOf("FIRE:1") >= 0) {
      fireDetected = true;

      int pxXIndex = incoming.indexOf("PX:");
      int pxYIndex = incoming.indexOf("PY:");
      if (pxXIndex >= 0 && pxYIndex >= 0) {
        firePxX = incoming.substring(pxXIndex + 3, incoming.indexOf(";", pxXIndex)).toInt();
        firePxY = incoming.substring(pxYIndex + 3, incoming.indexOf(";", pxYIndex)).toInt();
      }

      // Расчёт точных координат пожара
      calculateFireGPS(firePxX, firePxY, fireLat, fireLon);
    } else {
      fireDetected = false;
    }
  }

  // --- Отправка данных на пульт ---
  sendPacketToGround();

  // Здесь можно добавить другие функции полётного контроллера (стабилизация, управление)
}
