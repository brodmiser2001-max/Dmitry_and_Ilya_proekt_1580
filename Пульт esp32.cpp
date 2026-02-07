#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SoftwareSerial.h>

// Настройка UART для LoRa
#define LORA_RX 16
#define LORA_TX 17
SoftwareSerial loraSerial(LORA_RX, LORA_TX);

// Пины управления LoRa
#define M0_PIN 27
#define M1_PIN 14

// Джойстики
#define JOY1_X 34
#define JOY1_Y 35
#define JOY1_BTN 25
#define JOY2_X 36
#define JOY2_Y 39
#define JOY2_BTN 26

// Экран
#define TFT_CS 5
#define TFT_DC 2
#define TFT_RST 4
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Переменные
int joy1X, joy1Y, joy1Btn;
int joy2X, joy2Y, joy2Btn;
bool fireDetected = false;
String latitude = "";
String longitude = "";

// Виртуальный дуплекс
const int TX_DELAY_MS = 20;
const int RX_DELAY_MS = 50;

void setup() {
  Serial.begin(115200);
  loraSerial.begin(9600);

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);

  pinMode(JOY1_BTN, INPUT_PULLUP);
  pinMode(JOY2_BTN, INPUT_PULLUP);

  // LoRa в режиме приёма
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, HIGH);

  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
}

void loop() {
  // --- Считываем джойстики ---
  joy1X = analogRead(JOY1_X);
  joy1Y = analogRead(JOY1_Y);
  joy1Btn = digitalRead(JOY1_BTN);
  joy2X = analogRead(JOY2_X);
  joy2Y = analogRead(JOY2_Y);
  joy2Btn = digitalRead(JOY2_BTN);

  // Маппинг на диапазон 1000-2000
  joy1X = map(joy1X, 0, 4095, 1000, 2000);
  joy1Y = map(joy1Y, 0, 4095, 1000, 2000);
  joy2X = map(joy2X, 0, 4095, 1000, 2000);
  joy2Y = map(joy2Y, 0, 4095, 1000, 2000);

  // --- Формируем пакет для полётного контроллера ---
  String packet = "X1:" + String(joy1X) + ";Y1:" + String(joy1Y);
  packet += ";BTN1:" + String(joy1Btn) + ";X2:" + String(joy2X);
  packet += ";Y2:" + String(joy2Y) + ";BTN2:" + String(joy2Btn) + ";\n";

  // --- Передача по виртуальному дуплексу ---
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, LOW);
  delay(5);
  loraSerial.print(packet);
  delay(TX_DELAY_MS);

  // Перевод в режим приёма
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, HIGH);
  delay(RX_DELAY_MS);

  // --- Приём данных с полётного контроллера ---
  if (loraSerial.available()) {
    String incoming = loraSerial.readStringUntil('\n');

    fireDetected = (incoming.indexOf("FIRE:1") >= 0);

    int latIndex = incoming.indexOf("LAT:");
    int lonIndex = incoming.indexOf("LON:");

    if (latIndex >= 0 && lonIndex >= 0) {
      latitude = incoming.substring(latIndex + 4, incoming.indexOf(";", latIndex));
      longitude = incoming.substring(lonIndex + 4, incoming.indexOf(";", lonIndex));
    }
  }

  // --- Вывод на экран ---
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(10, 20);
  tft.print("Fire: "); tft.println(fireDetected ? "YES" : "NO");
  tft.print("Lat: "); tft.println(latitude);
  tft.print("Lon: "); tft.println(longitude);

  delay(50); // небольшая задержка для стабильной работы
}
