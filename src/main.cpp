// Core includes
#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include "ui.h"

// SPS30 sensor
#include <sps30.h>

// BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Display + sensor vars
#define BUZZER_PIN 15
#define LVGL_TICK_PERIOD 5
static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 240;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);

// BLE UUIDs
#define SERVICE_UUID        "d1f8ef04-1fa7-4806-ba3a-8115c0bcce6e"
#define CHARACTERISTIC_UUID "11d29a29-98ee-453e-8c87-2f95f8676519"

// BLE vars
BLECharacteristic *pCharacteristic;
BLEServer *pServer;
BLEDescriptor *pDescr;
BLE2902 *pBLE2902;
volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;
static bool initialNotified = false;
bool alertON = true;
static String lastSentPayload = "";

// Shared data
char blePayload[32];
float pm25 = 0, pm10 = 0;
SemaphoreHandle_t xPayloadSemaphore;

struct ISPUThreshold {
  float Xb, Xa; // Konsentrasi ambien bawah dan atas
  int Ib, Ia;   // ISPU batas bawah dan atas
};

// Ambang ISPU untuk PM2.5
ISPUThreshold ispuPM25[] = {
  {0, 15.5, 0, 50},
  {15.6, 55.4, 51, 100},
  {55.5, 150.4, 101, 200},
  {150.5, 250.4, 201, 300},
  {250.5, 500.0, 301, 500}
};

// Ambang ISPU untuk PM10
ISPUThreshold ispuPM10[] = {
  {0, 50, 0, 50},
  {51, 150, 51, 100},
  {151, 350, 101, 200},
  {351, 420, 201, 300},
  {421, 500, 301, 500}
};

// Fungsi untuk menghitung ISPU berdasarkan nilai dan ambang parameter
int hitungISPU(float Xx, ISPUThreshold* threshold, int n) {
  for (int i = 0; i < n; i++) {
    if (Xx >= threshold[i].Xb && Xx <= threshold[i].Xa) {
      float Xa = threshold[i].Xa;
      float Xb = threshold[i].Xb;
      int Ia = threshold[i].Ia;
      int Ib = threshold[i].Ib;
      return round(((Ia - Ib) / (Xa - Xb)) * (Xx - Xb) + Ib);
    }
  }
  // Di luar rentang tabel
  return -1;
}

// Fungsi untuk mengembalikan status berdasarkan ISPU
const char* kategoriISPU(int ispu) {
  if (ispu >= 0 && ispu <= 50) return "Very Good";
  else if (ispu <= 100) return "Good";
  else if (ispu <= 200) return "Fair";
  else if (ispu <= 300) return "Poor";
  else if (ispu > 300) return "Hazardous";
  return "Unknown";
}


// SPS30 Init
void sps_init() {
  delay(2000);
  sensirion_i2c_init();
  while (sps30_probe() != 0) {
    Serial.println("SPS sensor probe failed");
    delay(500);
  }
  Serial.println("SPS sensor ready");
  sps30_set_fan_auto_cleaning_interval_days(4);
  sps30_start_measurement();
  delay(2000);
}

// Read sensor
bool sps_read(float *pm25, float *pm10) {
  struct sps30_measurement m;
  uint16_t data_ready;
  int16_t ret;

  for (int i = 0; i < 10; ++i) {
    ret = sps30_read_data_ready(&data_ready);
    if (ret == 0 && data_ready) break;
    delay(100);
  }

  ret = sps30_read_measurement(&m);
  if (ret < 0) return false;

  *pm25 = 7.493066390949210 + (0.69661797 * m.mc_2p5);
  *pm10 = 8.861446488889847 + (0.91803023 * m.mc_10p0);

  return true;
}

// BLE server callback class
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    deviceConnected = true;
    lv_label_set_text(ui_title, "PM Analyzer (C)");
  }

  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    lv_label_set_text(ui_title, "PM Analyzer (D)");
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      uint8_t byteValue = value[0];
      Serial.print("Received byte: ");
      Serial.println(byteValue);

      alertON = (byteValue == 1);
    } else {
      Serial.println("Received empty write");
    }
  }
};

// BLE Init
void BLE_init() {
  BLEDevice::init("Particulate Analyzer");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pDescr = new BLEDescriptor((uint16_t)0x2901);
  pDescr->setValue("ESP32 Device");
  pCharacteristic->addDescriptor(pDescr);
  pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);
  pCharacteristic->addDescriptor(pBLE2902);
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  
  BLEDevice::startAdvertising();
  Serial.println("Waiting for connection...");
}

void BLETask(void *param) {
  BLE_init();

  while (true) {
    if (deviceConnected) {
      if (xSemaphoreTake(xPayloadSemaphore, portMAX_DELAY)) {
        String currentPayload = String(blePayload);

        if (currentPayload != lastSentPayload) {
          pCharacteristic->setValue(blePayload);
          pCharacteristic->notify();
          Serial.print("BLE Notify: ");
          Serial.println(blePayload);
          lastSentPayload = currentPayload;
        }
        xSemaphoreGive(xPayloadSemaphore);
      }
    }

    if (!deviceConnected && oldDeviceConnected) {
      Serial.println("Restart advertising...");
      pServer->startAdvertising();
      oldDeviceConnected = deviceConnected;
      initialNotified = false;
    }

    if (deviceConnected && !oldDeviceConnected) {
      long now = millis();
      Serial.println("Device connected");
      oldDeviceConnected = deviceConnected;
      initialNotified = false;
    }

    if (deviceConnected && !initialNotified) {
    float latestPm25, latestPm10;
    if (sps_read(&latestPm25, &latestPm10)) {
      if (xSemaphoreTake(xPayloadSemaphore, portMAX_DELAY)) {
        snprintf(blePayload, sizeof(blePayload), "%.2f#%.2f", latestPm25, latestPm10);
        pCharacteristic->setValue(blePayload);
        pCharacteristic->notify();
        Serial.print("Initial Notify on Connect: ");
        Serial.println(blePayload);
        lastSentPayload = String(blePayload);  // So it doesn’t re-send immediately after
        initialNotified = true;
        xSemaphoreGive(xPayloadSemaphore);
      }
    }
  }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Check every 1 second
  }
}

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();
  lv_disp_flush_ready(disp);
}

void SensorUITask(void *param) {
  float sumPm25 = 0;
  float sumPm10 = 0;
  int sampleCount = 0;

  unsigned long lastBeepTime = 0;
  bool buzzerOn = false;

  while (true) {
    if (sps_read(&pm25, &pm10)) {
      Serial.printf("PM2.5: %.2f | PM10: %.2f\n", pm25, pm10);

      // --- Buzzer Beep Logic (non-blocking, 500ms beep) ---
      static unsigned long buzzerStartTime = 0;
      static bool buzzerOn = false;
      static bool beepRequested = false;

      unsigned long currentMillis = millis();
      bool shouldAlert = (pm25 >= 55.0 || pm10 >= 75.0);

      if (shouldAlert && alertON) {
        // If buzzer is off and beep not requested yet, request beep now
        if (!buzzerOn && !beepRequested) {
          digitalWrite(BUZZER_PIN, HIGH);
          buzzerOn = true;
          beepRequested = true;
          buzzerStartTime = currentMillis;
        }
      } else {
        // No alert condition, turn buzzer off and reset flags
        digitalWrite(BUZZER_PIN, LOW);
        buzzerOn = false;
        beepRequested = false;
      }

      // If buzzer is on and 500 ms elapsed, turn buzzer off
      if (buzzerOn && (currentMillis - buzzerStartTime >= 500)) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerOn = false;
        // Allow next beep cycle if alert still true
        beepRequested = false;
      }

      // --- Update label ---
      static char pm25_buf[16], pm10_buf[16];
      snprintf(pm25_buf, sizeof(pm25_buf), "%.1f", pm25);
      snprintf(pm10_buf, sizeof(pm10_buf), "%.1f", pm10);
      lv_label_set_text(ui_pm25val, pm25_buf);
      lv_label_set_text(ui_pm10val, pm10_buf);
      
      int ispu25 = hitungISPU(pm25, ispuPM25, sizeof(ispuPM25) / sizeof(ISPUThreshold));
      int ispu10 = hitungISPU(pm10, ispuPM10, sizeof(ispuPM10) / sizeof(ISPUThreshold));

      int ispuFinal = max(ispu25, ispu10); // Gunakan ISPU tertinggi
      
      lv_label_set_text(ui_Status, kategoriISPU(ispuFinal));


      // --- Average for BLE transmission ---
      sumPm25 += pm25;
      sumPm10 += pm10;
      sampleCount++;

      if (sampleCount >= 12) { // 12 x 5s = 60s
        double avgPm25 = sumPm25 / sampleCount;
        double avgPm10 = sumPm10 / sampleCount;

        if (xSemaphoreTake(xPayloadSemaphore, portMAX_DELAY)) {
          snprintf(blePayload, sizeof(blePayload), "%.2f#%.2f", avgPm25, avgPm10);
          xSemaphoreGive(xPayloadSemaphore);
        }

        sumPm25 = 0;
        sumPm10 = 0;
        sampleCount = 0;
      }
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS); // Every 5 seconds
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);

  lv_init();
  tft.begin();
  tft.setRotation(1);
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  ui_init();

  sps_init();

  xPayloadSemaphore = xSemaphoreCreateMutex();

  // Create tasks
  xTaskCreatePinnedToCore(BLETask, "BLETask", 4096, NULL, 1, NULL, 0);   // Core 0
  xTaskCreatePinnedToCore(SensorUITask, "SensorTask", 8192, NULL, 1, NULL, 1); // Core 1
}

void loop() {
  lv_tick_inc(LVGL_TICK_PERIOD);
  lv_timer_handler();
  delay(LVGL_TICK_PERIOD);
}