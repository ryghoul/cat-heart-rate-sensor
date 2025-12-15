// INMP441 + ESP32 + ESP-NOW Sender
// SD=22, WS(LRCLK)=19, SCK(BCLK)=18, L/R strapped to GPIO13 (driven LOW => Left channel).

#include <Arduino.h>
#include "driver/i2s.h"
#include <WiFi.h>
#include <esp_now.h>

#define I2S_SAMPLE_RATE      16000
#define I2S_BITS_PER_SAMPLE  I2S_BITS_PER_SAMPLE_32BIT
#define I2S_CHANNELS         I2S_CHANNEL_FMT_ONLY_LEFT   // LEFT because we drive L/R LOW

const int I2S_BCK_PIN  = 18; // SCK/BCLK
const int I2S_WS_PIN   = 19; // WS/LRCLK
const int I2S_DATA_PIN = 22; // SD/DOUT
const int LR_PIN       = 13; // we'll use this as "ground" for L/R (logic LOW)

// === CHANGE THIS TO YOUR RECEIVER'S MAC ADDRESS ===
uint8_t receiverAddress[] = {0x4C, 0xC3, 0x82, 0xC3, 0x4F, 0xB0};

// Packet we send via ESP-NOW
typedef struct {
  float    rms;       // normalized RMS [-1,1)
  uint16_t samples;   // number of samples used to compute RMS
} mic_packet_t;

mic_packet_t micPacket;

// New-style send callback for ESP32 Arduino core v3+
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // --- Drive LR pin LOW so INMP441 uses LEFT channel ---
  pinMode(LR_PIN, OUTPUT);
  digitalWrite(LR_PIN, LOW);   // L/R tied LOW => left channel

  // --- WiFi + ESP-NOW init ---
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (true) { delay(1000); }
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
    while (true) { delay(1000); }
  }

  // --- I2S init ---
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE,
    .channel_format = I2S_CHANNELS,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 6,
    .dma_buf_len = 256,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_BCK_PIN,
    .ws_io_num    = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_DATA_PIN
  };

  i2s_driver_uninstall(I2S_NUM_0);  // in case it was already running
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_set_clk(I2S_NUM_0, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE, I2S_CHANNEL_MONO);

  Serial.println("I2S + ESP-NOW sender ready. Speak near the mic!");
}

void loop() {
  const int bufBytes = 1024;
  static uint8_t buf[bufBytes];
  size_t bytesRead = 0;

  // Block until we get data
  if (i2s_read(I2S_NUM_0, buf, bufBytes, &bytesRead, portMAX_DELAY) != ESP_OK || bytesRead == 0) {
    delay(10);
    return;
  }

  // Debug: show first few raw 32-bit words ~1x/s
  static uint32_t lastPrintRaw = 0;
  if (millis() - lastPrintRaw > 1000) {
    lastPrintRaw = millis();
    Serial.print("Raw: ");
    int words = min(8, (int)(bytesRead / 4));
    for (int i = 0; i < words; ++i) {
      uint32_t w = ((uint32_t*)buf)[i];
      Serial.printf("0x%08lX ", (unsigned long)w);
    }
    Serial.println();
  }

  // Convert 32-bit words to signed 24-bit, compute RMS
  const int samples = bytesRead / 4;
  long long sumSq = 0;

  for (int i = 0; i < samples; ++i) {
    int32_t v32 = ((int32_t*)buf)[i];
    int32_t s24 = v32 >> 8;    // 24-bit signed in upper bits
    long long s = s24;
    sumSq += s * s;
  }

  double meanSq = (double)sumSq / (double)samples;
  double rms = sqrt(meanSq) / (double)(1 << 23); // normalize to ~[-1,1)

  // Print locally for debugging
  Serial.printf("Samples: %d  RMS: %.6f\n", samples, rms);

  // --- Prepare packet and send via ESP-NOW ---
  micPacket.rms = (float)rms;
  micPacket.samples = (uint16_t)samples;  // fits easily

  esp_err_t result = esp_now_send(receiverAddress, (uint8_t*)&micPacket, sizeof(mic_packet_t));
  if (result != ESP_OK) {
    Serial.print("ESP-NOW send error: ");
    Serial.println(result);
  }

  // Throttle a bit so we don't absolutely spam
  delay(50);  // ~20 packets/sec
}
