// INMP441 + ESP32 with your pins:
// SD=22, WS(LRCLK)=19, SCK(BCLK)=18, L/R strapped (GND=Left, 3V3=Right).
// Prints RMS level; also prints a few raw words so you can verify bit alignment.

#include "driver/i2s.h"
#include <Arduino.h>

#define I2S_SAMPLE_RATE      16000
#define I2S_BITS_PER_SAMPLE  I2S_BITS_PER_SAMPLE_32BIT
#define I2S_CHANNELS         I2S_CHANNEL_FMT_ONLY_LEFT   // <- use ONLY_RIGHT if L/R is tied to 3V3

const int I2S_BCK_PIN  = 18; // SCK/BCLK
const int I2S_WS_PIN   = 19; // WS/LRCLK
const int I2S_DATA_PIN = 22; // SD/DOUT

void setup() {
  Serial.begin(115200);
  delay(200);

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

  i2s_driver_uninstall(I2S_NUM_0);
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  // Optional: set clock explicitly (sample rate, bits, and single-channel)
  i2s_set_clk(I2S_NUM_0, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE, I2S_CHANNEL_MONO);

  Serial.println("I2S ready. Speak near the mic; you should see RMS change.");
}

void loop() {
  const int bufBytes = 1024;
  static uint8_t buf[bufBytes];
  size_t bytesRead = 0;

  if (i2s_read(I2S_NUM_0, buf, bufBytes, &bytesRead, portMAX_DELAY) != ESP_OK || bytesRead == 0) {
    delay(10);
    return;
  }

  // Debug: show first few raw 32-bit words once per second-ish
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print("Raw: ");
    int words = min(8, (int)(bytesRead / 4));
    for (int i = 0; i < words; ++i) {
      uint32_t w = ((uint32_t*)buf)[i];
      Serial.printf("0x%08lX ", (unsigned long)w);
    }
    Serial.println();
  }

  // Convert 32-bit words to signed 24-bit (MSB-aligned), compute RMS
  const int samples = bytesRead / 4;
  long long sumSq = 0;
  for (int i = 0; i < samples; ++i) {
    int32_t v32 = ((int32_t*)buf)[i];
    int32_t s24 = v32 >> 8;          // 24-bit signed in upper bits
    long long s = s24;
    sumSq += s * s;
  }
  double meanSq = (double)sumSq / (double)samples;
  double rms = sqrt(meanSq) / (double)(1 << 23); // normalize to ~[-1,1)

  Serial.printf("Samples: %d  RMS: %.6f\n", samples, rms);
  delay(50);
}
