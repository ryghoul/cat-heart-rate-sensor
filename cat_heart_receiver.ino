// ESP-NOW Receiver + 1.54" TFT heart-rate style display
// Expects packets: struct { float rms; uint16_t samples; }

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// ---------- TFT PIN DEFINITIONS (adjust if wired differently) ----------
#define TFT_CS   5
#define TFT_DC   16
#define TFT_RST  17
// SCL -> GPIO 18 (hardware SCK)
// SDA -> GPIO 23 (hardware MOSI)

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ---------- ESP-NOW PACKET ----------
typedef struct {
  float    rms;       // normalized RMS [-1,1)
  uint16_t samples;   // number of samples used to compute RMS
} mic_packet_t;

// Shared between callback and loop
mic_packet_t latestPacket;
volatile bool newPacket = false;

// ---------- WAVEFORM BUFFER ----------
const int WAVE_POINTS = 120;  // number of points across the screen
float wave[WAVE_POINTS];
int waveIndex = 0;            // next position to write
int waveCount = 0;            // how many valid samples we have so far (<= WAVE_POINTS)
float lastRms = 0.0f;

// ---------- HEART RATE ESTIMATION ----------
const float THRESHOLD = 0.02f;          // adjust based on your signal level
const unsigned long REFRACTORY_MS = 300;  // minimum time between beats
const unsigned long WINDOW_MS = 8000;     // look at last 8s of beats
const int MAX_BEATS = 32;

unsigned long beatTimes[MAX_BEATS];
int beatWriteIndex = 0;
unsigned long lastBeatMs = 0;

// ---------- ESP-NOW RECEIVE CALLBACK (new style) ----------
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *incomingData,
                int len)
{
  if (len == sizeof(mic_packet_t)) {
    memcpy(&latestPacket, incomingData, sizeof(mic_packet_t));
    newPacket = true;
  }
}

// ---------- HEART RATE HELPERS ----------
void addBeat(unsigned long t) {
  beatTimes[beatWriteIndex] = t;
  beatWriteIndex = (beatWriteIndex + 1) % MAX_BEATS;
}

// Compute BPM from timestamps in last WINDOW_MS
int computeBpm(unsigned long now) {
  // Collect beats within the window
  unsigned long oldestAllowed = now - WINDOW_MS;
  unsigned long times[MAX_BEATS];
  int count = 0;

  // Copy in chronological order
  for (int i = 0; i < MAX_BEATS; ++i) {
    unsigned long ts = beatTimes[i];
    if (ts >= oldestAllowed && ts <= now && ts != 0) {
      times[count++] = ts;
    }
  }

  if (count < 2) {
    return 0;  // not enough beats to estimate
  }

  // Use first and last in that window
  unsigned long tFirst = times[0];
  unsigned long tLast  = times[count - 1];
  unsigned long dt = tLast - tFirst;  // ms

  if (dt == 0) return 0;

  float beats = (float)(count - 1);
  float minutes = (float)dt / 60000.0f;
  int bpm = (int)round(beats / minutes);
  return bpm;
}

// ---------- DRAWING ----------
void drawWaveform(float rms, int bpm) {
  // For speed we redraw the full frame; if it's slow, we can optimize later.
  uint16_t bgColor = ST77XX_BLACK;
  uint16_t lineColor = ST77XX_GREEN;
  uint16_t gridColor = 0x7BEF;
  uint16_t textColor = ST77XX_WHITE;

  tft.fillScreen(bgColor);

  int w = tft.width();
  int h = tft.height();

  // Top area: BPM text
  tft.setTextSize(2);
  tft.setTextColor(textColor, bgColor);
  tft.setCursor(10, 10);
  tft.print("BPM: ");
  if (bpm > 0 && bpm < 300) {
    tft.print(bpm);
  } else {
    tft.print("--");
  }

  // Show current RMS (for debugging / feedback)
  tft.setCursor(10, 35);
  tft.setTextSize(1);
  tft.print("RMS: ");
  tft.print(rms, 4);

  // Waveform area
  int graphTop = 60;
  int graphBottom = h - 10;
  int graphHeight = graphBottom - graphTop;

  // Draw a faint horizontal center line
  int midY = graphTop + graphHeight / 2;
  tft.drawLine(0, midY, w, midY, gridColor);

  // No waveform yet?
  if (waveCount < 2) return;

  // Find max RMS so we can auto-scale a bit (simple)
  float maxVal = 0.01f;  // avoid div by zero
  for (int i = 0; i < waveCount; ++i) {
    int idx = (waveIndex + i) % WAVE_POINTS;
    if (wave[idx] > maxVal) maxVal = wave[idx];
  }

  // Put a floor so the line doesn’t collapse if it’s quiet
  if (maxVal < 0.03f) maxVal = 0.03f;

  // Draw line from oldest -> newest across the width
  int pointsToDraw = waveCount;
  float xStep = (float)(w - 1) / (float)(pointsToDraw - 1);

  for (int i = 0; i < pointsToDraw - 1; ++i) {
    int idx1 = (waveIndex + i) % WAVE_POINTS;
    int idx2 = (waveIndex + i + 1) % WAVE_POINTS;
    float v1 = wave[idx1];
    float v2 = wave[idx2];

    // Map RMS [0, maxVal] -> [graphBottom, graphTop]
    auto mapY = [&](float v) {
      if (v < 0) v = 0;
      if (v > maxVal) v = maxVal;
      float norm = v / maxVal;  // 0..1
      int y = graphBottom - (int)(norm * (graphHeight - 1));
      return y;
    };

    int x1 = (int)round(i * xStep);
    int x2 = (int)round((i + 1) * xStep);
    int y1 = mapY(v1);
    int y2 = mapY(v2);

    tft.drawLine(x1, y1, x2, y2, lineColor);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // ----------- TFT INIT -----------
  tft.init(240, 240);     // If your screen is 240x240; adjust if different
  tft.setRotation(0);     // Rotate if you want landscape
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setCursor(10, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.println("Waiting for data...");

  // Clear beat timestamps
  for (int i = 0; i < MAX_BEATS; ++i) beatTimes[i] = 0;

  // ----------- ESP-NOW INIT -----------
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    tft.setCursor(10, 40);
    tft.setTextSize(1);
    tft.println("ESP-NOW init failed");
    while (true) { delay(1000); }
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP-NOW receiver + TFT ready.");
}

void loop() {
  if (newPacket) {
    noInterrupts();
    mic_packet_t pkt = latestPacket;
    newPacket = false;
    interrupts();

    float rms = pkt.rms;
    unsigned long now = millis();

    // ----------- HEARTBEAT DETECTION (simple threshold crossing) -----------
    // Detect rising edge across threshold (and respect refractory period)
    if (lastRms < THRESHOLD && rms >= THRESHOLD) {
      if (now - lastBeatMs > REFRACTORY_MS) {
        lastBeatMs = now;
        addBeat(now);
      }
    }
    lastRms = rms;

    int bpm = computeBpm(now);

    // ----------- UPDATE WAVE BUFFER -----------
    wave[waveIndex] = rms;
    waveIndex = (waveIndex + 1) % WAVE_POINTS;
    if (waveCount < WAVE_POINTS) waveCount++;

    // ----------- DRAW TFT -----------
    drawWaveform(rms, bpm);

    // For debugging to Serial too
    Serial.print("RMS: ");
    Serial.print(rms, 4);
    Serial.print("  BPM: ");
    Serial.println(bpm);
  }

  delay(5);
}


