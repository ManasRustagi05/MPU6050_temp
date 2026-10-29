/*
  Tilt-Sand: ESP32 + MPU6050 + 4-in-1 MAX7219 LED matrix (32x8)
  Same sketch for both boards. Only change BOARD_ID below.
  Library: LedControl (by Eberhard Fahle) -> Library Manager

  - Tilt the board -> sand slides toward the lowered side
  - Tilt past TRANSFER_G toward an edge -> grains leave through that edge
    and appear on the other board (ESP-NOW, no router, no MAC setup)
  - Calibration at power-up (keep board FLAT and still)
    Recalibrate: press BOOT button (GPIO0) or send 'c' in Serial Monitor
*/

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <LedControl.h>

// ================= CONFIG =================
#define BOARD_ID 'A'          // 'A' = starts with sand, 'B' = starts empty

// Pins
const int CS_PIN  = 5;        // MAX7219 CS   (DIN=23, CLK=18 are hardware SPI)
const int SDA_PIN = 21;       // MPU6050 SDA
const int SCL_PIN = 22;       // MPU6050 SCL
const int BTN_PIN = 0;        // BOOT button -> recalibrate

const uint8_t MPU_ADDR = 0x68;   // 0x69 if AD0 pin is HIGH

// Orientation tuning (change these if sand moves the wrong way)
const bool SWAP_XY        = false;  // swap sensor X and Y
const bool INVERT_X       = false;  // flip left/right sand direction
const bool INVERT_Y       = false;  // flip up/down sand direction
const bool FLIP_DISPLAY_X = false;  // mirror display left/right
const bool FLIP_DISPLAY_Y = false;  // mirror display up/down

// Sand behaviour
const int   START_GRAINS = 64;      // board A starts with this many grains
const float DEADZONE     = 0.08f;   // ignore tiny tilts (in g)
const float TRANSFER_G   = 0.60f;   // tilt needed to push sand to the other board (~37 deg)
const uint32_t FRAME_MS  = 35;      // simulation step time
// ==========================================

constexpr int W = 32;
constexpr int H = 8;
constexpr int MAXG = W * H;

// LedControl instance: LedControl(dataPin, clkPin, csPin, numDevices)
// Using ESP32 hardware SPI pins: DIN = 23, CLK = 18, CS = 5, 4 devices cascaded
LedControl lc = LedControl(23, 18, CS_PIN, 4);

// ---------- Sand state ----------
struct Grain { uint8_t x, y; };
Grain grains[MAXG];
int   nGrains = 0;
bool  occ[H][W];

// ---------- Calibration ----------
float offX = 0, offY = 0;      // raw accel offsets (counts)
const float LSB_PER_G = 16384.0f;

// ---------- ESP-NOW ----------
const uint8_t MAGIC = 0xA5;
const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct __attribute__((packed)) Packet {
  uint8_t magic;
  uint8_t type;      // 0 = heartbeat, 1 = sand
  uint8_t count;     // grains in this packet (max 8)
  uint8_t entryX;    // column where grains enter on the receiving board
  uint8_t y[8];
};

struct Pend { uint8_t x, y; };
constexpr int PEND_MAX = 128;
Pend pend[PEND_MAX];
volatile int pendN = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t lastHeard = 0;
volatile bool heardOnce = false;

bool peerAlive() { return heardOnce && (millis() - lastHeard < 1500); }

void handlePacket(const uint8_t *data, int len) {
  if (len < (int)sizeof(Packet)) return;
  const Packet *p = (const Packet *)data;
  if (p->magic != MAGIC) return;
  lastHeard = millis();
  heardOnce = true;
  if (p->type == 1 && p->entryX < W) {
    portENTER_CRITICAL(&mux);
    for (int i = 0; i < p->count && i < 8; i++) {
      if (p->y[i] < H && pendN < PEND_MAX) {
        pend[pendN++] = {p->entryX, p->y[i]};
      }
    }
    portEXIT_CRITICAL(&mux);
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { handlePacket(data, len); }
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) { handlePacket(data, len); }
#endif

// ---------- MPU6050 (raw I2C, no library needed) ----------
bool mpuInit() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);          // wake up
  if (Wire.endTransmission() != 0) return false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(0x03);          // low-pass filter ~44 Hz
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);          // +-2g range
  Wire.endTransmission();
  return true;
}

bool mpuReadAccel(int16_t &ax, int16_t &ay) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 6) != 6) return false;
  uint8_t h, l;
  h = Wire.read(); l = Wire.read(); ax = (int16_t)((h << 8) | l);
  h = Wire.read(); l = Wire.read(); ay = (int16_t)((h << 8) | l);
  Wire.read(); Wire.read();                    // skip Z
  return true;
}

// ---------- Display helpers ----------
inline void clearAll() {
  for (int i = 0; i < 4; i++) {
    lc.clearDisplay(i);
  }
}

inline void setPix(int x, int y, bool on) {
  int col = FLIP_DISPLAY_X ? x : (W - 1 - x);
  int row = FLIP_DISPLAY_Y ? (H - 1 - y) : y;
  int addr = col / 8;
  int locX = col % 8;
  lc.setLed(addr, row, locX, on);
}

void showProgress(float f) {
  clearAll();
  int cols = (int)(f * W);
  for (int x = 0; x < cols; x++) { setPix(x, 3, true); setPix(x, 4, true); }
}

void flashAll(int times) {
  for (int t = 0; t < times; t++) {
    for (int x = 0; x < W; x++) for (int y = 0; y < H; y++) setPix(x, y, true);
    delay(120);
    clearAll();
    delay(120);
  }
}

// ---------- Calibration ----------
void calibrate() {
  const int N = 400;
  while (true) {
    Serial.println("Calibrating: keep the board flat and still...");
    clearAll();
    delay(1500);

    long sx = 0, sy = 0;
    int cnt = 0;
    int16_t minX = 32767, maxX = -32768, minY = 32767, maxY = -32768;

    for (int i = 0; i < N; i++) {
      int16_t ax, ay;
      if (mpuReadAccel(ax, ay)) {
        sx += ax; sy += ay; cnt++;
        if (ax < minX) minX = ax; if (ax > maxX) maxX = ax;
        if (ay < minY) minY = ay; if (ay > maxY) maxY = ay;
      }
      if (i % 8 == 0) showProgress((float)i / N);
      delay(5);
    }

    if (cnt < N / 2 || (maxX - minX) > 2500 || (maxY - minY) > 2500) {
      Serial.println("Board moved during calibration - retrying");
      continue;
    }
    offX = (float)sx / cnt;
    offY = (float)sy / cnt;
    Serial.printf("Calibration done. offX=%.1f offY=%.1f\n", offX, offY);
    flashAll(2);
    return;
  }
}

// ---------- Sand engine ----------
inline bool cellFree(int x, int y) {
  return x >= 0 && x < W && y >= 0 && y < H && !occ[y][x];
}

bool tryMove(Grain &g, int nx, int ny) {
  if (!cellFree(nx, ny)) return false;
  occ[g.y][g.x] = false;
  g.x = nx; g.y = ny;
  occ[ny][nx] = true;
  return true;
}

void addGrain(int x, int y) {
  grains[nGrains++] = {(uint8_t)x, (uint8_t)y};
  occ[y][x] = true;
}

void stepSand(float gx, float gy) {
  if (gx == 0 && gy == 0) return;
  float px = min(1.0f, fabsf(gx) * 1.8f);
  float py = min(1.0f, fabsf(gy) * 1.8f);
  int sx = gx > 0 ? 1 : -1;
  int sy = gy > 0 ? 1 : -1;

  static uint16_t order[MAXG];
  for (int i = 0; i < nGrains; i++) order[i] = i;
  for (int i = nGrains - 1; i > 0; i--) {
    int j = random(i + 1);
    uint16_t t = order[i]; order[i] = order[j]; order[j] = t;
  }

  for (int k = 0; k < nGrains; k++) {
    Grain &g = grains[order[k]];
    int x = g.x, y = g.y;
    int dx = (random(1000) < (int)(px * 1000)) ? sx : 0;
    int dy = (random(1000) < (int)(py * 1000)) ? sy : 0;
    if (!dx && !dy) continue;

    if (dx && dy) {
      if (tryMove(g, x + dx, y + dy)) continue;
      if (random(2)) { if (tryMove(g, x + dx, y)) continue; tryMove(g, x, y + dy); }
      else           { if (tryMove(g, x, y + dy)) continue; tryMove(g, x + dx, y); }
    } else if (dx) {
      if (tryMove(g, x + dx, y)) continue;
      int s = random(2) ? 1 : -1;           
      if (tryMove(g, x + dx, y + s)) continue;
      tryMove(g, x + dx, y - s);
    } else {
      if (tryMove(g, x, y + dy)) continue;
      int s = random(2) ? 1 : -1;
      if (tryMove(g, x + s, y + dy)) continue;
      tryMove(g, x - s, y + dy);
    }
  }
}

void transferPass(float gx) {
  if (!peerAlive()) return;                 
  int edgeX, entryX;
  if (gx > TRANSFER_G)       { edgeX = W - 1; entryX = 0; }
  else if (gx < -TRANSFER_G) { edgeX = 0;     entryX = W - 1; }
  else return;

  Packet p;
  memset(&p, 0, sizeof(p));
  p.magic = MAGIC; p.type = 1; p.entryX = entryX;

  for (int i = nGrains - 1; i >= 0 && p.count < 8; i--) {
    if (grains[i].x == edgeX) {
      p.y[p.count++] = grains[i].y;
      occ[grains[i].y][grains[i].x] = false;
      grains[i] = grains[nGrains - 1];
      nGrains--;
    }
  }
  if (p.count) esp_now_send(BCAST, (uint8_t *)&p, sizeof(p));
}

void drainIncoming() {
  portENTER_CRITICAL(&mux);
  int w = 0;
  for (int i = 0; i < pendN; i++) {
    Pend q = pend[i];
    int inward = (q.x == 0) ? 1 : -1;
    bool placed = false;
    for (int d = 0; d < 3 && !placed; d++) {
      int xx = q.x + inward * d;
      if (xx < 0 || xx >= W) break;
      if (!occ[q.y][xx] && nGrains < MAXG) { addGrain(xx, q.y); placed = true; }
    }
    if (!placed) pend[w++] = q;             
  }
  pendN = w;
  portEXIT_CRITICAL(&mux);
}

void drawSand() {
  clearAll();
  for (int i = 0; i < nGrains; i++) setPix(grains[i].x, grains[i].y, true);
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);
  randomSeed(esp_random());

  // Initialize all 4 devices in the LedControl chain
  for (int i = 0; i < 4; i++) {
    lc.shutdown(i, false);
    lc.setIntensity(i, 3);
    lc.clearDisplay(i);
  }

  Serial.printf("\n=== Tilt-Sand (LedControl) BOARD %c ===\n", BOARD_ID);

  if (!mpuInit()) {
    Serial.println("MPU6050 not found! Check SDA/SCL/VCC/GND.");
    while (true) {                           
      clearAll();
      for (int y = 0; y < H; y++) setPix(0, y, true);
      delay(400);
      clearAll();
      delay(400);
    }
  }

  calibrate();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("My MAC: "); Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, restarting");
    delay(1000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  if (BOARD_ID == 'A') {                     
    int placed = 0;
    for (int x = 12; x < 20 && placed < START_GRAINS; x++)
      for (int y = 0; y < H && placed < START_GRAINS; y++) { addGrain(x, y); placed++; }
  }
  drawSand();
}

void loop() {
  static uint32_t lastFrame = 0, lastHb = 0, lastPrint = 0;
  static float fgx = 0, fgy = 0;

  uint32_t now = millis();
  if (now - lastFrame < FRAME_MS) return;
  lastFrame = now;

  if (Serial.available() && Serial.read() == 'c') calibrate();
  if (digitalRead(BTN_PIN) == LOW) { delay(50); if (digitalRead(BTN_PIN) == LOW) calibrate(); }

  int16_t ax, ay;
  if (mpuReadAccel(ax, ay)) {
    float sx = (ax - offX) / LSB_PER_G;
    float sy = (ay - offY) / LSB_PER_G;
    if (SWAP_XY) { float t = sx; sx = sy; sy = t; }
    float gx = -sx, gy = sy;
    if (INVERT_X) gx = -gx;
    if (INVERT_Y) gy = -gy;
    fgx = fgx * 0.7f + gx * 0.3f;
    fgy = fgy * 0.7f + gy * 0.3f;
  }
  float gx = fabsf(fgx) < DEADZONE ? 0 : fgx;
  float gy = fabsf(fgy) < DEADZONE ? 0 : fgy;

  stepSand(gx, gy);
  transferPass(gx);
  drainIncoming();
  drawSand();

  if (now - lastHb > 400) {
    lastHb = now;
    Packet hb;
    memset(&hb, 0, sizeof(hb));
    hb.magic = MAGIC; hb.type = 0;
    esp_now_send(BCAST, (uint8_t *)&hb, sizeof(hb));
  }

  if (now - lastPrint > 500) {
    lastPrint = now;
    Serial.printf("gx=%.2f gy=%.2f grains=%d peer=%s\n", fgx, fgy, nGrains, peerAlive() ? "ON" : "off");
  }
}
