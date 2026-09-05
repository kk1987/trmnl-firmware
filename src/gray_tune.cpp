// Standalone gray-scale tuner for the Seeed reTerminal Sticky (3.97" 800x480
// EP397 panel on an SSD1677 controller). Built only with -D GRAY_TUNE_MODE.
//
// The panel's two middle 4-gray levels come out too light with the stock
// firmware init (OTP waveform picked by temperature register 0x1A = 0x5A).
// This build draws a calibration frame -- four solid swatches, two dither
// ramps, a strip of real rendered text -- and lets the three front buttons
// walk through the waveform knobs: the OTP slot / display-update byte /
// voltage overrides, or a fully parameterised custom 105-byte LUT. Every
// change refreshes the panel and prints the active parameter set on the
// panel and over serial, so one flash covers the whole search.
//
// Buttons (active low): UP = GPIO5, DOWN = GPIO6, OK = GPIO4 (the power key).
//   UP / DOWN   step the selected parameter and refresh
//   OK          select the next parameter
//   long OK     next preset
//   long UP     sweep the selected parameter over its range (any key aborts)
//   long DOWN   reset the selected parameter to the preset's default
// The same operations exist as serial commands; type `help`.
//
// The panel is driven directly through bb_epaper's raw command API so the
// sequence is exactly what the parameters say: hardware reset, base init,
// engine-specific waveform setup, both RAM planes, 0x22/0x20, busy-wait.

#ifdef GRAY_TUNE_MODE

#include <Arduino.h>
#include <bb_epaper.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <gray_tune.h>
#include <gray_tune_assets.h>

extern BBEPAPER bbep; // display.cpp

namespace {

constexpr int kW = 800;
constexpr int kH = 480;
constexpr int kPlaneBytes = kW * kH / 8; // 48000, one 1-bpp plane
constexpr int kPinOk = 4, kPinUp = 5, kPinDown = 6, kPinBusy = 18;
constexpr int kPinDc = 16, kPinRst = 17, kPinCs = 15, kPinMosi = 14, kPinSck = 13;
constexpr uint32_t kBusyTimeoutMs = 30000;
constexpr uint32_t kLongPressMs = 1500;
constexpr uint32_t kDebounceMs = 30;
constexpr uint32_t kSweepDwellMs = 3000;

// ---------------------------------------------------------------- tables --

const char *const kPresetNames[] = {"bbep", "seeed", "bb2", "ws426"};
const char *const kEngineNames[] = {"OTP", "LUT"};
const char *const kBorderNames[] = {"00", "01", "80", "C0"};
const uint8_t kBorderVals[] = {0x00, 0x01, 0x80, 0xC0};
const char *const kUNames[] = {"D7", "DF", "C7", "CF", "F7"};
const uint8_t kUVals[] = {0xD7, 0xDF, 0xC7, 0xCF, 0xF7};
const char *const kLutBaseNames[] = {"bb2", "ws426"};
const char *const kLutLenNames[] = {"90", "105"};
const char *const kVsNames[] = {"VSS", "VSH1", "VSL", "VSH2"};
const char *const kYdirNames[] = {"dec", "inc"};
const char *const kOnOff[] = {"0", "1"};
// Which RAM code is black. The panel's OTP 4-gray waveforms (and upstream's
// custom LUT, written for bb_epaper's inverted palette) drive 11 = black,
// 01 = dark, 10 = light, 00 = white; Waveshare's LUT is the other way round.
const char *const kPolNames[] = {"norm", "inv"};

// Upstream bb_epaper a4bbf189 EP397 4-gray LUT ("custom LUT to get better
// grays than the built-in LUT"). It writes only 90 bytes; bytes 90..104 are
// padded here and only sent when len = 105.
const uint8_t kLutBb2[105] = {
    0x9a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // LUT0 (00) black
    0x99, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // LUT1 (01)
    0x96, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // LUT2 (10)
    0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // LUT3 (11) white
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // LUT4 VCOM
    0x0a, 0x0a, 0x07, 0x02, 0x00, // G0 TP A,B,C,D RP
    0x06, 0x01, 0x04, 0x00, 0x00, // G1
    0x00, 0x00, 0x00, 0x00, 0x00, // G2
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // G3, G4
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // G5, G6
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // G7, G8
    0x00, 0x00, 0x00, 0x00, 0x00, // G9
    0x00, 0x00, 0x00, 0x00, 0x00, // FR (not written upstream)
};

// Waveshare 4.26" (SSD1677, 800x480) LUT_DATA_4Gray, first 105 bytes.
const uint8_t kLutWs426[105] = {
    0x80, 0x48, 0x4A, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // LUT0 (00) black
    0x0A, 0x48, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // LUT1 (01) dark gray
    0x88, 0x48, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // LUT2 (10) light gray
    0xA8, 0x48, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // LUT3 (11) white
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // LUT4 VCOM
    0x07, 0x1E, 0x1C, 0x02, 0x00, // G0
    0x05, 0x01, 0x05, 0x01, 0x02, // G1
    0x08, 0x01, 0x01, 0x04, 0x04, // G2
    0x00, 0x02, 0x00, 0x02, 0x01, // G3
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // G4, G5
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // G6, G7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, // G8, G9
    0x22, 0x22, 0x22, 0x22, 0x22, // FR
};
// Waveshare's voltage tail: VGH 20 V, VSH1 15 V, VSH2 5 V, VSL -15 V, VCOM -1.2 V.
const uint8_t kWs426Volt[5] = {0x17, 0x41, 0xA8, 0x32, 0x30};

const uint8_t kIdentityLookup[16] = {0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};
const uint8_t kEmptyInit[] = {0};

const uint8_t kBayer4[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};

// ---------------------------------------------------------- parameters ----

struct Tune {
  // common
  int preset, engine, pol, swap, border, U, dbl, ydir;
  // OTP engine
  int T, split, ovVGH, ovVSH1, ovVSH2, ovVSL, ovVCOM;
  // LUT engine
  int base, len, lutEnd, volt, vgh, vsh1, vsh2, vsl, vcom, fr;
  int tp[3][4];
  int rp[3];
  int vs1[4]; // LUT1 phase voltages in the gray-defining group
  int vs2[4]; // LUT2 phase voltages in the gray-defining group
};

Tune t;
Tune tDefault; // values at the time the preset was applied

enum Kind { KIND_INT, KIND_HEX, KIND_ENUM, KIND_TEMP, KIND_VGH, KIND_VSH, KIND_VSL, KIND_VCOM };
enum Group { GRP_COMMON = 0, GRP_OTP = 1, GRP_LUT = 2 };

struct Param {
  const char *name;
  int *value;
  int lo, hi, step;
  Kind kind;
  const char *const *names; // KIND_ENUM
  bool allowOff;            // -1 = "off"
  uint8_t group;
};

#define P_ENUM(n, v, cnt, nm, g) {n, &v, 0, (cnt) - 1, 1, KIND_ENUM, nm, false, g}
#define P_INT(n, v, lo, hi, g) {n, &v, lo, hi, 1, KIND_INT, nullptr, false, g}

const Param kParams[] = {
    P_ENUM("preset", t.preset, 4, kPresetNames, GRP_COMMON),
    P_ENUM("engine", t.engine, 2, kEngineNames, GRP_COMMON),
    P_ENUM("pol", t.pol, 2, kPolNames, GRP_COMMON),
    P_ENUM("swap", t.swap, 2, kOnOff, GRP_COMMON),
    P_ENUM("border", t.border, 4, kBorderNames, GRP_COMMON),
    P_ENUM("U", t.U, 5, kUNames, GRP_COMMON),
    P_ENUM("dbl", t.dbl, 2, kOnOff, GRP_COMMON),
    P_ENUM("ydir", t.ydir, 2, kYdirNames, GRP_COMMON),
    // OTP
    {"T", &t.T, 0x40, 0x7F, 1, KIND_TEMP, nullptr, false, GRP_OTP},
    P_ENUM("split", t.split, 2, kOnOff, GRP_OTP),
    {"oVGH", &t.ovVGH, 0x07, 0x17, 1, KIND_VGH, nullptr, true, GRP_OTP},
    {"oVSH1", &t.ovVSH1, 0x23, 0x4B, 1, KIND_VSH, nullptr, true, GRP_OTP},
    {"oVSH2", &t.ovVSH2, 0x8E, 0xC3, 1, KIND_VSH, nullptr, true, GRP_OTP},
    {"oVSL", &t.ovVSL, 0x1A, 0x3A, 2, KIND_VSL, nullptr, true, GRP_OTP},
    {"oVCOM", &t.ovVCOM, 0x08, 0xA0, 4, KIND_VCOM, nullptr, true, GRP_OTP},
    // LUT
    P_ENUM("base", t.base, 2, kLutBaseNames, GRP_LUT),
    P_ENUM("len", t.len, 2, kLutLenNames, GRP_LUT),
    P_ENUM("lutend", t.lutEnd, 2, kOnOff, GRP_LUT),
    P_ENUM("volt", t.volt, 2, kOnOff, GRP_LUT),
    {"VGH", &t.vgh, 0x07, 0x17, 1, KIND_VGH, nullptr, false, GRP_LUT},
    {"VSH1", &t.vsh1, 0x23, 0x4B, 1, KIND_VSH, nullptr, false, GRP_LUT},
    {"VSH2", &t.vsh2, 0x8E, 0xC3, 1, KIND_VSH, nullptr, false, GRP_LUT},
    {"VSL", &t.vsl, 0x1A, 0x3A, 2, KIND_VSL, nullptr, false, GRP_LUT},
    {"VCOM", &t.vcom, 0x08, 0xA0, 4, KIND_VCOM, nullptr, false, GRP_LUT},
    P_ENUM("L1.A", t.vs1[0], 4, kVsNames, GRP_LUT),
    P_ENUM("L1.B", t.vs1[1], 4, kVsNames, GRP_LUT),
    P_ENUM("L1.C", t.vs1[2], 4, kVsNames, GRP_LUT),
    P_ENUM("L1.D", t.vs1[3], 4, kVsNames, GRP_LUT),
    P_ENUM("L2.A", t.vs2[0], 4, kVsNames, GRP_LUT),
    P_ENUM("L2.B", t.vs2[1], 4, kVsNames, GRP_LUT),
    P_ENUM("L2.C", t.vs2[2], 4, kVsNames, GRP_LUT),
    P_ENUM("L2.D", t.vs2[3], 4, kVsNames, GRP_LUT),
    P_INT("G0.A", t.tp[0][0], 0, 255, GRP_LUT),
    P_INT("G0.B", t.tp[0][1], 0, 255, GRP_LUT),
    P_INT("G0.C", t.tp[0][2], 0, 255, GRP_LUT),
    P_INT("G0.D", t.tp[0][3], 0, 255, GRP_LUT),
    P_INT("RP0", t.rp[0], 0, 255, GRP_LUT),
    P_INT("G1.A", t.tp[1][0], 0, 255, GRP_LUT),
    P_INT("G1.B", t.tp[1][1], 0, 255, GRP_LUT),
    P_INT("G1.C", t.tp[1][2], 0, 255, GRP_LUT),
    P_INT("G1.D", t.tp[1][3], 0, 255, GRP_LUT),
    P_INT("RP1", t.rp[1], 0, 255, GRP_LUT),
    P_INT("G2.A", t.tp[2][0], 0, 255, GRP_LUT),
    P_INT("G2.B", t.tp[2][1], 0, 255, GRP_LUT),
    P_INT("G2.C", t.tp[2][2], 0, 255, GRP_LUT),
    P_INT("G2.D", t.tp[2][3], 0, 255, GRP_LUT),
    P_INT("RP2", t.rp[2], 0, 255, GRP_LUT),
    {"FR", &t.fr, 0x00, 0xFF, 1, KIND_HEX, nullptr, false, GRP_LUT},
};
constexpr int kParamCount = sizeof(kParams) / sizeof(kParams[0]);

int selected = 0;         // index into kParams
uint32_t lastRefreshMs = 0;
bool lastRefreshOk = true;
int refreshCount = 0;
bool needRefresh = true;
bool abortSweep = false;

// ------------------------------------------------------------ formatting --

float vghVolts(int c) { return c == 0 ? 20.0f : 12.0f + (c - 7) * 0.5f; }
float vshVolts(int c) { return (c & 0x80) ? 2.4f + (c - 0x8E) * 0.1f : 9.0f + (c - 0x23) * 0.2f; }
float vslVolts(int c) { return -9.0f - (c - 0x1A) * 0.25f; }
float vcomVolts(int c) { return -c * 0.025f; }

// Short value for the on-panel list.
void fmtShort(const Param &p, char *buf, size_t n)
{
  int v = *p.value;
  if (p.allowOff && v < 0) { snprintf(buf, n, "off"); return; }
  switch (p.kind) {
    case KIND_INT: snprintf(buf, n, "%d", v); break;
    case KIND_HEX: snprintf(buf, n, "%02X", v); break;
    case KIND_ENUM: snprintf(buf, n, "%s", p.names[v]); break;
    case KIND_TEMP: snprintf(buf, n, "%02X", v); break;
    case KIND_VGH: snprintf(buf, n, "%02X", v); break;
    case KIND_VSH: snprintf(buf, n, "%02X", v); break;
    case KIND_VSL: snprintf(buf, n, "%02X", v); break;
    case KIND_VCOM: snprintf(buf, n, "%02X", v); break;
  }
}

// Long value for the headline and serial: adds the decoded physical meaning.
void fmtLong(const Param &p, char *buf, size_t n)
{
  int v = *p.value;
  if (p.allowOff && v < 0) { snprintf(buf, n, "off"); return; }
  switch (p.kind) {
    case KIND_TEMP: snprintf(buf, n, "%02X (%d C)", v, v); break;
    case KIND_VGH: snprintf(buf, n, "%02X (%.1f V)", v, vghVolts(v)); break;
    case KIND_VSH: snprintf(buf, n, "%02X (%.1f V)", v, vshVolts(v)); break;
    case KIND_VSL: snprintf(buf, n, "%02X (%.2f V)", v, vslVolts(v)); break;
    case KIND_VCOM: snprintf(buf, n, "%02X (%.2f V)", v, vcomVolts(v)); break;
    default: fmtShort(p, buf, n); break;
  }
}

bool visible(int i)
{
  const Param &p = kParams[i];
  return p.group == GRP_COMMON || p.group == (t.engine == 0 ? GRP_OTP : GRP_LUT);
}

int nextVisible(int from, int dir)
{
  int i = from;
  for (int k = 0; k < kParamCount; k++) {
    i = (i + dir + kParamCount) % kParamCount;
    if (visible(i)) return i;
  }
  return from;
}

int findParam(const char *name)
{
  for (int i = 0; i < kParamCount; i++) {
    if (strcasecmp(kParams[i].name, name) == 0) return i;
  }
  return -1;
}

// ---------------------------------------------------------------- presets --

void loadLutBase(void)
{
  const uint8_t *b = t.base == 0 ? kLutBb2 : kLutWs426;
  const uint8_t *v = kWs426Volt; // sensible voltages for either base
  for (int g = 0; g < 3; g++) {
    for (int k = 0; k < 4; k++) t.tp[g][k] = b[50 + g * 5 + k];
    t.rp[g] = b[50 + g * 5 + 4];
  }
  int gg = t.base == 0 ? 1 : 2; // the group that decides the grays
  uint8_t a = b[10 + gg], c = b[20 + gg];
  for (int k = 0; k < 4; k++) {
    t.vs1[k] = (a >> (6 - 2 * k)) & 3;
    t.vs2[k] = (c >> (6 - 2 * k)) & 3;
  }
  t.fr = b[100] ? b[100] : 0x22;
  t.vgh = v[0]; t.vsh1 = v[1]; t.vsh2 = v[2]; t.vsl = v[3]; t.vcom = v[4];
}

void applyPreset(int n)
{
  t.preset = n;
  t.dbl = 0; t.ydir = 0; t.split = 0;
  t.ovVGH = t.ovVSH1 = t.ovVSH2 = t.ovVSL = t.ovVCOM = -1;
  switch (n) {
    case 0: // bb_epaper 7df2bd1 as shipped: OTP slot 0x5A, 0x22 = D7, mid-levels swapped in png_draw
      t.engine = 0; t.pol = 1; t.swap = 0; t.border = 1; t.U = 0; t.T = 0x5A;
      t.base = 0; t.len = 0; t.lutEnd = 1; t.volt = 0; loadLutBase();
      break;
    case 1: // Seeed vendor driver: OTP slot 0x67 (103 C), 0x22 = D7, border 00
      t.engine = 0; t.pol = 1; t.swap = 0; t.border = 0; t.U = 0; t.T = 0x67;
      t.base = 0; t.len = 0; t.lutEnd = 1; t.volt = 0; loadLutBase();
      break;
    case 2: // upstream bb_epaper a4bbf189 custom LUT, 90 bytes, LUT END 07, still D7
      t.engine = 1; t.pol = 1; t.swap = 0; t.border = 1; t.U = 0; t.T = 0x67;
      t.base = 0; t.len = 0; t.lutEnd = 1; t.volt = 0; loadLutBase();
      break;
    default: // Waveshare 4.26" 4-gray: 105-byte LUT + voltages, 0x22 = C7
      t.engine = 1; t.pol = 0; t.swap = 0; t.border = 1; t.U = 2; t.T = 0x67;
      t.base = 1; t.len = 1; t.lutEnd = 0; t.volt = 1; loadLutBase();
      break;
  }
  tDefault = t;
}

// Build the 105-byte LUT from the base table plus the edited cells.
void buildLut(uint8_t out[105])
{
  memcpy(out, t.base == 0 ? kLutBb2 : kLutWs426, 105);
  for (int g = 0; g < 3; g++) {
    for (int k = 0; k < 4; k++) out[50 + g * 5 + k] = (uint8_t)t.tp[g][k];
    out[50 + g * 5 + 4] = (uint8_t)t.rp[g];
  }
  int gg = t.base == 0 ? 1 : 2;
  out[10 + gg] = (uint8_t)((t.vs1[0] << 6) | (t.vs1[1] << 4) | (t.vs1[2] << 2) | t.vs1[3]);
  out[20 + gg] = (uint8_t)((t.vs2[0] << 6) | (t.vs2[1] << 4) | (t.vs2[2] << 2) | t.vs2[3]);
  for (int k = 0; k < 5; k++) out[100 + k] = (uint8_t)t.fr;
}

// --------------------------------------------------------------- panel IO --

void cmdData(uint8_t cmd, const uint8_t *data, int len)
{
  bbep.writeCmd(cmd);
  bbep.writeData((uint8_t *)data, len);
}

bool waitBusy(uint32_t timeoutMs = kBusyTimeoutMs)
{
  uint32_t t0 = millis();
  delay(20); // let BUSY rise after 0x20 before polling it
  while (digitalRead(kPinBusy) == HIGH) {
    if (millis() - t0 > timeoutMs) {
      Serial.printf("GT: BUSY timeout after %lu ms\n", (unsigned long)timeoutMs);
      return false;
    }
    delay(5);
  }
  return true;
}

void writeVoltages(int vgh, int vsh1, int vsh2, int vsl, int vcom)
{
  if (vgh >= 0) bbep.writeCmd2(0x03, (uint8_t)vgh);
  if (vsh1 >= 0 || vsh2 >= 0 || vsl >= 0) {
    // 0x04 takes all three; fill unset ones with the POR values
    uint8_t v[3] = {(uint8_t)(vsh1 >= 0 ? vsh1 : 0x41), (uint8_t)(vsh2 >= 0 ? vsh2 : 0xA8),
                    (uint8_t)(vsl >= 0 ? vsl : 0x32)};
    cmdData(0x04, v, 3);
  }
  if (vcom >= 0) bbep.writeCmd2(0x2C, (uint8_t)vcom);
}

void setRamCounters(void)
{
  uint8_t x[2] = {0x00, 0x00};
  uint8_t y[2] = {0x00, 0x00};
  cmdData(0x4E, x, 2);
  cmdData(0x4F, y, 2);
}

// Base init, the epd397g_init_full sequence minus its 0x1A write.
void panelInit(void)
{
  bbep.wake(); // hardware reset pulse + busy wait
  bbep.writeCmd(0x12); // soft reset
  waitBusy();
  bbep.writeCmd2(0x18, 0x80); // internal temperature sensor
  const uint8_t softStart[5] = {0xAE, 0xC7, 0xC3, 0xC0, 0x80};
  cmdData(0x0C, softStart, 5);
  const uint8_t driverOut[3] = {0xDF, 0x01, 0x02}; // 480 gates
  cmdData(0x01, driverOut, 3);
  bbep.writeCmd2(0x3C, kBorderVals[t.border]);
  if (t.ydir == 0) { // as bb_epaper 7df2bd1: Y decrement from 0x1DF
    bbep.writeCmd2(0x11, 0x01);
    const uint8_t rx[4] = {0x00, 0x00, 0x1F, 0x03};
    const uint8_t ry[4] = {0xDF, 0x01, 0x00, 0x00};
    cmdData(0x44, rx, 4);
    cmdData(0x45, ry, 4);
  } else { // Y increment from 0
    bbep.writeCmd2(0x11, 0x03);
    const uint8_t rx[4] = {0x00, 0x00, 0x1F, 0x03};
    const uint8_t ry[4] = {0x00, 0x00, 0xDF, 0x01};
    cmdData(0x44, rx, 4);
    cmdData(0x45, ry, 4);
  }
  setRamCounters();
  waitBusy();
}

void writePlanes(void)
{
  uint8_t *p0 = (uint8_t *)bbep.getBuffer();
  uint8_t *p1 = p0 + kPlaneBytes;
  // The buffer holds level = code (00 black .. 11 white, bit0 in plane 0).
  // pol=inv wants 11 = black / 01 = dark / 10 = light / 00 = white, which is
  // "invert every bit, then exchange the planes"; swap exchanges the planes
  // once more so the two middle levels trade places.
  bool invert = t.pol == 1;
  bool exchange = (t.pol == 1) != (t.swap == 1);
  if (invert) {
    for (int i = 0; i < 2 * kPlaneBytes; i++) p0[i] ^= 0xFF;
  }
  uint8_t *toRam = exchange ? p1 : p0;
  uint8_t *toAlt = exchange ? p0 : p1;
  setRamCounters();
  bbep.startWrite(PLANE_0); // 0x24
  bbep.writeData(toRam, kPlaneBytes);
  setRamCounters();
  bbep.startWrite(PLANE_1); // 0x26
  bbep.writeData(toAlt, kPlaneBytes);
  if (invert) {
    for (int i = 0; i < 2 * kPlaneBytes; i++) p0[i] ^= 0xFF; // restore the buffer
  }
}

void refreshPanel(void)
{
  uint32_t t0 = millis();
  panelInit();
  uint8_t disp = kUVals[t.U];
  if (t.engine == 0) {
    uint8_t temp[2] = {(uint8_t)t.T, 0x00};
    cmdData(0x1A, temp, 2);
    if (t.split) {
      // load the OTP waveform for that temperature on its own, then the
      // voltage overrides can no longer be clobbered by the load
      bbep.writeCmd2(0x22, 0x91);
      bbep.writeCmd(0x20);
      waitBusy();
      disp = 0xC7;
    }
    writeVoltages(t.ovVGH, t.ovVSH1, t.ovVSH2, t.ovVSL, t.ovVCOM);
  } else {
    uint8_t lut[105];
    buildLut(lut);
    cmdData(0x32, lut, t.len == 0 ? 90 : 105);
    if (t.lutEnd) bbep.writeCmd2(0x3F, 0x07);
    if (t.volt) writeVoltages(t.vgh, t.vsh1, t.vsh2, t.vsl, t.vcom);
  }
  writePlanes();
  bbep.writeCmd2(0x22, disp);
  bbep.writeCmd(0x20);
  bool ok = waitBusy();
  if (t.dbl) {
    writePlanes();
    bbep.writeCmd2(0x22, disp);
    bbep.writeCmd(0x20);
    ok = waitBusy() && ok;
  }
  lastRefreshMs = millis() - t0;
  lastRefreshOk = ok;
  refreshCount++;
}

// ---------------------------------------------------------------- drawing --

// Levels 0..3 = black..white; with the identity lookup the buffer code is the level.
void blitStrip(const uint8_t *strip, int y0)
{
  for (int y = 0; y < GT_STRIP_H; y++) {
    const uint8_t *row = strip + y * (GT_STRIP_W / 4);
    for (int x = 0; x < GT_STRIP_W; x++) {
      int level = (row[x >> 2] >> (6 - 2 * (x & 3))) & 3;
      bbep.drawPixel(x, y0 + y, level);
    }
  }
}

void drawRamp(int y0, int h, int onLevel, int offLevel)
{
  const int cells = 6;
  const int cw = kW / cells;
  char label[8];
  for (int c = 0; c < cells; c++) {
    int pct = c * 20;
    int thresh = (pct * 16 + 50) / 100; // 0..16 of the 16 Bayer slots are "on"
    int x0 = c * cw + 2;
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < cw - 4; x++) {
        int on = kBayer4[y & 3][x & 3] < thresh;
        bbep.drawPixel(x0 + x, y0 + y, on ? onLevel : offLevel);
      }
    }
    bbep.drawRect(x0, y0, cw - 4, h, 0);
    snprintf(label, sizeof(label), "%d%%", pct);
    bbep.setFont(FONT_8x8);
    bbep.setTextColor(0, 3);
    bbep.drawString(label, x0 + 2, y0 + h + 1);
  }
}

void drawParams(int yTop)
{
  char val[40], line[120];
  const Param &sel = kParams[selected];
  fmtLong(sel, val, sizeof(val));
  snprintf(line, sizeof(line), "> %s = %s   [%s/%s] #%d %lums%s", sel.name, val,
           kPresetNames[t.preset], kEngineNames[t.engine], refreshCount,
           (unsigned long)lastRefreshMs, lastRefreshOk ? "" : " BUSY-TIMEOUT");
  bbep.setFont(FONT_12x16);
  bbep.setTextColor(0, 3);
  bbep.drawString(line, 4, yTop);

  // the visible parameters, 5 columns x 10 rows in the small font
  bbep.setFont(FONT_8x8);
  const int cols = 5, rows = 10, colW = kW / cols;
  int n = 0;
  for (int i = 0; i < kParamCount; i++) {
    if (!visible(i)) continue;
    fmtShort(kParams[i], val, sizeof(val));
    snprintf(line, sizeof(line), "%c%s=%s", i == selected ? '>' : ' ', kParams[i].name, val);
    int col = n / rows, row = n % rows;
    if (col >= cols) break;
    bbep.drawString(line, col * colW + 4, yTop + 20 + row * 8);
    n++;
  }
}

void drawFrame(void)
{
  bbep.fillScreen(3); // white
  // solid swatches
  const char *const names[4] = {"L0 black", "L1 dark", "L2 light", "L3 white"};
  bbep.setFont(FONT_12x16);
  for (int i = 0; i < 4; i++) {
    bbep.fillRect(i * 200, 0, 200, 96, i);
    bbep.setTextColor(i < 2 ? 3 : 0, i);
    bbep.drawString(names[i], i * 200 + 8, 74);
  }
  bbep.drawRect(600, 0, 200, 96, 0);
  // dither ramps: L0 over L1 (the zebra test) and L0 over white
  drawRamp(98, 38, 0, 1);
  drawRamp(148, 38, 0, 3);
  // rendered text from the server, canvas tones then present-pass tones
  blitStrip(gt_strip_canvas, 198);
  blitStrip(gt_strip_present, 288);
  bbep.fillRect(0, 287, kW, 1, 0);
  drawParams(380);
}

// ---------------------------------------------------------------- logging --

void logParams(const char *why)
{
  char val[40];
  Serial.printf("GT[%s] #%d %lu ms%s:", why, refreshCount, (unsigned long)lastRefreshMs,
                lastRefreshOk ? "" : " BUSY-TIMEOUT");
  for (int i = 0; i < kParamCount; i++) {
    if (!visible(i)) continue;
    fmtLong(kParams[i], val, sizeof(val));
    Serial.printf(" %s=%s", kParams[i].name, val);
  }
  Serial.println();
  if (t.engine == 1) {
    uint8_t lut[105];
    buildLut(lut);
    Serial.print("GT lut:");
    int n = t.len == 0 ? 90 : 105;
    for (int i = 0; i < n; i++) Serial.printf("%s%02x", (i % 10 == 0) ? " | " : " ", lut[i]);
    Serial.println();
  }
}

// ----------------------------------------------------------------- edits ---

void onChanged(int idx)
{
  const Param &p = kParams[idx];
  if (p.value == &t.preset) {
    applyPreset(t.preset);
  } else if (p.value == &t.base) {
    loadLutBase();
  }
  needRefresh = true;
}

void stepParam(int idx, int dir)
{
  const Param &p = kParams[idx];
  int v = *p.value;
  if (p.allowOff && v < 0) {
    if (dir > 0) v = p.lo;
    else return;
  } else {
    v += dir * p.step;
    if (v > p.hi) v = p.hi;
    if (v < p.lo) v = p.allowOff ? -1 : p.lo;
  }
  if (v == *p.value) return;
  *p.value = v;
  onChanged(idx);
}

void setParam(int idx, int v)
{
  const Param &p = kParams[idx];
  if (p.allowOff && v < 0) v = -1;
  else if (v < p.lo) v = p.lo;
  else if (v > p.hi) v = p.hi;
  *p.value = v;
  onChanged(idx);
}

void resetParam(int idx)
{
  const Param &p = kParams[idx];
  ptrdiff_t off = (char *)p.value - (char *)&t;
  int def = *(int *)((char *)&tDefault + off);
  if (def != *p.value) {
    *p.value = def;
    onChanged(idx);
  }
}

// -------------------------------------------------------------- buttons ----

struct Button {
  int pin;
  bool pressed;
  bool longFired;
  uint32_t changedAt;
  uint32_t pressedAt;
  int rawLast;
};

Button btnOk = {kPinOk, false, false, 0, 0, HIGH};
Button btnUp = {kPinUp, false, false, 0, 0, HIGH};
Button btnDown = {kPinDown, false, false, 0, 0, HIGH};

enum Event { EV_NONE, EV_SHORT, EV_LONG };

Event pollButton(Button &b)
{
  int raw = digitalRead(b.pin);
  uint32_t now = millis();
  if (raw != b.rawLast) {
    b.rawLast = raw;
    b.changedAt = now;
    return EV_NONE;
  }
  if (now - b.changedAt < kDebounceMs) return EV_NONE;
  bool down = raw == LOW;
  if (down && !b.pressed) {
    b.pressed = true;
    b.longFired = false;
    b.pressedAt = now;
    return EV_NONE;
  }
  if (down && b.pressed && !b.longFired && now - b.pressedAt >= kLongPressMs) {
    b.longFired = true;
    return EV_LONG;
  }
  if (!down && b.pressed) {
    b.pressed = false;
    return b.longFired ? EV_NONE : EV_SHORT;
  }
  return EV_NONE;
}

bool anyButtonDown(void)
{
  return digitalRead(kPinOk) == LOW || digitalRead(kPinUp) == LOW || digitalRead(kPinDown) == LOW;
}

void doRefresh(const char *why)
{
  drawFrame();
  refreshPanel();
  logParams(why);
  needRefresh = false;
}

void sweep(int idx, int from, int to, int step, uint32_t dwellMs)
{
  const Param &p = kParams[idx];
  if (step <= 0) step = p.step;
  Serial.printf("GT sweep %s from %d to %d step %d dwell %lu ms (any key or 'x' aborts)\n", p.name,
                from, to, step, (unsigned long)dwellMs);
  int saved = *p.value;
  abortSweep = false;
  for (int v = from; (step > 0) ? v <= to : v >= to; v += step) {
    if (v < p.lo && !(p.allowOff && v == -1)) continue;
    if (v > p.hi) break;
    *p.value = v;
    onChanged(idx);
    doRefresh("sweep");
    uint32_t t0 = millis();
    while (millis() - t0 < dwellMs) {
      if (anyButtonDown()) abortSweep = true;
      if (Serial.available() && Serial.peek() == 'x') { Serial.read(); abortSweep = true; }
      if (abortSweep) break;
      delay(10);
    }
    if (abortSweep) {
      Serial.println("GT sweep aborted");
      while (anyButtonDown()) delay(10);
      return; // leave the last value in place
    }
  }
  Serial.printf("GT sweep done, restoring %s=%d\n", p.name, saved);
  *p.value = saved;
  onChanged(idx);
}

// --------------------------------------------------------------- serial ----

void printHelp(void)
{
  Serial.println("GT commands: help | dump | next | prev | up | down | r (refresh)");
  Serial.println("  set <name> <value>      value: decimal, 0x.. hex, enum name, off");
  Serial.println("  sel <name>              select a parameter");
  Serial.println("  preset <name|n>         bbep | seeed | bb2 | ws426");
  Serial.println("  sweep <name> <from> <to> [step] [dwell_ms]   ('x' or any key aborts)");
  Serial.println("  params: ");
  for (int i = 0; i < kParamCount; i++) {
    Serial.printf("%s%s", i ? ", " : "    ", kParams[i].name);
  }
  Serial.println();
  Serial.println("  OTP engine: T = 0x1A temperature slot, U = 0x22 byte, split = load LUT with 91 first,");
  Serial.println("    o* = voltage overrides (only effective with split=1, D7/DF reload from OTP).");
  Serial.println("  LUT engine: base/len/lutend/volt, voltages, L1/L2 phase codes of the gray group");
  Serial.println("    (G1 for bb2, G2 for ws426), G0..G2 frame counts, RP repeats, FR frame rate.");
}

int parseValue(const Param &p, const char *s)
{
  if (strcasecmp(s, "off") == 0) return -1;
  if (p.names) {
    for (int i = p.lo; i <= p.hi; i++) {
      if (strcasecmp(p.names[i], s) == 0) return i;
    }
  }
  if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) return (int)strtol(s + 2, nullptr, 16);
  // hex-ish kinds accept bare hex when it contains a letter, else decimal
  bool hexKind = p.kind != KIND_INT && p.kind != KIND_ENUM;
  if (hexKind) return (int)strtol(s, nullptr, 16);
  return (int)strtol(s, nullptr, 10);
}

void handleLine(char *line)
{
  char *argv[6];
  int argc = 0;
  for (char *tok = strtok(line, " \t\r\n"); tok && argc < 6; tok = strtok(nullptr, " \t\r\n")) argv[argc++] = tok;
  if (argc == 0) return;
  const char *cmd = argv[0];
  if (!strcasecmp(cmd, "help") || !strcasecmp(cmd, "?")) { printHelp(); return; }
  if (!strcasecmp(cmd, "dump")) { logParams("dump"); return; }
  if (!strcasecmp(cmd, "next") || !strcasecmp(cmd, "n")) { selected = nextVisible(selected, +1); needRefresh = true; return; }
  if (!strcasecmp(cmd, "prev") || !strcasecmp(cmd, "p")) { selected = nextVisible(selected, -1); needRefresh = true; return; }
  if (!strcasecmp(cmd, "up") || !strcasecmp(cmd, "u")) { stepParam(selected, +1); return; }
  if (!strcasecmp(cmd, "down") || !strcasecmp(cmd, "d")) { stepParam(selected, -1); return; }
  if (!strcasecmp(cmd, "r")) { needRefresh = true; return; }
  if (!strcasecmp(cmd, "sel") && argc >= 2) {
    int i = findParam(argv[1]);
    if (i < 0) { Serial.printf("GT: no parameter '%s'\n", argv[1]); return; }
    selected = i;
    if (!visible(i)) Serial.println("GT: (parameter belongs to the other engine)");
    needRefresh = true;
    return;
  }
  if (!strcasecmp(cmd, "preset") && argc >= 2) {
    int n = -1;
    for (int i = 0; i < 4; i++) if (!strcasecmp(kPresetNames[i], argv[1])) n = i;
    if (n < 0 && argv[1][0] >= '0' && argv[1][0] <= '3') n = argv[1][0] - '0';
    if (n < 0) { Serial.println("GT: preset bbep|seeed|bb2|ws426"); return; }
    applyPreset(n);
    needRefresh = true;
    return;
  }
  if (!strcasecmp(cmd, "set") && argc >= 3) {
    int i = findParam(argv[1]);
    if (i < 0) { Serial.printf("GT: no parameter '%s'\n", argv[1]); return; }
    setParam(i, parseValue(kParams[i], argv[2]));
    selected = i;
    return;
  }
  if (!strcasecmp(cmd, "sweep") && argc >= 4) {
    int i = findParam(argv[1]);
    if (i < 0) { Serial.printf("GT: no parameter '%s'\n", argv[1]); return; }
    int from = parseValue(kParams[i], argv[2]);
    int to = parseValue(kParams[i], argv[3]);
    int step = argc >= 5 ? atoi(argv[4]) : kParams[i].step;
    uint32_t dwell = argc >= 6 ? (uint32_t)atoi(argv[5]) : kSweepDwellMs;
    selected = i;
    sweep(i, from, to, step, dwell);
    return;
  }
  Serial.printf("GT: unknown command '%s' (help)\n", cmd);
}

void handleSerial(void)
{
  static char buf[96];
  static int n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      buf[n] = 0;
      if (n) handleLine(buf);
      n = 0;
    } else if (n < (int)sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

} // namespace

void gray_tune_main(void)
{
  Serial.println("GT: gray tune mode, EP397/SSD1677 waveform tuner");
  pinMode(kPinOk, INPUT_PULLUP);
  pinMode(kPinUp, INPUT_PULLUP);
  pinMode(kPinDown, INPUT_PULLUP);
  pinMode(kPinBusy, INPUT);

  // Take the panel over from display_init(): 4-gray type, identity palette,
  // no library-driven init (we send every command ourselves).
  bbep.setPanelType(EP397_800x480_4GRAY);
  bbep._bbep.pInitFull = kEmptyInit;
  bbep._bbep.pInitFast = nullptr;
  bbep._bbep.pInitPart = nullptr;
  bbep._bbep.pColorLookup = kIdentityLookup;
  bbep._bbep.iFlags &= ~BBEP_NEEDS_EXTRA_INIT;
  bbep.initIO(kPinDc, kPinRst, kPinBusy, kPinCs, kPinMosi, kPinSck, 8000000);
  bbep.setLightSleep(false);
  if (bbep.allocBuffer() != BBEP_SUCCESS || bbep.getBuffer() == nullptr) {
    Serial.println("GT: allocBuffer failed");
    for (;;) delay(1000);
  }
  Serial.printf("GT: buffer %p, %d bytes per plane\n", bbep.getBuffer(), kPlaneBytes);

  applyPreset(0);
  selected = 0;
  printHelp();

  for (;;) {
    Event ok = pollButton(btnOk);
    Event up = pollButton(btnUp);
    Event down = pollButton(btnDown);
    if (ok == EV_SHORT) { selected = nextVisible(selected, +1); needRefresh = true; }
    if (ok == EV_LONG) { applyPreset((t.preset + 1) % 4); needRefresh = true; }
    if (up == EV_SHORT) stepParam(selected, +1);
    if (down == EV_SHORT) stepParam(selected, -1);
    if (down == EV_LONG) resetParam(selected);
    if (up == EV_LONG) {
      const Param &p = kParams[selected];
      while (anyButtonDown()) delay(10);
      sweep(selected, p.allowOff ? -1 : p.lo, p.hi, p.step, kSweepDwellMs);
      needRefresh = true;
    }
    handleSerial();
    if (needRefresh) doRefresh("refresh");
    delay(10);
  }
}

#endif // GRAY_TUNE_MODE
