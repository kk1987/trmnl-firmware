#pragma once
// Standalone gray-scale tuner for the Seeed reTerminal Sticky (EP397 / SSD1677).
// Built only with -D GRAY_TUNE_MODE; takes over bl_init() after the panel and
// filesystem are up and never returns (no WiFi, no server, no deep sleep).
void gray_tune_main(void);
