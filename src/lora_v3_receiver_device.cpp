#define HELTEC_LORA_V3
#define INTERVAL    1

#include <heltec_tracker.h>

String rxdata;
uint64_t last_print;

void setup() {
    heltec_setup();

    // // Initialize display
    // VextOn();
    // display.init();
    // display.setContrast(255);
    // display.flipScreenVertically();
    
    init_radio();
    
}

void loop() {
    heltec_loop();
    
    if (!last_print || millis() - last_print > INTERVAL * 1000) {
        Serial.printf("Listening... \n");
        last_print = millis();
    }
    
    // If a packet was received, display it and the RSSI and SNR
    if (rxFlag) {
        rxFlag = false;
        radio.readData(rxdata);
        if (_radiolib_status == RADIOLIB_ERR_NONE) {
        Serial.printf("RX [%s]\n", rxdata.c_str());
        Serial.printf("  RSSI: %.2f dBm\n", radio.getRSSI());
        Serial.printf("  SNR: %.2f dB\n", radio.getSNR());
        }
        RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    }

}

