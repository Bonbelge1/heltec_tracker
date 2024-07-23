#include <TinyGPSPlus.h>
#include <heltec_tracker_v11.h>

TinyGPSPlus gps;
float vbat;
bool ledEnabled = false;
long lastPPS = 0;

String rxdata;

long counter = 0;
uint64_t last_tx = 0;
uint64_t tx_time;
uint64_t minimum_pause;

void onPPS() { // on Pulse per Second
    ledEnabled = !ledEnabled;
    digitalWrite(LED, ledEnabled);
    if (lastPPS < millis() - 1000) {
        Serial.println(millis());
    }
    lastPPS = millis();
}

void setup() {
    Serial.begin(115200);
    // Serial.printf("Reason for wakeup : %s", esp_sleep_get_wakeup_cause()); not working
    VextOn(); // turn on
    
    // Initialize display
    spiST7735.begin(ST7735_SCLK_Pin, -1, ST7735_MOSI_Pin, ST7735_CS_Pin);            // SCK/CLK, MISO, MOSI, NSS/CS
    st7735.initR(ST7735_MODEL);                                                  // initialize ST7735S chip, mini display
    st7735.setRotation(2);
    st7735.fillScreen(ST7735_BLACK);
    
    // Initialize radio module
    spiRadio->begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    RADIOLIB_OR_HALT(radio.begin());
    // Set the callback function for received packets
    radio.setDio1Action(rx);
    // Set radio parameters
    Serial.printf("Frequency: %.2f MHz\n", FREQUENCY);
    RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
    Serial.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
    RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
    Serial.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
    RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
    Serial.printf("TX power: %i dBm\n", TRANSMIT_POWER);
    RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));
    // Start receiving
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));

    // Initialize GNSS module
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
    pinMode(GNSS_PPS, INPUT);
    attachInterrupt(GNSS_PPS, onPPS, RISING | FALLING);
    Serial1.begin(GNSS_BAUD, SERIAL_8N1, GNSS_TX, GNSS_RX);  // open GPS comms
}

void displayGPS() {
    st7735.setTextColor(ST7735_ORANGE);
    // st7735.fillRect(30, 96, 50, 30, ST7735_BLACK);
    st7735.fillScreen(ST7735_BLACK);
    st7735.setCursor(2,  2);
    st7735.printf("Battery: %d %%\r\n ", heltec_battery_percent(heltec_vbat()));
    st7735.setTextColor(ST7735_WHITE);
    st7735.setCursor(2,  96);
    st7735.printf("Lat: %8.04f", gps.location.lat());
    st7735.setCursor(2, 106);
    st7735.printf("Lng: %8.04f", gps.location.lng());
    st7735.setCursor(2, 116);
    st7735.printf("Alt: %6.2f m", gps.altitude.meters());
    // st7735.fillRect(30, 126, 50, 20, ST7735_BLACK);
    st7735.setCursor(2, 126);
    st7735.printf("HDOP: %7.1f", gps.hdop.hdop());
    st7735.setCursor(2, 136);
    st7735.printf("Sats: %7d", gps.satellites.value());
    st7735.fillRect(4, 146, 50, 8, ST7735_BLACK);
    st7735.setCursor(5, 146);
    st7735.printf("%02d:%02d:%02d", (gps.time.hour() + TIMEZONE) % 24, gps.time.minute(), gps.time.second());
}

void loop() {
    heltec_loop();
    while (Serial1.available()) {
        String input = Serial1.readStringUntil('\n');
        for (char c : input) {
            gps.encode(c);
        }
        // Serial.println(input);
    }
    displayGPS();
    Serial.printf("[%04d-%02d-%02d / %02d:%02d:%02d] ", 
                    gps.date.year(), gps.date.month(), gps.date.day(), (gps.time.hour() + TIMEZONE) % 24, gps.time.minute(), gps.time.second());
    Serial.printf("Lat: %8.5f | Lng: %8.5f | Alt: %5.2f | Sats: %d | HDOP: %5.2f", 
                    gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.satellites.value(), gps.hdop.value());
    Serial.printf(" | Battery: %d %%\r\n ", heltec_battery_percent(heltec_vbat()));
    delay(500);



    bool tx_legal = millis() > last_tx + minimum_pause;
    // Transmit a packet every PAUSE seconds or when the button is pressed
    if ((PAUSE && tx_legal && millis() - last_tx > (PAUSE * 1000)) || button.isSingleClick()) {
        // In case of button click, tell user to wait
        if (!tx_legal) {
            Serial.printf("Legal limit, wait %i sec.\n", (int)((minimum_pause - (millis() - last_tx)) / 1000) + 1);
            return;
        }
        Serial.printf("new TX sent, last was [%i] seconds ago", (int)((millis() - last_tx) / 1000) + 1);
        radio.clearDio1Action(); // clearDio1Action function disables the interrupt associated with the DIO1 pin of the SX126x module. This is typically done to stop the module from responding to certain events or signals on that pin.
        tx_time = millis();
        RADIOLIB(radio.transmit("hello world!"));
        tx_time = millis() - tx_time;
        if (_radiolib_status == RADIOLIB_ERR_NONE) {
            Serial.printf("OK (%i ms)\n", (int)tx_time);
        } else {
            Serial.printf("fail (%i)\n", _radiolib_status);
        }
        // Maximum 1% duty cycle
        minimum_pause = tx_time * 100;
        last_tx = millis();
        radio.setDio1Action(rx);
        RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
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




