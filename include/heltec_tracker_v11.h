#ifndef HELTEC_TRACKER_V11_H_
#define HELTEC_TRACKER_V11_H_

#include <stdint.h>
// #include <Arduino.h>
#include <HotButton.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
// #include <SPI.h>
#include "RadioLib.h"

/* Heltec Wireless Tracker V1.1 */

//
// TFT data from HT_st7735.h
//
#define ST7735_CS_Pin           GPIO_NUM_38
#define ST7735_REST_Pin         GPIO_NUM_39
#define ST7735_DC_Pin           GPIO_NUM_40
#define ST7735_SCLK_Pin         GPIO_NUM_41
#define ST7735_MOSI_Pin         GPIO_NUM_42
#define ST7735_LED_K_Pin        GPIO_NUM_21
#define ST7735_VTFT_CTRL_Pin    GPIO_NUM_3
#define ST7735_WIDTH            160
#define ST7735_HEIGHT           80
// mini 160x80, rotate left (INITR_MINI160x80_PLUGIN)
#define ST7735_MODEL            INITR_MINI160x80_PLUGIN

//
// SX1262 from esp32/libraries/LoraWan102/src/driver/board-config.h
//
#define BOARD_TCXO_WAKEUP_TIME  GPIO_NUM_5
// SPI pins
#define LORA_NSS                GPIO_NUM_8  // LoRa Negative Slave Select. The “N” in “NSS” indicates that it is an active-low signal, meaning the pin must be pulled low (0V) to enable communication with the LoRa module
#define LORA_SCK                GPIO_NUM_9  // Serial Clock, same as CLK
#define LORA_MOSI               GPIO_NUM_10 // Master Out Slave In
#define LORA_MISO               GPIO_NUM_11 // Master In Slave Out
// Radio pins
#define LORA_RESET              GPIO_NUM_12 // Used to reset the LoRa module
#define LORA_BUSY               GPIO_NUM_13 // Indicates the busy status of the LoRa module
#define LORA_DIO_1              GPIO_NUM_14 // Used for various interrupt-driven functions

//
// UC6580 GNSS
//
#define VGNSS_CTRL              GPIO_NUM_3
#define GNSS_TX                 GPIO_NUM_33
#define GNSS_RX                 GPIO_NUM_34
#define GNSS_RST                GPIO_NUM_35
#define GNSS_PPS                GPIO_NUM_36
#define GNSS_BAUD               115200UL

//
// misc from schematic
//
#define VEXT_CTRL               GPIO_NUM_3
#define VBAT_READ               GPIO_NUM_1
// VBAT = analogReadMilliVolts(VBAT_READ) * 4.9
#define ADC_CTRL                GPIO_NUM_2
#define USER_KEY                GPIO_NUM_0
#define LED                     GPIO_NUM_18
#define GNSS_BOOT_MODE          GPIO_NUM_47
#define GNSS_D_SEL              GPIO_NUM_48
#define U0RXD                   GPIO_NUM_44
#define U0TXD                   GPIO_NUM_43

#define TIMEZONE                -5

HotButton button(USER_KEY);

// --------------------- Display -------------------------
SPIClass spiST7735(HSPI); 
Adafruit_ST7735 st7735 = Adafruit_ST7735(&spiST7735, ST7735_CS_Pin, ST7735_DC_Pin, ST7735_REST_Pin);  

void VextOn(void) {
    pinMode(ST7735_VTFT_CTRL_Pin, OUTPUT);
    digitalWrite(ST7735_VTFT_CTRL_Pin, HIGH);  // VExt_Ctrl active HIGH
    pinMode(ST7735_LED_K_Pin, OUTPUT);
    digitalWrite(ST7735_LED_K_Pin, HIGH);  // Backlight active HIGH
}

void VextOff(void) {
    pinMode(ST7735_VTFT_CTRL_Pin, OUTPUT);
    digitalWrite(ST7735_VTFT_CTRL_Pin, LOW);
    // pinMode(ST7735_VTFT_CTRL_Pin, INPUT);

    pinMode(ST7735_LED_K_Pin, OUTPUT);
    digitalWrite(ST7735_LED_K_Pin, LOW);
    // pinMode(ST7735_LED_K_Pin, INPUT);
}


// --------------------- Battery -------------------------

const float min_voltage = 3.04;
const float max_voltage = 4.26;
const uint8_t scaled_voltage[100] = {
  254, 242, 230, 227, 223, 219, 215, 213, 210, 207,
  206, 202, 202, 200, 200, 199, 198, 198, 196, 196,
  195, 195, 194, 192, 191, 188, 187, 185, 185, 185,
  183, 182, 180, 179, 178, 175, 175, 174, 172, 171,
  170, 169, 168, 166, 166, 165, 165, 164, 161, 161,
  159, 158, 158, 157, 156, 155, 151, 148, 147, 145,
  143, 142, 140, 140, 136, 132, 130, 130, 129, 126,
  125, 124, 121, 120, 118, 116, 115, 114, 112, 112,
  110, 110, 108, 106, 106, 104, 102, 101, 99, 97,
  94, 90, 81, 80, 76, 73, 66, 52, 32, 7,
};

/**
 * @brief Measures the battery voltage.
 *
 * This function measures the battery voltage by controlling the VBAT_CTRL pin
 * and reading the analog value from the VBAT_ADC pin. The measured voltage is
 * then converted to a float value and returned.
 *
 * @return The battery voltage in volts.
 */
// float heltec_vbat() {
//   pinMode(ADC_CTRL, OUTPUT);
//   digitalWrite(ADC_CTRL, HIGH);
//   delay(5);
//   float vbat = analogRead(VBAT_READ) / 238.7;
//   // pulled up, no need to drive it
//   pinMode(ADC_CTRL, INPUT);
//   return vbat;
// }
float heltec_vbat() {
  pinMode(ADC_CTRL, INPUT_PULLUP);
  delay(5);
  float vbat = analogRead(VBAT_READ) / 238.7;
  return vbat;
}

/**
 * @brief Calculates the battery percentage based on the measured battery
 * voltage.
 *
 * This function calculates the battery percentage based on the measured battery
 * voltage. If the battery voltage is not provided as a parameter, it will be
 * measured using the heltec_vbat() function. The battery percentage is then
 * returned as an integer value.
 *
 * @param vbat The battery voltage in volts (default = -1).
 * @return The battery percentage (0-100).
 */
int heltec_battery_percent(float vbat = -1) {
  if (vbat == -1) {
    vbat = heltec_vbat();
  }
  for (int n = 0; n < sizeof(scaled_voltage); n++) {
    float step = (max_voltage - min_voltage) / 256;
    if (vbat > min_voltage + (step * scaled_voltage[n])) {
      return 100 - n;
    }
  }
  return 0;
}

// --------------------- LoRa -------------------------




SPIClass* spiRadio = new SPIClass(HSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO_1, LORA_RESET, LORA_BUSY, *spiRadio);

// make sure the power off button works when using RADIOLIB_OR_HALT
// (See RadioLib_convenience.h)
#define RADIOLIB_DO_DURING_HALT heltec_delay(10)
#include "RadioLib_convenience.h"

// Pause between transmited packets in seconds.
// Set to zero to only transmit a packet when pressing the user button
// Will not exceed 1% duty cycle, even if you set a lower value.
#define PAUSE               0

// Frequency in MHz. Keep the decimal point to designate float.
// Check your own rules and regulations to see what is legal where you are.
// #define FREQUENCY           866.3       // for Europe
#define FREQUENCY           905.2       // for US

// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH           250.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference. 
#define SPREADING_FACTOR    9

// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER      0

volatile bool rxFlag = false;
// Can't do Serial or display things here, takes too much time for the interrupt
void rx() {
  rxFlag = true;
}






// ------------------ Deep Sleep ----------------------
/**
 * @brief Puts the device into deep sleep mode.
 *
 * This function prepares the device for deep sleep mode by disconnecting from
 * WiFi, turning off the display, disabling external power, and turning off the
 * LED. It can also be configured to wake up after a certain number of seconds
 * using the optional parameter.
 *
 * @param seconds The number of seconds to sleep before waking up (default = 0).
 */
void heltec_deep_sleep(int seconds = 0) {
  #ifdef WiFi_h
    WiFi.disconnect(true);
  #endif
    st7735.enableSleep(true);
  #ifndef HELTEC_NO_RADIO_INSTANCE
    // It seems to make no sense to do a .begin() here, but in case the radio is
    // not interacted with at all before sleep, it will not respond to just
    // .sleep() and then consumes 800 µA more than it should in deep sleep.
    radio.begin();
    // 'false' here is to not have a warm start, we re-init the after sleep.
    radio.sleep(false);
  #endif
  // Turn off external power and turn off LED
  VextOff();
  esp_sleep_enable_ext0_wakeup(USER_KEY, LOW);
  button.waitForRelease();

  // Set timer wakeup if applicable
  if (seconds > 0) {
    esp_sleep_enable_timer_wakeup((int64_t)seconds * 1000000);
  }
  // and off to bed we go
  esp_deep_sleep_start();
}

// ------------------- Utilities ---------------------- 


/**
 * @brief Initializes the Heltec library.
 *
 * This function should be the first thing in setup() of your sketch. It
 * initializes the Heltec library by setting up serial port and display.
 */
void heltec_setup() {
  
}


/**
 * @brief The main loop function for the Heltec library.
 *
 * This function should be called in loop() of the Arduino sketch. It updates
 * the state of the power button and implements long-press power off if used.
 */

void heltec_loop() {
  button.update();
  // Power off button checking
  if (button.pressedFor(1000)) {
    // Visually confirm it's off so user releases button
    st7735.enableSleep(true);
    // Deep sleep (has wait for release so we don't wake up immediately)
    heltec_deep_sleep();
  }
}

/**
 * @brief Delays the execution of the program for the specified number of
 *        milliseconds.
 *
 * This function delays the execution of the program for the specified number of
 * milliseconds. During the delay, it also calls the heltec_loop() function to
 * allow for the power off button to be checked.
 *
 * @param ms The number of milliseconds to delay.
 */
void heltec_delay(int ms) {
  uint64_t start = millis();
  while (true) {
    heltec_loop();
    delay(1);
    if (millis() - start >= ms) {
      break;
    }
  }
}





#endif // HELTEC_TRACKER_V11_H_