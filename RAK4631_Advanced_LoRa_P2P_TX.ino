/**
  @file LoRaP2P_TX.ino
  @author rakwireless.com
  @brief Transmitter node for LoRa point to point communication
  @version 0.1
  @date 2020-08-21
  @copyright Copyright (c) 2020
  @note RAK4631 GPIO mapping to nRF52840 GPIO ports
  RAK4631 <-> nRF52840
  WB_IO1 <-> P0.17 (GPIO 17)
  WB_IO2 <-> P1.02 (GPIO 34)
  WB_IO3 <-> P0.21 (GPIO 21)
  WB_IO4 <-> P0.04 (GPIO 4)
  WB_IO5 <-> P0.09 (GPIO 9)
  WB_IO6 <-> P0.10 (GPIO 10)
  WB_SW1 <-> P0.01 (GPIO 1)
  WB_A0 <-> P0.04/AIN2 (AnalogIn A2)
  WB_A1 <-> P0.31/AIN7 (AnalogIn A7)
*/

#include <Arduino.h>
#include <SX126x-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
#include "Adafruit_nRFCrypto.h"
#include <Adafruit_TinyUSB.h> // for Serial

// Function declarations
void OnTxDone(void);
void OnTxTimeout(void);

#ifdef NRF52_SERIES
#define LED_BUILTIN 35
#endif

// Define LoRa parameters
#define RF_FREQUENCY 862.125e6 // Hz
#define TX_OUTPUT_POWER 22 // dBm
#define LORA_BANDWIDTH 0 // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 12 // [SF7..SF12]
#define LORA_CODINGRATE 1 // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH 8 // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0 // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

static RadioEvents_t RadioEvents;
static uint8_t TxdBuffer[64];
uint8_t OCP_value = 0x18;
uint32_t startMillis;

void hexDump(uint8_t* buf, uint16_t len) {
  // Something similar to the Unix/Linux hexdump -C command
  // Pretty-prints the contents of a buffer, 16 bytes a row
  char alphabet[17] = "0123456789abcdef";
  uint16_t i, index;
  Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
  Serial.print(F("   |.0 .1 .2 .3 .4 .5 .6 .7 .8 .9 .a .b .c .d .e .f | |      ASCII     |\n"));
  for (i = 0; i < len; i += 16) {
    if (i % 128 == 0) Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
    char s[] = "|                                                | |                |\n";
    // pre-formated line. We will replace the spaces with text when appropriate.
    uint8_t ix = 1, iy = 52, j;
    for (j = 0; j < 16; j++) {
      if (i + j < len) {
        uint8_t c = buf[i + j];
        // fastest way to convert a byte to its 2-digit hex equivalent
        s[ix++] = alphabet[(c >> 4) & 0x0F];
        s[ix++] = alphabet[c & 0x0F];
        ix++;
        if (c > 31 && c < 128) s[iy++] = c;
        else s[iy++] = '.'; // display ASCII code 0x20-0x7F or a dot.
      }
    }
    index = i / 16;
    // display line number then the text
    if (i < 256) Serial.write(' ');
    Serial.print(index, HEX); Serial.write('.');
    Serial.print(s);
  }
  Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
}

void setup() {
  // Initialize LoRa chip.
  lora_rak4630_init();
  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial) {
    if ((millis() - timeout) < 5000) {
      delay(100);
    } else {
      break;
    }
  }
  Serial.println("=====================================");
  Serial.println("LoRap2p Tx Test");
  Serial.println("=====================================");
  nRFCrypto.begin();
  nRFCrypto.Random.generate(TxdBuffer, 64);
  hexDump(TxdBuffer, 64);
  // Initialize the Radio callbacks
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = NULL;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = NULL;
  RadioEvents.RxError = NULL;
  RadioEvents.CadDone = NULL;
  // Initialize the Radio
  Radio.Init(&RadioEvents);
  // Set Radio channel
  Radio.SetChannel(RF_FREQUENCY);
  // Set Radio TX configuration
  Radio.SetTxConfig(
    MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  send();
}

void loop() {
  // Put your application tasks here, like reading of sensors,
  // Controlling actuators and/or other functions.
}

/**@brief Function to be executed on Radio Tx Done event
*/
void OnTxDone(void) {
  float timing = (millis() - startMillis) / 1000.0;
  Serial.printf(". Done! Time: %.3f ms.\n", timing);
  delay(4000);
  send();
}

/**@brief Function to be executed on Radio Tx Timeout event
*/
void OnTxTimeout(void) {
  Serial.println("OnTxTimeout");
}

void send() {
  SX126xWriteRegister(0x08e7, OCP_value);
  uint8_t mA = OCP_value * 2.5;
  Serial.printf("Sending 64 bytes at OCP_value: 0x%2x, ie %d mA", OCP_value, mA);
  if (OCP_value == 0x18) OCP_value = 0x38;
  else OCP_value = 0x18;
  startMillis = millis();
  Radio.Send(TxdBuffer, 64);
}
