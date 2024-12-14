/* 
 * This is an example of running an SX1278 in continuous AKA "RAW" mode.
 * Raw pulses can then be read out of DIO2, just like a RFM69HCW.
 *
 * Notes:
 * - If sharing an SPI bus (five devices here), ensure NSS=HIGH for all
 * - LongRangeMode=0 in RegOpMode sets the radio to FSK/OOK in sleep mode
 * - Do remaining setup in sleep mode - standby doesn't work first-time
 * - DataMode=0 in RegPacketConfig2 sets Continuous mode
 * - Data source has no preamble - use RSSI (DIO0) to wake up an interrupt
 * - Use ESP32 FreeRTOS multitasking to perform DIO2 non-blocking capture
 * - Use a queue to write between tasks / cores
 */

#include <SPI.h>

/*
 * M5Stack LoRa433 Module v1.0
 * Protoboard version - very hackable
 * RA-02 - SX1278 - 410-525MHz
 * 20MHz SPI
 */
#define NSS_433 18  // Resoldered from 1 - PoE conflict
#define RST_433 17  // Resoldered from 9 - PoE conflict
#define DIO0_433 8  // Default - for packet-mode
#define DIO2_433 2  // Soldered to add continuous-mode

// All SPI devices on same bus need their NSS=HIGH
#define NSS_868 6
#define NSS_POE 9
#define NSS_LCD 3
#define NSS_SXC 4

// Clock frequency
#define FXOSC 32000000.0 // Clock speed used with RxBw

// Raw data scheme
#define PULSE_LEAD  0x1 // HIGH of 500ms (488-544)
#define PULSE_SHORT 0x3 // LOW of 1000ms (980-1012)
#define PULSE_LONG  0x7 // LOW of 2000ms (1956-1996)
#define PULSE_STARTA 0xF // LOW of 3500ms (3328-3584)
#define PULSE_STARTB 0xE // LOW of 3700ms (3584-3840)
#define BIT_COUNT   36

// Global pointer for the SPI object
extern SPIClass *vspi = NULL;

QueueHandle_t sx_433_queue;

volatile bool sx_433_activity = false;

// Single Read/Write Operation
uint8_t spi_cmd(SPIClass *vspi, uint8_t nss, uint8_t addr, uint8_t value) {
  uint8_t status;

  // Indicate to device we are transmitting to it
  digitalWrite(nss, LOW);

  // Open SPI at 10MHz (FSCK in datasheet, also confirmed at 2.1.5.4)
  vspi->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

  // Transfer address value 
  vspi->transfer(addr);

  // Get status (0x00) or write (value | 0x80)
  status = vspi->transfer(value);

  // Stop transmission
  vspi->endTransaction();
  digitalWrite(nss, HIGH);

  return status;
}

// Single Write: Apply the wnr bit to write (section 2.2)
uint8_t spi_single_write(SPIClass *vspi, uint8_t nss, uint8_t addr, uint8_t value) {
  return spi_cmd(vspi, nss, addr | 0x80, value);
}

// Single Read: Write a zero to read
uint8_t spi_single_read(SPIClass *vspi, uint8_t nss, uint8_t addr) {
  return spi_cmd(vspi, nss, addr, 0x00);
}

// Start the radio
void init_sx(SPIClass *vspi, uint8_t rst, uint8_t nss, float freq, uint16_t bitrate, uint8_t bw) {
  uint8_t reg;
  // Set NSS off if not already
  digitalWrite(nss, HIGH);

  // Reset
  digitalWrite(rst, LOW);
  delayMicroseconds(150);
  digitalWrite(rst, HIGH);
  delay(15);

  // Version check
  if(spi_single_read(vspi, nss, 0x42) == 0x12)
    Serial.println("Radio: Found");
  else
    Serial.println("Radio: ERROR - Invalid / no silicon found!");

  // Sleep Mode for applying settings
  reg = spi_single_read(vspi, nss, 0x01) & 0xF8;     // Mask out bits 2-0
  spi_single_write(vspi, nss, 0x01, reg);            // Set bits 2-0 to 000
  delay(10);                                         // Wait until done

  // Enable FSK/OOK Radio while at sleep (disable LoRa)
  reg = spi_single_read(vspi, nss, 0x01) & 0xEF;     // Mask out bit 7
  spi_single_write(vspi, nss, 0x01, reg);            // Set bit 7 to 0

  // Enable OOK modulation
  reg = (spi_single_read(vspi, nss, 0x01) & 0x9F);   // Mask out bits 6-5
  spi_single_write(vspi, nss, 0x01, reg | 0x20);     // Set bits 6-5 to 01

  // Set RF Bandwidth 
  spi_single_write(vspi, nss, 0x12, bw);
  // Set AFC Bandwidth
  spi_single_write(vspi, nss, 0x13, bw);

  // Set Frequency
  uint32_t freqrf = (freq * 1000000) / (FXOSC / 524288);
  spi_single_write(vspi, nss, 0x06, (freqrf >> 16) & 0xFF);
  spi_single_write(vspi, nss, 0x07, (freqrf >> 8) & 0xFF);
  spi_single_write(vspi, nss, 0x08, freqrf & 0xFF);

  // Set Bitrate
  spi_single_write(vspi, nss, 0x02, (((uint32_t)FXOSC/bitrate) >> 8) & 0xFF);
  spi_single_write(vspi, nss, 0x03, ((uint32_t)FXOSC/bitrate) & 0xFF);

  // Enable continuous mode (disabling packet mode)
  reg = spi_single_read(vspi, nss, 0x31) & 0xBF;     // Mask out bit 6
  spi_single_write(vspi, nss, 0x31, reg);            // Set bit 6 to 0

  // Enable OOK Peak and Bit Synchronizer
  reg = spi_single_read(vspi, nss, 0x14) & 0xC7;     // Mask out bit 5-3
  spi_single_write(vspi, nss, 0x14, reg | 0x28);     // Set bits 5-3 to 101

  // RSSI trigger on DIO0
  spi_single_write(vspi, nss, 0x40, 0x40);           // RSSI pin assign to DIO
  spi_single_write(vspi, nss, 0x10, 0x90);           // Larger -45dBm seems to work

  // Receive Synth Mode
  reg = spi_single_read(vspi, nss, 0x01) & 0xF8;     // Mask out bits 2-0
  spi_single_write(vspi, nss, 0x01, reg | 0x04);     // Set bits 2-0 to 100
  while(!(spi_single_read(vspi, nss, 0x3E) & 0x80))  // Wait for ModeReady
  delayMicroseconds(120);                            // Wake-up to lock: ~60us

  // Receive Mode
  reg = spi_single_read(vspi, nss, 0x01) & 0xF8;     // Mask out bits 2-0
  spi_single_write(vspi, nss, 0x01, reg | 0x05);     // Set bits 2-0 to 101
  while(!(spi_single_read(vspi, nss, 0x3E) & 0x80)); // Wait for ModeReady
  delayMicroseconds(2450);                           // Receiver Start: 2.33ms +/- 5%
  
  Serial.println("Radio: Ready");
}

void sx_433_print() {
  uint32_t thisdata;

  xQueueReceive(sx_433_queue, &thisdata, (TickType_t) 10);
  Serial.print(millis() / 1000);
  uint8_t id = (thisdata >> 28) & 0xFF;
  uint8_t battery = (thisdata >> 27) & 0x1;
  uint8_t manual = (thisdata >> 26) & 0x1; 
  uint8_t channel = ((thisdata >> 24) & 0x3) + 1;
  float temperature = (((int16_t)((thisdata >> 8) & 0xFFF0)) >> 4) * 0.1;
  uint8_t humidity = thisdata & 0x7F;

  Serial.print(" - ID: 0x");
  Serial.print(id, HEX);
  Serial.print(", Battery?: ");
  Serial.print(battery);
  Serial.print(", Manual?: ");
  Serial.print(manual);
  Serial.print(", Chan: ");
  Serial.print(channel);
  Serial.print(", Temp: ");
  Serial.print(temperature);
  Serial.print(", RelH: ");
  Serial.println(humidity);
}

void sx_433_capture(void *pvParameters) {
  uint32_t thisTime = micros();
  uint32_t lastTime = thisTime;

  // Stores the current and last packet
  uint32_t thisdata, lastdata;
  uint8_t thisValue, lastValue, thisGap, captured, pulses;
  captured = 0; // unset when valid data found
  pulses = 0;
  lastValue = 0;

  while ((thisTime - lastTime) < 5000) { 
    thisValue = digitalRead(DIO2_433);
    thisTime = micros();

    // If there is a data change, process it
    if (thisValue != lastValue) {
      if (captured) {
        // if data already obtained, wait for a timeout
        vTaskDelay(1);
      } else {
        // not got data yet... 

        // Round-down the ms value to 256ms chunks and recenter
        thisGap = (thisTime - lastTime - 128) >> 8;

        // Check the size of the pulse we have received
        switch (thisGap) {

          // Pre-bit pulse 500us
          case PULSE_LEAD:
            // Wait for next bit
            break;

          // The pause indicating incoming data
          case PULSE_STARTA:
          case PULSE_STARTB:
            // Reset the data regardless
            thisdata = 0;
            pulses = 0;
            break; 

          // Valid Data to be processed
          case PULSE_SHORT: // 1000 us = 0
          case PULSE_LONG:  // 2000 us = 1
            // Make room for a new bit
            thisdata <<= 1;
            // Set the bit to the what the gap size represents
            thisdata |= (!(~thisGap & 0x04));
            // Increment the counter
            pulses++;
            // Wait for next bit
            break;
            
          // Soft reset, but don't reset - let timeout decide
          default:
            thisdata = 0;
            pulses = 0;
        }
        
        // If we have enough data, work with it and reset
        if (pulses == BIT_COUNT) {
          if (thisdata == lastdata) {
            // We have valid data - send and wait out remaining data
            xQueueSend(sx_433_queue, &thisdata, NULL);
            captured = 1;
          } else if (((thisdata >> 7) & 0x1F) == 0x1E) {
            // First sight of good data... save it and wait for another
            lastdata = thisdata;
          }
          // Do a soft reset
          thisdata = 0;
          pulses = 0;
        }      
      } 
      // Keep track of last-known state
      lastValue = thisValue;
      lastTime = thisTime;
    } 
  }
  // Reset the RSSI flag and wait for it to clear
  spi_single_write(vspi, NSS_433, 0x3E, 0x08);
  while((spi_single_read(vspi, NSS_433, 0x3E) & 0x08));
  
  // Kill-off this ad-hoc task
  vTaskDelete(NULL);
}

// 433MHz detection flag
void IRAM_ATTR sx_433_detection() {
  sx_433_activity = true;
}

void setup() {
  // Switch on serial
  Serial.begin(115200);

  // Delay, to hook up serial or avoid crash cycle
  delay(10000);

  // SPI setup
  vspi = new SPIClass();
  // All devices on same bus, so leave NSS unconfigured
  vspi->begin(SCK, MISO, MOSI, -1);

  // Set HIGH for all devices on shared SPI bus
  pinMode(NSS_433, OUTPUT);
  digitalWrite(NSS_433, HIGH);
  pinMode(NSS_868, OUTPUT);
  digitalWrite(NSS_868, HIGH);
  pinMode(NSS_POE, OUTPUT);
  digitalWrite(NSS_POE, HIGH);
  pinMode(NSS_SXC, OUTPUT);
  digitalWrite(NSS_SXC, HIGH);
  pinMode(NSS_LCD, OUTPUT);
  digitalWrite(NSS_LCD, HIGH);

  /*
   * 433MHz setup
   */
  // RESET - set HIGH for default
  pinMode(RST_433, OUTPUT);
  digitalWrite(RST_433, HIGH);

  // We use DIO2 as a raw data feed
  pinMode(DIO2_433, INPUT);

  // Use DIO0 to detect a incoming signal
  pinMode(DIO0_433, INPUT);

  // Set up 433 queue
  sx_433_queue = xQueueCreate(8, sizeof(uint32_t));

  // 433.73MHz / 2000bps / OOk@167kHz / Cont+sync+OOK+no-shaping
  init_sx(vspi, RST_433, NSS_433, 433.73, 2000, 0x11);

  // Interrupt for 433MHz activity to run once
  attachInterrupt(digitalPinToInterrupt(DIO0_433), sx_433_detection, RISING);
}

void loop() {
  // Flag on 433MHz RSSI "RISING" - stops duplicate tasks
  if (sx_433_activity) {
    // Task to grab the 433MHz traffic - 1.5Kbytes needed for task
    xTaskCreate(sx_433_capture, "SX 433 capture", 1536, NULL, 1, NULL);

    // Reset rising flag immediately
    sx_433_activity = false;
  }

  // Parse messages off the queue
  while (uxQueueMessagesWaiting(sx_433_queue)) {
    sx_433_print();
  }
}
