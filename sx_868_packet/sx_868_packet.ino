/* 
 * This is an example of running an SX127x in non-LoRa Packet Mode 
 *
 * Notes:
 * - If sharing an SPI bus (five in this example), ensure NSS=HIGH for all
 * - LongRangeMode=0 in RegOpMode sets the radio to FSK/OOK in sleep mode
 * - Do remaining setup in sleep mode - standby doesn't work first-time
 * - Preamble, syncword, address filtering and variable size work
 * - No CRC support other than CCITT / IBM, so may need other mechanisms
 */
#include <SPI.h>

/*
 * LoRa433 Module v1.0
 * Protoboard version - very hackable
 * RA-02 - SX1278 - 410-525MHz
 * 20MHz SPI
 */
#define NSS_868 6   // Resoldered from 1 - PoE conflict
#define RST_868 5   // Resoldered from 9 - PoE conflict
#define DIO0_868 10 // Default - for packet-mode

// All SPI devices on same bus need their NSS=HIGH
#define NSS_433 18
#define NSS_POE 9
#define NSS_LCD 3
#define NSS_SXC 4

// Clock frequency
#define FXOSC 32000000.0 // Clock speed used with RxBw

// Global pointer for the SPI object
extern SPIClass *vspi = NULL;

// Basic CRC-8: Two's Compliment
uint8_t crc8(uint8_t buf[], uint8_t size) {
  uint8_t crc = 0;

  for (int x = 0; x < size; x++)
      crc -= buf[x]; // Keep subtracting and rolling over

  return crc;
}

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
  spi_single_write(vspi, nss, 0x01, reg);            // Set bits 6-5 to 00 - FSK

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

  // Preamble and Syncword
  spi_single_write(vspi, nss, 0x27, 0x71); // Auto-restart, 0x55 preamble, syncword of 2 bytes
  spi_single_write(vspi, nss, 0x28, 0x2D); // Syncword 1 = 0x2D
  spi_single_write(vspi, nss, 0x29, 0xD4); // Syncword 2 = 0xD4

  // Packet Setup
  spi_single_write(vspi, nss, 0x30, 0x84); // Variable packet size, CRC off, Node+Broadcast address filtering
  spi_single_write(vspi, nss, 0x31, 0x40); // Packet mode
  spi_single_write(vspi, nss, 0x32, 0x20); // Packet Length - Maxiumum when variable packet size
  spi_single_write(vspi, nss, 0x33, 0x8B); // Node address filter (0x8B) (e.g. Remote to Base)
  spi_single_write(vspi, nss, 0x34, 0x0B); // Broadcast address filter (0x0B) (e.g. Base to all remotes)

  // IRQ setup
  spi_single_write(vspi, nss, 0x40, 0x00); // Default - DIO0 = PayloadReady (keeps SPI free)

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

  // RESET - set HIGH for default
  pinMode(RST_868, OUTPUT);
  digitalWrite(RST_868, HIGH);

  // We use DIO0 as a "packet-received" interrupt
  pinMode(DIO0_868, INPUT);

  // Wait for things to come alive
  delay(1000);

  // 868.29MHz / 9600bps / FSK@12kHz / Packet FSK NoShaping
  init_sx(vspi, RST_868, NSS_868, 868.299, 9600, 0x0D);
}

void loop() {
  // Look for PayloadReady on DIO0 (default in packet mode)
  if (digitalRead(DIO0_868)) {
    // Pointer to hold the payloadd
    uint8_t *buf = NULL;
    // Get the payload length from the first FIFO byte 
    uint8_t length = spi_single_read(vspi, NSS_868, 0x00);
    // Allocate memory for the payload
    buf = (uint8_t *)malloc(sizeof(uint8_t) * length);

    // Error if memory not allocated
    if (buf == NULL) {
      Serial.println(F("Cannot allocate memory"));
    } else {
      // Read in the remainder of the FIFO
      for (int x = 0; x < length; x++)
        buf[x] = spi_single_read(vspi, NSS_868, 0x00);

      // Print out the payload
      for (int x = 0; x < length; x++) {
        Serial.print(buf[x], HEX);
        Serial.print(" ");
      }

      /*
       * Optional: Custom CRC Check for Mitsubishi things
       * Skip the header (+7), and adjust the length (-8)
       * Check against the last byte
       */
      if (crc8(buf+7, length-8) == buf[length - 1])
        Serial.println("CRC-OK");
      else
        Serial.println("CRC-FAIL");

      // Free up the memory
      free(buf);
    }
  }
}
