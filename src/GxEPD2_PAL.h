#ifndef _GxEPD2_PAL_H_
#define _GxEPD2_PAL_H_

#include <stdint.h>

// SPI_HAS_TRANSACTION means SPI has beginTransaction(), endTransaction(),
// usingInterrupt(), and SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

// SPI_ATOMIC_VERSION means that SPI has atomicity fixes and what version.
// This way when there is a bug fix you can check this define to alert users
// of your code if it uses better version of this library.
// This also implies everything that SPI_HAS_TRANSACTION as documented above is
// available too.
#define SPI_ATOMIC_VERSION 1

// Uncomment this line to add detection of mismatched begin/end transactions.
// A mismatch occurs if other libraries fail to use SPI.endTransaction() for
// each SPI.beginTransaction().  Connect an LED to this pin.  The LED will turn
// on if any mismatch is ever detected.
//#define SPI_TRANSACTION_MISMATCH_LED 5

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR

class SPISettings {
public:
  SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
    if (__builtin_constant_p(clock)) {
      init_AlwaysInline(clock, bitOrder, dataMode);
    } else {
      init_MightInline(clock, bitOrder, dataMode);
    }
  }
  SPISettings() {
    init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0);
  }
private:
  void init_MightInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
    init_AlwaysInline(clock, bitOrder, dataMode);
  }
  void init_AlwaysInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
    __attribute__((__always_inline__)) {
    _clk = clock;
    _order = bitOrder;
    _mode = dataMode;
  }
  uint32_t _clk;
  uint8_t _order;
  uint8_t _mode;
  friend class SPIClass;
};

class SPIClass {
public:
  // Initialize the SPI library
  static void begin();

  // Note: the usingInterrupt and notUsingInterrupt functions should
  // not to be called from ISR context or inside a transaction.
  // For details see:
  // https://github.com/arduino/Arduino/pull/2381
  // https://github.com/arduino/Arduino/pull/2449

  // Before using SPI.transfer() or asserting chip select pins,
  // this function is used to gain exclusive access to the SPI bus
  // and configure the correct settings.
  inline static void beginTransaction(SPISettings settings) {
    //TODO
    (void) settings;
  }

  // Write to the SPI bus (MOSI pin) and also receive (MISO pin)
  inline static uint8_t transfer(uint8_t data) {
     //TODO
    (void) data;
    return 0;
  }
  // After performing a group of transfers and releasing the chip select
  // signal, this function allows others to access the SPI bus
  inline static void endTransaction(void) {
    //TODO
  }

  // Disable the SPI bus
  static void end();

private:
  static uint8_t initialized;
};

extern SPIClass SPI;

#define MOSI 0
#define MISO 1
#define SCK 2

extern "C" {
  typedef enum {
    LOW = 0,
    HIGH = 1
  } pinstate_t;
  typedef enum {
    INPUT,
    INPUT_PULLUP,
    OUTPUT,
    DISABLED
  } pinmode_t;
  void digitalWrite(int8_t pin, pinstate_t state);
  int8_t digitalRead(int8_t pin);
  void pinMode(int8_t pin, pinmode_t mode);
  unsigned long micros(void);
  void delay(unsigned long ms);
  void delayMicroseconds(unsigned int us);
}

#include <openthread/cli.h>

#define PROGMEM
#define printf otCliOutputFormat
#define pgm_read_byte(x) ((uint8_t)*(x))

class SerialClass {
public:
  static void begin(unsigned int baudrate){
    (void) baudrate;
  }

  static void println(void){
    otCliOutputFormat("\r\n");
  }

  static void println(const char* str){
    otCliOutputFormat("%s\r\n",str);
  }

  static void println(unsigned long val){
    otCliOutputFormat("%lu\r\n",val);
  }

  static void print(const char* str){
    otCliOutputFormat(str);
  }
};

extern SerialClass Serial;

#endif
