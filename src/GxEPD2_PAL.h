#ifndef _GxEPD2_PAL_H_
#define _GxEPD2_PAL_H_

#include <stdint.h>

#define MOSI 0
#define MISO 1
#define SCK 2
#define CS 3
#define DISP_DC 4
#define DISP_RST 5
#define DISP_BUSY 6

#if defined(EFR32MG12P332F1024GL125)
#define SPI_PERIPH USART2
#define SPI_PERIPH_CLK cmuClock_USART2
// MOSI on EXP 4 (SPI_MOSI)
#define MOSI_PORT gpioPortK
#define MOSI_PIN  0
#define MOSI_LOC  29
// MISO on EXP 6 (SPI_MISO)
#define MISO_PORT gpioPortK
#define MISO_PIN  2
#define MISO_LOC  30
// SCK on EXP 8 (SPI_SCLK)
#define SCK_PORT  gpioPortF
#define SCK_PIN   7
#define SCK_LOC   18
// CS on EXP 10 (SPI_CS)
#define CS_PORT   gpioPortA
#define CS_PIN    5
#define CS_LOC    27
// DC on EXP 12 (UART_TX)
#define DISP_DC_PORT   gpioPortF
#define DISP_DC_PIN    3
// RST on EXP 14 (UART_RX)
#define DISP_RST_PORT  gpioPortF
#define DISP_RST_PIN   4
// BUSY on EXP 16 (I2C_SDA)
#define DISP_BUSY_PORT gpioPortC
#define DISP_BUSY_PIN  10

#else
#error "Undefined pin mapping"
#endif

extern "C" {
  #include "em_gpio.h"
  #include "em_usart.h"
  #include "em_cmu.h"

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

  static GPIO_Port_TypeDef getEmlibPort(int8_t pin) {
    switch(pin) {
      case MOSI:
        return MOSI_PORT;
      case MISO:
        return MISO_PORT;
      case SCK:
        return SCK_PORT;
      case CS:
        return CS_PORT;
      case DISP_DC:
        return DISP_DC_PORT;
      case DISP_RST:
        return DISP_RST_PORT;
      case DISP_BUSY:
        return DISP_BUSY_PORT;
      default:
        return gpioPortA;
    }
  }

  static unsigned int getEmlibPin(int8_t pin) {
      switch(pin) {
        case MOSI:
          return MOSI_PIN;
        case MISO:
          return MISO_PIN;
        case SCK:
          return SCK_PIN;
        case CS:
          return CS_PIN;
        case DISP_DC:
          return DISP_DC_PIN;
        case DISP_RST:
          return DISP_RST_PIN;
        case DISP_BUSY:
          return DISP_BUSY_PIN;
        default:
          return gpioPortA;
      }
    }

  static void digitalWrite(int8_t pin, pinstate_t state)
  {
    if(state == LOW) {
        GPIO_PinOutClear(getEmlibPort(pin), getEmlibPin(pin));
    } else {
        GPIO_PinOutSet(getEmlibPort(pin), getEmlibPin(pin));
    }
  }
  static int8_t digitalRead(int8_t pin)
  {
    return GPIO_PinInGet(getEmlibPort(pin), getEmlibPin(pin));
  }

  static void pinMode(int8_t pin, pinmode_t mode)
  {
    switch(mode) {
      case INPUT:
        GPIO_PinModeSet(getEmlibPort(pin), getEmlibPin(pin), gpioModeInput, 0);
        return;
      case INPUT_PULLUP:
        GPIO_PinModeSet(getEmlibPort(pin), getEmlibPin(pin), gpioModeInputPull, 1);
        return;
      case OUTPUT:
        GPIO_PinModeSet(getEmlibPort(pin), getEmlibPin(pin), gpioModePushPull, 0);
        return;
      case DISABLED:
        GPIO_PinModeSet(getEmlibPort(pin), getEmlibPin(pin), gpioModeDisabled, 0);
        return;
    }
  }

  #include "sl_sleeptimer.h"
  static unsigned long micros(void) {
    return sl_sleeptimer_tick_to_ms(sl_sleeptimer_get_tick_count()) * 1000;
  }
  static void delay(unsigned long ms) {
    sl_sleeptimer_delay_millisecond(ms);
  }

  #include "sl_udelay.h"
  static void delayMicroseconds(unsigned int us) { sl_udelay_wait(us); }
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

// SPI_HAS_TRANSACTION means SPI has beginTransaction(), endTransaction(),
// usingInterrupt(), and SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// SPI_ATOMIC_VERSION means that SPI has atomicity fixes and what version.
// This way when there is a bug fix you can check this define to alert users
// of your code if it uses better version of this library.
// This also implies everything that SPI_HAS_TRANSACTION as documented above is
// available too.
#define SPI_ATOMIC_VERSION 1

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

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
  static void begin(void)
  {
    // Set up pins and clocks here
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(SPI_PERIPH_CLK, true);

    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(SCK, OUTPUT);

    USART_InitSync_TypeDef initsync = USART_INITSYNC_DEFAULT;
    USART_InitSync(SPI_PERIPH, &initsync);

    SPI_PERIPH->ROUTELOC0 =
        MISO_LOC << _USART_ROUTELOC0_RXLOC_SHIFT |
        MOSI_LOC << _USART_ROUTELOC0_TXLOC_SHIFT |
        SCK_LOC << _USART_ROUTELOC0_CLKLOC_SHIFT;

    SPI_PERIPH->ROUTEPEN =
        USART_ROUTEPEN_RXPEN |
        USART_ROUTEPEN_TXPEN |
        USART_ROUTEPEN_CLKPEN;
  }

  // Before using SPI.transfer() or asserting chip select pins,
  // this function is used to gain exclusive access to the SPI bus
  // and configure the correct settings.
  inline static void beginTransaction(SPISettings settings) {
    switch(settings._mode)
    {
      case SPI_MODE0:
        SPI_PERIPH->CTRL = (SPI_PERIPH->CTRL & ~(_USART_CTRL_CLKPOL_MASK | _USART_CTRL_CLKPHA_MASK)) | usartClockMode0;
        break;
      case SPI_MODE1:
        SPI_PERIPH->CTRL = (SPI_PERIPH->CTRL & ~(_USART_CTRL_CLKPOL_MASK | _USART_CTRL_CLKPHA_MASK)) | usartClockMode1;
        break;
      case SPI_MODE2:
        SPI_PERIPH->CTRL = (SPI_PERIPH->CTRL & ~(_USART_CTRL_CLKPOL_MASK | _USART_CTRL_CLKPHA_MASK)) | usartClockMode2;
        break;
      case SPI_MODE3:
        SPI_PERIPH->CTRL = (SPI_PERIPH->CTRL & ~(_USART_CTRL_CLKPOL_MASK | _USART_CTRL_CLKPHA_MASK)) | usartClockMode3;
        break;
    }

    switch(settings._order)
    {
      case LSBFIRST:
        SPI_PERIPH->CTRL = SPI_PERIPH->CTRL & ~USART_CTRL_MSBF;
        break;
      case MSBFIRST:
        SPI_PERIPH->CTRL = SPI_PERIPH->CTRL | USART_CTRL_MSBF;
        break;
    }

    USART_BaudrateSyncSet(SPI_PERIPH, 0, settings._clk);
  }

  // Write to the SPI bus (MOSI pin) and also receive (MISO pin)
  inline static uint8_t transfer(uint8_t data) {
    return USART_SpiTransfer(SPI_PERIPH, data);
  }
  // After performing a group of transfers and releasing the chip select
  // signal, this function allows others to access the SPI bus
  inline static void endTransaction(void) {
    // empty
  }

  // Disable the SPI bus
  static void end(void)
  {
    // turn off clocks here
    CMU_ClockEnable(cmuClock_USART1, false);
  }

private:
  static uint8_t initialized;
};

extern SPIClass SPI;

#endif
