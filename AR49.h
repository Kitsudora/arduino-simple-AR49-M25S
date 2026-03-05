#pragma once
#include <Arduino.h>
#include <SPI.h>
#define CS_PIN_AR49 10  // encoder select pin

#define ANGLE_RANGE 180

#define POS_DEFAULT_MASK(x) ~((~0xFFC00000)>>x)
#define STATUS_DEFAULT_MASK(x) ((0x00018000)>>x)

typedef struct DataFeedback_Basic_24bits
{
  uint32_t pos;
  boolean valid;
} DB24;

#ifndef AR49_SPI_SETTINGS
#define AR49_SPI_SETTINGS SPISettings(1000000, MSBFIRST, SPI_MODE3)
#endif

class AR49
{
public:
  explicit AR49(uint8_t csPin = 0xFF, SPIClass& spi = SPI);

  void begin(uint8_t csPin = CS_PIN_AR49);
  void set_Spi_Settings(const SPISettings& settings);

  void read_Pos(DB24* data);
  uint8_t update_Mask_Read_STB();
  void read_Status();

private:
  void select();
  void deselect();
  void set_Page(uint8_t page);
  uint8_t read_Reg(uint8_t addr);
  static uint32_t read_Bits_Msb_First(const uint8_t* buf, uint16_t bit_offset, uint8_t bit_len);

  SPIClass* _spi;
  SPISettings _settings;
  uint8_t _csPin;

  uint8_t _stSelect;
  uint8_t _mtSelect;
  uint8_t _stBits;
  uint8_t _mtBits;
  bool _spi4Ext;

  uint32_t _posMask;
  uint32_t _statusMask;
};
