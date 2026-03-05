#include "AR49.h"

AR49::AR49(uint8_t csPin, SPIClass& spi)
  : _spi(&spi),
    _settings(AR49_SPI_SETTINGS),
    _csPin(csPin),
    _stSelect(0),
    _mtSelect(0),
    _stBits(15),
    _mtBits(0),
    _spi4Ext(false),
    _posMask(POS_DEFAULT_MASK(0)),
    _statusMask(STATUS_DEFAULT_MASK(0))
{
}

void AR49::begin(uint8_t csPin)
{
  _csPin = csPin;
  pinMode(_csPin, OUTPUT);
  deselect();
  _spi->begin();
}

void AR49::set_Spi_Settings(const SPISettings& settings)
{
  _settings = settings;
}

void AR49::select()   { digitalWrite(_csPin, LOW); }
void AR49::deselect() { digitalWrite(_csPin, HIGH); }

void AR49::set_Page(uint8_t page)
{
  // OC=0xD2 Register Write (Single): MOSI D2 ADDR DATA
  _spi->beginTransaction(_settings);
  select();
  (void)_spi->transfer(0xD2);
  (void)_spi->transfer(0x7F);   // active page register
  (void)_spi->transfer(page);   // page number
  deselect();
  _spi->endTransaction();

  // Datasheet recommends ~18 ms after page change; keep your 20 ms.
  delay(20);
}

uint8_t AR49::read_Reg(uint8_t addr)
{
  // OC=0x81 Register Read (Continuous): MOSI 81 ADDR 00 ; MISO ... DATA1 DATA2...
  _spi->beginTransaction(_settings);
  select();
  (void)_spi->transfer(0x81);
  (void)_spi->transfer(addr);
  (void)_spi->transfer(0x00);         // required "delay" byte in the protocol
  uint8_t v = _spi->transfer(0x00);   // DATA1
  deselect();
  _spi->endTransaction();
  return v;
}

// Read MSB-first bitstream from buf[].
// bit_offset: 0 means the first bit of buf[0] (its MSB).
uint32_t AR49::read_Bits_Msb_First(const uint8_t* buf, uint16_t bit_offset, uint8_t bit_len)
{
  uint32_t out = 0;
  for (uint8_t i = 0; i < bit_len; i++) {
    uint16_t k = bit_offset + i;
    uint8_t  byte_idx = (uint8_t)(k / 8);
    uint8_t  bit_in_byte = 7u - (uint8_t)(k % 8); // MSB-first
    uint8_t  bit = (buf[byte_idx] >> bit_in_byte) & 0x01u;
    out = (out << 1) | bit;
  }
  return out;
}

void AR49::read_Pos(DB24* data)
{
  if (!data) return;

  // Compute expected payload bits AFTER the 8-bit echo of OC.
  // Position Read frame (SPI4):
  //   Basic:    [MT+ST] + nE + nW + CRC(6)
  //   Extended: [MT+ST] + nE + nW + SeqCnt(6) + CRC(16)
  const uint16_t pos_bits   = (uint16_t)_mtBits + (uint16_t)_stBits;
  const uint16_t tail_bits  = _spi4Ext ? (2u + 6u + 16u) : (2u + 6u);
  const uint16_t total_bits = pos_bits + tail_bits;
  const uint8_t  rx_bytes   = (uint8_t)((total_bits + 7u) / 8u); // bytes after echo

  // Safety: cap buffer (worst case here is small; rx_bytes <= 10 for your likely configs)
  uint8_t buf[10] = {0};
  const uint8_t n = (rx_bytes <= sizeof(buf)) ? rx_bytes : (uint8_t)sizeof(buf);

  _spi->beginTransaction(_settings);
  select();

  const uint8_t echo = _spi->transfer(0xA6); // OC=0xA6 Position Read; MISO echoes OC during first 8 clocks
  for (uint8_t i = 0; i < n; i++) {
    buf[i] = _spi->transfer(0x00);
  }

  deselect();
  _spi->endTransaction();

  // If echo mismatches, treat as invalid read (wiring/bus issue)
  if (echo != 0xA6) {
    data->pos = 0;
    data->valid = false;
    return;
  }

  // Parse bitstream: [MT][ST][nE][nW]...
  uint16_t off = 0;
  uint32_t mt = 0;
  if (_mtBits > 0) {
    mt = read_Bits_Msb_First(buf, off, _mtBits);
    off += _mtBits;
  }
  const uint32_t st = read_Bits_Msb_First(buf, off, _stBits);
  off += _stBits;

  const uint8_t nE = (uint8_t)read_Bits_Msb_First(buf, off, 1); off += 1;
  const uint8_t nW = (uint8_t)read_Bits_Msb_First(buf, off, 1); off += 1;

  // nError / nWarning are active-low in the datasheet logic:
  //   nE==1 and nW==1 => no error/warning
  data->valid = (nE == 1u) && (nW == 1u);

  // Default behavior: return ST only (fits your DB24).
  // If you truly need MT+ST, change DB24.pos to uint64_t and combine (mt<<st_bits)|st.
  (void)mt;
  data->pos = st;
}

uint8_t AR49::update_Mask_Read_STB()
{
  // Go to Page 7 (required for resolution and SPI4 CRC settings)
  set_Page(0x07);

  // Page 7, Addr 0x16: [6:4] MT_Select, [3:0] ST_Select
  const uint8_t cfg = read_Reg(0x16);
  _stSelect = cfg & 0x0Fu;
  _mtSelect = (cfg >> 4) & 0x07u;

  // Resolve ST bits (SPI/SSI 25-bit output table): 0000->15 ... 1001->24, 101x/11xx->25
  if (_stSelect <= 9) _stBits = (uint8_t)(15u + _stSelect);
  else                _stBits = 25u;

  // Resolve MT bits: 000->0, 001->12, 010->14, 011->16, 100->18, 101->20, 110->22, 111->24
  static const uint8_t MT_BITS_LUT[8] = {0, 12, 14, 16, 18, 20, 22, 24};
  _mtBits = MT_BITS_LUT[_mtSelect & 0x07u];

  // Page 7, Addr 0x25: SPI4_Ext_EN (6-bit CRC vs 16-bit CRC)
  // Datasheet table shows this as an "enable"; bit position is not explicit in your snippet,
  // so treat any non-zero as enabled to avoid guessing the exact bit.
  const uint8_t ext = read_Reg(0x25);
  _spi4Ext = (ext != 0);

  // Update your legacy mask scheme too (if you still use it elsewhere).
  // Your macros behave like: width = 10 + x, so choose x = ST_bits - 10 (clamped).
  uint8_t x = 0;
  if (_stBits > 10) x = (uint8_t)(_stBits - 10);
  _posMask    = (uint32_t)POS_DEFAULT_MASK(x);
  _statusMask = (uint32_t)STATUS_DEFAULT_MASK(x);

  return _stSelect; // raw ST_Select[3:0]
}

void AR49::read_Status()
{
  // OC=0x9C Read Status:
  // MISO: 9C 00 ERR[7:0] ERR[15:8] ERR[23:16] ERR[31:24] WARN[7:0] ... WARN[31:24]
  uint8_t z = 0;
  uint8_t b[8] = {0};

  _spi->beginTransaction(_settings);
  select();

  const uint8_t echo = _spi->transfer(0x9C);
  z = _spi->transfer(0x00); // documented 00h byte
  for (uint8_t i = 0; i < 8; i++) {
    b[i] = _spi->transfer(0x00);
  }

  deselect();
  _spi->endTransaction();

  if (echo != 0x9C) {
    Serial.println("AR49: Read Status echo mismatch");
    return;
  }
  (void)z;

  const uint32_t err  = (uint32_t)b[0]
                      | ((uint32_t)b[1] << 8)
                      | ((uint32_t)b[2] << 16)
                      | ((uint32_t)b[3] << 24);

  const uint32_t warn = (uint32_t)b[4]
                      | ((uint32_t)b[5] << 8)
                      | ((uint32_t)b[6] << 16)
                      | ((uint32_t)b[7] << 24);

  Serial.print("ERR=0x");  Serial.println(err, HEX);
  Serial.print("WARN=0x"); Serial.println(warn, HEX);
}
