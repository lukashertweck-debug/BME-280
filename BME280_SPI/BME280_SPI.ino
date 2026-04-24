/*
 * BME280 sensor driver over SPI for STM32 (Arduino IDE, 3.3 V logic).
 *
 * Wiring (default hardware SPI on most STM32duino boards, e.g. Nucleo/BluePill):
 *   BME280 VCC  -> 3.3 V
 *   BME280 GND  -> GND
 *   BME280 SCK  -> SPI1 SCK   (PA5)
 *   BME280 SDO  -> SPI1 MISO  (PA6)
 *   BME280 SDI  -> SPI1 MOSI  (PA7)
 *   BME280 CSB  -> CS pin     (PA4, see BME280_CS below)
 *
 * Output: Temperature [degC], Pressure [hPa], Humidity [%RH] via Serial @115200.
 */

#include <Arduino.h>
#include <SPI.h>

static const uint8_t BME280_CS = PA4;

static const uint8_t REG_ID         = 0xD0;
static const uint8_t REG_RESET      = 0xE0;
static const uint8_t REG_CTRL_HUM   = 0xF2;
static const uint8_t REG_STATUS     = 0xF3;
static const uint8_t REG_CTRL_MEAS  = 0xF4;
static const uint8_t REG_CONFIG     = 0xF5;
static const uint8_t REG_PRESS_MSB  = 0xF7;
static const uint8_t REG_CALIB_00   = 0x88;
static const uint8_t REG_CALIB_26   = 0xE1;

static const uint8_t CHIP_ID_BME280 = 0x60;
static const uint8_t RESET_WORD     = 0xB6;

static SPISettings bmeSpi(10000000, MSBFIRST, SPI_MODE0);

struct BME280Calib {
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;
  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  int8_t   dig_H6;
};

static BME280Calib calib;
static int32_t t_fine;

static void bmeWrite(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(bmeSpi);
  digitalWrite(BME280_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(BME280_CS, HIGH);
  SPI.endTransaction();
}

static uint8_t bmeRead8(uint8_t reg) {
  SPI.beginTransaction(bmeSpi);
  digitalWrite(BME280_CS, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(BME280_CS, HIGH);
  SPI.endTransaction();
  return value;
}

static void bmeReadBurst(uint8_t reg, uint8_t *buf, size_t len) {
  SPI.beginTransaction(bmeSpi);
  digitalWrite(BME280_CS, LOW);
  SPI.transfer(reg | 0x80);
  for (size_t i = 0; i < len; ++i) {
    buf[i] = SPI.transfer(0x00);
  }
  digitalWrite(BME280_CS, HIGH);
  SPI.endTransaction();
}

static void loadCalibration() {
  uint8_t b1[26];
  bmeReadBurst(REG_CALIB_00, b1, sizeof(b1));
  calib.dig_T1 = (uint16_t)(b1[0]  | (b1[1]  << 8));
  calib.dig_T2 = (int16_t) (b1[2]  | (b1[3]  << 8));
  calib.dig_T3 = (int16_t) (b1[4]  | (b1[5]  << 8));
  calib.dig_P1 = (uint16_t)(b1[6]  | (b1[7]  << 8));
  calib.dig_P2 = (int16_t) (b1[8]  | (b1[9]  << 8));
  calib.dig_P3 = (int16_t) (b1[10] | (b1[11] << 8));
  calib.dig_P4 = (int16_t) (b1[12] | (b1[13] << 8));
  calib.dig_P5 = (int16_t) (b1[14] | (b1[15] << 8));
  calib.dig_P6 = (int16_t) (b1[16] | (b1[17] << 8));
  calib.dig_P7 = (int16_t) (b1[18] | (b1[19] << 8));
  calib.dig_P8 = (int16_t) (b1[20] | (b1[21] << 8));
  calib.dig_P9 = (int16_t) (b1[22] | (b1[23] << 8));
  calib.dig_H1 = b1[25];

  uint8_t b2[7];
  bmeReadBurst(REG_CALIB_26, b2, sizeof(b2));
  calib.dig_H2 = (int16_t)(b2[0] | (b2[1] << 8));
  calib.dig_H3 = b2[2];
  calib.dig_H4 = (int16_t)((((int8_t)b2[3]) << 4) | (b2[4] & 0x0F));
  calib.dig_H5 = (int16_t)((((int8_t)b2[5]) << 4) | (b2[4] >> 4));
  calib.dig_H6 = (int8_t)b2[6];
}

static int32_t compensateT(int32_t adc_T) {
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) *
                  ((int32_t)calib.dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
                    ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
                  ((int32_t)calib.dig_T3)) >> 14;
  t_fine = var1 + var2;
  return (t_fine * 5 + 128) >> 8;
}

static uint32_t compensateP(int32_t adc_P) {
  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) +
         ((var1 * (int64_t)calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
  if (var1 == 0) {
    return 0;
  }
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)calib.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
  return (uint32_t)p;
}

static uint32_t compensateH(int32_t adc_H) {
  int32_t v = t_fine - ((int32_t)76800);
  v = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) -
         (((int32_t)calib.dig_H5) * v)) + ((int32_t)16384)) >> 15) *
       (((((((v * ((int32_t)calib.dig_H6)) >> 10) *
             (((v * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
          ((int32_t)2097152)) * ((int32_t)calib.dig_H2) + 8192) >> 14));
  v = v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)calib.dig_H1)) >> 4);
  if (v < 0)         v = 0;
  if (v > 419430400) v = 419430400;
  return (uint32_t)(v >> 12);
}

static bool bmeBegin() {
  pinMode(BME280_CS, OUTPUT);
  digitalWrite(BME280_CS, HIGH);
  SPI.begin();
  delay(10);

  if (bmeRead8(REG_ID) != CHIP_ID_BME280) {
    return false;
  }

  bmeWrite(REG_RESET, RESET_WORD);
  delay(10);
  while (bmeRead8(REG_STATUS) & 0x01) {
    delay(2);
  }

  loadCalibration();

  bmeWrite(REG_CTRL_HUM, 0x01);
  bmeWrite(REG_CTRL_MEAS, (0x01 << 5) | (0x01 << 2) | 0x03);
  bmeWrite(REG_CONFIG, (0x04 << 2));
  return true;
}

static void bmeRead(float &tempC, float &pressHPa, float &humRH) {
  uint8_t raw[8];
  bmeReadBurst(REG_PRESS_MSB, raw, sizeof(raw));
  int32_t adc_P = ((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[2] >> 4);
  int32_t adc_T = ((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | (raw[5] >> 4);
  int32_t adc_H = ((uint32_t)raw[6] << 8)  |  raw[7];

  int32_t  T = compensateT(adc_T);
  uint32_t P = compensateP(adc_P);
  uint32_t H = compensateH(adc_H);

  tempC    = T / 100.0f;
  pressHPa = (P / 256.0f) / 100.0f;
  humRH    = H / 1024.0f;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  if (!bmeBegin()) {
    Serial.println("BME280 not found on SPI (check wiring / CS pin).");
    while (true) { delay(1000); }
  }
  Serial.println("BME280 ready.");
}

void loop() {
  float tempC, pressHPa, humRH;
  bmeRead(tempC, pressHPa, humRH);

  Serial.print("T = ");   Serial.print(tempC, 2);    Serial.print(" degC  ");
  Serial.print("P = ");   Serial.print(pressHPa, 2); Serial.print(" hPa  ");
  Serial.print("RH = ");  Serial.print(humRH, 2);    Serial.println(" %");

  delay(1000);
}
